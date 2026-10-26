"""Switch maps, discover compatible signal junctions and prepare shared profiles.

Run without arguments for an interactive workflow. Explicit commands are also
available for scripts. Stop-line visual confirmation is never inferred.
"""
import argparse
import hashlib
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4
from evaluation.map_context import junctions, read_inventory
from evaluation.approach_geometry import build as build_geometry, DEFAULT_APPROACH_CONFIG
from evaluation.topology import build as build_lines
from evaluation.detection_lines import DetectionLines
from evaluation.map_profile import ROOT, ACTIVE_PROFILE


def write_json(path,data):
    with Path(path).open('x',encoding='utf-8') as handle:
        json.dump(data,handle,ensure_ascii=False,indent=2)


def switch_map(client,name):
    available=client.get_available_maps()
    matches=[p for p in available if p==name or p.split('/')[-1]==name]
    if len(matches)!=1:
        raise ValueError('Map name is missing or ambiguous; use --list-maps')
    world=client.get_world()
    if world.get_map().name.split('/')[-1]==matches[0].split('/')[-1]:
        return world
    actors=world.get_actors()
    if (world.get_settings().synchronous_mode or actors.filter('vehicle.*') or
            actors.filter('walker.*') or actors.filter('sensor.*')):
        raise ValueError('End the current experiment and clean its vehicles/sensors before switching maps')
    world=client.load_world(matches[0])
    if world.get_map().name.split('/')[-1]!=matches[0].split('/')[-1]:
        raise RuntimeError('Server did not load the selected map')
    return world


def inspect_junction(world,jid,stop_config=None,detection_config=None):
    """Structural capability check, not a claim of physical metric acceptance."""
    lights=list(world.get_traffic_lights_in_junction(jid))
    result=dict(junction_id=jid,status='unsupported',reasons=[],signal_count=len(lights),
                lanes_per_signal=[],calibration_status='reference_only')
    if not lights:
        result['reasons']=['无信号路口，不在当前工作流支持范围']
        return result,None
    poles=[light.get_pole_index() for light in lights]
    if len(set(poles))!=len(poles):
        result['reasons']=['灯杆 pole_index 重复，当前固定配时控制器不支持']
        return result,None
    inventory=read_inventory(world,jid)
    stops=[light['stop_waypoints'] for light in inventory['traffic_lights']]
    result['lanes_per_signal']=[len({(s['road_id'],s['section_id'],s['lane_id']) for s in group}) for group in stops]
    if not all(stops):
        result['reasons']=['至少一盏控制灯没有可用停止参考车道']
        return result,None
    ownership={}
    for light in inventory['traffic_lights']:
        for stop in light['stop_waypoints']:
            key=(stop['road_id'],stop['section_id'],stop['lane_id'])
            ownership.setdefault(key,set()).add(light['opendrive_id'])
    if any(len(ids)>1 for ids in ownership.values()):
        result['reasons']=['同一进口车道关联多盏控制灯，需先定义信号映射']
        return result,None
    config=json.loads(DEFAULT_APPROACH_CONFIG.read_text(encoding='utf-8'))
    try:
        geometry=build_geometry(world,inventory,config,stop_config)
        invalid=[lane['id'] for lane in geometry['lanes'] if len(lane['samples'])<2]
        if invalid:
            raise ValueError('进口缺少有效上游几何，需检查停止参考点及路口内部拓扑：'+', '.join(invalid))
    except ValueError as exc:
        result['reasons']=[str(exc)]
        return result,None
    result['calibration_status']=('surveyed_stop_lines' if geometry['lanes'] and
        all(lane['reference_kind']=='surveyed_stop_line' for lane in geometry['lanes']) else
        'partial_or_waypoint_calibration' if stop_config else 'reference_only')
    try:
        lines=detection_config if detection_config is not None else build_lines(world,jid,approach_geometry=geometry)
        if (lines['map_name'].split('/')[-1]!=inventory['map_name'].split('/')[-1] or
                int(lines['junction_id'])!=jid or any(int(line['junction_id'])!=jid for line in lines['lines'])):
            raise ValueError('检测线地图或路口不匹配')
        DetectionLines(lines['lines'])
        if not {'arrival','exit'}.issubset({line['kind'] for line in lines['lines'] if line.get('enabled',True)}):
            raise ValueError('需要有效的 arrival 和 exit 检测线')
    except ValueError as exc:
        result.update(status='needs_detection_config',reasons=[str(exc)])
        return result,None
    result.update(status='ready',reasons=[],approach_count=len(geometry['lanes']),
                  detection_line_count=len(lines['lines']),
                  note='结构预检通过；不代表逐转向独立车道、相位安全或物理指标已验收')
    return result,dict(inventory=inventory,approach_geometry=geometry,detection_config=lines,
                       stop_reference_config=stop_config)


def scan(world):
    results=[]
    for jid in sorted(junctions(world)):
        results.append(inspect_junction(world,jid)[0])
    return dict(map_name=world.get_map().name,
        opendrive_sha256=hashlib.sha256(world.get_map().to_opendrive().encode('utf-8')).hexdigest(),
        junctions=results)


def prepare_profile(world,jid,output,host='localhost',port=2000,stop_config=None,detection_config=None):
    result,files=inspect_junction(world,jid,stop_config,detection_config)
    if result['status']!='ready':
        raise ValueError('路口不可直接配置：'+'; '.join(result['reasons']))
    output=Path(output)
    output.mkdir(parents=True,exist_ok=False)
    inventory=files['inventory']
    profile=dict(schema_version='map_profile.1',map_name=inventory['map_name'],junction_id=jid,
        opendrive_sha256=inventory['opendrive_sha256'],server=dict(host=host,port=port),
        calibration_status=result['calibration_status'],preflight=result,files={})
    for name,data in files.items():
        if data is None:
            continue
        path=output/(name+'.json');write_json(path,data)
        profile['files'][name]=dict(path=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    write_json(output/'profile.json',profile)
    return output/'profile.json'


def activate(profile_path,active_path=ACTIVE_PROFILE):
    from evaluation.map_profile import read_profile
    read_profile(profile_path)
    active_path=Path(active_path);active_path.parent.mkdir(parents=True,exist_ok=True)
    temporary=active_path.with_name(active_path.name+'.'+uuid4().hex+'.tmp')
    write_json(temporary,dict(profile=os.path.relpath(str(Path(profile_path).resolve()),str(active_path.parent.resolve()))))
    temporary.replace(active_path)


def calibrate(world,jid,output,host,port):
    """Human alignment on the live render; never auto-accept CARLA references."""
    import carla
    from Toolkits.CollectionTools.stop_reference_review import main as review
    inventory=read_inventory(world,jid)
    if not inventory['traffic_lights']:
        raise ValueError('无信号路口不支持校准流程')
    if world.get_settings().synchronous_mode:
        raise ValueError('先结束同步实验，再执行现场校准')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    result={key:inventory[key] for key in ('map_name','junction_id','opendrive_sha256')}
    result.update(schema_version='stop_line.1',approaches={})
    stops={}
    for light in inventory['traffic_lights']:
        for stop in light['stop_waypoints']:
            stops['{}_{}_{}'.format(stop['road_id'],stop['section_id'],stop['lane_id'])]=stop
    print('校准边界：车辆接近侧的白线边缘。输入沿原方向的前移米数（可负）；a 确认并拍照；q 中止。')
    print('紫线显示 5 秒；Enter 重画。旧线消失后再判读。弯道/斜线无法对齐时中止并使用完整人工配置。')
    for key,stop in stops.items():
        offset=0.;angle=math.radians(stop['yaw']);nx,ny=math.cos(angle),math.sin(angle)
        while True:
            p=dict(x=stop['location']['x']+offset*nx,y=stop['location']['y']+offset*ny,z=stop['location']['z'])
            width=stop['lane_width_m']
            ends=[carla.Location(p['x']-sign*ny*width/2,p['y']+sign*nx*width/2,p['z']+.1) for sign in (-1,1)]
            world.debug.draw_line(*ends,thickness=.04,color=carla.Color(255,0,255),life_time=5)
            world.get_spectator().set_transform(carla.Transform(carla.Location(p['x'],p['y'],p['z']+16),
                                                             carla.Rotation(pitch=-90,yaw=stop['yaw'])))
            command=input('{} offset={:.3f}m > '.format(key,offset)).strip().lower()
            if command=='q':
                raise ValueError('校准已中止，未启用新配置')
            if command=='a':
                wp=world.get_map().get_waypoint(carla.Location(**p),project_to_road=False)
                if wp is None:
                    print('位置不在车道上，请调整');continue
                try:
                    error=float(input('输入目视误差估计（米，例如 0.15）> '))
                    if not math.isfinite(error) or error<0:
                        raise ValueError()
                except ValueError:
                    print('误差必须是非负有限米数，请重新确认');continue
                entry=dict(calibration_id=output.name+'-'+key,stop_line=dict(center=p,normal_xy=[nx,ny],
                    reference_waypoint=dict(road_id=wp.road_id,section_id=wp.section_id,lane_id=wp.lane_id,s=wp.s),
                    convention='approach_side_edge',estimated_error_m=error,
                    evidence=str(output/key/(key+'.png')),method='interactive human alignment'))
                trial=dict(result,approaches=dict(result['approaches'],**{key:entry}))
                build_geometry(world,inventory,json.loads(DEFAULT_APPROACH_CONFIG.read_text()),trial)
                review(['--host',host,'--port',str(port),'--junction-id',str(jid),'--approach',key,
                        '--forward-m',str(offset),'--draw','--capture','--output',str(output/key)])
                result=trial;break
            if command:
                try:
                    value=float(command)
                    if not math.isfinite(value) or abs(value)>30:
                        raise ValueError()
                    offset=value
                except ValueError:
                    print('请输入 -30 到 30 之间的有限米数，或 a/q')
    path=output/'stop_reference_config.json';write_json(path,result)
    return path


def show_report(report,show_all=False):
    print('当前地图：'+report['map_name'])
    for item in report['junctions']:
        if not show_all and not item['signal_count']:
            continue
        label={'ready':'可配置','needs_detection_config':'需人工检测线','unsupported':'暂不支持'}[item['status']]
        print('{}  {}  信号数={}  各信号进口车道数={}  {}'.format(item['junction_id'],label,
            item['signal_count'],item['lanes_per_signal'],'; '.join(item['reasons'])))
    print('可配置仅表示结构预检通过；真实停止线与物理指标需分别验收。')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='localhost');parser.add_argument('--port',type=int,default=2000)
    parser.add_argument('--list-maps',action='store_true');parser.add_argument('--map')
    parser.add_argument('--scan',action='store_true');parser.add_argument('--all-junctions',action='store_true')
    parser.add_argument('--report',type=Path);parser.add_argument('--junction-id',type=int)
    parser.add_argument('--stop-reference-config',type=Path);parser.add_argument('--detection-config',type=Path)
    parser.add_argument('--calibrate',action='store_true');parser.add_argument('--prepare',action='store_true')
    parser.add_argument('--activate',action='store_true');parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.calibrate and args.stop_reference_config:
        parser.error('--calibrate and --stop-reference-config are mutually exclusive')
    import carla
    client=carla.Client(args.host,args.port);client.set_timeout(60)
    interactive=not any((args.list_maps,args.map,args.scan,args.junction_id is not None,args.report,
                         args.calibrate,args.prepare,args.activate,args.all_junctions))
    if args.list_maps:
        print('\n'.join(client.get_available_maps()));return
    if interactive:
        print('可用地图：\n'+'\n'.join(p.split('/')[-1] for p in client.get_available_maps()))
        args.map=input('输入目标地图名，Enter 保留当前地图 > ').strip() or None
    world=switch_map(client,args.map) if args.map else client.get_world()
    report=scan(world);show_report(report,args.all_junctions)
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True);write_json(args.report,report)
    if interactive:
        value=input('选择“可配置”的路口 ID，Enter 仅查看后退出 > ').strip()
        if not value:
            return
        args.junction_id=int(value);args.prepare=True;args.activate=True
        choice=input('现在校准真实停止线？y=现场校准，n=先使用明确标记的 CARLA 参考点 > ').strip().lower()
        if choice not in ('y','n'):
            raise ValueError('请输入 y 或 n')
        args.calibrate=choice=='y'
    if not (args.prepare or args.activate or args.calibrate):
        return
    if args.junction_id is None:
        parser.error('--junction-id is required when preparing or calibrating')
    tag=world.get_map().name.split('/')[-1]+'_j'+str(args.junction_id)+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output=args.output or ROOT/'map_profiles'/tag
    if output.exists():
        raise ValueError('输出目录已存在，请选择新目录：'+str(output))
    if args.calibrate:
        args.stop_reference_config=calibrate(world,args.junction_id,output.with_name(output.name+'_survey'),args.host,args.port)
    stop=json.loads(args.stop_reference_config.read_text(encoding='utf-8-sig')) if args.stop_reference_config else None
    lines=json.loads(args.detection_config.read_text(encoding='utf-8-sig')) if args.detection_config else None
    path=prepare_profile(world,args.junction_id,output,args.host,args.port,stop,lines)
    if args.activate:
        activate(path)
    print('配置已生成：'+str(path))
    print('已设为当前配置。' if args.activate else '使用 --profile '+str(path)+' 显式选择此配置。')
    print('终端 1: python -B control_junction_phases.py --collect --collisions'+('' if args.activate else ' --profile "'+str(path)+'"'))
    print('终端 2: python -B spawn_junction_traffic.py'+('' if args.activate else ' --profile "'+str(path)+'"'))


if __name__=='__main__':
    try:
        main()
    except (ValueError,RuntimeError) as exc:
        raise SystemExit(str(exc))
