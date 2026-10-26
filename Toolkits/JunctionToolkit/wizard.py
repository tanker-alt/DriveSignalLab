"""Interactive map configuration and generation; compatible with Python 3.7+."""
import argparse
import datetime
import json
import math
import re
import subprocess
import sys
from pathlib import Path
from layout import ARM_NAMES, arm_turns

ROOT = Path(__file__).resolve().parent


def map_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', value):
        raise ValueError('名称须为 1–64 位英文字母、数字、下划线或短横线，首位为字母/数字')
    if value.upper() in {'CON', 'PRN', 'AUX', 'NUL'} | {'COM'+str(i) for i in range(1,10)} | {'LPT'+str(i) for i in range(1,10)}:
        raise ValueError('该名称是 Windows 保留名称')
    return value


def number(kind, low, high):
    def parse(value):
        result = kind(value)
        if not math.isfinite(result) or not low <= result <= high:
            raise ValueError('请输入 {} 至 {} 之间的数值'.format(low, high))
        return result
    return parse


def ask(label, default, parse=str):
    while True:
        try:
            return parse(input('{} [{}]: '.format(label, default)).strip() or str(default))
        except ValueError as exc:
            print('输入无效：', exc)


def yes(label, default=False):
    def parse(value):
        if value.lower() in ('y','yes','是','1'): return True
        if value.lower() in ('n','no','否','0'): return False
        raise ValueError('请输入 y 或 n')
    return ask(label, 'y' if default else 'n', parse)


def turns(value):
    result = [part.strip().lower() for part in value.split(',')]
    if not result or len(set(result)) != len(result) or set(result)-{'left','straight','right'}:
        raise ValueError('使用 left、straight、right；多个转向用英文逗号分隔，不重复')
    return result


def config(args, interactive):
    name = args.name or 'map_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    name = ask('地图名称', name, map_name) if interactive else map_name(name)
    kind = args.type
    if interactive:
        def choose(value):
            if value not in ('cross','grid','t'): raise ValueError('输入 cross（十字）、grid（规则网格）或 t（T 型）')
            return value
        kind = ask('地图类型：cross / grid / t', kind, choose)
    rows, columns = (args.rows if args.rows is not None else 2), (args.columns if args.columns is not None else 2)
    if kind == 'grid' and interactive:
        rows = ask('网格行数', rows, number(int,1,5))
        columns = ask('网格列数', columns, number(int,1,5))
    if kind in ('cross','t'):
        if not interactive and (args.rows is not None or args.columns is not None):
            raise ValueError('单路口不接受行列设置；需要组合请使用 --type grid')
        rows = columns = 1
    fields = [('incoming_lanes','每方向进口车道数（出口同数）',args.lanes,int,1,4),
              ('arm_length_m','外围道路臂长/米',args.arm_length,float,80,500),
              ('lane_width_m','车道宽度/米',args.lane_width,float,3,4.5)]
    c = dict(name=name,rows=rows,columns=columns)
    if kind=='t':
        def missing(value):
            if value not in ARM_NAMES:raise ValueError('输入 north/south/east/west')
            return value
        c['type']='t'
        c['missing_arm']=ask('缺失方向（北=north，南=south，东=east，西=west）',args.missing_arm or 'north',missing) if interactive else args.missing_arm or 'north'
    elif args.missing_arm:
        raise ValueError('--missing-arm 仅用于 --type t')
    for key,label,default,typ,low,high in fields:
        c[key] = ask(label,default,number(typ,low,high)) if interactive else number(typ,low,high)(default)
    n = c['incoming_lanes']
    minimum = n*c['lane_width_m']+8
    radius = args.radius if args.radius is not None else max(22.,minimum)
    c['junction_radius_m'] = ask('路口中心至道路端点距离/米',radius,number(float,minimum,1000)) if interactive else number(float,minimum,1000)(radius)
    spacing = args.spacing if args.spacing is not None else max(344.,2*(c['junction_radius_m']+80))
    c['spacing_m'] = ask('路口中心间距/米',spacing,number(float,2*(c['junction_radius_m']+80),10000)) if interactive and kind=='grid' else spacing
    c['sidewalk_width_m'] = ask('人行道宽度/米（0 关闭）',args.sidewalk,number(float,0,5)) if interactive else args.sidewalk
    c['roadside_props'] = yes('添加长椅等路边小设施（没有建筑）',True) if interactive else not args.no_props
    c['seed'] = args.seed
    if kind=='t':
        if args.turns:raise ValueError('T 型路口每方向转向不同，请用交互模式逐方向设置，不能用共享 --turns')
        c['arm_turns']={ARM_NAMES[i]:lanes for i,lanes in arm_turns(c).items()}
        print('各进口从内侧至外侧的转向分配：',c['arm_turns'])
        if interactive and not yes('使用上述默认转向分配',True):
            for direction,lanes in list(c['arm_turns'].items()):
                while True:
                    proposed=[ask('{} 进口车道 {} 允许转向'.format(direction,i+1),','.join(t),turns) for i,t in enumerate(lanes)]
                    previous=c['arm_turns'][direction];c['arm_turns'][direction]=proposed
                    try:arm_turns(c);break
                    except ValueError as exc:
                        c['arm_turns'][direction]=previous;print('该分配不可用，请重新输入：',exc)
    else:
        c['allowed_turns'] = {1:[['left','straight','right']],2:[['left','straight'],['right']],
                              3:[['left'],['straight'],['right']],4:[['left'],['straight'],['straight'],['right']]}[n]
        print('从内侧至外侧的转向分配：',c['allowed_turns'])
        if interactive and not yes('使用上述默认转向分配',True):
            c['allowed_turns'] = [ask('车道 {} 允许转向（英文逗号分隔）'.format(i+1),','.join(t),turns) for i,t in enumerate(c['allowed_turns'])]
        if args.turns:c['allowed_turns'] = [turns(t) for t in args.turns.split(';')]
        if len(c['allowed_turns']) != n: raise ValueError('--turns 须为每车道一组，用分号分隔')
    number(int,1,5)(rows);number(int,1,5)(columns)
    number(float,0,5)(c['sidewalk_width_m'])
    number(float,2*(c['junction_radius_m']+80),10000)(c['spacing_m'])
    return c


def main():
    p = argparse.ArgumentParser(description='输入参数，自动保存配置、生成并检查地图。未带 --name 时启动交互向导。')
    p.add_argument('--name');p.add_argument('--interactive',action='store_true')
    p.add_argument('--type',choices=['cross','grid','t'],default='cross')
    p.add_argument('--missing-arm',choices=['north','south','east','west'],help='T 型路口缺失方向，默认 north')
    p.add_argument('--lanes',type=int,default=3);p.add_argument('--arm-length',type=float,default=150)
    p.add_argument('--rows',type=int);p.add_argument('--columns',type=int)
    p.add_argument('--lane-width',type=float,default=3.5);p.add_argument('--radius',type=float)
    p.add_argument('--spacing',type=float);p.add_argument('--sidewalk',type=float,default=2)
    p.add_argument('--seed',type=int,default=42);p.add_argument('--no-props',action='store_true')
    p.add_argument('--turns',help='由内向外，如 "left;straight;right"')
    p.add_argument('--load',action='store_true',help='拓扑检查通过后加载 CARLA')
    a = p.parse_args();interactive = a.interactive or a.name is None
    print('支持十字路口、单个 T 型路口和十字规则网格；各保留方向进出车道同数。环岛和各方向独立车道数尚未开放。')
    c = config(a,interactive)
    out = ROOT.parents[1]/'Maps/MapDataset/maps'/c['name'];path = ROOT/'configs/generated'/(c['name']+'.json')
    if out.exists() or path.exists(): raise ValueError('名称已使用，请换一个名称；不会覆盖已有地图或配置')
    print(json.dumps(c,ensure_ascii=False,indent=2))
    print('地图目录：',out)
    if interactive and not yes('确认生成',True):
        print('已取消，没有写入文件');return
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as f: json.dump(c,f,ensure_ascii=False,indent=2)
    subprocess.run([sys.executable,'-B',str(ROOT/'toolkit.py'),'generate','--config',str(path),'--out',str(out)],check=True)
    print('生成及拓扑检查通过。配置：',path)
    print('尚未做实车验收；新地图实验须重新准备 profile，不能套用默认 Cross3 配置。')
    load = a.load or (interactive and yes('现在加载到 CARLA（需已启动且无实验运行）',False))
    if load:
        subprocess.run([sys.executable,'-B',str(ROOT/'toolkit.py'),'view',str(out)],check=True)
    else:
        print('稍后查看：python toolkit.py view "{}"'.format(out))


if __name__ == '__main__':
    try: main()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print('未完成：{}\n若生成阶段失败，已写入的配置/候选目录会保留，请检查后使用新名称重试。'.format(exc),file=sys.stderr)
        raise SystemExit(1)
    except (KeyboardInterrupt,EOFError):
        print('\n已中止向导；若已开始生成，请检查保留的输出。');raise SystemExit(130)
