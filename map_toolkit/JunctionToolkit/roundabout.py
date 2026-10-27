"""Experimental four-arm roundabout: two circulating lanes, two lanes each way.

Generate only; never connects to or switches a running CARLA server.
"""
import argparse,hashlib,json,math
import xml.etree.ElementTree as ET
from pathlib import Path
from runtime_config import map_output


def node(parent,tag,**attrs):
    return ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})


def generate(out,radius=30.,arm_length=120.,width=3.5):
    if not 26<=radius<=60:raise ValueError('ring radius must be 26..60 m')
    if not 80<=arm_length<=300:raise ValueError('arm length must be 80..300 m')
    if not 3<=width<=4:raise ValueError('lane width must be 3..4 m')
    out=map_output(out)
    if out.exists() and any(out.iterdir()):raise ValueError('Output must be empty; existing maps are never overwritten')
    root=ET.Element('OpenDRIVE');h=node(root,'header',revMajor=1,revMinor=4,name=out.name,version=1,north=0,south=0,east=0,west=0)
    node(h,'geoReference').text='+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs'
    rr=radius-width/2;delta=math.radians(30);mouth=radius+24
    specs={};junctions=[];signalmap={};paths=[]
    def road(rid,length,geometry,left=0,right=1,junction=-1,pre=None,suc=None,pre_lane=-1,suc_lane=-1):
        r=node(root,'road',name='roundabout_'+str(rid),length=length,id=rid,junction=junction);link=node(r,'link')
        for tag,info in [('predecessor',pre),('successor',suc)]:
            if info:
                typ,target,contact=info;attrs=dict(elementType=typ,elementId=target)
                if typ=='road':attrs['contactPoint']=contact
                node(link,tag,**attrs)
        typ=node(r,'type',s=0,type='town');node(typ,'speed',max=20,unit='km/h')
        plan=node(r,'planView');g=node(plan,'geometry',s=0,x=geometry[0],y=geometry[1],hdg=geometry[2],length=length)
        node(g,geometry[3],**geometry[4]);node(node(r,'elevationProfile'),'elevation',s=0,a=0,b=0,c=0,d=0)
        section=node(node(r,'lanes'),'laneSection',s=0)
        centre=node(node(section,'center'),'lane',id=0,type='none',level='false')
        node(centre,'roadMark',sOffset=0,type='solid',weight='standard',color='yellow',width=.15)
        for side,count,sign in [('left',left,1),('right',right,-1)]:
            if not count:continue
            group=node(section,side)
            for k in range(1,count+1):
                lane=node(group,'lane',id=sign*k,type='driving',level='false');ln=node(lane,'link')
                if junction!=-1:
                    node(ln,'predecessor',id=pre_lane[k-1] if isinstance(pre_lane,list) else pre_lane)
                    node(ln,'successor',id=suc_lane[k-1] if isinstance(suc_lane,list) else suc_lane)
                node(lane,'width',sOffset=0,a=width,b=0,c=0,d=0)
                node(lane,'roadMark',sOffset=0,type='solid' if k==count else 'broken',weight='standard',color='white',width=.15,laneChange='none')
        specs[rid]=dict(length=length,geometry=geometry,junction=junction)
        return r
    def arc(theta,sweep):
        return rr*sweep,(rr*math.cos(theta),rr*math.sin(theta),theta+math.pi/2,'arc',dict(curvature=1/rr))
    def add_signals(r,rid,jid,slot,role):
        spec=specs[rid];station=spec['length']-3
        x,y,heading,kind,params=spec['geometry']
        if kind=='arc':
            k=params['curvature'];end_heading=heading+k*station
            x+=(math.sin(end_heading)-math.sin(heading))/k
            y-=(math.cos(end_heading)-math.cos(heading))/k;heading=end_heading
        else:x+=station*math.cos(heading);y+=station*math.sin(heading)
        signals=node(r,'signals')
        for lane in (1,2):
            sid=(1000 if role=='entry' else 20000)+rid*10+lane;lateral=-(lane-.5)*width-6
            sig=node(signals,'signal',s=station,t=lateral,id=sid,name=role+'_lane_'+str(sid),dynamic='yes',orientation='+',zOffset=0,country='OpenDRIVE',type='1000001',subtype=-1,value=-1,height=3,width=.3,hOffset=0)
            node(sig,'validity',fromLane=-lane,toLane=-lane)
            node(sig,'positionInertial',x=x-lateral*math.sin(heading),y=y+lateral*math.cos(heading),z=0,hdg=heading,pitch=0,roll=0)
            ctrl=node(root,'controller',id=2000+sid,name='Independent_'+str(sid),sequence=lane);node(ctrl,'control',signalId=sid,type=0)
            signalmap[str(sid)]=dict(road_id=rid,junction_id=jid,phase_slot=slot+lane-1,approach_slot=slot//2,reference_s=spec['length']-6,incoming_lanes=[-lane],role=role)
    def curve(p0,h0,p1,h1):
        # Cubic Hermite reference curve with endpoint tangent continuity.
        dx,dy=p1[0]-p0[0],p1[1]-p0[1];u=dx*math.cos(h0)+dy*math.sin(h0);v=-dx*math.sin(h0)+dy*math.cos(h0)
        handle=math.hypot(dx,dy)*1.05;eu=handle*math.cos(h1-h0);ev=handle*math.sin(h1-h0)
        cu,du=3*u-2*handle-eu,-2*u+handle+eu;cv,dv=3*v-ev,-2*v+ev
        def speed(t):return math.hypot(handle+2*cu*t+3*du*t*t,2*cv*t+3*dv*t*t)
        steps=1000;length=sum((speed(k/steps)+4*speed((k+.5)/steps)+speed((k+1)/steps))/(6*steps) for k in range(steps))
        return length,(p0[0],p0[1],h0,'paramPoly3',dict(aU=0,bU=handle,cU=cu,dU=du,aV=0,bV=0,cV=cv,dV=dv,pRange='normalized'))
    # Arms indexed west, south, east, north; circulation increases polar angle.
    for i,name in enumerate(('west','south','east','north')):
        theta=math.pi+i*math.pi/2;heading=theta+math.pi;jid=10+i
        start=((mouth+arm_length)*math.cos(theta),(mouth+arm_length)*math.sin(theta))
        r=road(i+1,arm_length,(*start,heading,'line',{}),left=2,right=2,suc=('junction',jid,None))
        add_signals(r,i+1,jid,4*i,'entry')
        length,g=arc(theta+delta,math.pi/2-2*delta)
        r=road(20+i,length,g,right=2,pre=('junction',jid,None),suc=('junction',10+(i+1)%4,None))
        add_signals(r,20+i,10+(i+1)%4,4*((i+1)%4)+2,'circulating')
        junctions.append(dict(id=jid,center=[mouth*math.cos(theta),mouth*math.sin(theta)],roads=[i+1,20+(i-1)%4],lengths=[arm_length,length],arms={name:i+1}))
    for i in range(4):
        theta=math.pi+i*math.pi/2;jid=10+i;incoming=20+(i-1)%4;outgoing=20+i;arm=i+1
        j=node(root,'junction',id=jid,name='Roundabout_access_'+str(i));counter=0
        def connect(rid,length,g,source,source_lane,target,target_lane,target_contact,count=1):
            nonlocal counter
            road(rid,length,g,right=count,junction=jid,pre=('road',source,'end'),suc=('road',target,target_contact),pre_lane=source_lane,suc_lane=target_lane)
            con=node(j,'connection',id=counter,incomingRoad=source,connectingRoad=rid,contactPoint='start');counter+=1
            for k in range(count):
                src=source_lane[k] if isinstance(source_lane,list) else source_lane;dst=target_lane[k] if isinstance(target_lane,list) else target_lane
                node(con,'laneLink',**{'from':src,'to':-(k+1)})
                paths.append(dict(road=rid,lane=-(k+1),source=source,source_lane=src,target=target,target_lane=dst))
        length,g=arc(theta-delta,2*delta);connect(100+i*10,length,g,incoming,[-1,-2],outgoing,[-1,-2],'start',count=2)
        for lane in (1,2):
            # Parent +lane is opposite reference direction; connector -1 aligns
            # with it by using the parent left edge and outward end tangent.
            hin=theta+math.pi;left=(-math.sin(hin),math.cos(hin));end=(mouth*math.cos(theta),mouth*math.sin(theta))
            p0=(end[0]-left[0]*(lane-1)*width,end[1]-left[1]*(lane-1)*width)
            lane_radius=rr+(lane-1)*width
            p1=(lane_radius*math.cos(theta+delta),lane_radius*math.sin(theta+delta))
            length,g=curve(p0,hin,p1,theta+delta+math.pi/2);connect(100+i*10+lane,length,g,arm,-lane,outgoing,-lane,'start')
            p0=(lane_radius*math.cos(theta-delta),lane_radius*math.sin(theta-delta))
            p1=(end[0]+left[0]*(lane-1)*width,end[1]+left[1]*(lane-1)*width)
            length,g=curve(p0,theta-delta+math.pi/2,p1,theta);connect(100+i*10+lane+2,length,g,incoming,-lane,arm,lane,'end')
        for sid,cfg in signalmap.items():
            if cfg['junction_id']==jid:node(j,'controller',id=2000+int(sid),type=0,sequence=cfg['phase_slot'])
    root[:]=[h]+root.findall('road')+root.findall('controller')+root.findall('junction')
    out.mkdir(parents=True,exist_ok=True)
    ET.indent(root);ET.ElementTree(root).write(str(out/'map.xodr'),encoding='utf-8',xml_declaration=True)
    config=dict(type='roundabout',name=out.name,rows=1,columns=1,incoming_lanes=2,outgoing_lanes=2,circulating_lanes=2,
                internal_signals=True,lane_routing='lane_preserving_no_lane_changes',
                ring_centerline_radius_m=radius,junction_radius_m=radius,arm_length_m=arm_length,lane_width_m=width,spacing_m=2*(mouth+arm_length),sidewalk_width_m=2,roadside_props=False,seed=42)
    design=dict(schema_version=1,signal_policy='one_signal_per_incoming_lane',status='experimental_offline_only',config=config,
                junctions=junctions,signals=signalmap,connections=paths,ring_roads=[20,21,22,23],ring_continuations=[100,110,120,130],
                coordinate_system='OpenDRIVE XY; CARLA flips Y',circulation='counterclockwise',phase_plan=None,
                opendrive_sha256=hashlib.sha256((out/'map.xodr').read_bytes()).hexdigest(),road_geometry=specs,
                limits=['Lane 1 enters inner ring lane and exits to lane 1; lane 2 uses outer ring lane and exit lane 2. No circulating lane changes.',
                        'Inner-lane entry/exit trajectories cross the outer circulating stream; signal conflict design is required.',
                        'Four CARLA junction IDs form one logical roundabout; existing single-junction evaluator is not yet integrated.',
                        'No signal phase assignment or live driving certification.'])
    road_groups={1:3,2:2,3:1,4:4,20:7,21:6,22:5,23:8}
    names={1:'east_entry',2:'south_entry',3:'west_entry',4:'north_entry',5:'northeast_internal',6:'southeast_internal',7:'southwest_internal',8:'northwest_internal'}
    groups={str(g):dict(name=name,signal_ids=[]) for g,name in names.items()}
    for sid,cfg in signalmap.items():
        group=road_groups[cfg['road_id']];cfg['signal_group']=group;groups[str(group)]['signal_ids'].append(sid)
    design['signal_groups']=groups
    phase_example=dict(status='visual_demo_only_not_traffic_validated',source='User-supplied eight-group roundabout phase diagram',
        phases=[dict(name='A',green_groups=[5,6,7,8],seconds=24),dict(name='B',green_groups=[2,4,6,8],seconds=36),
                dict(name='C',green_groups=[5,6,7,8],seconds=24),dict(name='D',green_groups=[1,3,5,7],seconds=36)],
        yellow_seconds=3,all_red_seconds=1,
        transition_note='3s yellow + 1s all-red are provisional demo additions, not provided by the reference image; total demo cycle 136s.',
        conflict_note='Inner-lane entry/exit crosses outer circulating trajectories. This demo rejects vehicles; no safe traffic phase claim.')
    (out/'phase_example.json').write_text(json.dumps(phase_example,indent=2),encoding='utf-8')
    for name,data in [('config.json',config),('manifest.json',design)]:
        (out/name).write_text(json.dumps(data,indent=2),encoding='utf-8')
    print('Generated roundabout:',out,'signals:',len(signalmap),'(8 entry + 8 circulating)')
    return design


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',required=True);p.add_argument('--radius',type=float,default=30);p.add_argument('--arm-length',type=float,default=120);p.add_argument('--lane-width',type=float,default=3.5)
    a=p.parse_args();generate(a.out,a.radius,a.arm_length,a.lane_width)
