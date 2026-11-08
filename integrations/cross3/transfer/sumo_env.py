"""Real SUMO Cross3, imported from the same OpenDRIVE as CARLA."""
import json
import random
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from .core import Metrics, Signals, observation, phase_for

# transfer/ -> cross3/ -> integrations/
MAP = Path(__file__).resolve().parents[2]/'carla_handoff/upstream_20261023_v4/map_dataset/junction_toolkit/maps/cross3'

def demand(seed, horizon, rate=.035, egos=False):
    rng=random.Random(seed); rows=[]
    for m in json.loads((MAP/'manifest.json').read_text())['movements']:
        t=rng.uniform(0,5)
        while t<horizon:
            rows.append(dict(m,time=t,ego=False))
            t+=rng.expovariate(rate)
    if egos:
        for road in (1,3):
            m=next(m for m in json.loads((MAP/'manifest.json').read_text())['movements'] if m['road']==road and m['lane']==-2)
            rows.append(dict(m,time=0.,ego=True))
    rows.sort(key=lambda x:(x['time'],x['road'],x['lane']))
    for i,r in enumerate(rows): r['id']='v%d'%i
    return rows

class SumoEnv:
    dt=.1
    def __init__(self, folder, rows, seed, limit, policy=None, joint=False, train=False, **kwargs):
        if joint: raise ValueError('VLA joint training requires CARLA sensors, not SUMO')
        import traci
        self.folder=folder; folder.mkdir(parents=True,exist_ok=False)
        self.rows=rows; self.by_id={r['id']:r for r in rows}; self.limit=limit
        self.signals=Signals(); self.metrics=Metrics(rows); self.time=0.; self.ego_ids=[]
        self.connection=None
        net=folder/'cross3.net.xml'
        subprocess.run([shutil.which('netconvert') or 'netconvert','--opendrive-files',str(MAP/'map.xodr'),
                        '--opendrive.import-all-lanes','true','--output-file',str(net)],check=True,
                       stdout=subprocess.DEVNULL,stderr=(folder/'netconvert.log').open('w'))
        root=ET.parse(net).getroot()
        conns=[c for c in root.findall('connection') if c.get('tl')=='10']
        self.links={}
        for row in rows:
            key=(row['road'],row['lane'])
            matches=[c for c in conns if c.get('from')=='-%d'%row['road'] and
                     int(c.get('fromLane'))==4+row['lane'] and c.get('to')==str(row['target_road']) and
                     int(c.get('toLane'))==3-row['target_lane']]
            if len(matches)!=1: raise ValueError('OpenDRIVE movement does not map uniquely: %r'%key)
            self.links[int(matches[0].get('linkIndex'))]=phase_for(*key)
        # Validate all twelve links, even when the demand is sparse.
        for c in conns:
            self.links[int(c.get('linkIndex'))]=phase_for(-int(c.get('from')),int(c.get('fromLane'))-4)
        if len(self.links)!=12: raise ValueError('Expected twelve Cross3 controlled links')
        routes=ET.Element('routes')
        ET.SubElement(routes,'vType',id='car',accel='2.5',decel='4.5',length='4.7',minGap='2.5',maxSpeed=str(20/3.6),sigma='0')
        for row in rows:
            v=ET.SubElement(routes,'vehicle',id=row['id'],type='car',depart=str(row['time']),
                departLane=str(4+row['lane']),departPos='60',departSpeed='0',
                arrivalLane=str(3-row['target_lane']),arrivalPos='62')
            ET.SubElement(v,'route',edges='-%d %d'%(row['road'],row['target_road']))
        ET.ElementTree(routes).write(str(folder/'routes.rou.xml'))
        label='transfer_'+str(seed)+'_'+str(id(self))
        traci.start([shutil.which('sumo') or 'sumo','-n',str(net),'-r',str(folder/'routes.rou.xml'),
                     '--step-length',str(self.dt),'--seed',str(seed),'--time-to-teleport','-1',
                     '--no-step-log','true','--collision.action','warn','--collision.check-junctions','true'],
                    label=label,stdout=(folder/'sumo.log').open('w'))
        self.connection=traci.getConnection(label)
        self.trace=(folder/'decisions.jsonl').open('w')
    def vehicles(self):
        c=self.connection; result={}
        for vid in c.vehicle.getIDList():
            row=self.by_id[vid]; lane=c.vehicle.getLaneID(vid); edge=c.vehicle.getRoadID(vid)
            leader=c.vehicle.getLeader(vid,150)
            if leader and c.vehicle.getLaneID(leader[0]) != lane:
                leader=None
            result[vid]=dict(road=row['road'],lane=row['lane'],speed=c.vehicle.getSpeed(vid),
                approach=edge=='-%d'%row['road'],outgoing=edge==str(row['target_road']),
                distance=c.lane.getLength(lane)-c.vehicle.getLanePosition(vid),
                gap=leader[1]+c.vehicle.getMinGap(vid) if leader else None,
                leader_speed=c.vehicle.getSpeed(leader[0]) if leader else c.vehicle.getSpeed(vid),
                occupied=edge.startswith(':'))
        return result
    def observe(self):
        vs=self.vehicles()
        for row in self.rows:
            if row['time']<=self.time and row['id'] not in vs and self.metrics.rows[row['id']]['finish'] is None:
                vs[row['id']]=dict(road=row['road'],lane=row['lane'],speed=0.,approach=True,distance=90.)
        return observation(vs,self.signals)
    def step(self, action):
        obs=self.observe(); self.signals.request(action,obs['incoming'])
        before=self.metrics.vehicle_time; collisions=[]; elapsed=0.
        for _ in range(round(1/self.dt)):
            vs=self.vehicles()
            chars=['r']*12
            for link,phase in self.links.items():
                if phase==self.signals.active:
                    chars[link]={'GREEN':'G','YELLOW':'y','RED':'r'}[self.signals.stage]
            self.connection.trafficlight.setRedYellowGreenState('10',''.join(chars))
            self.connection.simulationStep(); self.time+=self.dt; elapsed+=self.dt
            collisions.extend((event.collider,event.victim) for event in self.connection.simulation.getCollisions())
            complete=self.connection.simulation.getArrivedIDList()
            # Pre-tick states integrate exposure over the interval; exits retain their last speed.
            self.metrics.sample(self.time,self.dt,vs,complete,collisions)
            current=self.vehicles()
            self.signals.tick(self.dt,any(v['occupied'] for v in current.values()))
            if self.time>=self.limit-1e-7 or collisions or all(r['finish'] is not None for r in self.metrics.rows.values()): break
        terminated=bool(collisions) or all(r['finish'] is not None for r in self.metrics.rows.values())
        truncated=self.time>=self.limit-1e-7 and not terminated
        reward=-(self.metrics.vehicle_time-before)/20-100*bool(collisions)
        self.trace.write(json.dumps(dict(time=self.time,action=action,phase=self.signals.active,stage=self.signals.stage,reward=reward))+'\n')
        return self.observe(),reward,terminated,truncated,dict(duration=elapsed,head_events=[])
    def close(self):
        if self.connection is not None: self.connection.close(); self.connection=None
        self.trace.close()
