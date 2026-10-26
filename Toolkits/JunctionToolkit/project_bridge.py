"""Explicit adapter into the existing evaluation project, with fresh map profiles."""
import argparse
import hashlib
import json
import runpy
import sys
from pathlib import Path
import carla


class Light:
    def __init__(self,native,cfg,road_map):
        self.native=native;self.cfg=cfg
        self.stops=[road_map.get_waypoint_xodr(cfg['road_id'],lane,cfg['reference_s']) for lane in cfg['incoming_lanes']]
        if any(w is None or w.is_junction for w in self.stops):raise ValueError('Invalid stop reference')
    def __getattr__(self,name):return getattr(self.native,name)
    def get_pole_index(self):return self.cfg['phase_slot']
    def get_stop_waypoints(self):return self.stops


class World:
    def __init__(self,native,design):
        self.native=native;self.design=design
        if not native.get_settings().synchronous_mode:native.wait_for_tick(15)
        digest=hashlib.sha256(native.get_map().to_opendrive().encode()).hexdigest()
        if digest!=design['opendrive_sha256']:raise ValueError('Map fingerprint mismatch: load the requested toolkit map first')
    def __getattr__(self,name):return getattr(self.native,name)
    def get_traffic_lights_in_junction(self,jid):
        configs={sid:cfg for sid,cfg in self.design['signals'].items() if cfg['junction_id']==jid}
        if not configs:return self.native.get_traffic_lights_in_junction(jid)
        lights={l.get_opendrive_id():l for l in self.native.get_traffic_lights_in_junction(jid)}
        # A newly connected synchronous client may not yet have its first
        # actor snapshot. Wait for the existing controller; never tick here.
        if not lights and self.native.get_settings().synchronous_mode:
            self.native.wait_for_tick(5)
            lights={l.get_opendrive_id():l for l in self.native.get_traffic_lights_in_junction(jid)}
        if set(lights)!=set(configs):raise ValueError('Signal snapshot mismatch. Finish/recover previous synchronous run before restarting.')
        road_map=self.native.get_map();result=[]
        for sid,cfg in sorted(configs.items()):
            light=lights[sid]
            actual={(w.road_id,w.lane_id) for w in light.get_affected_lane_waypoints()}
            if actual!={(cfg['road_id'],lane) for lane in cfg['incoming_lanes']}:raise ValueError('Signal validity mismatch '+sid)
            result.append(Light(light,cfg,road_map))
        return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['prepare','control','spawn']);p.add_argument('--map-dir',required=True)
    p.add_argument('--project',default=str(Path(__file__).resolve().parents[2]/'InformationCollector'))
    a,rest=p.parse_known_args();design=json.loads((Path(a.map_dir)/'manifest.json').read_text())
    native=carla.Client
    class Client:
        def __init__(self,*args,**kw):self.native=native(*args,**kw)
        def __getattr__(self,name):return getattr(self.native,name)
        def get_world(self):return World(self.native.get_world(),design)
    carla.Client=Client
    sys.path.insert(0,a.project)
    modules=dict(prepare='Toolkits.CollectionTools.map_setup',control='control_junction_phases',spawn='spawn_junction_traffic')
    sys.argv=[modules[a.mode]]+rest
    runpy.run_module(modules[a.mode],run_name='__main__')


if __name__=='__main__':main()
