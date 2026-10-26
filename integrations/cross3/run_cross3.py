#!/usr/bin/env python3
"""Cross3 native four-phase CARLA harness, two sensor-driven LMDrive egos.

One process owns ticking, signals and route-bound background BasicAgents.
Online DQN uses real CARLA counts; it is a PressLight-style implementation.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import random
import sys
import time
from collections import deque

ROOT = Path(__file__).resolve().parents[3]
DRIVE = ROOT / 'Drive'
MDRIVE = DRIVE / 'repos/MDrive'
UPSTREAM = Path(__file__).resolve().parents[1] / 'carla_handoff/upstream_20261023_v4'
MAP = UPSTREAM / 'map_dataset/junction_toolkit/maps/cross3'
for folder in (MDRIVE, MDRIVE/'simulation/leaderboard', MDRIVE/'simulation/scenario_runner',
               DRIVE/'software/CARLA_0.9.12_MDrive/PythonAPI/carla', UPSTREAM):
    sys.path.insert(0, str(folder))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agent_interfaces import load_plugin, load_driving_agent, validate_controls, SignalSelector


def write(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def load_map(client, output):
    import carla
    from control_four_phases import normalize_design, bind
    raw = (MAP/'map.xodr').read_bytes()
    design = normalize_design(json.loads((MAP/'manifest.json').read_text()))
    assert hashlib.sha256(raw).hexdigest() == design['opendrive_sha256']
    world = client.get_world()
    if world.get_settings().synchronous_mode or any(world.get_actors().filter(p)
            for p in ('vehicle.*', 'sensor.*', 'walker.*')):
        raise RuntimeError('Existing experiment owns the world; finish it first')
    world = client.generate_opendrive_world(raw.decode(), carla.OpendriveGenerationParameters(
        vertex_distance=2., max_road_length=50., wall_height=0., additional_width=0.,
        smooth_junctions=False, enable_mesh_visibility=True, enable_pedestrian_navigation=False))
    world.wait_for_tick(30)
    lights = bind(world, design)
    report = dict(client_version=client.get_client_version(), server_version=client.get_server_version(),
                  map_sha256=design['opendrive_sha256'], signal_count=len(lights),
                  lane_validity_passed=True)
    if report['client_version'] != report['server_version']:
        raise RuntimeError('Client/server versions must match')
    bp = world.get_blueprint_library().find('static.prop.mesh')
    if bp.has_attribute('mesh_path'):
        sys.path.insert(0, str(UPSTREAM/'map_dataset/junction_toolkit'))
        from scenery import build
        from lane_signals import build as gantries
        report['scenery'] = build(world, design)
        report['gantries'] = gantries(world, design)
        report['appearance'] = 'upstream_0.9.15'
    else:
        report['appearance'] = 'native_opendrive_0.9.12'
        report['appearance_limitation'] = 'mesh_path unavailable; upstream asphalt tiles and gantries not restored'
    world.set_weather(carla.WeatherParameters(cloudiness=25, sun_altitude_angle=48))
    world.get_spectator().set_transform(carla.Transform(carla.Location(x=-48,y=-38,z=55),
                                                     carla.Rotation(pitch=-45,yaw=38)))
    write(output/'map_load.json', report)
    print(json.dumps(report), flush=True)
    return world, design, lights


def route_for(road_map, move, start=100, end=85):
    wp = road_map.get_waypoint_xodr(move['road'], move['lane'], start)
    route = [wp]
    for _ in range(700):
        if wp.road_id == move['target_road'] and wp.lane_id == move['target_lane'] and wp.s <= end:
            return route
        choices = wp.next(.5)
        if len(choices) != 1:
            raise RuntimeError('Dedicated lane must have exactly one route: %r' % move)
        wp = choices[0]
        route.append(wp)
    raise RuntimeError('Route did not reach manifest target')


class Learner:
    name = 'cross3_presslight'
    movements = 4

    def __init__(self, args, design):
        import numpy as np
        import torch
        self.np, self.torch, self.args = np, torch, args
        torch.set_num_threads(1)
        torch.manual_seed(args.seed)
        self.net = torch.nn.Sequential(torch.nn.Linear(28,64), torch.nn.ReLU(),
                                       torch.nn.Linear(64,64), torch.nn.ReLU(), torch.nn.Linear(64,4))
        self.target = copy.deepcopy(self.net)
        self.optimizer = torch.optim.Adam(self.net.parameters(), lr=.0003)
        self.memory = deque(maxlen=20000)
        self.steps, self.updates, self.pending, self.last_loss = 0, 0, None, None
        self.fingerprint = design['opendrive_sha256']
        self.metadata = dict(schema='cross3_presslight_carla_v1', state_dim=28, action_dim=4,
            phase_order=['NS_STRAIGHT','NS_LEFT','EW_STRAIGHT','EW_LEFT'],
            lane_order=[(m['road'],m['lane']) for m in design['movements']],
            map_sha256=self.fingerprint, training_environment='CARLA',
            scope='PressLight-style DQN; not official PressLight implementation',
            right_turn=args.right_turn, timing=dict(min_green=5,max_green=20,yellow=3,clearance=2,decision=2))
        if args.checkpoint and args.checkpoint.exists():
            data = torch.load(str(args.checkpoint), map_location='cpu')
            for key in ('schema','state_dim','action_dim','phase_order','lane_order','map_sha256','right_turn','timing'):
                if data['metadata'][key] != self.metadata[key]:
                    raise ValueError('Incompatible Cross3 checkpoint: '+key)
            self.net.load_state_dict(data['model'])
            self.target.load_state_dict(data['target'])
            self.optimizer.load_state_dict(data['optimizer'])
            self.steps, self.updates = data['steps'], data['updates']
        elif not args.train:
            raise ValueError('Inference requires a trained Cross3 checkpoint')

    def vector(self, obs, phase):
        return self.np.asarray([float(i==phase) for i in range(4)] + obs['lane_incoming'] + obs['lane_outgoing'], dtype='float32')

    def transition(self, obs, phase, terminal=False):
        if self.pending is None:
            return
        state, action, origin = self.pending
        duration = max(.05, obs['elapsed']-origin)
        self.memory.append((state, action, self.reward, self.vector(obs,phase), duration, terminal))
        self.pending = None
        if len(self.memory) >= 32:
            torch = self.torch
            for _ in range(4):
                b = random.sample(self.memory,32)
                s = torch.tensor(self.np.stack([r[0] for r in b]))
                a = torch.tensor([r[1] for r in b])
                r = torch.tensor([r[2] for r in b],dtype=torch.float32)
                ns = torch.tensor(self.np.stack([r[3] for r in b]))
                d = torch.tensor([r[4] for r in b])
                done = torch.tensor([r[5] for r in b],dtype=torch.float32)
                with torch.no_grad():
                    y = r + .95**d * self.target(ns).max(1).values*(1-done)
                loss = torch.nn.functional.smooth_l1_loss(self.net(s).gather(1,a[:,None]).squeeze(1),y)
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.net.parameters(),5)
                self.optimizer.step()
                self.updates += 1
                self.last_loss = float(loss)
                if self.updates % 100 == 0:
                    self.target.load_state_dict(self.net.state_dict())

    def select(self, observation, current_action=None):
        if self.args.train:
            self.transition(observation,current_action)
        state = self.vector(observation,current_action)
        with self.torch.no_grad():
            q = self.net(self.torch.from_numpy(state))
        epsilon = max(.05,1-self.steps/2000) if self.args.train else 0
        action = random.randrange(4) if random.random()<epsilon else int(q.argmax())
        self.steps += 1
        if self.args.train:
            self.pending = state,action,observation['elapsed']
            self.reward = 0.
        return action

    def accumulate(self, obs):
        if self.pending is not None:
            age = obs['elapsed']-self.pending[2]
            self.reward -= .95**age * sum(abs(a-b) for a,b in zip(obs['lane_incoming'],obs['lane_outgoing']))*.05

    def save(self):
        if not self.args.train:
            return
        p = self.args.checkpoint
        p.parent.mkdir(parents=True,exist_ok=True)
        temp = p.with_suffix('.tmp')
        self.torch.save(dict(metadata=self.metadata,model=self.net.state_dict(),target=self.target.state_dict(),
                             optimizer=self.optimizer.state_dict(),steps=self.steps,updates=self.updates),str(temp))
        temp.replace(p)
        write(p.with_suffix('.json'),dict(self.metadata,decisions=self.steps,gradient_updates=self.updates,
                                        last_loss=self.last_loss,replay_size=len(self.memory)))


def observation(world, design, elapsed, road_map=None):
    road_map = road_map or world.get_map()
    incoming, outgoing, queued = [0]*12, [0]*12, [0]*12
    for actor in world.get_actors().filter('vehicle.*'):
        wp = road_map.get_waypoint(actor.get_location())
        if wp is None:
            continue
        v = actor.get_velocity()
        speed = (v.x*v.x+v.y*v.y+v.z*v.z)**.5
        for i,m in enumerate(design['movements']):
            if (wp.road_id,wp.lane_id)==(m['road'],m['lane']) and 64<=wp.s<=150:
                incoming[i] += 1
                queued[i] += int(speed<.3)
            if (wp.road_id,wp.lane_id)==(m['target_road'],m['target_lane']) and 70<=wp.s<=150:
                outgoing[i] += 1
    from control_four_phases import active_ids, PHASES
    groups = [active_ids(design,p,'red') for p in PHASES]
    keys = [(m['road'],m['lane']) for m in design['movements']]
    indices = [[keys.index((design['signals'][sid]['road_id'],design['signals'][sid]['incoming_lanes'][0]))
                for sid in g] for g in groups]
    inc = [sum(incoming[i] for i in g) for g in indices]
    out = [sum(outgoing[i] for i in g) for g in indices]
    return dict(incoming=inc,outgoing=out,pressure=[a-b for a,b in zip(inc,out)],
                lane_incoming=incoming,lane_outgoing=outgoing,lane_queued=queued,elapsed=elapsed)


def episode(client, world, design, lights, args, output, learner=None, agent=None, signal_agent=None):
    import carla
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.local_planner import RoadOption
    from common.traffic_signal_control import SafeAdaptivePolicy, MaxPressureSelector
    from control_four_phases import expected_states, PHASES
    from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
    from srunner.scenariomanager.timer import GameTime
    road_map=world.get_map()
    lane_lights={(cfg['road_id'],cfg['incoming_lanes'][0]):lights[sid]
                 for sid,cfg in design['signals'].items()}
    class LaneBasicAgent(BasicAgent):
        def _affected_by_traffic_light(self, lights_list=None, max_distance=None):
            # Stock 0.9.12 checks road_id only, so a neighbouring lane's red
            # incorrectly stops every car on a per-lane-signal map.
            wp = self._map.get_waypoint(self._vehicle.get_location())
            light=lane_lights.get((wp.road_id,wp.lane_id))
            relevant=[light] if light else []
            if self._last_traffic_light not in relevant:
                self._last_traffic_light = None
            if not relevant:
                return False,None
            return super()._affected_by_traffic_light(relevant,max_distance)
    output.mkdir(parents=True)
    original = world.get_settings()
    saved = [(l,l.get_state(),l.is_frozen()) for l in lights.values()]
    actors, records, sensors, collisions, backgrounds = [], [], [], [], []
    wrapper = None
    summary = dict(status='starting',controller='plugin' if signal_agent else args.controller,
                   ego_driver=('BasicAgent_expert' if args.teacher else type(agent).__name__) if args.egos else None,
                   right_turn=args.right_turn,vehicles=records,collisions=collisions,
                   evaluation_scope='Harness endpoint/route/collision audit, not official Leaderboard scoring')
    try:
        settings = world.get_settings()
        settings.synchronous_mode=True
        settings.fixed_delta_seconds=.05
        world.apply_settings(settings)
        CarlaDataProvider.set_client(client)
        CarlaDataProvider.set_world(world)
        GameTime.restart()
        for l in lights.values():
            l.freeze(True)
            l.set_state(carla.TrafficLightState.Red)
        selected = [next(m for m in design['movements'] if m['road']==r and m['lane']==-2) for r in (1,3)] if args.egos else []
        plans = []
        for i,m in enumerate(selected):
            plans.append((m,'hero_%d'%i,100+random.uniform(-2,2)))
        for m in design['movements']:
            for j in range(args.background_per_lane):
                start = 80-12*j if m in selected else 100-12*j
                plans.append((m,'cross3_background',start))
        for m,role,start in plans:
            route = route_for(road_map,m,start)
            bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2017' if role.startswith('hero') else 'vehicle.tesla.model3')
            bp.set_attribute('role_name',role)
            transform = route[0].transform
            transform.location.z += .4
            actor = world.spawn_actor(bp,transform)
            actors.append(actor)
            actor.apply_control(carla.VehicleControl(brake=1))
            record = dict(m,actor_id=actor.id,role=role,completed=False,route_violation=False)
            records.append(record)
            sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=actor)
            sensor.listen(lambda e: collisions.append(dict(frame=e.frame,actor=e.actor.id,other=e.other_actor.id)))
            sensors.append(sensor)
            if role.startswith('hero'):
                CarlaDataProvider._carla_actor_pool[actor.id]=actor
                CarlaDataProvider.register_actor(actor)
                if args.teacher:
                    basic=LaneBasicAgent(actor,target_speed=18,opt_dict={})
                    basic.set_global_plan([(w,RoadOption.LANEFOLLOW) for w in route])
                    agent.teachers[int(role.split('_')[-1])]=basic
            else:
                basic = LaneBasicAgent(actor,target_speed=20,opt_dict={})
                basic.set_global_plan([(w,RoadOption.LANEFOLLOW) for w in route])
                backgrounds.append((actor,basic,record))
        for _ in range(20):
            world.tick()
        if agent is not None:
            from leaderboard.autoagents.agent_wrapper import AgentWrapper
            from leaderboard.utils.route_manipulation import location_route_to_gps, _get_latlon_ref
            agent.town_id='OpenDriveMap'
            routes = [[(w.transform,RoadOption.LANEFOLLOW) for w in route_for(road_map,m,plans[i][2])]
                      for i,m in enumerate(selected)]
            write(output/'ego_endpoints.json',[dict(x=r[-1][0].location.x,y=r[-1][0].location.y) for r in routes])
            lat,lon = _get_latlon_ref(world)
            agent.set_global_plan([location_route_to_gps(r,lat,lon) for r in routes],routes)
            wrapper = AgentWrapper(agent)
            for i in range(args.egos):
                wrapper.setup_sensors(actors[i],i,output)
        selector = SignalSelector(signal_agent) if signal_agent else (learner or MaxPressureSelector())
        policy = SafeAdaptivePolicy(selector,movements=4,max_green_seconds=20)
        start_time=world.get_snapshot().timestamp.elapsed_seconds
        mismatches=0
        application_lag_frames=0
        previous_desired=None
        pending_application=False
        application_wait=0
        seen=set()
        signal_log=(output/'signals.jsonl').open('w')
        trajectory_log=(output/'vehicles.jsonl').open('w')
        try:
            for step in range(int(args.duration/.05)):
                snap=world.get_snapshot()
                elapsed=snap.timestamp.elapsed_seconds-start_time
                obs=observation(world,design,elapsed,road_map)
                if learner and args.train:
                    learner.accumulate(obs)
                if args.controller=='fixed' and signal_agent is None:
                    offset=elapsed-2
                    index=int(max(0,offset)//25)%4
                    age=max(0,offset)%25
                    stage='GREEN' if age<20 else 'YELLOW' if age<23 else 'RED'
                    phase=PHASES[index] if offset>=0 else None
                    if offset<0: stage='RED'
                else:
                    # The safety layer can force a maximum-green switch without
                    # calling select(). Record that executed action too.
                    if (learner and args.train and policy.mode=='green'
                            and elapsed-policy.mode_start>=policy.max_green_seconds):
                        waiting=[(policy.active+i)%4 for i in range(1,4)
                                 if obs['incoming'][(policy.active+i)%4]>0]
                        if waiting:
                            learner.transition(obs,policy.active)
                            learner.pending=(learner.vector(obs,policy.active),waiting[0],elapsed)
                            learner.reward=0.
                            learner.steps+=1
                    policy.update(elapsed,obs)
                    d=policy.decide(elapsed)
                    index=d['active_movement']
                    phase=PHASES[index] if index is not None else None
                    stage=d['state'].upper()
                desired=expected_states(design,phase,stage,args.right_turn)
                changing=desired!=previous_desired or pending_application
                for sid,state in desired.items():
                    lights[sid].set_state(getattr(carla.TrafficLightState,state))
                world.get_settings()  # Drain preceding signal RPCs before the tick.
                for actor,basic,record in backgrounds:
                    if actor.is_alive:
                        actor.apply_control(carla.VehicleControl(brake=1) if record['completed'] else basic.run_step())
                GameTime.on_carla_tick(snap.timestamp)
                CarlaDataProvider.on_carla_tick()
                if wrapper:
                    controls=validate_controls(wrapper(), args.egos)
                    for i in range(args.egos):
                        if actors[i].is_alive:
                            actors[i].apply_control(carla.VehicleControl(brake=1) if records[i]['completed'] else controls[i])
                world.tick()
                snap=world.get_snapshot()
                actual={sid:str(l.get_state()).split('.')[-1] for sid,l in lights.items()}
                mismatch=actual!=desired
                if mismatch and changing:
                    application_lag_frames+=1
                    application_wait+=1
                    start_time+=.05  # Do not count unconfirmed frames as phase time.
                elif mismatch:
                    mismatches+=1
                else:
                    application_wait=0
                record_stage='APPLYING_'+stage if mismatch and changing else stage
                signal_log.write(json.dumps(dict(frame=snap.frame,time=elapsed,phase=phase,stage=record_stage,
                                                desired=desired,actual=actual,observation=obs))+'\n')
                if mismatches or application_wait>10:
                    raise RuntimeError('Unconfirmed/stable signal mismatch: %r vs %r' % (desired,actual))
                pending_application=mismatch
                previous_desired=desired
                if not mismatch and stage=='GREEN' and phase: seen.add(phase)
                for actor,record in zip(actors,records):
                    if record['completed']: continue
                    wp=road_map.get_waypoint(actor.get_location())
                    if wp and wp.road_id in (1,2,3,4) and wp.lane_id>0:
                        if (wp.road_id,wp.lane_id)!=(record['target_road'],record['target_lane']):
                            record['route_violation']=True
                        elif wp.s<=88:
                            record.update(completed=True,completion_seconds=elapsed)
                            print('Completed',record['role'],record['road'],record['lane'],flush=True)
                    if step%10==0:
                        p=actor.get_location()
                        trajectory_log.write(json.dumps(dict(frame=snap.frame,actor=actor.id,x=p.x,y=p.y,
                            road=wp.road_id if wp else None,lane=wp.lane_id if wp else None))+'\n')
                    if record['completed']:
                        actor_index=actors.index(actor)
                        if actor_index<args.egos:
                            wrapper.cleanup_single(actor_index)
                            wrapper.del_ego_sensor(actor_index)
                            CarlaDataProvider._carla_actor_pool.pop(actor.id,None)
                        sensor=sensors[actor_index]
                        sensor.stop();sensor.destroy()
                        actor.destroy()
                if step%10==0:
                    signal_log.flush()
                    trajectory_log.flush()
                    write(output/'live.json',dict(frame=snap.frame,elapsed=elapsed,
                        completed=sum(r['completed'] for r in records),planned=len(records),
                        vehicles=records))
                if step%200==0:
                    print('time=%.1f completed=%d/%d phase=%s %s updates=%s' % (elapsed,sum(r['completed'] for r in records),len(records),phase,stage,learner.updates if learner else 0),flush=True)
                    signal_log.flush()
                    write(output/'progress.json',dict(elapsed=elapsed,completed=sum(r['completed'] for r in records),planned=len(records)))
                if all(r['completed'] for r in records):
                    break
            if learner and args.train:
                learner.transition(observation(world,design,elapsed,road_map),policy.active,terminal=True)
                learner.save()
            summary.update(status='finished',completed=sum(r['completed'] for r in records),
                           planned=len(records),signal_mismatches=mismatches,phases_seen=sorted(seen),elapsed=elapsed)
            summary['signal_application_lag_frames']=application_lag_frames
        finally:
            signal_log.close()
            trajectory_log.close()
    except BaseException as exc:
        summary.update(status='aborted',error=repr(exc))
        raise
    finally:
        if wrapper: wrapper.cleanup()
        for s in sensors:
            if s.is_alive:
                s.stop(); s.destroy()
        for a in actors:
            if a.is_alive: a.destroy()
        for l,state,frozen in saved:
            l.set_state(state); l.freeze(frozen)
        world.apply_settings(original)
        CarlaDataProvider._carla_actor_pool.clear()
        summary['world_settings_restored']=True
        write(output/'summary.json',summary)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=31600)
    p.add_argument('--egos',type=int,choices=[0,2],default=0)
    p.add_argument('--teacher',action='store_true',help='BasicAgent expert data collection; never VLA evaluation')
    p.add_argument('--waypoint-head',type=Path,help='Explicit scene adaptation; omitted means original model weights')
    p.add_argument('--controller',choices=['fixed','maxpressure','presslight'],default='fixed')
    p.add_argument('--train',action='store_true')
    p.add_argument('--episodes',type=int,default=1)
    p.add_argument('--duration',type=float,default=180)
    p.add_argument('--background-per-lane',type=int,default=1)
    p.add_argument('--right-turn',choices=['always-green','with-straight','red'],default='always-green')
    p.add_argument('--seed',type=int,default=2026)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path)
    p.add_argument('--signal-agent',type=Path,help='Signal plugin JSON; overrides built-in controller for inference')
    p.add_argument('--driving-agent',type=Path,help='Multi-ego Leaderboard plugin JSON; replaces LMDrive')
    args=p.parse_args()
    if args.signal_agent and (args.train or args.checkpoint):
        p.error('--signal-agent is inference-only; do not combine with --train or --checkpoint')
    if args.driving_agent and (not args.egos or args.teacher or args.waypoint_head):
        p.error('--driving-agent requires --egos 2 and cannot combine with --teacher/--waypoint-head')
    if args.train and (args.controller!='presslight' or args.checkpoint is None):
        p.error('--train requires --controller presslight --checkpoint FILE')
    if args.egos and args.episodes!=1:
        p.error('LMDrive runs use one episode per process')
    if args.teacher and not args.egos:
        p.error('--teacher requires --egos 2')
    if args.duration<=0 or not 1<=args.background_per_lane<=5 or args.episodes<1:
        p.error('Invalid duration/episodes/background count')
    args.output=args.output.resolve()
    for key in ('signal_agent', 'driving_agent'):
        if getattr(args, key):
            setattr(args, key, getattr(args, key).resolve())
    args.output.mkdir(parents=True,exist_ok=False)
    import shutil
    shutil.copyfile(__file__,str(args.output/'runner_source.py'))
    for key in ('signal_agent', 'driving_agent'):
        if getattr(args, key):
            shutil.copyfile(getattr(args, key), args.output/(key+'.json'))
    random.seed(args.seed)
    import carla
    client=carla.Client('127.0.0.1',args.port)
    client.set_timeout(120)
    world,design,lights=load_map(client,args.output)
    write(args.output/'config.json',{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()})
    learner=Learner(args,design) if args.controller=='presslight' and not args.signal_agent else None
    agent=None
    signal_agent=None
    try:
        if args.signal_agent:
            signal_agent=load_plugin(args.signal_agent, 'signal')
        if args.driving_agent:
            agent=load_driving_agent(args.driving_agent, args.egos)
        elif args.egos:
            import torch
            torch.set_num_threads(4)
            os.environ.setdefault('HF_HUB_OFFLINE','1')
            os.environ.setdefault('TRANSFORMERS_OFFLINE','1')
            os.environ.setdefault('LMDRIVE_SKIP_BACKBONE_PRETRAIN','1')
            os.environ['SAVE_PATH']=str(args.output/'lmdrive')
            os.environ['LMDRIVE_LIGHT_MESSAGES']='off'
            os.environ['ROUTES']='cross3_two_ego'
            if args.waypoint_head:
                os.environ['LMDRIVE_WAYPOINT_HEAD']=str(args.waypoint_head.resolve())
            else:
                os.environ.pop('LMDRIVE_WAYPOINT_HEAD',None)
            os.chdir(str(MDRIVE))
            if args.teacher:
                from team_code.teacher_signal_lmdrive import TeacherSignalLMDrive as Agent
            else:
                from team_code.fixed_signal_lmdrive import FixedSignalLMDrive as Agent
            agent=Agent(str(DRIVE/'configs/lmdrive_four_arm_config.py'),2)
        summaries=[]
        for i in range(args.episodes):
            summaries.append(episode(client,world,design,lights,args,args.output/('episode_%03d'%i),learner,agent,signal_agent))
            write(args.output/'summary.json',summaries)
    finally:
        try:
            if learner: learner.save()
        finally:
            try:
                if agent: agent.destroy()
            finally:
                if signal_agent: signal_agent.close()


if __name__=='__main__':
    main()
