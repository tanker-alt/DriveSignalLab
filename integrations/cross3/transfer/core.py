"""Simulator-independent measurement and signal contracts (SI units)."""
import numpy as np

PHASES = ('NS_STRAIGHT', 'NS_LEFT', 'EW_STRAIGHT', 'EW_LEFT')
CONTRACT = dict(schema='cross3_transfer_v2', phases=list(PHASES),
    light_features='queue4,incoming4,outgoing4,phase4,green_yellow_red3,age1',
    vehicle_features='speed,distance,green,leader_gap,approach,active',
    count_scale=20., speed_scale=15., distance_scale=150., stop_speed=.1,
    decision_seconds=1., min_green=5., max_green=20., yellow=3., all_red=2.,
    background_driver='lane_signal_front_distance_v2', right_turn='with-straight', measurement='scheduled_arrival_to_vehicle_front_exit_62m',
    stop_line_incoming_s=144., spawn_center_s=60., metrics_schema='cross3_metrics_v2',
    reward='negative_all_vehicle_time_integral_div20_collision100',
    vehicle_action='internal_waypoints_predictor_final_linear_gaussian_std_0.05m')

def phase_for(road, lane):
    return (0 if road in (2, 4) else 2) + int(lane == -1)

class Signals:
    def __init__(self):
        self.active = self.target = 0
        self.stage = 'RED'
        self.age = 0.
        self.switches = 0
    def mask(self, incoming):
        mask = np.zeros(4, dtype=bool)
        mask[self.active] = True
        if self.stage == 'GREEN' and self.age >= CONTRACT['min_green'] - 1e-7:
            mask[:] = True
            if self.age >= CONTRACT['max_green'] - 1e-7 and any(
                    n > 0 for i, n in enumerate(incoming) if i != self.active):
                mask[self.active] = False
        return mask
    def request(self, action, incoming):
        if not self.mask(incoming)[action]:
            raise ValueError('Action violates stored signal mask')
        if action != self.active:
            self.target = int(action)
            self.stage, self.age = 'YELLOW', 0.
            self.switches += 1
    def tick(self, dt, occupied=False):
        self.age += dt
        if self.stage == 'YELLOW' and self.age >= CONTRACT['yellow'] - 1e-7:
            self.stage, self.age = 'RED', 0.
        elif self.stage == 'RED' and self.age >= CONTRACT['all_red'] - 1e-7 and not occupied:
            self.active = self.target
            self.stage, self.age = 'GREEN', 0.
    def permits(self, road, lane):
        return self.stage == 'GREEN' and phase_for(road, lane) == self.active

class Metrics:
    """Left-endpoint time integration, with explicit external spawn waiting.

    Completed travel time is separate from the right-censored observation lower
    bound. Physical queue excludes vehicles still outside the simulator.
    """
    def __init__(self, demand):
        import math
        if len({x['id'] for x in demand}) != len(demand):
            raise ValueError('Duplicate demand ID')
        if any(not math.isfinite(x['time']) or x['time']<0 for x in demand):
            raise ValueError('Invalid scheduled arrival')
        self.rows={x['id']:dict(x,spawn=None,finish=None,waiting=0.,external_waiting=0.,
            in_sim_waiting=0.,observed_time=0.,stops=0,stop_duration=0.,stop_counted=False) for x in demand}
        self.time=self.queue_integral=self.vehicle_time=self.ttc_exposure=0.
        self.min_ttc=None
        self.max_queue=self.max_external_queue=0
        self.collisions=set()
        self.safety=None

    def spawned(self, vid, when):
        row=self.rows[vid]
        if when<row['time']-1e-6 or row['spawn'] is not None:
            raise ValueError('Invalid/duplicate spawn: '+vid)
        row['spawn']=float(when)

    def sample(self, now, dt, vehicles, completed=(), collisions=(), after=None):
        import math
        if not math.isfinite(now) or not math.isfinite(dt) or dt<=0 or abs(now-dt-self.time)>1e-6:
            raise ValueError('Measurements must cover contiguous, non-overlapping intervals')
        start=self.time
        for vid in completed:
            row=self.rows[vid]
            if row['time']>now: raise ValueError('Completion before scheduled arrival')
            if row['finish'] is None: row['finish']=now
        self.time=now
        for vid,row in self.rows.items():
            end=min(now,row['finish'] if row['finish'] is not None else now)
            begin=max(start,row['time'])
            exposure=max(0.,end-begin)
            self.vehicle_time+=exposure; row['observed_time']+=exposure
            if not exposure: continue
            v=vehicles.get(vid)
            if v is not None:
                if not math.isfinite(v['speed']) or v['speed']<0: raise ValueError('Invalid speed')
                if row['spawn'] is None: row['spawn']=max(start,row['time'])
            spawn=row['spawn'] if row['spawn'] is not None else end
            external=max(0.,min(end,spawn)-begin)
            in_sim=exposure-external
            if in_sim>1e-6 and v is None:
                # Newly spawned during this interval has known zero initial speed.
                if spawn<start-1e-6: raise ValueError('Spawned vehicle disappeared without completion: '+vid)
                speed=0.
            else: speed=v['speed'] if v is not None else 0.
            waiting=in_sim if speed<CONTRACT['stop_speed'] else 0.
            row['external_waiting']+=external; row['in_sim_waiting']+=waiting
            row['waiting']+=external+waiting; self.queue_integral+=external+waiting
            if in_sim:
                if speed<CONTRACT['stop_speed']:
                    row['stop_duration']+=in_sim
                    if row['stop_duration']>=1.-1e-7 and not row['stop_counted']:
                        row['stops']+=1; row['stop_counted']=True
                elif speed>.5:
                    row['stop_duration']=0.; row['stop_counted']=False
            if v is not None:
                closing=v['speed']-v.get('leader_speed',v['speed'])
                if v.get('gap') is not None and closing>1e-9:
                    ttc=max(0.,v['gap'])/closing
                    self.min_ttc=ttc if self.min_ttc is None else min(self.min_ttc,ttc)
                    if ttc<1.5: self.ttc_exposure+=in_sim
        present=after if after is not None else vehicles
        physical=sum(v['speed']<CONTRACT['stop_speed'] for vid,v in present.items() if self.rows[vid]['finish'] is None)
        external=sum(r['time']<=now and r['spawn'] is None and r['finish'] is None for r in self.rows.values())
        self.max_queue=max(self.max_queue,physical); self.max_external_queue=max(self.max_external_queue,external)
        for pair in collisions: self.collisions.add(tuple(sorted(map(str,pair))))

    def report(self):
        due=[r for r in self.rows.values() if r['time']<=self.time]
        finished=[r for r in due if r['finish'] is not None]
        trips=[r['finish']-r['time'] for r in finished]
        waits=[r['waiting'] for r in due]
        report=dict(schema='cross3_metrics_v2',duration_s=self.time,planned=len(self.rows),
            due=len(due),completed=len(finished),unfinished=len(due)-len(finished),
            future_demand=len(self.rows)-len(due),completion_rate=len(finished)/len(due) if due else None,
            completed_travel_time_mean_s=float(np.mean(trips)) if trips else None,
            completed_travel_time_p95_s=float(np.percentile(trips,95)) if trips else None,
            censored_vehicle_time_s=self.vehicle_time,
            observed_time_per_due_vehicle_s=self.vehicle_time/len(due) if due else None,
            waiting_vehicle_seconds=self.queue_integral,
            external_waiting_vehicle_seconds=sum(r['external_waiting'] for r in due),
            in_sim_waiting_vehicle_seconds=sum(r['in_sim_waiting'] for r in due),
            mean_observed_wait_s=float(np.mean(waits)) if waits else None,
            max_observed_wait_s=max(waits) if waits else None,
            max_queue_vehicles=self.max_queue,max_external_queue_vehicles=self.max_external_queue,
            throughput_veh_per_hour=3600*len(finished)/self.time if self.time else None,
            stops=sum(r['stops'] for r in due),collision_pairs=len(self.collisions),
            rear_end_min_ttc_s=self.min_ttc,rear_end_ttc_below_1_5_vehicle_seconds=self.ttc_exposure,
            red_light_violations=None,crossing_min_ttc_s=None,
            by_approach={str(r):dict(due=sum(v['road']==r for v in due),
                waiting_vehicle_seconds=sum(v['waiting'] for v in due if v['road']==r)) for r in (1,2,3,4)})
        if self.safety is not None: report.update(self.safety.report())
        return report


def observation(vehicles, signals, ego_ids=()):
    incoming, outgoing, queue = (np.zeros(4, np.float32) for _ in range(3))
    for v in vehicles.values():
        p = phase_for(v['road'], v['lane'])
        if v['approach']:
            incoming[p] += 1
            queue[p] += v['speed'] < CONTRACT['stop_speed']
        elif v.get('outgoing'):
            outgoing[p] += 1
    light = np.r_[queue/20, incoming/20, outgoing/20, np.eye(4)[signals.active],
                  [signals.stage==s for s in ('GREEN','YELLOW','RED')], min(signals.age/20,2)].astype(np.float32)
    vo, vm, valid = [], [], []
    for vid in ego_ids:
        v = vehicles.get(vid)
        if v is None:
            vo.append([0]*6); vm.append([True,False,False,False]); valid.append(False)
            continue
        green = signals.permits(v['road'], v['lane']) and v['approach']
        gap = v.get('gap')
        vo.append([v['speed']/15, max(0,v['distance'])/150, green,
                   min(150,gap if gap is not None else 150)/150, v['approach'], 1])
        vm.append([True, bool(v['approach']), bool(v['approach']), bool(green and (gap is None or gap>8))])
        valid.append(True)
    # A dummy inactive vehicle keeps the centralized schema identical in signal-only training.
    if not vo:
        vo, vm, valid = [[0]*6], [[True,False,False,False]], [False]
    vo, valid = np.asarray(vo,np.float32), np.asarray(valid,bool)
    mean = vo[valid].mean(0) if valid.any() else np.zeros(6,np.float32)
    glob = np.r_[light, mean, valid.sum()/2].astype(np.float32)
    return dict(light=light, vehicle=vo, global_state=glob, light_mask=signals.mask(incoming),
                vehicle_mask=np.asarray(vm,bool), valid=valid, incoming=incoming)
