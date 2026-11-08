"""Simulator-independent measurement and signal contracts (SI units)."""
import numpy as np

PHASES = ('NS_STRAIGHT', 'NS_LEFT', 'EW_STRAIGHT', 'EW_LEFT')
CONTRACT = dict(schema='cross3_transfer_v1', phases=list(PHASES),
    light_features='queue4,incoming4,outgoing4,phase4,green_yellow_red3,age1',
    vehicle_features='speed,distance,green,leader_gap,approach,active',
    count_scale=20., speed_scale=15., distance_scale=150., stop_speed=.1,
    decision_seconds=1., min_green=5., max_green=20., yellow=3., all_red=2.,
    right_turn='with-straight', measurement='scheduled_entry_to_exit_s88',
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
    """Include every due demand, including failed spawns and censored trips.

    Queue: speed < .1 m/s, plus due but unspawned demand. TTC is same-lane
    rear-end TTC only; crossing-conflict TTC and red-light violations are unknown.
    """
    def __init__(self, demand):
        self.rows = {x['id']:dict(x, spawn=None, finish=None, waiting=0., stops=0,
                                 stopped=False, stop_duration=0., stop_counted=False) for x in demand}
        self.time = self.queue_integral = self.vehicle_time = self.ttc_exposure = 0.
        self.min_ttc = None
        self.max_queue = 0
        self.collisions = set()
    def sample(self, now, dt, vehicles, completed=(), collisions=()):
        if dt <= 0 or now < self.time:
            raise ValueError('Metrics require monotonic simulation time')
        self.time = now
        for vid in completed:
            if self.rows[vid]['finish'] is None:
                self.rows[vid]['finish'] = now
        queue = 0
        for vid, row in self.rows.items():
            # Integrate overlap, so arrivals and exits at tick boundaries are not overcounted.
            end = min(now, row['finish'] if row['finish'] is not None else now)
            exposure = max(0., end - max(now-dt, row['time']))
            self.vehicle_time += exposure
            if not exposure:
                continue
            v = vehicles.get(vid)
            if v is not None and row['spawn'] is None:
                row['spawn'] = now-dt
            stopped = v is None or v['speed'] < CONTRACT['stop_speed']
            if stopped:
                row['waiting'] += exposure
                self.queue_integral += exposure
                queue += 1
            if v is not None:
                if stopped:
                    row['stop_duration'] += exposure
                    if row['stop_duration'] >= 1.-1e-7 and not row['stop_counted']:
                        row['stops'] += 1
                        row['stop_counted'] = True
                elif v['speed'] > .5:
                    row['stop_duration'] = 0.
                    row['stop_counted'] = False
                row['stopped'] = stopped
                closing = v['speed'] - v.get('leader_speed', v['speed'])
                gap = v.get('gap')
                if gap is not None and closing > 0:
                    ttc = max(0., gap) / closing
                    self.min_ttc = ttc if self.min_ttc is None else min(self.min_ttc, ttc)
                    if ttc < 1.5:
                        self.ttc_exposure += exposure
        self.max_queue = max(self.max_queue, queue)
        for pair in collisions:
            self.collisions.add(tuple(sorted(map(str, pair))))
    def report(self):
        due = [r for r in self.rows.values() if r['time'] <= self.time]
        finished = [r for r in due if r['finish'] is not None]
        trips = [r['finish']-r['time'] for r in finished]
        waits = [r['waiting'] for r in due]
        return dict(schema='cross3_metrics_v1', duration_s=self.time, planned=len(self.rows),
            due=len(due), completed=len(finished), unfinished=len(due)-len(finished),
            future_demand=len(self.rows)-len(due), completion_rate=len(finished)/len(due) if due else None,
            completed_travel_time_mean_s=float(np.mean(trips)) if trips else None,
            completed_travel_time_p95_s=float(np.percentile(trips,95)) if trips else None,
            censored_vehicle_time_s=self.vehicle_time,
            observed_time_per_due_vehicle_s=self.vehicle_time/len(due) if due else None,
            waiting_vehicle_seconds=self.queue_integral,
            mean_observed_wait_s=float(np.mean(waits)) if waits else None,
            max_observed_wait_s=max(waits) if waits else None,
            max_queue_vehicles=self.max_queue,
            throughput_veh_per_hour=3600*len(finished)/self.time if self.time else None,
            stops=sum(r['stops'] for r in due), collision_pairs=len(self.collisions),
            rear_end_min_ttc_s=self.min_ttc, rear_end_ttc_below_1_5_vehicle_seconds=self.ttc_exposure,
            red_light_violations=None, crossing_ttc=None,
            by_approach={str(r):dict(due=sum(v['road']==r for v in due),
                waiting_vehicle_seconds=sum(v['waiting'] for v in due if v['road']==r)) for r in (1,2,3,4)})

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
