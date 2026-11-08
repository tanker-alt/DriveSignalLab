"""Geometry-only safety measurements, independent of controller decisions.

Crossing TTC extrapolates oriented rectangles with constant velocity and heading
for at most 10 seconds. It is a risk proxy, not a prediction of future steering.
"""
import math
from itertools import combinations


def rectangle_ttc(a, b, horizon=10.):
    """First overlap time using continuous separating-axis intervals; None=no overlap."""
    axes=[]
    for box in (a,b):
        c,s=math.cos(box['yaw']),math.sin(box['yaw'])
        axes.extend(((c,s),(-s,c)))
    delta=(b['x']-a['x'],b['y']-a['y'])
    velocity=(b['vx']-a['vx'],b['vy']-a['vy'])
    enter,leave=0.,horizon
    for ux,uy in axes:
        radius=0.
        for box in (a,b):
            c,s=math.cos(box['yaw']),math.sin(box['yaw'])
            radius+=box['half_length']*abs(ux*c+uy*s)+box['half_width']*abs(-ux*s+uy*c)
        position=delta[0]*ux+delta[1]*uy
        speed=velocity[0]*ux+velocity[1]*uy
        if abs(speed)<1e-10:
            if abs(position)>radius: return None
            continue
        t0,t1=sorted(((-radius-position)/speed,(radius-position)/speed))
        enter=max(enter,t0); leave=min(leave,t1)
        if enter>leave+1e-10: return None
    return max(0.,enter) if enter<=horizon and leave>=0 else None


class SafetyMetrics:
    def __init__(self):
        self.crossed=set()
        self.events=[]
        self.min_crossing_ttc=None
        self.crossing_exposure=0.
        self.rear_ttc=None
        self.rear_exposure=0.
        self.samples=0
        self.covered_vehicle_seconds=0.

    def sample(self, now, dt, before, after, colors):
        """colors are measured physical signals held during [now-dt, now]."""
        self.samples+=1
        for vid,v in before.items():
            post=after.get(vid)
            if (post is not None and vid not in self.crossed and v.get('approach')
                    and v['front_stop_distance']>0>=post['front_stop_distance']):
                color=colors.get(vid)
                if color not in ('RED','YELLOW','GREEN'): raise ValueError('Missing measured signal for crossing')
                fraction=v['front_stop_distance']/(v['front_stop_distance']-post['front_stop_distance'])
                self.events.append(dict(vehicle=vid,time=now-dt+fraction*dt,
                                        signal=color,red_violation=color=='RED'))
                self.crossed.add(vid)
            closing=v['speed']-v.get('leader_speed',v['speed'])
            if v.get('gap') is not None and closing>1e-9:
                ttc=max(0.,v['gap'])/closing
                self.rear_ttc=ttc if self.rear_ttc is None else min(self.rear_ttc,ttc)
                if ttc<1.5: self.rear_exposure+=dt
            self.covered_vehicle_seconds+=dt
        for (_,a),(_,b) in combinations(before.items(),2):
            if a.get('lane_key')==b.get('lane_key'): continue
            if not (a.get('occupied') or b.get('occupied') or
                    (a.get('approach') and a['front_stop_distance']<60) or
                    (b.get('approach') and b['front_stop_distance']<60)): continue
            ttc=rectangle_ttc(a,b)
            if ttc is not None:
                self.min_crossing_ttc=ttc if self.min_crossing_ttc is None else min(self.min_crossing_ttc,ttc)
                if ttc<1.5: self.crossing_exposure+=dt

    def report(self):
        return dict(red_light_violations=sum(e['red_violation'] for e in self.events),
                    stop_line_crossings=len(self.events),
                    crossing_ttc_model='constant_velocity_fixed_heading_oriented_rectangles_10s',
                    crossing_min_ttc_s=self.min_crossing_ttc,
                    crossing_ttc_below_1_5_pair_seconds=self.crossing_exposure,
                    rear_end_min_ttc_s=self.rear_ttc,
                    rear_end_ttc_below_1_5_vehicle_seconds=self.rear_exposure,
                    safety_samples=self.samples,safety_observed_vehicle_seconds=self.covered_vehicle_seconds)
