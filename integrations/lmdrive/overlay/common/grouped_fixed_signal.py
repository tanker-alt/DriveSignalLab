"""Verified opposite straight movements, with fixed yellow/all-red transitions."""
import math


def point_segment_distance(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    norm = dx*dx+dy*dy
    q = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/norm)) if norm else 0.
    return math.hypot(p[0]-a[0]-q*dx, p[1]-a[1]-q*dy)


def segment_distance(a, b, c, d):
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    if cross(a,b,c)*cross(a,b,d) < 0 and cross(c,d,a)*cross(c,d,b) < 0:
        return 0.
    return min(point_segment_distance(a,c,d), point_segment_distance(b,c,d),
               point_segment_distance(c,a,b), point_segment_distance(d,a,b))


class GroupedFixedPolicy:
    movements = 4
    requires_verified_mapping = True

    def __init__(self, green_seconds=20., yellow_seconds=3., clearance_seconds=2., initial_red_seconds=2.):
        self.green, self.yellow, self.clearance = map(float, (green_seconds, yellow_seconds, clearance_seconds))
        self.initial_red = float(initial_red_seconds)
        if any(not math.isfinite(v) or v <= 0 for v in (self.green,self.yellow,self.clearance,self.initial_red)):
            raise ValueError('Signal durations must be positive and finite')
        self.phase_groups = None

    def configure(self, mappings):
        if len(mappings) != 4 or any(m['movement'] != 'STRAIGHT' for m in mappings):
            raise ValueError('Paired fixed baseline requires four verified straight movements')
        groups = {'NS': [], 'EW': []}
        for i, m in enumerate(mappings):
            yaw = m['stop'].transform.rotation.yaw % 360
            cardinal = round(yaw/90)*90
            if abs((yaw-cardinal+180)%360-180) > 15:
                raise ValueError('Approach not aligned with a cardinal axis')
            groups['NS' if round(yaw/90)%2 else 'EW'].append(i)
        self.clearances = {}
        for name, indices in groups.items():
            if len(indices) != 2:
                raise ValueError('Each axis must contain exactly two approaches')
            a,b = [mappings[i] for i in indices]
            delta = abs((a['stop'].transform.rotation.yaw-b['stop'].transform.rotation.yaw+180)%360-180)
            if abs(delta-180) > 15:
                raise ValueError('Paired movements must approach from opposite directions')
            paths = [[(p.x,p.y) for p in m['points']] for m in (a,b)]
            clearance = min(segment_distance(p,q,r,s) for p,q in zip(paths[0],paths[0][1:])
                            for r,s in zip(paths[1],paths[1][1:]))
            if clearance < 2.8:
                raise ValueError('Paired routes intersect or lack lateral clearance: %.3fm' % clearance)
            self.clearances[name] = clearance
        self.phase_groups = [('NS', groups['NS']), ('EW', groups['EW'])]
        self.compatible_green_sets = [{mappings[i]['light'].id for i in indices}
                                      for _,indices in self.phase_groups]

    def decide(self, elapsed):
        if self.phase_groups is None:
            raise RuntimeError('Configure and validate geometry before controlling signals')
        if elapsed < self.initial_red:
            return self._result('initial_all_red', [], 'Red', self.initial_red-elapsed)
        interval = self.green+self.yellow+self.clearance
        offset = (elapsed-self.initial_red) % (2*interval)
        name, indices = self.phase_groups[int(offset//interval)]
        age = offset % interval
        if age < self.green:
            return self._result(name+'_green', indices, 'Green', self.green-age)
        if age < self.green+self.yellow:
            return self._result(name+'_yellow', indices, 'Yellow', self.green+self.yellow-age)
        return self._result(name+'_all_red', [], 'Red', interval-age)

    @staticmethod
    def _result(phase, indices, state, remaining):
        return dict(phase=phase, active_ego=None, active_movements=list(indices), state=state,
                    remaining_s=remaining, controller='fixed_paired_straight')
