"""Deterministic navigation text from the assigned route's junction geometry.

This adapter supplies navigation only. It never reads signal state or controls a car.
CARLA coordinates use positive yaw for right turns.
"""
import math


def turn_text(entry_yaw, exit_yaw):
    delta = (exit_yaw - entry_yaw + 180.0) % 360.0 - 180.0
    if delta > 35.0:
        return 'Turn right at the next intersection.'
    if delta < -35.0:
        return 'Turn left at the next intersection.'
    return 'Go straight through the next intersection.'


class RouteInstructionAdapter:
    def __init__(self, delegate, route, world_map, training_vocabulary=False):
        self.delegate = delegate
        self.training_vocabulary = training_vocabulary
        self.route = [entry[0] for entry in route]
        self.progress = 0
        self.arc = [0.0]
        for a, b in zip(self.route, self.route[1:]):
            self.arc.append(self.arc[-1] + math.hypot(b.location.x-a.location.x,
                                                     b.location.y-a.location.y))
        flags = [world_map.get_waypoint(t.location).is_junction for t in self.route]
        self.turns = []
        index = 0
        while index < len(flags):
            if not flags[index]:
                index += 1
                continue
            start = index
            while index + 1 < len(flags) and flags[index+1]:
                index += 1
            end = index
            # MDrive postprocessing can return location-only Transforms whose
            # yaw is zero. Derive tangent directions from the route coordinates.
            before = self._heading(max(1, start-1))
            after = self._heading(min(len(flags)-1, end+1))
            self.turns.append(dict(start=start, end=end, entry_yaw=before,
                                   exit_yaw=after, text=turn_text(before, after)))
            index += 1
        self.last_navigation = None

    def _heading(self, end):
        b = self.route[end].location
        for start in range(end-1, -1, -1):
            a = self.route[start].location
            if math.hypot(b.x-a.x, b.y-a.y) > 0.05:
                return math.degrees(math.atan2(b.y-a.y, b.x-a.x))
        raise ValueError('Cannot infer heading from degenerate assigned route')

    def __getattr__(self, key):
        return getattr(self.delegate, key)

    def command2instruct(self, town_id, tick_data, routes=None, ego_id=0, dis_on=True):
        state = tick_data['ego_world']
        x, y = state['x'], state['y']
        self.progress = min(range(self.progress, min(len(self.route), self.progress+60)),
                            key=lambda i: (self.route[i].location.x-x)**2 +
                                          (self.route[i].location.y-y)**2)
        text = 'Follow the road.'
        for turn in self.turns:
            if self.progress > turn['end']:
                continue
            if self.arc[turn['start']] - self.arc[self.progress] <= 30.0:
                text = turn['text']
            break
        if self.training_vocabulary:
            text = {'Go straight through the next intersection.': 'Please proceed straight at the next intersection.',
                    'Turn left at the next intersection.': 'Next intersection, turn left.',
                    'Turn right at the next intersection.': 'Next intersection, turn right.',
                    'Follow the road.': 'Follow the road.'}.get(text, text)
        self.last_navigation = dict(source='assigned_route_junction_geometry',
                                    route_index=self.progress, text=text)
        return text
