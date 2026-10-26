"""Deterministic per-vehicle Safety events; independent of CARLA and controllers."""
import json
import math
from collections import Counter

METRICS = ('red_light', 'out_of_bounds', 'wrong_way', 'speeding', 'reversing')


def finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None




def forward(row):
    yaw = finite(row.get('yaw'))
    pitch = finite(row.get('pitch', 0))
    if yaw is None or pitch is None:
        return None
    yaw, pitch = math.radians(yaw), math.radians(pitch)
    return (math.cos(yaw)*math.cos(pitch), math.sin(yaw)*math.cos(pitch), math.sin(pitch))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))




class SafetyEvaluator:
    def __init__(self, config, geometry=None):
        self.config, self.geometry = config, geometry
        self.states = {key: {} for key in METRICS}
        self.previous_values = {key: {} for key in METRICS}
        self.previous_rows = {}
        self.previous_signals = {}
        self.last_frame = self.last_time = None
        self.events = []
        self.counts = {key: Counter() for key in METRICS}
        self.coverage = {key: dict(valid_samples=0, unavailable_samples=0, not_applicable_samples=0, covered_vehicle_seconds=0.) for key in METRICS}
        self.by_vehicle = {}
        self.speed_audit = {}
        self.closed = False
        self.geometry_region = None
        self.stop_lines = {}
        if geometry:
            try:
                from shapely.geometry import shape
                self.geometry_region = shape(geometry['driving_region']).buffer(config['geometry']['boundary_uncertainty_m'])
            except (KeyError, TypeError, ValueError):
                pass
            for line in geometry.get('stop_lines', []):
                self.stop_lines[(str(line.get('road_id')), str(line.get('lane_id')), str(line.get('signal_opendrive_id')))] = line

    def emit(self, metric, vid, state, status, frame, now, reason=None, **extra):
        event = dict(event_id=len(self.events)+1, metric=metric, vehicle_id=vid,
                     status=status, start_frame=state['start_frame'], start_sim_time=state['start'],
                     end_frame=frame, end_sim_time=now, observed_duration_s=now-state['start'],
                     qualified=state.get('qualified', True), start_observed=state.get('start_observed', True),
                     reason=reason, **extra)
        self.events.append(event)
        self.counts[metric][status] += 1
        key = (vid, metric)
        self.by_vehicle.setdefault(key, Counter())[status] += 1

    def break_state(self, metric, vid, reason):
        state = self.states[metric].pop(vid, None)
        if state:
            self.emit(metric, vid, state, 'censored', state['last_frame'], state['last'], reason)
        self.previous_values[metric].pop(vid, None)

    def duration_event(self, metric, vid, frame, now, trigger, release, duration):
        state = self.states[metric].get(vid)
        previous = self.previous_values[metric].get(vid)
        if state is not None:
            if release:
                if state['qualified']:
                    status = 'completed' if state['start_observed'] else 'censored'
                    self.emit(metric, vid, state, status, frame, now,
                              None if status == 'completed' else 'episode_already_present_at_observation_start')
                self.states[metric].pop(vid)
                state = None
            elif not state['qualified'] and not trigger:
                self.states[metric].pop(vid)
                state = None
        if state is None and trigger:
            state = dict(start=now, start_frame=frame, last=now, last_frame=frame,
                         qualified=duration == 0, start_observed=previous is not None)
            self.states[metric][vid] = state
        if state is not None:
            state.update(last=now, last_frame=frame)
            if trigger and now-state['start'] >= duration-1e-9:
                state['qualified'] = True
        return 'active' if state and state['qualified'] else ('candidate' if state else 'clear')

    @staticmethod
    def footprint(row):
        from shapely.geometry import MultiPoint
        points=row.get('bbox_vertices_world_json')
        if isinstance(points,str): points=json.loads(points)
        if not points or len(points)<4 or not all(len(p)>=2 and all(finite(v) is not None for v in p[:2]) for p in points):
            raise ValueError('Invalid body geometry')
        poly=MultiPoint([(p[0],p[1]) for p in points]).convex_hull
        if poly.area<=0: raise ValueError('Invalid body area')
        return poly

    def geometry_values(self, vid, row, frame, now, signals, contiguous):
        out={k:dict(status='unavailable',reason='Required geometry unavailable') for k in ('red_light','out_of_bounds','wrong_way')}
        try: body=self.footprint(row)
        except (ValueError,TypeError,KeyError):body=None
        if body is not None and self.geometry_region is not None:
            outside=not self.geometry_region.covers(body)
            state=self.duration_event('out_of_bounds',vid,frame,now,outside,not outside,0)
            out['out_of_bounds']=dict(status='valid',state=state,violation=outside)
        previous=self.previous_rows.get(vid) if contiguous else None
        if previous is not None and body is not None:
            try: before=self.footprint(previous)
            except (ValueError,TypeError,KeyError):before=None
        else:before=None
        lines=[line for key,line in self.stop_lines.items() if key[:2]==(str(previous.get('road_id')),str(previous.get('lane_id')))] if previous else []
        current_lines=[line for key,line in self.stop_lines.items() if key[:2]==(str(row.get('road_id')),str(row.get('lane_id')))]
        if not lines:lines=current_lines
        if len(lines)==1 and body is not None:
            line=lines[0];cx,cy=line['center_world_m'][:2];fx,fy=line['forward_world'][:2]
            def front(poly):return max((x-cx)*fx+(y-cy)*fy for x,y in poly.exterior.coords)
            sid=str(line['signal_opendrive_id']);state=signals.get(sid)
            prior=self.previous_signals.get(sid)
            if state in ('Red','Yellow','Green'):
                crossed=before is not None and front(before)<0<=front(body)
                # Bound lateral location to the finite map stop line, not its infinite plane.
                if crossed:
                    alpha=-front(before)/(front(body)-front(before))
                    center=before.centroid
                    x=center.x+alpha*(body.centroid.x-center.x);y=center.y+alpha*(body.centroid.y-center.y)
                    crossed=abs(-(x-cx)*fy+(y-cy)*fx)<=line['lane_width_m']/2
                out['red_light']=dict(status='valid',crossed=bool(crossed),signal_state=state)
                if crossed:
                    status='completed' if state==prior=='Red' else 'indeterminate' if prior!=state else None
                    if status:
                        event_state=dict(start_frame=frame,start=now,qualified=True,start_observed=True)
                        self.emit('red_light',vid,event_state,status,frame,now,
                                  'map_stop_line_red_crossing' if status=='completed' else 'signal_transition_during_crossing',stop_line_id=line['id'])
        if row.get('legal_direction_status')=='unique_lane':
            direction=[finite(row.get(k)) for k in ('legal_forward_x','legal_forward_y')]
            velocity=[finite(row.get(k)) for k in ('vx','vy')]
            if all(v is not None for v in direction+velocity):
                wrong=dot(direction,velocity)<-.1
                state=self.duration_event('wrong_way',vid,frame,now,wrong,not wrong,.5)
                out['wrong_way']=dict(status='valid',state=state,violation=wrong)
        elif row.get('legal_direction_status')=='junction_route_unresolved':
            out['wrong_way']=dict(status='not_applicable',reason='Junction movement direction unresolved')
        return out

    def sample(self, frame, now, rows, signals=None):
        frame, now = int(frame), float(now)
        if self.closed:
            raise ValueError('Safety evaluator is closed')
        if self.last_frame is not None and (frame <= self.last_frame or now <= self.last_time):
            raise ValueError('Safety frame/time must increase')
        signals = {str(k): str(v).split('.')[-1] for k,v in (signals or {}).items()}
        rows = list(rows)
        ids = [str(row['vehicle_id']) for row in rows]
        if len(set(ids)) != len(ids):
            raise ValueError('Duplicate vehicle in Safety frame')
        contiguous = self.last_frame is not None and frame == self.last_frame+1
        begin = len(self.events)
        for metric in METRICS:
            for vid in list(set(self.previous_values[metric]) | set(self.states[metric])):
                if not contiguous or vid not in ids:
                    self.break_state(metric, vid, 'frame_gap' if not contiguous else 'vehicle_missing')
        results = []
        for row, vid in zip(rows, ids):
            values = {}
            values.update(self.geometry_values(vid, row, frame, now, signals, contiguous))
            speed = finite(row.get('speed_mps'))
            cfg = self.config['speeding']
            values['speeding'] = dict(status='unavailable', reason='Missing finite speed')
            if not cfg['enabled']:
                values['speeding'] = dict(status='disabled')
            elif speed is not None and speed >= 0:
                kmh = speed*3.6
                audit=self.speed_audit.setdefault(vid,dict(max_speed_mps=0.,exceeding_limit_samples=0))
                audit['max_speed_mps']=max(audit['max_speed_mps'],speed)
                audit['exceeding_limit_samples']+=int(speed>cfg['limit_kmh']/3.6)
                state = self.duration_event('speeding', vid, frame, now,
                    speed > (cfg['limit_kmh']+cfg['tolerance_kmh'])/3.6, speed <= cfg['limit_kmh']/3.6, cfg['minimum_duration_s'])
                values['speeding'] = dict(status='valid', state=state, speed_kmh=kmh,
                    limit_kmh=cfg['limit_kmh'], trigger_kmh=cfg['limit_kmh']+cfg['tolerance_kmh'])
            cfg = self.config['reversing']
            heading = forward(row)
            velocity = [finite(row.get(k, 0 if k == 'vz' else None)) for k in ('vx','vy','vz')]
            values['reversing'] = dict(status='unavailable', reason='Missing velocity/heading')
            if not cfg['enabled']:
                values['reversing'] = dict(status='disabled')
            elif heading is not None and all(v is not None for v in velocity):
                longitudinal = dot(heading, velocity)
                state = self.duration_event('reversing', vid, frame, now, longitudinal < -cfg['trigger_speed_mps'],
                    longitudinal >= -cfg['release_speed_mps'] if cfg['end_at_zero'] else longitudinal > cfg['release_speed_mps'], cfg['minimum_duration_s'])
                values['reversing'] = dict(status='valid', state=state, longitudinal_speed_mps=longitudinal)
            for metric, value in values.items():
                coverage = self.coverage[metric]
                if value['status'] == 'valid':
                    coverage['valid_samples'] += 1
                    if vid in self.previous_values[metric] and contiguous:
                        coverage['covered_vehicle_seconds'] += now-self.last_time
                    self.previous_values[metric][vid] = now
                else:
                    coverage['not_applicable_samples' if value['status']=='not_applicable' else 'unavailable_samples'] += 1
                    self.break_state(metric, vid, value.get('reason', value['status']))
            results.append(dict(vehicle_id=vid, **values))
        self.previous_rows = {str(r['vehicle_id']): r for r in rows}
        self.previous_signals = signals
        self.last_frame, self.last_time = frame, now
        return dict(vehicles=results, events=self.events[begin:])

    def summary(self):
        result = {}
        for metric in METRICS:
            counts, coverage = self.counts[metric], self.coverage[metric]
            result[metric] = dict(value=counts['completed'] if coverage['valid_samples'] else None,
                unit='events', status='observed' if coverage['valid_samples'] else 'unavailable',
                reason=None if coverage['valid_samples'] else 'No valid applicable samples; inspect per-frame status and configuration',
                completed_events=counts['completed'], censored_events=counts['censored'],
                qualified_censored_events=sum(e['metric']==metric and e['status']=='censored' and e['qualified'] for e in self.events),
                indeterminate_events=counts['indeterminate'],
                active_events=sum(s['qualified'] for s in self.states[metric].values()),
                coverage=coverage,
                by_vehicle={vid: dict(c) for (vid, m), c in self.by_vehicle.items() if m == metric})
        result['speeding']['instantaneous_limit_audit']=dict(definition='MDrive MaxVelocityTest comparison: speed_mps > limit_mps; event duration/hysteresis is a separate extension',vehicles=self.speed_audit)
        return result

    def close(self):
        if not self.closed:
            for metric in METRICS:
                for vid in list(self.states[metric]):
                    self.break_state(metric, vid, 'evaluation_ended')
            self.closed = True
        return self.summary()
