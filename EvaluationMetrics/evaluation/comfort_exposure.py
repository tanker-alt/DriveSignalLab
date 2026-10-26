"""Observed forward-driving exposure using linear threshold crossings."""


class ComfortExposure:
    def __init__(self, ids, config):
        self.config = config
        self.previous = {}
        self.frame = self.time = None
        self.data = {vid: dict(valid_samples=0, seconds=0., metrics={
            name: dict(seconds=0., longest=0., current=0.) for name in config}) for vid in ids}

    def sample(self, frame, now, values):
        contiguous = self.frame is not None and frame == self.frame + 1
        for vid, data in self.data.items():
            if vid in values:
                data['valid_samples'] += 1
            if not contiguous or vid not in values or vid not in self.previous:
                for stat in data['metrics'].values():
                    stat['current'] = 0.
                continue
            dt = now - self.time
            data['seconds'] += dt
            for name, sign in (('harsh_braking', -1), ('harsh_acceleration', 1)):
                threshold = sign*self.config[name]['trigger_mps2']
                a = sign*self.previous[vid] - threshold
                b = sign*values[vid] - threshold
                stat = data['metrics'][name]
                if a > 0 and b > 0:
                    duration = dt
                    stat['current'] += duration
                elif a > 0:
                    duration = dt*a/(a-b)
                    stat['current'] += duration
                elif b > 0:
                    duration = dt*b/(b-a)
                    stat['current'] = duration
                else:
                    duration = 0.
                    stat['current'] = 0.
                stat['seconds'] += duration
                stat['longest'] = max(stat['longest'], stat['current'])
                if b <= 0:
                    stat['current'] = 0.
        self.previous = values
        self.frame, self.time = frame, now

    def summary(self, events, states):
        vehicles = {}
        for vid, data in self.data.items():
            seconds = data['seconds']
            metrics = {}
            for name, stat in data['metrics'].items():
                completed = sum(e['vehicle_id'] == vid and e['metric'] == name and
                                e['status'] == 'completed' for e in events)
                metrics[name] = dict(
                    completed_event_count=completed if data['valid_samples'] else None,
                    censored_event_count=sum(e['vehicle_id'] == vid and e['metric'] == name and
                                             e['status'] == 'censored' for e in events),
                    active_event_count=int(states.get((vid, name), {}).get('confirmed', False)),
                    events_per_minute=completed*60/seconds if seconds else None,
                    exceedance_seconds=stat['seconds'] if seconds else None,
                    exceedance_ratio=stat['seconds']/seconds if seconds else None,
                    longest_continuous_exceedance_seconds=stat['longest'] if seconds else None)
            total = sum(s['seconds'] for s in data['metrics'].values())
            vehicles[vid] = dict(valid_forward_seconds=seconds, valid_sample_count=data['valid_samples'],
                                 metrics=metrics, combined_exceedance_seconds=total if seconds else None,
                                 combined_exceedance_ratio=total/seconds if seconds else None)
        return dict(status='observed_unfiltered' if any(d['seconds'] for d in self.data.values()) else 'unavailable',
                    scope='per_explicit_vla_forward_only', filter='none', vehicles=vehicles,
                    definition='Linear acceleration interpolation on consecutive frames with both endpoints forward and valid; '
                    'includes short excursions and censored observed intervals, excludes hysteresis-only time; '
                    'longest is observed duration, not a claim of complete episode coverage; ratios are fractions')
