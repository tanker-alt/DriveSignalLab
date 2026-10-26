"""Unfiltered VLA acceleration measurements; no comfort score or jerk assessment."""
import math


def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def statistics(values):
    if not values:
        return dict(sample_count=0, mean=None, rms=None, p95_abs=None,
                    peak_abs=None, maximum=None, minimum=None)
    ordered = sorted(abs(v) for v in values)
    position = .95 * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return dict(sample_count=len(values), mean=sum(values)/len(values),
                rms=math.sqrt(sum(v*v for v in values)/len(values)),
                p95_abs=ordered[lower] + (position-lower)*(ordered[upper]-ordered[lower]),
                peak_abs=ordered[-1], maximum=max(values), minimum=min(values))


class AccelerationMetrics:
    """IDs must be explicitly supplied; never infer VLA identity from background actors."""
    def __init__(self, vehicle_ids=None):
        self.ids = {str(v) for v in vehicle_ids} if vehicle_ids is not None else set()
        self.values = {vid: {axis: [] for axis in ('longitudinal', 'lateral')}
                       for vid in self.ids}
        self.previous = {}
        self.duration = {vid: dict(longitudinal=0., lateral=0.) for vid in self.ids}
        self.integrals = {vid: dict(longitudinal=0., lateral=0.) for vid in self.ids}
        self.last_frame = self.last_time = None

    def sample(self, frame, now, rows):
        if self.last_frame is not None and (frame <= self.last_frame or now <= self.last_time):
            raise ValueError('Comfort frame/time must increase')
        current, result = {}, []
        for row in rows:
            vid = str(row['vehicle_id'])
            if vid not in self.ids:
                continue
            if vid in current:
                raise ValueError('Duplicate Comfort vehicle in frame')
            values = {axis: finite(row.get(axis + '_accel_mps2')) for axis in self.values[vid]}
            for axis, value in values.items():
                if value is None:
                    continue
                self.values[vid][axis].append(value)
                old = self.previous.get(vid, {}).get(axis)
                if self.last_frame is not None and frame == self.last_frame + 1 and old is not None:
                    dt = now - self.last_time
                    self.duration[vid][axis] += dt
                    self.integrals[vid][axis] += .5*(old*old + value*value)*dt
            current[vid] = values
            result.append(dict(vehicle_id=vid, **{axis+'_accel_mps2': value for axis, value in values.items()}))
        self.previous = current
        self.last_frame, self.last_time = frame, now
        return result

    def summary(self):
        vehicles = {}
        for vid, axes in self.values.items():
            vehicle = {}
            for axis, values in axes.items():
                stats = statistics(values)
                duration = self.duration[vid][axis]
                stats.update(covered_seconds=duration,
                             time_weighted_rms=math.sqrt(self.integrals[vid][axis]/duration) if duration else None)
                if axis == 'longitudinal':
                    stats.update(max_acceleration_mps2=max(0., max(values)) if values else None,
                                 max_braking_mps2=max(0., -min(values)) if values else None)
                vehicle[axis + '_accel_mps2'] = stats
            vehicles[vid] = vehicle
        has_data = any(values for axes in self.values.values() for values in axes.values())
        return dict(status=('available' if has_data else 'no_valid_samples') if self.ids else 'pending_vla_identity',
                    scope='explicit_vla_vehicle_ids', vehicle_ids=sorted(self.ids),
                    filter='none', unit='m/s^2', vehicles=vehicles,
                    aggregation='per vehicle; sample mean/RMS and linear-interpolated absolute P95; '
                    'time_weighted_rms integrates squared acceleration by trapezoid on consecutive valid frames',
                    axes='yaw-only horizontal projection; longitudinal forward positive, lateral CARLA right positive')
