"""Observed stopped waiting per junction, independent of signal association."""
import math


class JunctionWaitingTime:
    def __init__(self,junction_id,stop_speed_mps=.1,resume_speed_mps=.3,observation_warmup_seconds=1.):
        if not all(math.isfinite(v) for v in (stop_speed_mps,resume_speed_mps,observation_warmup_seconds)) or not 0<=stop_speed_mps<resume_speed_mps or observation_warmup_seconds<0:
            raise ValueError('Invalid waiting configuration')
        self.junction_id=str(junction_id);self.stop=stop_speed_mps;self.resume=resume_speed_mps;self.warmup=observation_warmup_seconds
        self.active={};self.vehicles={};self.frame=self.now=self.start=None;self.missing_frames=0;self.invalid_samples=0

    def sample(self,frame,now,rows):
        frame,now=int(frame),float(now)
        if not math.isfinite(now) or self.frame is not None and (frame<=self.frame or now<=self.now):raise ValueError('Increasing frame/time required')
        if self.start is None:self.start=now
        ids=[str(r['vehicle_id']) for r in rows]
        if len(ids)!=len(set(ids)):raise ValueError('Duplicate vehicle in junction frame')
        gap=self.frame is not None and frame!=self.frame+1
        if self.frame is not None:self.missing_frames+=frame-self.frame-1
        for vid in list(self.active):
            if gap or vid not in ids:self.active.pop(vid)
        for vid,row in zip(ids,rows):
            totals=self.vehicles.setdefault(vid,dict(wait_s=0.,covered_s=0.,segments=0,invalid_samples=0))
            try:
                vx,vy=float(row['vx']),float(row['vy']);speed=math.hypot(vx,vy)
                if not math.isfinite(speed):raise ValueError()
            except (KeyError,TypeError,ValueError):
                self.active.pop(vid,None);self.invalid_samples+=1;totals['invalid_samples']+=1;continue
            state=self.active.get(vid)
            if state is None:
                totals['segments']+=1
                state=dict(start=now,ready=False,waiting=False,time=now)
                self.active[vid]=state
            ready=now-state['start']>=self.warmup-1e-9
            waiting=ready and (speed<self.resume if state['waiting'] else speed<=self.stop)
            if ready and state['ready']:
                dt=now-state['time'];totals['covered_s']+=dt
                # Require both endpoints waiting; no interpolation across missing coverage.
                if waiting and state['waiting']:totals['wait_s']+=dt
            state.update(time=now,ready=ready,waiting=waiting)
        self.frame,self.now=frame,now
        return self.summary()

    def summary(self):
        valid={k:v for k,v in self.vehicles.items() if v['covered_s']>0}
        total=sum(v['wait_s'] for v in valid.values())
        return dict(junction_id=self.junction_id,value=total/len(valid) if valid else None,unit='s/vehicle',
            status='observed' if valid else 'unavailable',total_wait_vehicle_seconds=total,
            valid_vehicle_count=len(valid),observed_vehicle_count=len(self.vehicles),
            excluded_vehicle_count=len(self.vehicles)-len(valid),covered_vehicle_seconds=sum(v['covered_s'] for v in valid.values()),
            missing_frames=self.missing_frames,invalid_samples=self.invalid_samples,
            start_sim_time=self.start,end_sim_time=self.now,
            config=dict(stop_speed_mps=self.stop,resume_speed_mps=self.resume,observation_warmup_seconds=self.warmup),
            by_vehicle=self.vehicles.copy(),
            definition='mean observed stopped waiting per unique valid vehicle within configured junction region; includes zero-wait and unfinished vehicles; not full-trip delay')


def aggregate_junction_waiting(reports):
    by_junction={};window=None
    for report in reports:
        key=str(report['junction_id'])
        if key in by_junction:raise ValueError('Duplicate junction report')
        current=(report['start_sim_time'],report['end_sim_time'],report['config'])
        if window is not None and current!=window:raise ValueError('Junction windows and waiting rules must match')
        window=current;by_junction[key]=report
    valid=[r['value'] for r in by_junction.values() if r['value'] is not None]
    return dict(value=sum(valid)/len(valid) if valid else None,unit='s/vehicle',
        definition='unweighted arithmetic mean of valid junction means; not vehicle-weighted',
        valid_junction_count=len(valid),excluded_junction_ids=[k for k,v in by_junction.items() if v['value'] is None],
        by_junction=by_junction)