"""Auditable junction efficiency aggregation; no simulation control."""
import math
from collections import deque


class TrafficEfficiency:
    def __init__(self, lines=(), approach_ids=(), window_seconds=60.):
        if not math.isfinite(window_seconds) or window_seconds<=0:raise ValueError('Invalid flow window')
        self.exits={line['id'] for line in lines if line.get('enabled') and line['kind']=='exit'}
        self.approach_ids=set(approach_ids)
        self.window=window_seconds
        self.frame=self.now=self.start=None
        self.intervals=deque();self.crossings=deque();self.total=0;self.unique=set()
        self.by_line={key:0 for key in self.exits};self.ambiguous=0
        self.covered=0.;self.missing=0;self.previous_total=None
        self.queue_integral=self.queue_seconds=0.;self.invalid_queue_frames=0;self.boundary_frames=0
        self.queue_max=None

    def update(self, frame, now, crossings, approaches):
        frame,now=int(frame),float(now)
        if not math.isfinite(now) or self.frame is not None and (frame<=self.frame or now<=self.now):raise ValueError('Frame/time must increase')
        if self.start is None:self.start=now
        contiguous=self.frame is not None and frame==self.frame+1
        if self.frame is not None:
            self.missing+=frame-self.frame-1
            if contiguous:
                self.intervals.append((self.now,now));self.covered+=now-self.now
        groups={}
        for event in crossings:
            if event['line_id'] not in self.exits:continue
            t=float(event['estimated_crossing_time'])
            if not contiguous or not self.now<=t<=now:raise ValueError('Exit crossing outside covered frame interval')
            groups.setdefault(str(event['vehicle_id']),[]).append(event)
        for vid,events in groups.items():
            # Duplicate detector matches cannot inflate global crossings.
            if len({e['line_id'] for e in events})>1:
                self.ambiguous+=1
                continue
            event=min(events,key=lambda e:e['estimated_crossing_time'])
            self.total+=1;self.unique.add(vid);self.by_line[event['line_id']]+=1
            self.crossings.append(float(event['estimated_crossing_time']))
        self.crossings=deque(sorted(t for t in self.crossings if t>now-self.window))
        while self.intervals and self.intervals[0][1]<=now-self.window:self.intervals.popleft()
        queues={a['approach_id']:a for a in approaches}
        complete=bool(self.approach_ids) and set(queues)==self.approach_ids and len(queues)==len(approaches)
        values=[]
        for key in self.approach_ids:
            a=queues.get(key,{})
            value=a.get('queue_length_m')
            if a.get('metric_status')!='valid' or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
                complete=False
            else:values.append(value)
        total=sum(values) if complete else None
        if total is None:self.invalid_queue_frames+=1
        else:
            self.queue_max=total if self.queue_max is None else max(self.queue_max,total)
            if contiguous and self.previous_total is not None:
                self.queue_integral+=(self.previous_total+total)*.5*(now-self.now)
                self.queue_seconds+=now-self.now
        if any(a.get('queue_vehicle_count',0) and a.get('queue_head_distance_m') is not None and
               a['queue_head_distance_m']+a['queue_length_m']>=a['detection_length_m'] for a in approaches if a.get('metric_status')=='valid'):
            self.boundary_frames+=1
        self.previous_total=total;self.frame,self.now=frame,now

    def summary(self, trips):
        window_covered=sum(max(0.,b-max(a,self.now-self.window)) for a,b in self.intervals) if self.now is not None else 0.
        duration=0. if self.now is None else self.now-self.start
        complete_window=window_covered>=self.window-1e-6
        paired=[t for t in trips if t.get('status')=='paired' and isinstance(t.get('travel_time_s'),(int,float)) and math.isfinite(t['travel_time_s']) and t['travel_time_s']>=0]
        values=sorted(t['travel_time_s'] for t in paired)
        def percentile(q):
            if not values:return None
            x=(len(values)-1)*q;i=int(x)
            return values[i]+(values[min(i+1,len(values)-1)]-values[i])*(x-i)
        excluded={}
        for trip in trips:
            if trip not in paired:excluded[trip.get('reason','invalid_duration')]=excluded.get(trip.get('reason','invalid_duration'),0)+1
        return dict(travel_time=dict(value=sum(values)/len(values) if values else None,unit='s',
            status='observed' if values else 'unavailable',paired_count=len(values),excluded_count=len(trips)-len(values),
            censored_by_reason=excluded,median_s=percentile(.5),p95_s=percentile(.95),max_s=max(values) if values else None,
            definition='complete arrival-exit actor-origin visits; all vehicles; incomplete visits excluded, not zero'),
            queue=dict(value=self.queue_integral/self.queue_seconds if self.queue_seconds else None,unit='m',
                definition='time mean of sum of all configured approach queue lengths; complete simultaneous coverage only',
                approach_ids=sorted(self.approach_ids),covered_seconds=self.queue_seconds,
                coverage_ratio=self.queue_seconds/duration if duration else None,max_total_queue_m=self.queue_max,
                invalid_frames=self.invalid_queue_frames,boundary_reached_frames=self.boundary_frames,
                boundary_status='observed_queue_may_be_truncated' if self.boundary_frames else 'no_observed_boundary_contact_not_proof_of_no_overflow'),
            throughput=dict(value=self.total if self.exits else None,unit='crossings',exit_line_ids=sorted(self.exits),
                by_exit=self.by_line,unique_vehicle_count=len(self.unique) if self.exits else None,
                ambiguous_excluded=self.ambiguous,covered_seconds=self.covered,missing_frames=self.missing,
                coverage_ratio=self.covered/duration if duration else None,
                window_count=len(self.crossings) if self.exits else None,window_seconds=self.window,window_covered_seconds=window_covered,
                window_status='complete' if complete_window else 'partial',
                nominal_rate_veh_per_hour=len(self.crossings)*3600/self.window if self.exits else None,
                complete_window_rate_veh_per_hour=len(self.crossings)*3600/self.window if complete_window and self.exits else None,
                definition='unique per-vehicle exit crossing within each frame; repeated visits counted; ambiguous multiple exits excluded'))