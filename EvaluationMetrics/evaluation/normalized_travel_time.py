"""Per-visit ideal/actual time from clipped observed path, never endpoint chord."""
import math
from collections import Counter

class NormalizedTravelTime:
    def __init__(self, limit_kmh):
        if not isinstance(limit_kmh,(int,float)) or not math.isfinite(limit_kmh) or limit_kmh<=0:
            raise ValueError('A positive finite configured speed limit is required')
        self.limit=limit_kmh/3.6
        self.history={}
        self.results=[]

    def observe(self,frame,now,rows):
        present=set()
        for row in rows:
            vid=str(row['vehicle_id']);present.add(vid)
            try:
                point=tuple(float(row[k]) for k in ('x','y','z'))
                if not all(math.isfinite(v) for v in point):raise ValueError()
            except (ValueError,KeyError,TypeError):
                self.history.pop(vid,None);continue
            h=self.history.setdefault(vid,[])
            if h and (frame!=h[-1][0]+1 or now<=h[-1][1]):h.clear()
            h.append((frame,now,point))
        for vid in set(self.history)-present:self.history.pop(vid,None)

    def annotate(self,trip):
        result=dict(value=None,ideal_time_s=None,path_length_m=None,actual_time_s=trip.get('travel_time_s'),
            limit_kmh=self.limit*3.6,limit_source='safety_config.speeding.limit_kmh',
            path_definition='3D piecewise-linear observed actor-origin path clipped to arrival/exit times',
            status='unavailable',reason=trip.get('reason','incomplete_trip'))
        if trip.get('status')=='paired':
            a,b=trip['arrival_time'],trip['exit_time'];h=self.history.get(str(trip['vehicle_id']),[])
            actual=b-a
            if not math.isfinite(actual) or actual<=0:result['reason']='invalid_actual_time'
            elif not h or h[0][1]>a or h[-1][1]<b:result['reason']='path_coverage_missing'
            else:
                length=covered=0.
                for previous,current in zip(h,h[1:]):
                    t0,t1=previous[1],current[1];dt=min(b,t1)-max(a,t0)
                    if dt>0:
                        length+=math.sqrt(sum((x-y)**2 for x,y in zip(previous[2],current[2])))*dt/(t1-t0)
                        covered+=dt
                if not math.isclose(covered,actual,rel_tol=1e-8,abs_tol=1e-8):result['reason']='path_coverage_missing'
                elif length<=0:result['reason']='nonpositive_path_length'
                else:result.update(value=(length/self.limit)/actual,ideal_time_s=length/self.limit,
                    path_length_m=length,actual_time_s=actual,status='observed',reason=None)
        trip['normalized_travel_time']=result
        self.results.append(result)

    def prune(self,active):
        for vid,h in self.history.items():
            if vid not in active:self.history[vid]=h[-1:]

    def summary(self):
        valid=[r['value'] for r in self.results if r['value'] is not None]
        return dict(value=sum(valid)/len(valid) if valid else None,unit='ratio',
            status='observed' if valid else 'unavailable',valid_trip_count=len(valid),
            excluded_trip_count=len(self.results)-len(valid),
            excluded_reasons=dict(Counter(r['reason'] for r in self.results if r['value'] is None)),
            definition='arithmetic mean of per-complete-visit ideal_time / actual_time; no clipping',
            ideal_time_definition='observed clipped path length / configured speed limit',
            limit_kmh=self.limit*3.6,limit_source='safety_config.speeding.limit_kmh',
            above_one_count=sum(v>1 for v in valid))