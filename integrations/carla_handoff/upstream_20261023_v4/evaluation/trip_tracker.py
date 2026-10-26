"""Pair observed arrival/exit crossings without stitching across missing data.

Travel time is between actor-origin counting sections. Waiting is the existing
approach-stop metric within that interval, never total delay or a full-route wait.
"""


class TripTracker:
    def __init__(self, junction_id):
        self.junction_id=str(junction_id)
        self.active={}
        self.previous={}
        self.completed=[]
        self.frame=None
        self.now=None
        self.sequence=0

    def finish(self,vid,reason,exit_event=None,wait_at_exit=None):
        state=self.active.pop(vid)
        paired=reason=="paired" and state["seen_junction"]
        if reason=="paired" and not state["seen_junction"]:
            reason="junction_not_observed"
        if paired and wait_at_exit is None:
            state["wait_issues"].add("waiting_endpoint_unavailable")
        observed_wait=(max(0.,wait_at_exit-state["wait_at_entry"])
                       if wait_at_exit is not None and state["wait_at_entry"] is not None else None)
        result=dict(trip_id=state["trip_id"],vehicle_id=vid,junction_id=self.junction_id,
            arrival_line_id=state["arrival_line_id"],arrival_time=state["arrival_time"],
            exit_line_id=exit_event["line_id"] if exit_event else None,
            exit_time=exit_event["estimated_crossing_time"] if exit_event else None,
            last_observed_time=state["last_time"],status="paired" if paired else "censored",reason=reason,
            travel_time_s=exit_event["estimated_crossing_time"]-state["arrival_time"] if paired else None,
            observed_approach_wait_s=observed_wait,
            approach_wait_s=observed_wait if paired and not state["wait_issues"] else None,
            wait_quality="covered" if paired and not state["wait_issues"] and observed_wait is not None else "incomplete",
            wait_issues=sorted(state["wait_issues"]))
        self.completed.append(result)
        return result

    def update(self,frame,now,rows,crossings,tracker):
        frame,now=int(frame),float(now)
        if self.frame is not None and (frame<=self.frame or now<=self.now):
            raise ValueError("Trip frames/time must increase")
        emitted=[]
        current={str(row["vehicle_id"]):row for row in rows}
        gap=self.frame is not None and frame!=self.frame+1
        for vid in list(self.active):
            if gap or vid not in current:
                emitted.append(self.finish(vid,"frame_gap" if gap else "observation_lost"))
        if gap:
            self.previous={}
        groups={}
        for event in crossings:
            groups.setdefault(str(event["vehicle_id"]),[]).append(event)

        def wait_at(vid,time):
            state=tracker.active.get(vid)
            old=self.previous.get(vid)
            if (state is None or old is None or
                    old["observation_start"] != state["observation_start"] or now<=old["time"]):
                return None
            if vid in self.active and self.active[vid]["tracker_visit"]!=state["visit_id"]:
                return None
            fraction=max(0.,min(1.,(time-old["time"])/(now-old["time"])))
            return old["wait_s"]+fraction*(state["observation_wait_s"]-old["wait_s"])

        def check_wait(vid):
            state=self.active[vid]
            observation=tracker.active.get(vid)
            if observation is None or observation["visit_id"]!=state["tracker_visit"]:
                state["wait_issues"].add("waiting_visit_changed")
            elif (not observation["entered"] and not observation["passed"] and
                  current[vid].get("signal_match")!="unique"):
                state["wait_issues"].add("approach_signal_unresolved")
            state["seen_junction"] |= str(current[vid].get("current_junction_id",""))==self.junction_id
            state["last_time"]=now

        for vid in current:
            if vid in self.active:
                check_wait(vid)
            events=sorted(groups.get(vid,[]),key=lambda event:event["estimated_crossing_time"])
            if any(sum(event["kind"]==kind for event in events)>1 for kind in ("arrival","exit")):
                if vid in self.active:
                    emitted.append(self.finish(vid,"ambiguous_crossing"))
                self.sequence+=1
                result=dict(trip_id=self.sequence,vehicle_id=vid,junction_id=self.junction_id,status="censored",
                    reason="ambiguous_crossing",arrival_time=None,exit_time=None,travel_time_s=None,
                    approach_wait_s=None,observed_approach_wait_s=None,wait_quality="incomplete",wait_issues=["ambiguous_crossing"])
                self.completed.append(result); emitted.append(result)
                continue
            for event in events:
                if event["kind"]=="arrival":
                    if vid in self.active:
                        emitted.append(self.finish(vid,"repeated_arrival"))
                    self.sequence+=1
                    observation=tracker.active.get(vid)
                    baseline=wait_at(vid,event["estimated_crossing_time"])
                    issues=set()
                    if baseline is None:
                        issues.add("waiting_baseline_unavailable")
                    if observation and event["estimated_crossing_time"]<observation["observation_start"]+tracker.warmup:
                        issues.add("observation_warmup")
                    self.active[vid]=dict(trip_id=self.sequence,arrival_line_id=event["line_id"],
                        arrival_time=event["estimated_crossing_time"],last_time=now,seen_junction=False,
                        tracker_visit=observation["visit_id"] if observation else None,
                        wait_at_entry=baseline,wait_issues=issues)
                    check_wait(vid)
                elif event["kind"]=="exit":
                    if vid in self.active:
                        emitted.append(self.finish(vid,"paired",event,wait_at(vid,event["estimated_crossing_time"])))
                    else:
                        self.sequence+=1
                        result=dict(trip_id=self.sequence,vehicle_id=vid,junction_id=self.junction_id,
                            arrival_time=None,exit_time=event["estimated_crossing_time"],exit_line_id=event["line_id"],
                            status="censored",reason="exit_without_arrival",travel_time_s=None,approach_wait_s=None,
                            observed_approach_wait_s=None,wait_quality="incomplete",wait_issues=["arrival_not_observed"])
                        self.completed.append(result); emitted.append(result)
        self.previous={vid:dict(time=now,visit_id=state["visit_id"],
                               observation_start=state["observation_start"],wait_s=state["observation_wait_s"])
                       for vid,state in tracker.active.items()}
        self.frame,self.now=frame,now
        return emitted

    def close(self):
        return [self.finish(vid,"experiment_end") for vid in list(self.active)]

    def summary(self):
        paired=[trip for trip in self.completed if trip["status"]=="paired"]
        covered=[trip["approach_wait_s"] for trip in paired if trip["approach_wait_s"] is not None]
        return dict(paired_trip_count=len(paired),censored_trip_count=len(self.completed)-len(paired),
            open_trip_count=len(self.active),
            mean_paired_travel_time_s=sum(t["travel_time_s"] for t in paired)/len(paired) if paired else None,
            wait_sample_count=len(covered),wait_excluded_paired_count=len(paired)-len(covered),
            mean_paired_approach_wait_s=sum(covered)/len(covered) if covered else None,
            definition="arrival_to_exit_actor_origin; observed_approach_stop_wait; repeated_visits_separate")
