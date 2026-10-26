"""Keep entry signal provenance without inventing a current in-junction rule."""


class SignalAssociation:
    def __init__(self,junction_id):
        self.junction_id=str(junction_id)
        self.previous={}
        self.frame=None

    def update(self,frame,rows,signals):
        frame=int(frame)
        previous=self.previous if self.frame is not None and frame==self.frame+1 else {}
        current={}
        result={}
        for row in rows:
            vid=str(row["vehicle_id"])
            inside=str(row.get("current_junction_id",""))==self.junction_id
            signal_id=None
            source="unresolved"
            if not inside and row.get("signal_match")=="unique" and row.get("signal_opendrive_id") not in (None,""):
                signal_id=str(row["signal_opendrive_id"])
                source="coordinate_unique"
            elif inside and vid in previous:
                signal_id=previous[vid]
                source="retained_entry_signal"
            if signal_id:
                current[vid]=signal_id
            result[vid]=dict(approach_signal_id=signal_id,signal_association_source=source,
                            approach_signal_state=signals.get(signal_id) if signal_id else None)
        self.previous,self.frame=current,frame
        return result
