"""Lossless per-tick physical evidence; excludes nonserializable simulator objects."""
import gzip
import json


class Recorder:
    def __init__(self, folder, backend):
        self.file=gzip.open(folder/'measurements.jsonl.gz','wt',encoding='utf-8')
        self.backend=backend

    def write(self, now, dt, before, after, completed, colors, physical_signals, collisions):
        clean=lambda values:{vid:{k:x for k,x in v.items() if k!='wp'} for vid,v in values.items()}
        record=dict(schema='cross3_measurements_v2',backend=self.backend,time=now,dt=dt,
                    before=clean(before),after=clean(after),completed=list(completed),
                    colors=colors,physical_signals=physical_signals,collisions=list(collisions))
        self.file.write(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n')

    def close(self):
        self.file.close()
