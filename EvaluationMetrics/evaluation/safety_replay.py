"""Replay per-frame Safety JSONL from the four-phase demo; never overwrite runs."""
import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from .data_products import AssessmentWriter
from .safety_config import load_config


def replay(run, safety_config=None, start_frame=None, end_frame=None):
    run=Path(run)
    source=run/'assessment/input_frames.jsonl'
    if not source.exists():
        source=run/'assessment/safety_input.jsonl'
    if not source.exists():
        source=run/'observations/agent_observations.jsonl'
    if not source.exists():
        raise ValueError('No per-frame JSONL; use evaluation.summarize for collector CSV runs')
    if start_frame is not None and end_frame is not None and start_frame>end_frame:
        raise ValueError('Invalid frame window')
    saved=run/'safety_config_used.json'
    config=load_config(safety_config or (saved if saved.exists() else None))
    geo_path=run/'safety_geometry_used.json'
    geo=json.loads(geo_path.read_text(encoding='utf-8')) if geo_path.exists() else None
    output=run/('safety_analysis_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid4().hex[:6])
    output.mkdir()
    (output/'replay_scope.json').write_text(json.dumps(dict(source=str(source),start_frame=start_frame,end_frame=end_frame,
        preparation_period='Caller must select release frame for formal evaluation'),indent=2),encoding='utf-8')
    writer=AssessmentWriter(output,config,geo)
    status='failed'
    try:
        count=0
        with source.open(encoding='utf-8') as stream:
            for line in stream:
                packet=json.loads(line)
                frame=int(packet['frame'])
                if start_frame is not None and frame<start_frame:continue
                if end_frame is not None and frame>end_frame:continue
                signals={} if packet.get('read_overlapped_tick') else packet.get('signals',{})
                writer.sample(frame,packet['sim_time'],packet['vehicles'],[],[],{},signals)
                count+=1
        if not count:raise ValueError('No frames in requested window')
        status='completed'
    finally:
        writer.close(status,{})
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run')
    p.add_argument('--safety-config')
    p.add_argument('--start-frame',type=int)
    p.add_argument('--end-frame',type=int)
    a=p.parse_args()
    print(replay(a.run,a.safety_config,a.start_frame,a.end_frame))
