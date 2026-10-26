"""Produce a deterministic, explicitly synthetic Safety acceptance fixture."""
import json
import csv
from pathlib import Path
from datetime import datetime, timezone
from uuid import uuid4
import carla
from evaluation.data_products import AssessmentWriter
from evaluation.safety_config import load_config,map_geometry
from evaluation.safety_replay import replay


def vehicle(vid,x,y,speed=0,vx=0):
    return dict(vehicle_id=vid,x=x,y=y,z=0,yaw=0,pitch=0,speed_mps=speed/3.6,
        vx=vx,vy=0,vz=0,road_id=1,lane_id=-1,
        bbox_vertices_world_json=json.dumps([[x+u,y+v,z] for u in (-2,2) for v in (-1,1) for z in (0,1)]))


def main():
    root=(Path(__file__).resolve().parents[3] / 'InformationCollector')
    folder=root.parent/'Maps/MapDataset/maps/cross3'
    road_map=carla.Map('cross3',(folder/'map.xodr').read_bytes().decode('utf-8'))
    config=load_config()
    geo=map_geometry(road_map,config,folder)
    output=root/'evaluation/outputs/safety_acceptance'/('20261025_'+uuid4().hex[:8])
    output.mkdir(parents=True)
    writer=AssessmentWriter(output,config,geo)
    inputs=[]
    times=[0,.1,.4,.6,.7,.8]
    for index,t in enumerate(times):
        rows=[vehicle(1,-60,1.75,[60,66,66,66,63,60][index]),
              vehicle(2,[-31,-30,-29.5,-29,-28.5,-28][index],1.75),
              vehicle(3,-70,5.25,vx=[1,-1,-1,-1,-1,0][index]),
              vehicle(4,-80,[9,10,10,10,10,9][index])]
        signals={line['signal_opendrive_id']:'Red' for line in geo['stop_lines']}
        packet=dict(frame=index+1,sim_time=t,vehicles=rows,signals=signals)
        inputs.append(packet)
        writer.sample(index+1,t,rows,[],[],{},signals)
    report=writer.close('synthetic_acceptance',{})
    (output/'assessment/safety_input.jsonl').write_text('\n'.join(json.dumps(p) for p in inputs)+'\n',encoding='utf-8')
    expected={'speeding':1,'reversing':1}
    actual={key:report['Safety'][key]['value'] for key in expected}
    assert actual==expected,(actual,expected)
    replayed=replay(output)
    rerun=json.loads((replayed/'assessment/evaluation_report.json').read_text(encoding='utf-8'))
    assert report['Safety']==rerun['Safety']
    # Exercise the full collector CSV replay path with the same synthetic inputs.
    from evaluation.summarize import replay as replay_csv
    def write_csv(name,fields,rows):
        with (output/name).open('w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    frame_rows=[dict(frame=p['frame'],sim_time=p['sim_time'],vehicle_count=len(p['vehicles'])) for p in inputs]
    vehicle_rows=[dict(frame=p['frame'],sim_time=p['sim_time'],**r) for p in inputs for r in p['vehicles']]
    signal_rows=[dict(frame=p['frame'],signal_opendrive_id=sid,state=state) for p in inputs for sid,state in p['signals'].items()]
    write_csv('frames.csv',list(frame_rows[0]),frame_rows)
    write_csv('vehicles.csv',list(vehicle_rows[0]),vehicle_rows)
    write_csv('signals.csv',list(signal_rows[0]),signal_rows)
    (output/'metadata.json').write_text(json.dumps(dict(map='Carla/Maps/OpenDriveMap',junction_id=10,
        inventory=dict(opendrive_sha256=geo['opendrive_sha256']),source='synthetic')),encoding='utf-8')
    (output/'metrics_config.json').write_text(json.dumps(dict(junction_id=10)),encoding='utf-8')
    csv_output=replay_csv(output,output/'metrics_config.json')
    csv_report=json.loads((csv_output/'assessment/evaluation_report.json').read_text(encoding='utf-8'))
    assert csv_report['Safety']==report['Safety']
    evidence=dict(source='Synthetic trajectories and velocities; real Cross3 map geometry; no physical vehicle experiment',
                  created_utc=datetime.now(timezone.utc).isoformat(),expected=expected,actual=actual,
                  replay_equal=True,replay_directory=str(replayed),csv_replay_equal=True,csv_replay_directory=str(csv_output))
    (output/'acceptance.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    print(output)
    print(json.dumps(evidence,indent=2))


if __name__=='__main__':main()
