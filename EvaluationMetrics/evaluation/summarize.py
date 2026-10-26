"""从已有逐帧 CSV 回放事件与等待统计，不需要 CARLA 服务端。"""

import argparse
import csv
import json
from pathlib import Path
from datetime import datetime, timezone
from uuid import uuid4

try:
    from .recorder import EvaluationRecorder, DEFAULT_CONFIG
except ImportError:
    from recorder import EvaluationRecorder, DEFAULT_CONFIG


def replay(run, config_path=DEFAULT_CONFIG, spawn_manifest=None, approach_geometry_path=None, safety_config_path=None):
    run = Path(run)
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    config.setdefault("junction_id", int(metadata["junction_id"]))
    if int(metadata["junction_id"]) != config["junction_id"]:
        raise ValueError("Config junction does not match run")
    start_frame = None
    fleet = None
    if spawn_manifest:
        fleet = json.loads(Path(spawn_manifest).read_text(encoding="utf-8"))
        if (fleet["map_name"].split("/")[-1] != metadata["map"].split("/")[-1]
                or int(fleet["junction_id"]) != int(metadata["junction_id"])):
            raise ValueError("Spawn manifest map/junction does not match run")
        vehicles = fleet["vehicles"]
        if not vehicles or any("release_frame" not in vehicle for vehicle in vehicles):
            raise ValueError("Fleet was not fully released; no valid evaluation start frame")
        start_frame = max(vehicle["release_frame"] for vehicle in vehicles) + 1
    # 每次创建新的派生目录，保留原始 CSV 和之前的分析结果。
    output = run / ("analysis_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex[:6])
    output.mkdir()
    (output / "replay_scope.json").write_text(json.dumps(dict(start_frame=start_frame,
        spawn_manifest=fleet, source_run=str(run)), indent=2), encoding="utf-8")
    line_path = run / "detection_lines_used.json"
    line_config = json.loads(line_path.read_text(encoding="utf-8")) if line_path.exists() else {"lines": []}
    geometry_path=Path(approach_geometry_path) if approach_geometry_path else run/"approach_geometry_used.json"
    geometry=json.loads(geometry_path.read_text(encoding="utf-8")) if geometry_path.exists() else None
    if approach_geometry_path and geometry is None:
        raise ValueError("Requested approach geometry file missing")
    if geometry and (int(geometry["junction_id"])!=int(metadata["junction_id"]) or
            geometry["map_name"].split("/")[-1]!=metadata["map"].split("/")[-1] or
            (metadata.get("inventory",{}).get("opendrive_sha256") is not None and
             geometry.get("opendrive_sha256")!=metadata["inventory"]["opendrive_sha256"])):
        raise ValueError("Approach geometry does not match recorded map/junction")
    safety_path = run / 'safety_config_used.json'
    safety_settings = (safety_config_path or (safety_path if safety_path.exists() else None))
    safety_geometry_path = run / 'safety_geometry_used.json'
    safety_geometry = json.loads(safety_geometry_path.read_text(encoding='utf-8')) if safety_geometry_path.exists() else None
    recorded_hash = metadata.get('inventory', {}).get('opendrive_sha256')
    if safety_geometry and recorded_hash and safety_geometry['opendrive_sha256'] != recorded_hash:
        raise ValueError('Safety geometry does not match recorded map')
    recorder = EvaluationRecorder(output, config, line_config, geometry, safety_settings, safety_geometry)
    try:
        from .observations import ObservationWriter, observation_packet
    except ImportError:
        from observations import ObservationWriter, observation_packet
    observations = ObservationWriter(output)
    status = "failed"
    sampled_frames = 0
    signal_path=run/"signals.csv"
    signal_handle=signal_path.open(encoding="utf-8-sig",newline="") if signal_path.exists() else None
    signal_rows=iter(csv.DictReader(signal_handle)) if signal_handle else iter([])
    signal_current=next(signal_rows,None)
    try:
        with (run / "frames.csv").open(encoding="utf-8-sig", newline="") as ff, (run / "vehicles.csv").open(encoding="utf-8-sig", newline="") as vf:
            vehicles = iter(csv.DictReader(vf))
            current = next(vehicles, None)
            for frame in csv.DictReader(ff):
                rows = []
                while current is not None and int(current["frame"]) == int(frame["frame"]):
                    rows.append(current)
                    current = next(vehicles, None)
                if current is not None and int(current["frame"]) < int(frame["frame"]):
                    raise ValueError("Vehicle rows out of frame order")
                if len(rows) != int(frame["vehicle_count"]):
                    raise ValueError("Vehicle count mismatch")
                signals={}
                while signal_current is not None and int(signal_current["frame"])==int(frame["frame"]):
                    signals[str(signal_current["signal_opendrive_id"])]=signal_current["state"]
                    signal_current=next(signal_rows,None)
                if signal_current is not None and int(signal_current["frame"])<int(frame["frame"]):
                    raise ValueError("Signal rows out of frame order")
                if start_frame is not None and int(frame["frame"]) < start_frame:
                    continue
                contexts = recorder.sample(frame["frame"], frame["sim_time"], rows,
                    {} if str(frame.get("read_overlapped_tick", "")).lower() in ("true", "1") else signals)
                observations.write(observation_packet(frame['frame'], frame['sim_time'], rows, signals, contexts,
                    run_id=metadata.get('run_id'), junction_id=metadata['junction_id'],
                    frame_gap=int(frame.get('frame_gap') or 0)))
                sampled_frames += 1
            if current is not None:
                raise ValueError("Vehicle rows outside frame log")
            if signal_current is not None:
                raise ValueError("Signal rows outside frame log")
        if start_frame is not None and not sampled_frames:
            raise ValueError("No recorded frames after fleet release")
        status = "completed"
    finally:
        if signal_handle:
            signal_handle.close()
        recorder.close(status)
        observations.close()
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", help="Directory containing metadata/frames/vehicles")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--spawn-manifest", help="Exclude fleet setup before all vehicles were released")
    parser.add_argument("--approach-geometry", help="Verified lane geometry for older runs without a snapshot")
    parser.add_argument('--safety-config', help='Override saved Safety configuration for this new analysis only')
    args = parser.parse_args()
    print(replay(args.run, args.config, args.spawn_manifest, args.approach_geometry, args.safety_config))
