"""Call the sibling MDrive source directly. No copied metric formulas.

PDM is retrospective: never return it through the agent observation interface.
Raw data and route identity must be supplied explicitly; missing data is not a pass.
"""
import os
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

SOURCE = Path(os.environ.get('MDRIVE_ROOT', str(Path(__file__).resolve().parents[2] / 'MDrive-main'))) / 'simulation/leaderboard/leaderboard/utils/pdm_metrics.py'


def unavailable(reason):
    return dict(available=False, value=None, reason=reason)


class MDriveMetrics:
    def __init__(self, vehicle_ids=(), road_map=None, routes=None, source=SOURCE):
        self.ids = tuple(str(v) for v in (vehicle_ids or ()))
        if len(set(self.ids)) != len(self.ids):
            raise ValueError('Duplicate evaluated vehicle ID')
        self.road_map = road_map
        self.routes = {str(k): v for k, v in (routes or {}).items()}
        self.source = Path(source).resolve()
        self.frames = []
        self.route_records = {}
        self.module = None
        self.result = None

    def sample(self, packet):
        if self.result is not None:
            raise RuntimeError('MDrive evaluator is closed')
        if self.frames and (packet['frame'] <= self.frames[-1]['frame'] or
                            packet['sim_time'] <= self.frames[-1]['sim_time']):
            raise ValueError('MDrive frame/time must increase')
        # Detach retained evaluation evidence from mutable caller dictionaries.
        self.frames.append(json.loads(json.dumps(packet, allow_nan=False)))

    def set_route_record(self, vehicle_id, record):
        """Use the original MDrive StatisticsManager result; do not rescore it.

        Call after compute_route_statistics, before close. A simulator failure
        must keep its original status and infractions, not be converted to success.
        """
        vid = str(vehicle_id)
        if vid not in self.ids:
            raise ValueError('Route record vehicle was not registered')
        data = record if isinstance(record, dict) else vars(record)
        if not {'scores', 'infractions', 'status'} <= set(data):
            raise ValueError('Expected an original MDrive RouteRecord')
        self.route_records[vid] = json.loads(json.dumps(data, allow_nan=False))

    def _load(self):
        if self.module is not None:
            return self.module
        # Upstream allows dependency fallbacks which can yield passing metrics;
        # our adapter requires the real geometry/filtering dependencies instead.
        import numpy
        import scipy.signal
        import shapely.geometry
        if not self.source.is_file():
            raise FileNotFoundError('MDrive metric source not found: ' + str(self.source))
        name = '_my_controller_mdrive_pdm_' + str(id(self))
        spec = importlib.util.spec_from_file_location(name, self.source)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        try:
            exec(compile(self.source.read_text(encoding='utf-8'), str(self.source), 'exec'), module.__dict__)
        finally:
            sys.modules.pop(name, None)
        # Isolated module namespace: never change MDrive files or its live provider.
        module.CarlaDataProvider = SimpleNamespace(get_map=lambda: self.road_map)
        self.module = module
        return module

    def _trace(self, vid):
        trace, worlds, polygons = [], [], {}
        last_frame = last_time = last_ang = None
        missing_after_start = False
        for packet in self.frames:
            world = packet.get('world_snapshot')
            if not world or not world.get('complete'):
                return None, 'Missing or incomplete same-frame world snapshot'
            if world.get('frame') != packet['frame'] or world.get('t') != packet['sim_time']:
                return None, 'World/vehicle timestamps differ'
            matches = [a for a in world['actors'] if str(a['id']) == vid]
            if not matches:
                if trace:
                    missing_after_start = True
                continue
            if len(matches) != 1:
                return None, 'Duplicate actor in world snapshot'
            if missing_after_start:
                return None, 'Evaluated actor disappeared and reappeared'
            now = packet['sim_time']
            if last_frame is not None and packet['frame'] != last_frame + 1:
                return None, 'Trajectory has missing frames'
            actor = matches[0]
            required = ('x','y','yaw','vel_x','vel_y','accel_x','accel_y','ang_vel','extent_x','extent_y')
            if any(k not in actor or not math.isfinite(float(actor[k])) for k in required):
                return None, 'Missing/nonfinite actor state'
            if actor['extent_x'] <= 0 or actor['extent_y'] <= 0:
                return None, 'Invalid actor bounding box'
            row = {k: actor[k] for k in required}
            row.update(t=now, actor_id=actor['id'],
                       ang_acc=0.0 if last_time is None else (actor['ang_vel']-last_ang)/(now-last_time))
            trace.append(row)
            # MDrive collision helper expects this ego to be excluded, other egos retained.
            worlds.append(dict(actors=[a for a in world['actors'] if str(a['id']) != vid],
                               traffic_lights=world['traffic_lights']))
            polygons.update(world['traffic_light_polygons'])
            last_frame, last_time, last_ang = packet['frame'], now, actor['ang_vel']
        if len(trace) < 2:
            return None, 'At least two observed ego frames are required'
        intervals = [trace[i]['t']-trace[i-1]['t'] for i in range(1,len(trace))]
        if any(abs(dt-intervals[0]) > max(1e-6,intervals[0]*1e-4) for dt in intervals):
            return None, 'MDrive derivatives require a fixed sampling interval'
        return (trace, worlds, polygons), None

    def close(self):
        if self.result is not None:
            return self.result
        result = dict(source=str(self.source), source_sha256=None,
                      definition='MDrive PDM, unmodified upstream implementation',
                      retrospective=True, vehicles={}, available=False)
        if not self.ids:
            result.update(reason='No explicit VLA/ego vehicle IDs')
            self.result = result
            return result
        try:
            mod = self._load()
            result['source_sha256'] = hashlib.sha256(self.source.read_bytes()).hexdigest()
        except (ImportError, OSError) as exc:
            result['reason'] = str(exc)
            for vid in self.ids:
                result['vehicles'][vid] = dict(pdm=unavailable(str(exc)),
                    leaderboard=self.route_records.get(vid, unavailable('Original MDrive RouteRecord not supplied')))
            self.result = result
            return result
        for vid in self.ids:
            output = dict(leaderboard=self.route_records.get(vid,
                unavailable('Original MDrive RouteRecord not supplied; no invented driving score')))
            result['vehicles'][vid] = output
            prepared, error = self._trace(vid)
            if error:
                output['pdm'] = unavailable(error)
                continue
            if self.road_map is None or mod.carla is None:
                output['pdm'] = unavailable('CARLA map and Python API required for PDM geometry')
                continue
            route = self.routes.get(vid)
            if route is None or len(route) < 2:
                output['pdm'] = unavailable('Reference route missing for evaluated vehicle')
                continue
            trace, worlds, polygons = prepared
            if any(set(w['traffic_lights']) - set(polygons) for w in worlds):
                output['pdm'] = unavailable('Traffic light trigger geometry missing')
                continue
            config = SimpleNamespace(trajectory=route, multi_traj=[route])
            output['pdm'] = mod.compute_pdm_route_metrics(None, trace,
                pdm_world_trace=worlds, pdm_tl_polygons=polygons, config=config, ego_id=0)
            result['available'] |= bool(output['pdm'].get('available'))
        self.result = result
        return result
