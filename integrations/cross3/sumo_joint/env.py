"""Small TraCI adapter for CoTV-style light/CAV training.

With ``--sumo-cfg`` it drives a real SUMO process through TraCI; without it
the same API runs a synthetic intersection for smoke tests.
"""
from __future__ import annotations
import os, subprocess, time
from typing import Optional
import numpy as np
from .config import TARGET_SPEED, MAX_ACCEL, MAX_DECEL, SIM_STEP, HORIZON

class SumoJointEnv:
    def __init__(self, sumo_cfg: Optional[str] = None, tls_id: str = "", cav_ids=None,
                 horizon: int = HORIZON, seed: int = 0, gui: bool = False):
        self.cfg, self.tls_id, self.cav_ids = sumo_cfg, tls_id, list(cav_ids or [])
        self.horizon, self.seed, self.gui = horizon, seed, gui
        self.rng = np.random.default_rng(seed); self.traci = None; self.proc = None
        self.t = 0; self.phase = 0; self.q = np.zeros(4, dtype=np.float32)
        self._real = bool(sumo_cfg)
    @property
    def real(self): return self._real
    def reset(self, seed: Optional[int] = None):
        if seed is not None: self.rng = np.random.default_rng(seed)
        self.t = 0; self.phase = 0; self.q = self.rng.uniform(1, 5, 4).astype(np.float32)
        if self._real: self._start_sumo()
        return self.observe()
    def _start_sumo(self):
        try: import traci
        except ImportError as e: raise RuntimeError("--sumo-cfg requires Python traci") from e
        self.traci = traci
        binary = os.environ.get("SUMO_BINARY", "sumo-gui" if self.gui else "sumo")
        self.proc = subprocess.Popen([binary, "-c", self.cfg, "--step-length", str(SIM_STEP), "--seed", str(self.seed), "--no-step-log", "true"])
        for _ in range(50):
            try: self.traci.init(8813); break
            except Exception: time.sleep(.1)
        else: raise RuntimeError("Could not connect to SUMO TraCI on port 8813")
    def _close_sumo(self):
        if self.traci:
            try: self.traci.close()
            except Exception: pass
        if self.proc: self.proc.terminate()
        self.traci = self.proc = None
    def observe(self):
        if self._real:
            ids = self.traci.vehicle.getIDList(); q = np.zeros(4, dtype=np.float32); speed = np.zeros(4, dtype=np.float32)
            for vid in ids:
                try:
                    idx = abs(hash(self.traci.vehicle.getLaneID(vid))) % 4; v = self.traci.vehicle.getSpeed(vid); speed[idx] += v
                    if v < .1: q[idx] += 1
                except Exception: continue
            mean_speed = np.clip(speed / np.maximum(q, 1) / TARGET_SPEED, 0, 1); self.q = q
        else: mean_speed = np.clip(1.0 - self.q / 20.0, 0, 1)
        light = np.zeros(4, dtype=np.float32); light[self.phase % 4] = 1
        return np.concatenate([np.clip(self.q / 20, 0, 1), mean_speed, light, np.array([self.t / max(1, self.horizon)], dtype=np.float32)]).astype(np.float32)
    def vehicle_observation(self, n=None):
        n = len(self.cav_ids) if n is None else n
        if self._real and self.cav_ids:
            rows=[]
            for vid in self.cav_ids[:n]:
                try: rows.append([self.traci.vehicle.getSpeed(vid)/TARGET_SPEED, self.traci.vehicle.getAcceleration(vid)/MAX_ACCEL, self.traci.vehicle.getLanePosition(vid)/300., 1., 0., 0.])
                except Exception: rows.append([0,0,0,1,0,0])
            return np.asarray(rows, dtype=np.float32)
        return np.tile(np.array([.5,0,.5,1,0,0], dtype=np.float32), (n,1))
    def step(self, light_action: int, accelerations):
        self.phase = int(light_action) if int(light_action) >= 0 else self.phase; a = np.clip(np.asarray(accelerations, dtype=np.float32), -MAX_DECEL, MAX_ACCEL)
        if self._real:
            if self.tls_id:
                try: self.traci.trafficlight.setPhase(self.tls_id, self.phase)
                except Exception: pass
            for vid, acc in zip(self.cav_ids, a):
                try: self.traci.vehicle.setAcceleration(vid, float(acc), SIM_STEP)
                except Exception: pass
            self.traci.simulationStep(); self.t += 1; arrived = set(self.traci.simulation.getArrivedIDList()); collision = bool(self.traci.simulation.getCollidingVehiclesIDList())
        else:
            arrivals = self.rng.poisson(.25, 4); served = self.rng.poisson(1.2, 4) * (self.phase == np.arange(4)); self.q = np.maximum(0, self.q + arrivals - served); self.q += np.maximum(0, -a.mean()) * .02; self.t += 1; arrived = set(); collision = False
        queue = float(self.q.sum()); reward = -queue - 0.02 * float(np.square(a).mean())
        if collision: reward -= 100.0
        return self.observe(), reward, self.t >= self.horizon, {'collision': collision, 'arrived': len(arrived), 'queue': queue}
    def close(self): self._close_sumo()
