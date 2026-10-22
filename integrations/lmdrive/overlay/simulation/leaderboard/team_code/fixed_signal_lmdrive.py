"""LMDrive under fixed physical signals, with auditable per-ego model calls."""
import json
import time
import os
import torch
import types
from team_code.lmdrive_signal_notice import signal_notice
from team_code.lmdriver_agent import LMDriveAgent
from common.fixed_traffic_light import TrafficLightAgent, CarlaTrafficLightRuntime
from common.traffic_signal_control import build_signal_policy
from common.background_traffic import BackgroundTrafficRuntime
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


def get_entry_point():
    return 'FixedSignalLMDrive'


class FixedSignalLMDrive(LMDriveAgent):
    def _init(self):
        super()._init()
        if getattr(self.lm_config, 'navigation_mode', 'legacy') == 'route_geometry':
            from team_code.lmdrive_route_instructions import RouteInstructionAdapter
            self._instruction_planners = [RouteInstructionAdapter(
                planner, self._global_plan_world_coord_all[i], self._carla_map)
                for i, planner in enumerate(self._instruction_planners)]
            previous_ego = self.active_ego
            for i, planner in enumerate(self._instruction_planners):
                self.active_ego = i
                self._write(dict(type='navigation_plan', turns=planner.turns))
            self.active_ego = previous_ego

    def sensors(self):
        sensors = super().sensors()
        for spec in sensors:
            if spec['type'] == 'sensor.camera.rgb':
                spec.update(lens_circle_multiplier=3.0, lens_circle_falloff=3.0)
        return sensors

    def setup(self, path_to_conf_file, ego_vehicles_num):
        self.signal_runtime = None
        self.background_runtime = None
        self.active_ego = None
        self.active_camera_frame = None
        super().setup(path_to_conf_file, ego_vehicles_num)
        self.light_messages = os.environ.get('LMDRIVE_LIGHT_MESSAGES', 'off')
        if self.light_messages not in ('off', 'on'):
            raise ValueError('LMDRIVE_LIGHT_MESSAGES must be off or on')
        # Enable only the explicit roadside adapter, never privileged planner notices.
        self.net.use_notice_prompt = self.light_messages == 'on'
        self.net.llm_model.register_forward_pre_hook(self._audit_llm, with_kwargs=True)
        original_concat = self.net.concat_text_image_input_with_notice
        def audited_concat(module, *args, **kwargs):
            result = original_concat(*args, **kwargs)
            frame_id, notice_text = args[5], args[6]
            self.notice_span = None
            if frame_id[0] >= 0:
                tokens = module.llm_tokenizer(notice_text, return_tensors='pt', padding='longest',
                    truncation=True, max_length=module.max_txt_len).to(result[0].device)
                count = int(tokens.attention_mask[0].sum())
                start = int(args[1][0].sum()) + frame_id[0] * args[2].size(2)
                expected = module.llm_model.get_input_embeddings()(tokens.input_ids)[0, :count]
                assert torch.equal(result[0][0, start:start+count], expected)
                assert bool(result[1][0, start:start+count].all())
                assert result[3][-1][1] >= start + count
                self.notice_span = dict(start=start, count=count, token_ids=tokens.input_ids[0,:count].tolist())
            return result
        self.net.concat_text_image_input_with_notice = types.MethodType(audited_concat, self.net)
        self.audit = (self.save_path / 'model_audit.jsonl').open('w')
        self.audited_actor_ids = set()
        (self.save_path / 'experiment_config.json').write_text(json.dumps(dict(
            config_path=str(path_to_conf_file), ego_count=ego_vehicles_num,
            light_messages=self.light_messages, notice_adapter='camera_snapshot_v1',
            signal_controller=os.environ.get('LMDRIVE_SIGNAL_CONTROLLER', 'fixed').lower(),
            presslight_checkpoint=os.environ.get('PRESSLIGHT_CHECKPOINT'),
            background_traffic_profile=os.environ.get('BACKGROUND_TRAFFIC_PROFILE', 'none'),
            background_traffic_seed=int(os.environ.get('BACKGROUND_TRAFFIC_SEED', '2000')),
            background_vehicles_per_movement=os.environ.get('BACKGROUND_VEHICLES_PER_MOVEMENT'),
            background_drain_timeout_s=float(os.environ.get('BACKGROUND_DRAIN_TIMEOUT_SECONDS', '120')),
            agent_use_notice=self.agent_use_notice, launch_assist=self.lm_config.launch_assist,
            pid_speed_mode=self.lm_config.pid_speed_mode,
            inference_stride=self.lm_config.inference_stride,
            target_point_distance=self.lm_config.target_point_distance,
            native_traffic=getattr(self.lm_config, 'native_traffic', False),
            expected_vehicle_model=getattr(self.lm_config, 'expected_vehicle_model', None),
            navigation_mode=getattr(self.lm_config, 'navigation_mode', 'legacy'),
            traffic_light=getattr(self.lm_config, 'traffic_light', {})), indent=2))
        self.replay_saved = set()
        self.net.register_forward_pre_hook(self._before_model, with_kwargs=True)
        self.net.register_forward_hook(self._after_model)

    def _write(self, record):
        record.update(ego_index=self.active_ego, camera_frame=self.active_camera_frame,
                      frame=CarlaDataProvider.get_world().get_snapshot().frame)
        self.audit.write(json.dumps(record) + '\n')
        self.audit.flush()

    def _before_model(self, module, inputs, kwargs):
        self.infer_start = time.perf_counter()
        data = inputs[0]
        signal = self.signal_runtime.messages.get(self.active_ego, {}) if self.signal_runtime else {}
        notice, source = signal_notice(signal, self.active_ego, self.active_camera_frame)
        if self.light_messages == 'on':
            data['notice_text'] = [notice]
            data['notice_frame_id'] = [int(data['valid_frames'][0])-1 if notice else -1]
        else:
            data.pop('notice_text', None)
            data.pop('notice_frame_id', None)
        self.notice_span = None
        self.notice_source = source if self.light_messages == 'on' else None
        capture_key = (self.active_ego, signal.get('actual_state'))
        if os.environ.get('LMDRIVE_CAPTURE_REPLAY') == '1' and capture_key not in self.replay_saved:
            def cpu(value):
                if torch.is_tensor(value):
                    return value.detach().cpu()
                if isinstance(value, dict):
                    return {k:cpu(v) for k,v in value.items()}
                return value
            torch.save(dict(samples=cpu(data), image_embeds=cpu(kwargs['image_embeds']),
                            ego_index=self.active_ego, frame=self.active_camera_frame,
                            signal_state=signal.get('actual_state')),
                       self.save_path / ('replay_ego%d_%s.pt' % capture_key))
            self.replay_saved.add(capture_key)
        self._write(dict(type='model_input', text_input=data.get('text_input'),
                         navigation=getattr(self._instruction_planners[self.active_ego], 'last_navigation', None),
                         notice_text=data.get('notice_text'), notice_frame_id=data.get('notice_frame_id'),
                         notice_source=self.notice_source, light_messages=self.light_messages, valid_frames=data.get('valid_frames'),
                         token_ids=module.llm_tokenizer(data['text_input'], truncation=True,
                                                       max_length=module.max_txt_len)['input_ids']))

    def _audit_llm(self, module, inputs, kwargs):
        span = self.notice_span
        if span:
            assert bool(kwargs['attention_mask'][0, span['start']:span['start']+span['count']].all())
        self._write(dict(type='llm_notice_input', notice_span=span, notice_source=self.notice_source,
                         sequence_length=int(kwargs['inputs_embeds'].size(1))))

    def _after_model(self, module, inputs, outputs):
        waypoints = outputs[0].detach().float().cpu().tolist()
        end_logits = outputs[1].detach().float().cpu().tolist()
        last = outputs[0][-1].detach().float().reshape(5, 2)
        target_speed = float((last[0] - last[1]).norm() * 2)
        self._write(dict(type='model_output', elapsed_s=time.perf_counter()-self.infer_start,
                         first_segment_target_speed_mps=target_speed,
                         predicted_brake_by_speed=target_speed < float(self.lm_config.brake_speed),
                         waypoints=waypoints, end_logits=end_logits))

    def start_traffic_signals(self):
        if getattr(self.lm_config, 'native_traffic', False):
            return
        if self.signal_runtime is None:
            controller = os.environ.get('LMDRIVE_SIGNAL_CONTROLLER', 'fixed').lower()
            if controller == 'fixed':
                policy = TrafficLightAgent(**getattr(self.lm_config, 'traffic_light', {}))
            else:
                policy = build_signal_policy(
                    controller,
                    checkpoint=os.environ.get('PRESSLIGHT_CHECKPOINT'),
                    initial_red_seconds=float(os.environ.get('SIGNAL_INITIAL_RED_SECONDS', 2)),
                    min_green_seconds=float(os.environ.get('SIGNAL_MIN_GREEN_SECONDS', 5)),
                    decision_interval_seconds=float(os.environ.get('SIGNAL_DECISION_INTERVAL_SECONDS', 2)),
                    yellow_seconds=float(os.environ.get('SIGNAL_YELLOW_SECONDS', 3)),
                    clearance_seconds=float(os.environ.get('SIGNAL_CLEARANCE_SECONDS', 2)),
                    max_green_seconds=float(os.environ.get('SIGNAL_MAX_GREEN_SECONDS', 15)))
            self.signal_runtime = CarlaTrafficLightRuntime(CarlaDataProvider.get_world(),
                self._global_plan_world_coord_all, self.save_path,
                policy)
            profile = os.environ.get('BACKGROUND_TRAFFIC_PROFILE', 'none').lower()
            count = os.environ.get('BACKGROUND_VEHICLES_PER_MOVEMENT')
            self.background_runtime = BackgroundTrafficRuntime(
                CarlaDataProvider.get_world(), self.signal_runtime.mappings, self.save_path,
                profile=profile,
                seed=int(os.environ.get('BACKGROUND_TRAFFIC_SEED', '2000')),
                count_override=None if count in (None, '') else int(count),
                spawn_distance_m=float(os.environ.get('BACKGROUND_SPAWN_DISTANCE_M', '12')),
                headway_s=float(os.environ.get('BACKGROUND_HEADWAY_SECONDS', '6')),
                speed_difference=float(os.environ.get('BACKGROUND_SPEED_DIFFERENCE', '0')))
            self.signal_runtime.background_runtime = self.background_runtime

    def before_world_tick(self):
        if self.signal_runtime:
            world = CarlaDataProvider.get_world()
            next_time = (world.get_snapshot().timestamp.elapsed_seconds
                         + world.get_settings().fixed_delta_seconds)
            self.signal_runtime.prepare(next_time)
            if self.background_runtime:
                self.background_runtime.prepare(next_time - self.signal_runtime.origin)

    def after_world_tick(self):
        if self.signal_runtime:
            self.signal_runtime.record_snapshot()
            if self.background_runtime and self.signal_runtime.latest_lane_state:
                elapsed = (CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
                           - self.signal_runtime.origin)
                self.background_runtime.observe(self.signal_runtime.latest_lane_state, elapsed)

    def background_traffic_status(self):
        if not self.background_runtime:
            return {'enabled': False, 'drained': True, 'planned': 0, 'spawned': 0,
                    'pending': 0, 'exited': 0}
        return self.background_runtime.status()

    def run_step(self, input_data, timestamp):
        actors = [CarlaDataProvider.get_hero_actor(hero_id=i) for i in range(self.ego_vehicles_num)]
        frames = {i:input_data['rgb_front_%d' % i][0] for i in range(self.ego_vehicles_num)
                  if 'rgb_front_%d' % i in input_data}
        # Only the opt-in adapter supplies camera-aligned snapshots to the model.
        if self.signal_runtime:
            self.signal_runtime.observe(actors, frames)
        return super().run_step(input_data, timestamp)

    def run_step_single_vehicle(self, ego_id, input_data, timestamp):
        self.active_ego = ego_id
        self.active_camera_frame = input_data.get('rgb_front_%d' % ego_id, (None,))[0]
        actor = CarlaDataProvider.get_hero_actor(hero_id=ego_id)
        if actor.id not in self.audited_actor_ids:
            expected = getattr(self.lm_config, 'expected_vehicle_model', None)
            self._write(dict(type='vehicle_identity', actual_model=actor.type_id,
                             expected_model=expected, actor_id=actor.id,
                             sensor_specs=self.sensors()))
            if expected and actor.type_id != expected:
                raise RuntimeError('LMDrive vehicle mismatch: expected %s, got %s; refusing silent fallback'
                                   % (expected, actor.type_id))
            self.audited_actor_ids.add(actor.id)
        control = super().run_step_single_vehicle(ego_id, input_data, timestamp)
        actor = CarlaDataProvider.get_hero_actor(hero_id=ego_id)
        loc, vel = actor.get_location(), actor.get_velocity()
        self._write(dict(type='final_control', x=loc.x, y=loc.y,
                         speed_mps=(vel.x**2+vel.y**2+vel.z**2)**.5,
                         throttle=control.throttle, brake=control.brake, steer=control.steer,
                         launch_assist_enabled=self.lm_config.launch_assist))
        return control

    def destroy(self):
        try:
            if self.background_runtime:
                self.background_runtime.close()
                self.background_runtime = None
            if self.signal_runtime:
                self.signal_runtime.close()
                self.signal_runtime = None
            super().destroy()
        finally:
            if hasattr(self, 'audit'):
                self.audit.close()
