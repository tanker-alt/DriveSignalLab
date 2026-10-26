"""Training-data collector only: BasicAgent drives, LMDrive features are recorded.

This agent must never be reported as a VLA driving evaluation.
"""
from pathlib import Path
import json
import torch
from agents.navigation.basic_agent import BasicAgent
from team_code.fixed_signal_lmdrive import FixedSignalLMDrive
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


def get_entry_point():
    return 'TeacherSignalLMDrive'


class TeacherSignalLMDrive(FixedSignalLMDrive):
    def setup(self, path_to_conf_file, ego_vehicles_num):
        self.teachers = {}
        self.training_rows = []
        self.head_feature = None
        super().setup(path_to_conf_file, ego_vehicles_num)
        self.net.waypoints_predictor.register_forward_pre_hook(self._capture_head)
        (Path(self.save_path)/'teacher_manifest.json').write_text(
            json.dumps(dict(driver='BasicAgent', evaluation=False)))

    def _capture_head(self, module, inputs):
        self.head_feature = inputs[0][0, -1].detach().float().cpu()

    def _after_model(self, module, inputs, outputs):
        super()._after_model(module, inputs, outputs)
        actor = CarlaDataProvider.get_hero_actor(hero_id=self.active_ego)
        transform = actor.get_transform()
        self.training_rows.append(dict(
            ego=self.active_ego, time=CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds,
            x=transform.location.x, y=transform.location.y, yaw=transform.rotation.yaw,
            feature=self.head_feature, predicted=outputs[0][-1].detach().float().cpu()))

    def run_step_single_vehicle(self, ego_id, input_data, timestamp):
        super().run_step_single_vehicle(ego_id, input_data, timestamp)
        actor = CarlaDataProvider.get_hero_actor(hero_id=ego_id)
        if ego_id not in self.teachers:
            agent = BasicAgent(actor, target_speed=18)
            world_map = CarlaDataProvider.get_world().get_map()
            plan = [(world_map.get_waypoint(t.location), cmd)
                    for t, cmd in self._global_plan_world_coord_all[ego_id]]
            agent.set_global_plan(plan)
            self.teachers[ego_id] = agent
        control = self.teachers[ego_id].run_step()
        self._write(dict(type='teacher_control', driver='BasicAgent',
                         throttle=control.throttle, steer=control.steer, brake=control.brake))
        return control

    def destroy(self):
        if getattr(self, 'training_rows', None):
            torch.save(dict(schema='lmdrive_teacher_features_v1', driver='BasicAgent',
                            evaluation=False, rows=self.training_rows),
                       Path(self.save_path)/'teacher_features.pt')
        super().destroy()
