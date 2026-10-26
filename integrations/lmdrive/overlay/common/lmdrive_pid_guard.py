"""Avoid undefined steering and PID windup for stationary/backward predictions.

Uses only model-predicted waypoints, never a map or desired route.
"""
import numpy as np


def stabilize_stop_control(result, waypoints, brake_speed, turn_controller, speed_controller):
    points = np.asarray(waypoints, dtype=float)
    desired_speed = float(np.linalg.norm(points[1]-points[0])*2)
    aim = (points[0]+points[1])/2
    degenerate = desired_speed < brake_speed or np.linalg.norm(aim) < .15 or aim[1] >= 0
    if not degenerate:
        return result
    for controller in (turn_controller, speed_controller):
        window = controller._window
        window.clear()
        window.extend([0.] * window.maxlen)
    metadata = dict(result[3], steer=0., throttle=0., brake=1.,
                    stop_prediction_guard=True, launch_assist=False)
    return 0., 0., 1., metadata
