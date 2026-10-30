"""Adapter for MDrive's ``notice_text`` channel."""
from .vehicle_intent_policy import SignalNotice, build_notice

def make_notice(phase_name, remaining_s, permitted, intent, consensus="灯控智能体建议保持当前相位", phase=0):
    return build_notice(SignalNotice(phase=int(phase), phase_name=str(phase_name),
                                     remaining_s=float(remaining_s), permitted=bool(permitted),
                                     consensus=consensus), intent)
