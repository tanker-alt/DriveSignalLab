from __future__ import annotations
"""Vehicle intent policy and truthful signal negotiation messages.

The policy only emits a discrete request.  It never changes CARLA controls or
traffic-light state; the VLA/controller decides whether the request is safe.
"""
from dataclasses import dataclass
from typing import Dict, Iterable, Optional
import torch
from torch import nn

INTENTS = ("none", "approach_smoothly", "prepare_stop", "start_if_clear")
INTENT_TEXT = {
    "none": "保持当前驾驶意图，不额外引导。",
    "approach_smoothly": "建议平缓接近停止线，保持安全车距。",
    "prepare_stop": "建议准备停车；若信号不允许请勿进入路口。",
    "start_if_clear": "若当前信号和前方空间均允许，建议平缓起步。",
}

@dataclass(frozen=True)
class SignalNotice:
    phase: int
    phase_name: str
    remaining_s: float
    permitted: bool
    consensus: str = "灯控智能体建议保持当前相位"

def intent_to_text(intent: int | str) -> str:
    key = INTENTS[int(intent)] if isinstance(intent, int) else intent
    return INTENT_TEXT.get(key, INTENT_TEXT["none"])

def build_notice(signal: SignalNotice, intent: int | str) -> str:
    allow = "允许通行" if signal.permitted else "禁止通行"
    return (f"真实信号：{signal.phase_name}，剩余约{max(0.0, signal.remaining_s):.1f}秒，{allow}。"
            f"协商结果：{signal.consensus}。车辆建议：{intent_to_text(intent)}")

class VehicleIntentActor(nn.Module):
    """Small categorical actor used by MAPPO; output is an intent only."""
    def __init__(self, obs_dim: int, hidden: int = 64):
        super().__init__()
        self.body = nn.Sequential(nn.Linear(obs_dim, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh())
        self.logits = nn.Linear(hidden, len(INTENTS))
    def forward(self, obs, mask: Optional[torch.Tensor] = None):
        logits = self.logits(self.body(obs))
        if mask is not None: logits = logits.masked_fill(~mask.bool(), -1e9)
        return torch.distributions.Categorical(logits=logits)
