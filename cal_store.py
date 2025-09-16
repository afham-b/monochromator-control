
from __future__ import annotations
import json, os
from dataclasses import dataclass, asdict

CAL_PATH = os.path.join(os.path.dirname(__file__), "state", "cal.json")
os.makedirs(os.path.dirname(CAL_PATH), exist_ok=True)

@dataclass
class CalData:
    S2_STEPS_PER_REV: float | None = None
    HOME_OFFSET_DIR: dict | None = None
    BACKLASH_STEPS: int | None = None
    DIR_REVERSE_SCALE: float | None = None
    WL_MODEL: dict | None = None

def load_cal() -> 'CalData':
    if not os.path.exists(CAL_PATH):
        return CalData(HOME_OFFSET_DIR={"+1": 0, "-1": 0})
    with open(CAL_PATH, "r") as f:
        raw = json.load(f)
    return CalData(**raw)

def save_cal(cal: 'CalData') -> None:
    tmp = CAL_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(asdict(cal), f, indent=2)
    os.replace(tmp, CAL_PATH)
