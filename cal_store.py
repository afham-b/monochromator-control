# cal_store.py
import json, os, time
from typing import Dict, Any

DEFAULT_CAL = {
    "version": 1,
    "saved_at": None,               # unix ts
    "s1": {                         # small disk / sensor 1
        "steps_per_rev": None,
        "edge_holdoff_steps": None  # optional; if you measure it
    },
    "s2": {                         # worm gear / sensor 2
        "steps_per_rev": None,
        "steps_per_deg": None
    },
    "backlash": {                   # directional bias/backlash block
        "backlash_steps": None,
        "reverse_scale": 1.0
    },
    "grating": {                    # for wavelength mapping helpers (optional)
        "d_mm": None,              # groove spacing (mm)
        "theta_zero_deg": 0.0      # θ offset for your zero-order alignment (optional)
    }
}

def _cal_path(state_dir: str) -> str:
    os.makedirs(state_dir, exist_ok=True)
    return os.path.join(state_dir, "calibration.json")

def load_cal(state_dir: str) -> Dict[str, Any]:
    """Load calibration from state/calibration.json. Returns a dict (always with defaults merged)."""
    path = _cal_path(state_dir)
    cal = json.loads(json.dumps(DEFAULT_CAL))  # deep copy
    try:
        with open(path, "r") as f:
            data = json.load(f)
        # shallow-merge (keep DEFAULT_CAL fields present even if missing in file)
        for k, v in data.items():
            if isinstance(v, dict) and k in cal and isinstance(cal[k], dict):
                cal[k].update(v)
            else:
                cal[k] = v
    except FileNotFoundError:
        # no file yet: keep defaults
        pass
    except Exception as e:
        print(f"[cal_store] Could not read {path}: {e} — using defaults")
    return cal

def save_cal(
    state_dir: str,
    *,
    s1_steps_per_rev=None,
    s2_steps_per_rev=None,
    steps_per_deg=None,
    backlash_steps=None,
    reverse_scale=None,
    d_mm=None,
    theta_zero_deg=None,
    edge_holdoff_steps=None
) -> str:
    """
    Update any subset of calibration values and save atomically.
    Returns the absolute path of the saved JSON.
    """
    path = _cal_path(state_dir)
    cal = load_cal(state_dir)

    if s1_steps_per_rev is not None:
        cal["s1"]["steps_per_rev"] = int(s1_steps_per_rev)
    if edge_holdoff_steps is not None:
        cal["s1"]["edge_holdoff_steps"] = int(edge_holdoff_steps)

    if s2_steps_per_rev is not None:
        cal["s2"]["steps_per_rev"] = int(s2_steps_per_rev)
    if steps_per_deg is not None:
        cal["s2"]["steps_per_deg"] = float(steps_per_deg)

    if backlash_steps is not None:
        cal["backlash"]["backlash_steps"] = int(backlash_steps)
    if reverse_scale is not None:
        cal["backlash"]["reverse_scale"] = float(reverse_scale)

    if d_mm is not None:
        if cal.get("grating") is None:
            cal["grating"] = {}
        cal["grating"]["d_mm"] = float(d_mm)
    if theta_zero_deg is not None:
        if cal.get("grating") is None:
            cal["grating"] = {}
        cal["grating"]["theta_zero_deg"] = float(theta_zero_deg)

    cal["saved_at"] = time.time()

    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cal, f, indent=2, sort_keys=True)
    os.replace(tmp, path)   # atomic save on POSIX/NTFS
    return os.path.abspath(path)

def apply_cal_to_globals(cal: Dict[str, Any], g: Dict[str, Any]) -> None:
    """
    Apply a loaded cal dict onto your module globals (dict returned by globals()).
    Only assigns when values are present.
    """
    # S1
    s1 = cal.get("s1", {})
    if s1.get("steps_per_rev") is not None:
        g["steps_per_rev"] = int(s1["steps_per_rev"]) 
    if s1.get("edge_holdoff_steps") is not None:
        g["EDGE_HOLDOFF_STEPS"] = int(s1["edge_holdoff_steps"])

    # S2
    s2 = cal.get("s2", {})
    if s2.get("steps_per_rev") is not None:
        g["S2_STEPS_PER_REV"] = int(s2["steps_per_rev"])
    if s2.get("steps_per_deg") is not None:
        g["STEPS_PER_DEG"] = float(s2["steps_per_deg"])

    # Backlash
    b = cal.get("backlash", {})
    if b.get("backlash_steps") is not None:
        g["BACKLASH_STEPS"] = int(b["backlash_steps"])
    if b.get("reverse_scale") is not None:
        g["DIR_REVERSE_SCALE"] = float(b["reverse_scale"])

    # Grating (optional)
    gr = cal.get("grating", {})
    if gr.get("d_mm") is not None:
        g["GRATING_D_MM"] = float(gr["d_mm"])
    if gr.get("theta_zero_deg") is not None:
        g["THETA_ZERO_DEG"] = float(gr["theta_zero_deg"])
