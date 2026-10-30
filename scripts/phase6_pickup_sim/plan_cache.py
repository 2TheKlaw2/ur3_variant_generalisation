"""Tiny on-disk cache for expensive offline plans (CAD contact classification ~1 min each).

Key = hash of the explicit inputs (rounded numbers) + the contents of every file the plan
depends on, so any change to code, CAD, CDMPs, calibration or registration re-plans.
Cache dir: results/.plan_cache/ (not committed). Delete it to force re-planning.
"""
import hashlib
import json
import os
import pickle

import scene_model as sm

DIR = os.path.join(sm.PKG, "results", ".plan_cache")


def key(tag, inputs, files):
    h = hashlib.sha256(tag.encode())
    h.update(json.dumps(inputs, sort_keys=True, default=lambda x: repr(x)).encode())
    for f in sorted(set(files)):
        with open(f if os.path.isabs(f) else os.path.join(sm.PKG, f), "rb") as fh:
            h.update(hashlib.sha256(fh.read()).digest())
    return h.hexdigest()[:24]


def get(k):
    p = os.path.join(DIR, k + ".pkl")
    if os.path.exists(p):
        try:
            with open(p, "rb") as f:
                return pickle.load(f)
        except Exception:
            return None
    return None


def put(k, obj):
    os.makedirs(DIR, exist_ok=True)
    tmp = os.path.join(DIR, k + ".tmp")
    with open(tmp, "wb") as f:
        pickle.dump(obj, f)
    os.replace(tmp, os.path.join(DIR, k + ".pkl"))
