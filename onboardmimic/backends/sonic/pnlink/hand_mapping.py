"""Revo2 mapping from host_setup/g1_hands/mocap.py (Apache-2.0).

Source: GR00T-WholeBodyControl; see PROVENANCE.md and the repository LICENSE.
Pure math only: no hardware, publisher, or calibration side effects.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation

CHANNELS = ("thumb_flex", "thumb_aux", "index", "middle", "ring", "pinky")
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
# SMPL-X joint ordering: index, middle, pinky, ring, thumb (three joints each).
GROUPS = ((13, 14), (12,), (0, 1, 2), (3, 4, 5), (9, 10, 11), (6, 7, 8))
# One measured flexion per human finger. Both robot thumb actuators follow the
# thumb's measured chain, including its CMC joint; six independent glove DOFs
# are not required to follow five fingers.
FLEX_GROUPS = ((12, 13, 14), (12, 13, 14), *GROUPS[2:])
CHANNEL_FINGERS = ("thumb", "thumb", *FINGERS[1:])
MIN_SPAN = 0.15  # radians, aggregate rotation distance for each actuator group
CAPS = np.array([0.6, 0.75, 1., 1., 1., 1.])


def pose_array(value):
    arr = np.asarray(value, dtype=float)
    if arr.shape not in ((45,), (1, 45), (15, 3)) or not np.isfinite(arr).all():
        raise ValueError("Expected 15 finite hand axis-angle rotations")
    arr = arr.reshape(15, 3)
    if np.max(np.linalg.norm(arr, axis=1)) > math.pi + 1e-3:
        raise ValueError("Hand rotation outside canonical axis-angle range")
    return arr


def relative_pose(origin, pose):
    return (Rotation.from_rotvec(origin).inv() * Rotation.from_rotvec(pose)).as_rotvec()


class Mapping:
    def __init__(self, calibration, mode, closure=0.7, allow_partial=False, hand='right'):
        version = calibration.get("version")
        if hand not in ('left', 'right') or version not in (1, 2) or calibration.get("hand") != hand:
            raise ValueError("Expected version 1 or 2 " + str(hand) + "-hand calibration")
        if "open" not in calibration or "closed" not in calibration:
            raise ValueError("Calibration incomplete; capture both calibrate-open and calibrate-closed")
        if mode not in ("grasp", "fingers") or not math.isfinite(closure) or not 0.05 <= closure <= 1:
            raise ValueError("Invalid tracking mode or closure amplitude")
        self.open = pose_array(calibration["open"])
        self.groups = GROUPS if version == 1 else FLEX_GROUPS
        closed = pose_array(calibration["closed"])
        overrides = calibration.get("finger_closed", {}) if version == 2 else {}
        if not isinstance(overrides, dict) or set(overrides) - set(FINGERS):
            raise ValueError("Invalid per-finger calibration")
        self.directions = [relative_pose(self.open, pose_array(overrides.get(finger, closed)))
                           for finger in CHANNEL_FINGERS]
        self.spans = np.array([np.linalg.norm(d[list(g)])
                               for d, g in zip(self.directions, self.groups)])
        self.valid = self.spans >= MIN_SPAN
        self.mode, self.closure = mode, closure
        if mode == "fingers" and not self.valid.all() and not allow_partial:
            missing = [n for n, valid in zip(CHANNELS, self.valid) if not valid]
            raise ValueError(f"Insufficient glove response for {missing}; recalibrate that finger "
                             "or use --allow-partial to hold missing channels at startup feedback")
        if mode == "fingers" and not self.valid.any():
            raise ValueError("No measurable finger response")
        if mode == "grasp" and np.count_nonzero(self.valid[2:]) < 2:
            raise ValueError("Grasp needs measurable open/closed response in at least two fingers")

    def target(self, pose):
        delta = relative_pose(self.open, pose_array(pose))
        amounts = np.zeros(6)
        for i, group in enumerate(self.groups):
            if self.valid[i]:
                g = list(group)
                amounts[i] = np.sum(delta[g] * self.directions[i][g]) / self.spans[i] ** 2
        amounts = np.clip(amounts, 0., 1.)
        if self.mode == "grasp":
            grip = float(np.mean(amounts[2:][self.valid[2:]]))
            amounts[:] = grip
        # A small dead zone suppresses static glove jitter near the open pose.
        amounts = np.clip((amounts - 0.03) / 0.97, 0., 1.)
        return amounts * CAPS * self.closure
