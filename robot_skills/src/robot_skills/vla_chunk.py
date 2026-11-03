"""
Helpers to turn an action chunk of the HSR-SmolVLA policy into robot commands.

The policy (https://arxiv.org/abs/2606.00253, https://github.com/paumontagut/per-group-mse-vla) emits
chunks of shape (T, 11) with the columns

    0-4   arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll   per-step joint deltas [m, rad]
    5     gripper                                                 command, ~ -0.8 (closed) .. ~ +1.24 (open)
    6-7   head_pan, head_tilt                                     per-step joint deltas [rad]
    8-10  base_x, base_y, base_theta                              velocities [m/s, m/s, rad/s]

These semantics are inferred from the normalisation statistics shipped with the checkpoint and the 30 fps
of the AIRoA MoMa dataset it was trained on: the arm and head columns have a median of 0 and a small
spread, the base columns saturate at +-0.30 like a velocity limit and the gripper column is bimodal.
Validate them in shadow mode (VLAGrasp(..., execute=False)) before executing on a robot.

This module deliberately has no ROS dependencies so it can be tested with plain nosetests.
"""
from typing import NamedTuple, Optional, Sequence

import numpy as np

ACTION_DIM = 11
ARM = slice(0, 5)
GRIPPER = 5
HEAD = slice(6, 8)
BASE = slice(8, 11)
GROUPS = {"arm": ARM, "gripper": slice(GRIPPER, GRIPPER + 1), "head": HEAD, "base": BASE}

# Limits of the HSR arm joints (hsrb4s.urdf) in the order of the arm columns
HSR_ARM_LOWER = (0.0, -2.62, -2.09, -1.92, -1.92)
HSR_ARM_UPPER = (0.69, 0.0, 3.84, 1.22, 3.67)
HSR_ARM_MAX_VEL = (0.2, 1.2, 2.0, 1.5, 1.5)


class ChunkLimits(NamedTuple):
    """
    Rate and safety limits applied when converting a chunk into commands

    :param fps: rate at which the policy was trained; one chunk row corresponds to 1/fps seconds
    :param arm_lower: lower joint limits of the arm columns
    :param arm_upper: upper joint limits of the arm columns
    :param arm_max_vel: maximum joint velocities of the arm columns [m/s, rad/s]
    :param base_max_vel: maximum base velocities [m/s, m/s, rad/s]
    :param gripper_open_threshold: gripper column value at or above which the gripper is opened
    :param gripper_close_threshold: gripper column value at or below which the gripper is closed
    """
    fps: float = 30.0
    arm_lower: Sequence[float] = HSR_ARM_LOWER
    arm_upper: Sequence[float] = HSR_ARM_UPPER
    arm_max_vel: Sequence[float] = HSR_ARM_MAX_VEL
    base_max_vel: Sequence[float] = (0.15, 0.15, 0.3)
    gripper_open_threshold: float = 0.5
    gripper_close_threshold: float = -0.3


class ChunkPlan(NamedTuple):
    """
    Commands derived from (a prefix of) an action chunk

    :param arm_positions: (n, 5) absolute arm joint positions
    :param times_from_start: (n,) time [s] at which each arm position should be reached
    :param base_velocities: (n, 3) base velocities [m/s, m/s, rad/s], one per step
    :param gripper: (n,) raw gripper column
    """
    arm_positions: np.ndarray
    times_from_start: np.ndarray
    base_velocities: np.ndarray
    gripper: np.ndarray


def validate_chunk(chunk) -> np.ndarray:
    """
    Check that a chunk has shape (T, 11) and contains finite values

    :param chunk: array-like action chunk
    :return: the chunk as a float64 array
    :raises ValueError: on a wrong shape or non-finite values
    """
    chunk = np.asarray(chunk, dtype=np.float64)
    if chunk.ndim != 2 or chunk.shape[1] != ACTION_DIM:
        raise ValueError("Expected an action chunk of shape (T, {}), got {}".format(ACTION_DIM, chunk.shape))
    if not np.isfinite(chunk).all():
        raise ValueError("Action chunk contains NaN or Inf")
    return chunk


def group_max_abs(chunk) -> dict:
    """
    Largest absolute value per joint group, for logging

    :param chunk: (T, 11) action chunk
    :return: dict with the group names as keys
    """
    chunk = validate_chunk(chunk)
    return {name: float(np.abs(chunk[:, slc]).max()) for name, slc in GROUPS.items()}


def integrate_arm(deltas, current, limits: ChunkLimits) -> np.ndarray:
    """
    Cumulatively sum per-step deltas onto the current arm position

    Every step is clipped to the maximum joint velocity and the resulting positions to the joint limits.

    :param deltas: (n, 5) per-step joint deltas
    :param current: (5,) current arm joint positions
    :param limits: limits to apply
    :return: (n, 5) absolute joint positions
    """
    deltas = np.asarray(deltas, dtype=np.float64)
    max_step = np.asarray(limits.arm_max_vel, dtype=np.float64) / limits.fps
    steps = np.clip(deltas, -max_step, max_step)
    positions = np.asarray(current, dtype=np.float64) + np.cumsum(steps, axis=0)
    return np.clip(positions, limits.arm_lower, limits.arm_upper)


def gripper_command(value: float, limits: ChunkLimits, previous: Optional[str] = None) -> Optional[str]:
    """
    Map the raw gripper column to a gripper command with hysteresis

    :param value: raw gripper value
    :param limits: limits holding the thresholds
    :param previous: previously commanded state ('open', 'close' or None)
    :return: 'open', 'close' or None when the value is between the thresholds or the state did not change
    """
    if value >= limits.gripper_open_threshold:
        state = "open"
    elif value <= limits.gripper_close_threshold:
        state = "close"
    else:
        return None
    return None if state == previous else state


def plan_chunk(chunk, current_arm, limits: ChunkLimits, n_steps: Optional[int] = None) -> ChunkPlan:
    """
    Convert (a prefix of) an action chunk into robot commands

    :param chunk: (T, 11) action chunk
    :param current_arm: (5,) current arm joint positions
    :param limits: limits to apply
    :param n_steps: number of rows to use, defaults to the whole chunk
    :return: the commands
    """
    chunk = validate_chunk(chunk)
    n = chunk.shape[0] if n_steps is None else min(n_steps, chunk.shape[0])
    chunk = chunk[:n]
    base_max = np.asarray(limits.base_max_vel, dtype=np.float64)
    return ChunkPlan(
        arm_positions=integrate_arm(chunk[:, ARM], current_arm, limits),
        times_from_start=np.arange(1, n + 1, dtype=np.float64) / limits.fps,
        base_velocities=np.clip(chunk[:, BASE], -base_max, base_max),
        gripper=chunk[:, GRIPPER],
    )
