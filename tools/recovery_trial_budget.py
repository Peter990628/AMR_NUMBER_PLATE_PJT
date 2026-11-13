"""ROS-free time planning for one local translation trial and its return.

This is an estimate, not a deadline guarantee. The live runner must retain its
wall deadline, fresh-input checks, independent command lease, and stop on faults.
The simulation below uses the same turn/drive/align controller as the live trial.
"""

import math
from numbers import Real

from recovery_trial_math import PoseController, angle


STEP_SEC = 0.02
MAX_LEG_SIM_SEC = 120.0
MEASUREMENT_RESERVE_SEC = 44.0  # (8 s settling + 14 s sampling) x two windows
CLEANUP_RESERVE_SEC = 5.0


def _finite_number(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _pose(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or not all(_finite_number(coordinate) for coordinate in value)):
        raise ValueError('pose must contain three finite numbers')
    return tuple(float(coordinate) for coordinate in value)


def _leg(start, target):
    controller = PoseController(target)
    x, y, yaw = start
    # Bound both CPU work and the simulated horizon even if a future controller
    # change fails to converge. Feed the actual predicted endpoint into return.
    maximum_steps = math.ceil(MAX_LEG_SIM_SEC / STEP_SEC)
    for step in range(maximum_steps + 1):
        linear, angular = controller.command((x, y, yaw))
        if controller.phase == 'done':
            return {'converged': True, 'sim_sec': step * STEP_SEC,
                    'predicted_pose': [x, y, angle(yaw)]}
        if step == maximum_steps:
            break
        # Integrate a constant twist over this controller period exactly.
        next_yaw = yaw + angular * STEP_SEC
        if abs(angular) > 1e-12:
            x += linear / angular * (math.sin(next_yaw) - math.sin(yaw))
            y += linear / angular * (math.cos(yaw) - math.cos(next_yaw))
        else:
            x += linear * math.cos(yaw) * STEP_SEC
            y += linear * math.sin(yaw) * STEP_SEC
        yaw = angle(next_yaw)
    return {'converged': False, 'sim_sec': MAX_LEG_SIM_SEC,
            'predicted_pose': [x, y, yaw]}


def assess_translation_budget(current, target, initial, *, remaining_wall_sec,
                              baseline_rate, recent_rate, margin=1.25,
                              per_leg_wall_limit_sec=100.0):
    """Estimate candidate motion, two observations, and return before departure.

    ``baseline_rate`` and ``recent_rate`` are measured positive ROS-seconds per
    wall-second rates. The slower rate is used. The returned feasibility is only
    a predeparture budget check; no commands or I/O are performed here.
    """
    current, target, initial = (_pose(p) for p in (current, target, initial))
    values = (remaining_wall_sec, baseline_rate, recent_rate, margin,
              per_leg_wall_limit_sec)
    if not all(_finite_number(value) for value in values):
        raise ValueError('time budget inputs must be finite numbers')
    if remaining_wall_sec < 0 or baseline_rate <= 0 or recent_rate <= 0:
        raise ValueError('remaining time must be nonnegative and clock rates positive')
    if margin < 1 or per_leg_wall_limit_sec <= 0:
        raise ValueError('margin must be at least one and leg deadline positive')
    rate = min(float(baseline_rate), float(recent_rate))
    outbound = _leg(current, target)
    returning = _leg(outbound['predicted_pose'], initial)
    for leg in (outbound, returning):
        leg['estimated_wall_sec'] = leg['sim_sec'] / rate
        leg['reserved_wall_sec'] = leg['estimated_wall_sec'] * margin
    required = (outbound['reserved_wall_sec'] + returning['reserved_wall_sec']
                + MEASUREMENT_RESERVE_SEC + CLEANUP_RESERVE_SEC)
    if not outbound['converged'] or not returning['converged']:
        reason = 'controller_prediction_not_converged'
    elif max(outbound['reserved_wall_sec'], returning['reserved_wall_sec']) > per_leg_wall_limit_sec:
        reason = 'per_leg_wall_limit'
    elif required > remaining_wall_sec:
        reason = 'insufficient_session_budget'
    else:
        reason = 'ok'
    return {
        'feasible': reason == 'ok', 'reason': reason,
        'required_wall_sec': required, 'remaining_wall_sec': float(remaining_wall_sec),
        'rate_used': rate, 'baseline_rate': float(baseline_rate),
        'recent_rate': float(recent_rate), 'margin': float(margin),
        'per_leg_wall_limit_sec': float(per_leg_wall_limit_sec),
        'measurement_reserve_sec': MEASUREMENT_RESERVE_SEC,
        'cleanup_reserve_sec': CLEANUP_RESERVE_SEC,
        'outbound': outbound, 'return': returning,
        'limitation': 'Planning estimate only; live freshness and wall deadlines remain mandatory.',
    }
