"""The measurement tools, measured.

`tools/measure_drive.py` is what every number in `docs/control-demos.md` was made with, so its own arithmetic
is worth a test — in particular the one place where it was wrong in a way that flattered the robots. Path
length is a sum of steps, and a sum of steps is exactly what an odometer with noise turns into a lie: the
simulator's odometry carries a few millimetres into every update and the `/odom` bridge runs at about 87 Hz,
which adds up fast. `split_path` is the answer, and these three cases are the three things it has to get
right, one of which is a robot that never moved at all.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

from measure_drive import split_path                                    # noqa: E402


def jittered(n, where, hz=87.0, noise=0.003):
    """`n` positions at the bridge's own rate, each pushed 3 mm off on alternate samples.

    3 mm is the size of what the live tool sees: on a run of its own the robot, standing still and reporting
    0.00 m/s, accumulated 6.49 m of step-sum in 22 s, which is 6.49 m / (22 s × 87 Hz) ≈ 3.4 mm a step.
    """
    return [where(i / hz) for i in range(int(n))]


def test_a_robot_standing_still_did_not_go_anywhere_here_either():
    """The case that started all this, and the one the first version of the tool got wrong by 6.5 m."""
    parked = jittered(87 * 22, lambda t: (1.0 + (0.003 if int(t * 87) % 2 else 0.0), 2.0))
    path, left_out = split_path(parked, 0.0, 22.0)
    assert left_out > 4.0, f"the noise is still in the step-sum, so the fix is not doing anything: {left_out:.2f}"
    assert path < 1.0, f"a parked robot's path has to read about nil, and it read {path:.2f} m"


def test_the_decimation_that_removes_the_noise_leaves_the_driving_alone():
    """Killing the noise must not be a way of losing the robot's actual travel with it."""
    straight = jittered(87 * 22, lambda t: (0.3 * t + (0.003 if int(t * 87) % 2 else 0.0), 2.0))
    path, _ = split_path(straight, 0.0, 22.0)
    assert abs(path - 6.6) < 0.3, f"0.3 m/s for 22 s is 6.6 m of straight; the tool said {path:.2f} m"


def test_a_curve_is_still_a_curve_at_ten_positions_a_second():
    """Decimating a *straight* line costs nothing; decimating a tight arc cuts corners, so the arc is the case
    that says whether 10 Hz is too coarse. On a 10 m radius at 0.3 m/s — a gentle sweep, not a pivot — the
    chord of one 0.03 rad step is 0.03 % short of the arc, which is why this tolerance is 2 %."""
    arc = jittered(87 * 22, lambda t: (10.0 * math.cos(0.03 * t), 10.0 * math.sin(0.03 * t)), noise=0.0)
    true = 0.3 * 22.0
    path, _ = split_path(arc, 0.0, 22.0)
    assert abs(path - true) / true < 0.02, f"6.6 m of 10 m-radius arc measured {path:.2f} m, 2 % of which is loss"
