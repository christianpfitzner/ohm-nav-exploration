"""The measurement tools, measured.

`tools/measure_drive.py` is what every number in `docs/control-demos.md` was made with, so its own arithmetic
needs a test — in particular the one place where it was wrong in a direction that flattered the robots. Path
length looks like the easiest number on the page: add up the distance between consecutive positions. On this
odometry that is a measure of the odometry. `path_length` is the answer and these cases are the three things it
has to get right, one of which is a robot that never moved at all.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

from measure_drive import DEADBAND, path_length                         # noqa: E402

HZ = 87.0                                               # what the `/odom` bridge actually runs at
REDRAW = 0.040                                          # m: p99 of what the odometry does to a still robot


def redrawn(n, where):
    """`n` positions at the bridge's rate, each one pushed `REDRAW` off every third sample.

    That is the shape measured on a live run rather than a guess at one: over 50 s the step between two
    consecutive odometry readings had a median of 10 mm, a p90 of 30 mm and a p99 of 40 mm — while the robot
    those readings described was standing still with its wheels commanded to exactly zero.
    """
    return [(x + (REDRAW if i % 3 == 0 else 0.0), y)
            for i in range(int(n)) for (x, y) in [where(i / HZ)]]


def test_a_robot_standing_still_did_not_go_anywhere_here_either():
    """The case that started all this, and the one a plain step-sum got wrong by 32 m in 30 s."""
    path, raw = path_length(redrawn(HZ * 30, lambda t: (1.0, 2.0)))
    assert raw > 20.0, f"the noise has to be big enough to be worth testing against, got {raw:.1f} m"
    assert path == 0.0, f"a robot that never moved has to read nil, and it read {path:.2f} m"


def test_the_dead_band_that_eats_the_noise_is_between_the_noise_and_the_measuring():
    """`DEADBAND` is only right if it is bigger than what the sensor does to a still robot and small against
    what the docs then go on to claim. The first half is the p99 of the redraw; the second is why the band
    cannot simply be made enormous: what it gives up is under one band per walk — measured, 0.03 m of a 6.6 m
    run — and the numbers this package is written down against are gaps of half a metre and an arrival declared
    at 0.12 m, so a band that lost half a metre would be a band blind to both of them."""
    assert path_length(redrawn(HZ * 22, lambda t: (1.0, 2.0)))[0] == 0.0, "the band has to be over the noise"
    crawl, _ = path_length(redrawn(HZ * 22, lambda t: (0.3 * t, 0.0)))
    assert abs(crawl - 6.6) < 0.3, f"0.3 m/s for 22 s is 6.6 m and the tool said {crawl:.2f} m"
    assert DEADBAND > REDRAW, f"a {DEADBAND} m band cannot swallow a {REDRAW} m redraw"
    assert abs(crawl - 6.6) < 1.5 * DEADBAND, "the loss has to be under about one band, not one per sample"


def test_a_curve_is_still_a_curve_once_the_noise_is_gone():
    """Decimating a straight line costs nothing; a dead band cuts corners, so the arc is the case that says
    whether 50 mm is too coarse. On a 10 m radius at 0.3 m/s — a gentle sweep, not a pivot — the chord of a
    50 mm step is 0.01 % short of its arc, which is why the tolerance below is 2 % and not 20 %."""
    arc = [(10.0 * math.cos(0.03 * t), 10.0 * math.sin(0.03 * t)) for t in [i / HZ for i in range(int(HZ * 22))]]
    true = 0.3 * 22.0
    path, _ = path_length(arc)
    assert abs(path - true) / true < 0.02, f"6.6 m of 10 m-radius arc measured {path:.2f} m"


def test_what_was_thrown_away_is_reported_rather_than_hidden():
    """The tool prints the step-sum it left out, so a reader can see how big the thing was that the dead band
    swallowed instead of having to take on trust that it was small."""
    still = path_length(redrawn(HZ * 22, lambda t: (2.0, 2.0)))
    moving = path_length(redrawn(HZ * 22, lambda t: (0.3 * t, 2.0)))
    assert still[1] - still[0] > 20.0, "the noise discarded from a still robot should be visible in the output"
    assert moving[1] - moving[0] > 20.0, "and so should the noise discarded from one that was moving"
