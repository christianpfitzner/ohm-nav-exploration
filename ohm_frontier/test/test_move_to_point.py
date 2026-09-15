"""The orient-then-drive controller, checked as arithmetic.

The same argument as in `test_reactive.py` for the other two: `orient_then_drive` is a function of pose and
goal, so the phase it chooses is a fact about angles and not about a graph — and every claim in the module
docstring, which is written as numbers a lecture can compute, is checkable here without ROS, a hall or a
robot. `move_to_point.py` guards its ROS imports for exactly this.

What is *not* tested here is where the robot actually ends up, which is a claim about a simulator, a slip
factor and an odometry that integrates metres it never drove. That claim is `test_integration.py`'s, and the
line between the two is the reason the phases are a function at all.
"""
from math import atan2, cos, degrees, hypot, pi, sin

import pytest

from ohm_frontier.angles import wrap
from ohm_frontier.move_to_point import ARRIVED, DRIVING, NO_GOAL, ORIENTING, Motion
from ohm_frontier.move_to_point import orient_then_drive, parse_command

# The node's defaults, spelled out here so a changed default in the module shows up as a changed number
# here rather than as a controller that quietly stopped re-aiming.
TIGHT = dict(gain_turn=1.8, turn_limit=1.2, speed=0.3, decelerate_over=0.25,
             aim_tolerance=0.08, arrive_distance=0.12, aim_commit=0.25)


def test_a_place_ahead_and_to_the_left_asks_for_a_left_turn_and_nothelse():
    """Face east at the origin, place 2 m north: 90° off, so it turns, and turns left, and does not move."""
    decided = orient_then_drive((0.0, 0.0, 0.0), (0.0, 2.0), **TIGHT)
    assert decided.phase == ORIENTING
    assert decided.forward == 0.0, "a sequential controller does not drive while it aims; that is the " \
                                   "difference between this file and turn_and_move.drive_to"
    assert decided.turn > 0, "left is positive, which is the same sign convention as the wheels'"
    assert abs(decided.heading_error - pi / 2) < 1e-9
    assert abs(decided.remaining - 2.0) < 1e-9, "remaining is distance to the place, not distance driven"


def test_the_turn_is_proportional_and_ceilinged_at_what_the_wheels_reach():
    """1.8 per radian, then the ceiling — the two numbers on the slide about a turn that saturates."""
    slightly_off = orient_then_drive((0.0, 0.0, 0.0), (2.0, 0.5), **TIGHT)        # 14° off the nose
    assert abs(slightly_off.turn - 1.8 * 0.2449786631) < 1e-6, "0.44 rad/s: proportional, and under the "\
                                         "ceiling, which is the pair this test is about"
    behind = orient_then_drive((0.0, 0.0, 0.0), (-1.0, -0.1), **TIGHT)          # 174° off, to the right
    assert abs(behind.turn) == 1.2, "a half-turn would ask 1.8 · 3.04 = 5.5 rad/s, which these wheels never " \
                                    "reach; the ceiling is the honest command, and it bites at " \
                                    "1.2 / 1.8 = 0.67 rad = 38° off"
    assert behind.turn < 0, "clockwise: the place is behind and to the right, and the sign is the wheels'"


def test_a_place_inside_the_cone_is_driven_not_turned_and_the_last_metres_are_slow():
    """The cone is the whole controller: in it, straight ahead at full speed until the deceleration distance."""
    decided = orient_then_drive((0.0, 0.0, 0.02), (2.0, 0.0), **TIGHT)     # 1.1° off, inside 0.08 rad
    assert decided.phase == DRIVING and decided.turn == 0.0
    assert abs(decided.forward - 0.3) < 1e-9, "2 m away is past the last 0.15 m, so full speed"
    nearly = orient_then_drive((0.0, 0.0, 0.0), (0.18, 0.0), **TIGHT)      # 18 cm left: inside the ramp
    assert nearly.phase == DRIVING and abs(nearly.forward - 0.3 * 0.18 / 0.25) < 1e-9
    assert nearly.forward < 0.3, "the last quarter metre is driven proportionally slower, which is what makes " \
                                 "the stop a stop rather than a one-tick halt; see the rule in the function"


def test_arrived_is_measured_to_the_place_and_not_along_the_way_there():
    """0.1 m from the place is arrived; 2 m of driving that ends 1 m away is not."""
    assert orient_then_drive((0.0, 0.0, 0.0), (0.1, 0.0), **TIGHT).phase == ARRIVED
    stopped = orient_then_drive((0.0, 0.0, 0.0), (0.1, 0.0), **TIGHT)
    assert stopped.forward == 0.0 and stopped.turn == 0.0, "arrived means stopped, not coasting through it"


def test_the_place_behind_is_reached_by_the_short_way_round():
    """Due west of a robot facing due east is 180°, not 0° — and a place 4° off behind is 176°, not -184°.

    The wrap bug in its most useful disguise: without it the controller would spin three-quarters of a turn
    to go somewhere it could reach in a quarter, and the sign of the turn would depend on which side of the
    calendar the arithmetic landed on.
    """
    due_behind = orient_then_drive((0.0, 0.0, 0.0), (-2.0, 0.0001), **TIGHT)
    assert abs(due_behind.heading_error) > pi - 0.01, "just short of a half-turn, on one side of it"
    assert abs(due_behind.heading_error) <= pi, "and never more than a half-turn, which is the whole point"
    assert abs(due_behind.turn) <= 1.2
    just_past = orient_then_drive((0.0, 0.0, 0.0), (-2.0, -0.0001), **TIGHT)
    assert due_behind.turn * just_past.turn < 0, "4° either side of due west, the two turns must be opposite; " \
                                                "both same sign would mean no wrap in the subtraction"


def test_the_tolerance_is_the_accuracy_you_can_compute_before_it_moves():
    """0.08 rad at 3 m is 24 cm it will accept; 0.25 rad is 75 cm. Both arrive, which is the lesson."""
    wide = orient_then_drive((0.0, 0.0, 0.0), (3.0, 0.6), **{**TIGHT, "aim_tolerance": 0.25})
    assert wide.phase == DRIVING, "11 cm off the line at 3 m, inside a 0.25 rad cone, so it drives at it"
    assert abs(degrees(atan2(0.6, 3.0)) - 11.3) < 0.2, "the arithmetic on the slide, checked here"
    tight = orient_then_drive((0.0, 0.0, 0.0), (3.0, 0.6), **TIGHT)
    assert tight.phase == ORIENTING and tight.forward == 0.0, "the same place, the same robot, the same hall, " \
                                                             "and the only thing that changed is a number " \
                                                             "that decides what this controller can see"


def test_a_command_that_is_not_two_numbers_says_why_in_the_words_a_person_typed():
    """`parse_command`'s second answer is for the person at the terminal, so it is part of the contract."""
    assert parse_command("go 3.0 2.0") == ("go", (3.0, 2.0))
    assert parse_command("  TO -1.5 4 ") == ("go", (-1.5, 4.0)), "one word, two spellings, one answer"
    assert parse_command("stop")[0] == "stop"
    for typed, says in (("go 3.0", "two numbers"), ("dance 1 2", "unknown command"),
                        ("go a b", "two numbers"), ("", "empty")):
        what, reason = parse_command(typed)
        assert what is None and says in reason, f"'{typed}' should explain itself with '{says}'"


def test_the_phases_are_four_named_states_and_nothing_else():
    """A student reading `decided.phase` should see four possibilities, and the fifth is the empty graph."""
    assert {ORIENTING, DRIVING, ARRIVED, NO_GOAL} == {"orienting", "driving", "arrived", "no goal yet"}
    assert orient_then_drive((0, 0, 0), (5, 5), **TIGHT).phase in (ORIENTING, DRIVING)
    assert Motion._fields == ("phase", "forward", "turn", "heading_error", "remaining"), \
        "the two numbers the view labels are the two numbers the log line quotes; they come from here"


def test_the_cone_still_decides_the_approach_and_the_commit_decides_the_finish():
    """Two distances, two rules, and the reason they are different: authority, not preference.

    Out on the approach the cone is the whole controller and 5° off a place 2 m away is a turn, with no
    forward drive — that half is the difference between this file and `turn_and_move.drive_to`, and it has to
    survive every fix to the other half.

    Inside `aim_commit` the cone is not asked, because at the edge of it the orient phase can only turn at
    `gain_turn · aim_tolerance` = 0.144 rad/s while the bearing to a place 0.3 m away swings at 0.08 rad/s.
    The committed line is bounded rather than trusted: from 0.25 m at 0.3 rad off the nose — four times the
    cone, deliberately exaggerated — it still passes the place inside 0.08 m of the tolerance it has to meet.
    """
    far_and_off = orient_then_drive((0.0, 0.0, 0.0), (2.0, 0.2), **TIGHT)     # the place 5.7° off the nose
    assert far_and_off.phase == ORIENTING and far_and_off.forward == 0.0, \
        "2 m out and 5.7° off the line: this is still the sequential controller, and it still pays with a stop"

    near_and_off = orient_then_drive((0.0, 0.0, 0.3), (0.20, 0.15), **TIGHT)     # 0.25 m away, 0.3 rad off
    assert near_and_off.phase == DRIVING, "0.25 m out, and the argument is over: driving is the only phase " \
                                          "that can close the last stretch"
    assert near_and_off.turn == 0.0 and near_and_off.forward > 0.0, "straight, slower, and not stopped"
    assert near_and_off.remaining * abs(near_and_off.heading_error) < 0.12, \
        "the perpendicular miss this line can still make, against the tolerance it has to arrive inside"


def test_the_commit_and_the_tolerance_are_not_independent_numbers():
    """`aim_commit · sin(aim_tolerance) <= arrive_distance`, which is what makes committing safe.

    The default pair leaves 6x of margin, so `aim_tolerance:=0.25` — the setting the launch file advertises
    for the "arrive wide" demo — is still safe at 0.062 m. At 0.5 rad the committed line can miss by the whole
    0.12 m tolerance, and that is the edge of the parameter, not a thing to discover in front of a hall.
    """
    assert 0.25 * sin(0.08) == pytest.approx(0.020, abs=0.001), "the default: 2 cm against a 12 cm tolerance"
    assert 0.25 * sin(0.25) < 0.12, "the wide demo setting still arrives, at 6.2 cm"
    assert 0.25 * sin(0.5) == pytest.approx(0.120, abs=0.001), "and 0.5 rad is the end of it: asin(0.12 / 0.25) " \
                                                               "= 0.50 rad = 29° is the widest cone a 0.25 m " \
                                                               "commit can be paired with"


def test_the_committed_approach_arrives_and_stops_quarrelling_on_the_way_in():
    """The acceptance of the whole file, closed on an ideal odometry: 6.0 m, and it ends at the place.

    The odometry is the truth here — the pose is the command integrated, no slip, no sensor noise — because
    the claim is about the rule. On an odometry that cannot lie the old rule still crept in eventually (23.9 s,
    0.115 m), which is why this test compares the two settings rather than asserting that one of them fails:
    what the argument costs is *time* near the place, and the live hall makes that difference an arrival or a
    miss. Measured on the same 6.0 m in `open`, the version without a commit distance ended 0.187 m short of
    the place at the end of 28 s, still printing `orienting`, after 36 changes of phase in 25 s of driving.

    Two things are asserted beyond arriving: no `orienting` cycle at all inside the commit radius, and a
    higher share of forward drive over the last half metre than the always-asking rule manages (measured
    offline: 76 % against 64 %).
    """
    spawn, goal = (2.25, 6.25, 0.0), (2.25, 12.25)        # the `open` spawn and a place 6.0 m due north

    def run(commit, seconds=40.0, dt=0.05):
        pose, out = spawn, []
        for _ in range(int(seconds / dt)):
            decided = orient_then_drive(pose, goal, **{**TIGHT, "aim_commit": commit})
            out.append((pose, decided))
            heading = pose[2] + 0.5 * decided.turn * dt
            pose = (pose[0] + decided.forward * cos(heading) * dt,
                    pose[1] + decided.forward * sin(heading) * dt, pose[2] + decided.turn * dt)
            if decided.phase == ARRIVED:
                break
        return out, pose

    def duty_in_the_last(out, goal, within):
        near = [(pose, d) for pose, d in out if hypot(pose[0] - goal[0], pose[1] - goal[1]) < within]
        return sum(1 for _, d in near if d.forward > 0.02) / len(near), near

    committed, where = run(TIGHT["aim_commit"])
    always_asking, _ = run(0.0001)                       # the rule as it was: the cone asked on every cycle

    residue = hypot(where[0] - goal[0], where[1] - goal[1])
    assert residue <= TIGHT["arrive_distance"], f"ends {residue:.3f} m from the place"
    assert len(committed) * 0.05 < 30.0, f"{len(committed) * 0.05:.1f} s for 6 m at 0.3 m/s is driving"
    path = sum(hypot(b[0][0] - a[0][0], b[0][1] - a[0][1]) for a, b in zip(committed, committed[1:]))
    net = hypot(where[0] - spawn[0], where[1] - spawn[1])
    assert path / net < 1.5, f"{path:.2f} m of path for {net:.2f} m of net: a straight line with a turn at " \
                             "the start of it, not a approach that spends its path re-aiming"

    inside = [d.phase for pose, d in committed
              if hypot(pose[0] - goal[0], pose[1] - goal[1]) <= TIGHT["aim_commit"]]
    assert ORIENTING not in inside, "inside the commit radius this node never stops to look at the place again"

    committed_duty, _ = duty_in_the_last(committed, goal, 0.5)
    asked_duty, _ = duty_in_the_last(always_asking, goal, 0.5)
    assert committed_duty > asked_duty, f"over the last half metre: {committed_duty:.0%} of the cycles drive " \
                                        f"against {asked_duty:.0%} when the cone is asked all the way in"


def test_the_angle_helper_this_package_now_shares_in_one_place():
    """`wrap` used to be written three times; now it is written once, and the boundary is worth naming."""
    assert abs(wrap(0.0)) == 0.0
    assert abs(wrap(3 * pi) - pi) < 1e-9 or abs(wrap(3 * pi) + pi) < 1e-9
    assert abs(wrap(pi) - wrap(-pi)) < 1e-9, "the two ends of the range are the same direction, and which " \
                                            "name this returns does not matter — a test on that would be " \
                                            "testing the modulo, not the robot"
    for raw in (-7.0, -0.1, 0.0, 0.1, 6.9, 100.0):
        assert -pi - 1e-12 <= wrap(raw) <= pi + 1e-12
