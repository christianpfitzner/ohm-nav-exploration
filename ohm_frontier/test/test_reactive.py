"""The reactive family's maths, tested without ROS and without a hall.

    python3 -m pytest test          # needs nothing but numpy

`wall_following.py`, `obstacle_avoidance.py` and `turn_and_move.py` each keep their arithmetic in a plain
function — `steer`, `field`, `turn_towards`, `drive_straight`, `drive_to` — for exactly this file: a rule
like these is worth believing before someone watches a robot for ten minutes in front of a lecture hall,
and a simulation is the slowest possible way to find out whether a sign is right. A scan is therefore drawn
here as a list of 360 numbers, the way `sensors.py` builds one, and the answer is a number.

The three tests at the end are about the split itself: the modules must still import when ROS is not there,
because that is what makes the functions above testable at all, and `main` is what has to complain instead.
"""
from math import atan, cos, degrees, hypot, inf, nan, pi, radians, sin
from pathlib import Path
import re
import sys

import pytest

sys.path.insert(0, ".")
from ohm_frontier import move_to_point, obstacle_avoidance, turn_and_move, view_markers, wall_following   # noqa: E402
from ohm_frontier.obstacle_avoidance import (BLOCKED as FIELD_BLOCKED, CLEAR, NOWHERE, TURNING,   # noqa: E402
                                             avoid, floor_down, menu, planning_reach, runway,
                                             way_round, ways, wrap)
from ohm_frontier.turn_and_move import (arrive_words, drive_straight, drive_to, parse_command,   # noqa: E402
                                        turn_towards)
from ohm_frontier.wall_following import (APPROACHING, BLOCKED, DIAGONAL, FOLLOWING, NO_WALL,   # noqa: E402
                                         SEARCHING, TOO_CLOSE, beam, side_of, state_line, steer)

BEAMS, INCREMENT, RANGE_MAX = 360, 2 * pi / 360, 8.0        # what `sensors.py` actually publishes


def a_scan(walls=(), no_echo="inf"):
    """A laser scan drawn by hand: `walls` is ((degrees, metres), ...), everything else no echo.

    Degrees, because that is how the beam index is thought about: beam 0 is the nose and the index grows
    counterclockwise, so the right hand is at negative angles and the right-wall beam of the wall follower
    is index 270. A wall covers five beams rather than one, which is what a flat wall in front of a robot
    with 1° beams really does.

    `no_echo` picks which dialect of "nothing came back" to write, because both exist in this ecosystem —
    the simulator's own `range_max` unless it was started with `lidar_no_echo:=inf`, and infinity when it
    was — and the rules are supposed to answer them the same way.
    """
    blank = {"inf": inf, "range": RANGE_MAX, "nan": nan}[no_echo]
    ranges = [blank] * BEAMS
    for angle, distance in walls:
        middle = round(radians(angle) / INCREMENT) % BEAMS
        for offset in range(-2, 3):
            ranges[(middle + offset) % BEAMS] = distance
    return ranges


PACKAGE = Path(__file__).resolve().parents[1]      # where setup.py and launch/ are, whoever ran pytest from

# ------------------------------------------------------------------------------- wall following


def test_a_wall_closer_than_the_wanted_gap_is_answered_by_turning_away_from_it():
    """The gap term of `steer`, in the direction a person standing beside the robot expects.

    At 0.35 m rather than the 0.25 m this test used to use: below `min_wall_gap` a different rule answers, and
    that one has its own test below.
    """
    decided = steer(a_scan([(-90, 0.35)]), INCREMENT, RANGE_MAX)
    assert decided.state == FOLLOWING and decided.wall == pytest.approx(0.35)
    assert decided.turn > 0, "a wall at 35 cm where 50 cm was wanted must turn left, away from it"
    assert decided.speed > 0, "the rule follows a wall it has, it does not stop for it"


def test_a_wall_farther_than_the_wanted_gap_is_answered_by_turning_towards_it():
    decided = steer(a_scan([(-90, 0.90)]), INCREMENT, RANGE_MAX)
    assert decided.state == FOLLOWING
    assert decided.turn < 0, "90 cm of gap where 50 cm was wanted must turn right, back towards the wall"


def test_nothing_on_the_right_stops_the_robot_and_turns_until_a_wall_comes_back():
    """The state the lecture waits for. Driving forward without a reference is how this demo crashes."""
    decided = steer(a_scan(), INCREMENT, RANGE_MAX)
    assert decided.state == SEARCHING and decided.wall is None
    assert decided.speed == 0.0 and decided.turn < 0, "spin on the spot to the right, do not drive"


def test_the_right_hand_is_the_half_of_the_scan_with_the_negative_angles():
    """A wall on the left must not be mistaken for a wall on the right — the classic off-by-half-circle.

    0.40 m on the right, not the 0.30 m this used to use: the floor under the gap now answers anything nearer,
    and this test is about which half of the scan is the right hand.
    """
    assert steer(a_scan([(90, 0.30)]), INCREMENT, RANGE_MAX).state == SEARCHING
    assert steer(a_scan([(-90, 0.40)]), INCREMENT, RANGE_MAX).state == FOLLOWING


def test_a_missing_echo_is_not_a_wall_at_the_far_end_of_the_lidar():
    """All three dialects of "nothing reflected" mean the same thing; "no echo" as 8.0 m is the trap."""
    for dialect in ("inf", "range", "nan"):
        decided = steer(a_scan(no_echo=dialect), INCREMENT, RANGE_MAX)
        assert decided.state == SEARCHING, f"{dialect} arrived as a wall"


def test_a_wall_is_read_with_the_beams_beside_it_and_an_empty_direction_still_answers_nothing():
    """`beam` with a window: one echo in the middle of the window counts, an empty window answers None."""
    ranges = a_scan()
    ranges[round(radians(-90) / INCREMENT)] = 0.40        # one single echoed beam, nothing beside it
    assert beam(ranges, INCREMENT, -pi / 2, window=2) == pytest.approx(0.40)
    assert beam(ranges, INCREMENT, pi / 2, window=2) is None


def test_a_wall_straight_ahead_brakes_without_losing_the_wall_on_the_right():
    decided = steer(a_scan([(0, 0.30), (-90, 0.40)]), INCREMENT, RANGE_MAX)
    assert decided.state == BLOCKED and decided.speed == 0.0
    assert decided.wall == pytest.approx(0.40), "the reference on the right is still what it is following"


def test_a_wall_beyond_the_range_is_a_direction_to_drive_to_and_not_a_gap_to_keep():
    """The measured bug: **17.15 m of path and 0.45 m of net** in 45 s, `wz` at its clamp the whole time, and
    the state line printing `following: wall 3.93 m`. A wall 3.93 m off is not a reference at 0.50 m; it is
    somewhere off the right shoulder, and the answer is to drive towards it at walking pace.
    """
    decided = steer(a_scan([(-90, 3.93)]), INCREMENT, RANGE_MAX)
    assert decided.state == APPROACHING, "a wall outside max_wall_range is being followed, which is the bug"
    assert decided.wall == pytest.approx(3.93)
    assert 0 < decided.speed < 0.35, "closing on a wall whose end the lidar cannot see round is find_speed"
    assert decided.turn < 0, "and it leans towards the wall while it closes"


def test_the_gap_error_becomes_an_angle_which_a_far_wall_cannot_saturate():
    """Metres times a gain clamps and stays clamped; metres leaned over a lookahead settle.

    Three numbers, three reasons. Half a metre of error asks for atan(0.5/1.5) = 18°. A wall 6 m out and a
    wall 7 m out ask for the same 20°, because past `max_lean` there is nothing left to ask for — and 20° is
    the largest lean that keeps both diagonal beams on the wall, which is the measurement the loop closes on.
    The turn that results, 1.6 × 0.35 = 0.56 rad/s, is inside `turn_limit`: an error this large is the case
    that used to hold `wz` at its clamp for 45 s, so the bound has to bite before the wheels do.
    """
    half_a_metre_out = steer(a_scan([(-90, 1.00)]), INCREMENT, RANGE_MAX)
    far, further = steer(a_scan([(-90, 7.00)]), INCREMENT, RANGE_MAX), \
        steer(a_scan([(-90, 6.50)]), INCREMENT, RANGE_MAX)
    assert degrees(half_a_metre_out.aim) == pytest.approx(-degrees(atan(0.5 / 1.5)), abs=0.6)
    assert degrees(far.aim) == pytest.approx(-degrees(0.35), abs=0.6), "the lean is bounded by max_lean"
    assert far.aim == pytest.approx(further.aim), "two errors past the bound ask for one heading"
    assert abs(far.turn) < 1.1, "the largest ask is inside the wheel limit, so the robot still translates"


def test_the_lean_is_bounded_where_the_measurement_that_closes_the_loop_survives():
    """`max_lean` is not a comfort limit. At 45° of nose-swing the right-behind beam lies along the wall's own
    surface, the two-diagonal angle goes blind, `side_of` falls back to parallel and `aim` degenerates to a
    constant turn — which is how a robot ends up in a 0.23 m circle printing `leaning -45°` for a minute.
    At 20° the beam is 25° off the surface, the angle is still measured, the loop is still closed.
    """
    seen_at_the_full_lean = a_scan([(-90, 2.00), (-45, 1.60), (-135, 2.45)])
    assert side_of(seen_at_the_full_lean, INCREMENT, RANGE_MAX).angle != 0.0, \
        "a wall seen at the full lean still reports its angle, which is the whole feedback path"
    rooms_spawn = steer(a_scan([(-90, 5.23)]), INCREMENT, RANGE_MAX)
    assert degrees(rooms_spawn.aim) > -21, "the 5.23 m wall at the rooms spawn is what the bound was fitted to"
    assert rooms_spawn.speed > 0, "and it is still driven towards, at the lean rather than at a pivot"


def test_the_gap_band_has_two_edges_because_one_edge_is_a_relay():
    """`at_gap` comes back in from the previous cycle, the way `looking` does.

    With one edge the 75 s run printed `converging` and `following` 19 times apiece, the commanded speed
    swinging between 0.25 and 0.35 with them: a boundary the robot sits on is a switch, and a switch at 20 Hz
    is a limit cycle that nobody can see in a still frame.
    """
    scan = a_scan([(-90, 0.63)])                      # 0.13 m of error: outside the band, inside the hysteresis
    arrived = steer(scan, INCREMENT, RANGE_MAX, at_gap=False)
    holding = steer(scan, INCREMENT, RANGE_MAX, at_gap=True)
    assert not arrived.at_gap and arrived.speed == pytest.approx(0.25)
    assert holding.at_gap and holding.speed == pytest.approx(0.35), "the same gap, and the last cycle decides"


def test_a_wall_nearer_than_the_robot_is_wide_stops_the_wheels_and_turns_off_it():
    """0.30 m is not comfort margin: the robot is 0.46 m wide, so its flank sits 0.23 m from the centre, and
    the run without this floor recorded 26 readings inside 0.30 m, the nearest 0.24 m. The error term is
    overridden here — the one state where it does not get to vote, because driving along a wall you are
    touching is not following it.
    """
    decided = steer(a_scan([(-90, 0.24)]), INCREMENT, RANGE_MAX)
    assert decided.state == TOO_CLOSE and decided.speed == 0.0
    assert decided.turn == pytest.approx(0.5), "a fixed rate off the wall, not a term that can cancel itself"
    assert decided.close_guard and "scrape" in state_line(decided), state_line(decided)


def test_the_floor_keeps_holding_while_the_wall_is_still_too_near_to_leave():
    """The same two-edge band on the way out of the guard, and the same reason: at one edge the robot sits on
    the boundary and the log alternates. 0.33 m is above the 0.30 m floor and below the 0.35 m release.
    """
    scan = a_scan([(-90, 0.33)])
    assert steer(scan, INCREMENT, RANGE_MAX, close_guard=True).state == TOO_CLOSE
    assert steer(scan, INCREMENT, RANGE_MAX, close_guard=False).state == FOLLOWING


def test_the_guard_turns_off_the_wall_even_when_the_wall_is_the_reason_it_is_there():
    """The corner that the first version of the guard died in: a wall falling away ahead, which the
    follow-the-wall term reads as −0.35 rad — the very steer that walked the robot into it. Add the full lean
    to that and the two cancel, which is how a robot ends a 75 s run sitting 0.25 m from a corner with both
    wheels still. Here the angle is ignored and the nose comes off at the fixed rate.
    """
    into_the_corner = a_scan([(-90, 0.25), (-45, 0.55), (-135, 0.20)])
    assert side_of(into_the_corner, INCREMENT, RANGE_MAX).angle < -0.3, "the geometry that cancelled the turn"
    decided = steer(into_the_corner, INCREMENT, RANGE_MAX)
    assert decided.state == TOO_CLOSE and decided.turn == pytest.approx(0.5)
    assert degrees(decided.aim) > 0, "the overlay still points off the wall, because that is what it is doing"


def test_being_at_the_gap_and_still_converging_onto_it_are_two_answers():
    """`follow_tolerance` is what separates the phase on the screen and the pace of the wheels: a robot still
    25 cm off its line has no business driving along it at full speed.
    """
    at_it = steer(a_scan([(-90, 0.55)]), INCREMENT, RANGE_MAX)
    still_off = steer(a_scan([(-90, 0.75)]), INCREMENT, RANGE_MAX)
    assert at_it.at_gap and at_it.state == FOLLOWING and at_it.speed == pytest.approx(0.35)
    assert not still_off.at_gap and still_off.speed == pytest.approx(0.25)
    assert state_line(still_off, 0.50).startswith("converging"), state_line(still_off, 0.50)


def test_the_gap_is_measured_from_the_point_the_wheels_turn_about():
    """The same wall in the same place is a different error to a sensor that is not at the centre. The
    simulator mounts the lidar at [0, 0, 0], which is why `laser_offset_y` is 0.0 and why nothing in this
    repo noticed the difference until a robot with a nose-mounted lidar arrived.
    """
    scan = a_scan([(-90, 0.50)])
    centred = steer(scan, INCREMENT, RANGE_MAX)
    assert centred.wall == pytest.approx(0.50) and centred.turn == pytest.approx(0.0, abs=0.01)
    sensor_7_cm_left = steer(scan, INCREMENT, RANGE_MAX, laser_offset_y=0.07)
    assert sensor_7_cm_left.wall == pytest.approx(0.43)
    assert sensor_7_cm_left.turn > 0, (
        "the same wall in the same place is now too close, and the robot steers away from it — 7 cm of mount "
        "error is most of `follow_tolerance`, so it is not nothing")


def test_the_two_diagonal_beams_are_the_walls_own_angle():
    """A wall that falls away to the right ahead is a corridor bending right, not a nearer wall: the robot
    turns with it, harder than the gap alone would ask.
    """
    bending = a_scan([(-90, 0.60), (-45, 1.20), (-135, 0.60)])
    parallel = a_scan([(-90, 0.60), (-45, 0.60), (-135, 0.60)])
    assert side_of(bending, INCREMENT, RANGE_MAX).angle < 0, "falling away ahead is a right turn, which is negative"
    assert steer(bending, INCREMENT, RANGE_MAX).turn < steer(parallel, INCREMENT, RANGE_MAX).turn, \
        "the same gap in the same place, and the bend is what the nose is for"


def test_the_side_beam_can_be_blind_and_the_two_diagonals_still_answer_for_the_gap():
    """Two points 45° off the shoulder are a line: each is 0.707 of its slant range to the side, so the gap
    at the flank is the mean of them. A post between the robot and the wall costs the reference nothing.
    """
    side = side_of(a_scan([(-45, 0.80), (-135, 0.80)]), INCREMENT, RANGE_MAX)
    assert side.distance == pytest.approx(0.80 * DIAGONAL)
    assert steer(a_scan([(-45, 0.80), (-135, 0.80)]), INCREMENT, RANGE_MAX).state == FOLLOWING


def test_one_diagonal_blind_leaves_a_gap_and_no_opinion_about_the_walls_bearing():
    """One point is not a line. The reference is kept — a wall is a wall — but the angle term goes to zero
    rather than guessing, which is the difference between following a wall and inventing one.
    """
    side = side_of(a_scan([(-90, 0.60), (-45, 0.90)]), INCREMENT, RANGE_MAX)
    assert side.angle == 0.0 and side.distance == pytest.approx(0.60)
    assert steer(a_scan([(-90, 0.60), (-45, 0.90)]), INCREMENT, RANGE_MAX).state == FOLLOWING


def test_the_search_is_counted_across_cycles_and_then_said_out_loud():
    """`looking` is how cycle 401 knows how long it has been blind: the node hands it back what it measured.
    Turning for ever beside an open hall is the old behaviour; after `search_timeout` it stops and says what
    it cannot do, because this rule has no map to go and look somewhere else with.
    """
    blind = a_scan()
    first = steer(blind, INCREMENT, RANGE_MAX, looking=0.0, dt=0.05)
    assert first.state == SEARCHING and first.looking == pytest.approx(0.05)
    assert first.speed == 0.0 and first.turn < 0, "spin towards the wall you want, do not drive blind"
    given_up = steer(blind, INCREMENT, RANGE_MAX, looking=19.99, dt=0.05)
    assert given_up.state == NO_WALL and (given_up.speed, given_up.turn) == (0.0, 0.0)
    assert "max_wall_range" in state_line(given_up), "a stop has to come with the next thing to try"


def test_the_lidars_own_maximum_is_no_wall_once_a_range_is_named():
    """`beam` on its own reports what came back, which for the simulator's own dialect of 'nothing' is its
    maximum range — 8.0 m of wall, which no rule is allowed to read. Naming `range_max` is what turns that
    reading back into an absence; the rule-level version of this is the dialect test above.
    """
    ranges = a_scan(no_echo="range")
    assert beam(ranges, INCREMENT, -pi / 2, 2) == pytest.approx(RANGE_MAX)
    assert beam(ranges, INCREMENT, -pi / 2, 2, RANGE_MAX) is None


# ------------------------------------------------------------------------------- the vector field


def test_the_rule_points_away_from_a_wall_and_towards_the_open_side():
    """Both halves in one test, because a rule that only ever turns one way is a sign error — and this rule
    has a tie to break, which is where a left/right bias would show up.

    A wall dead ahead at 0.9 m is a dead end rather than an obstacle: its 0.9 m of floor is what keeps the
    nose off the menu, and a second wall on the right takes that side out too, so what is left to drive is
    the opening on the left — 30° of it, which is where the ranking lands once the headings are ranked by how
    wide their opening is rather than by how little turning they cost.
    """
    walled = [(0, 0.90), (-40, 0.80)]
    to_the_left = avoid(a_scan(walled), INCREMENT, RANGE_MAX)
    assert to_the_left.state == TURNING and 0.3 < to_the_left.direction < 0.7, to_the_left.why
    assert to_the_left.turn > 0, "the open side is to the left, so the turn goes left"
    assert to_the_left.why, "and it says which gap it picked"

    mirrored = avoid(a_scan([(0, 0.90), (40, 0.80)]), INCREMENT, RANGE_MAX)
    assert mirrored.direction == pytest.approx(-to_the_left.direction), \
        "the same wall mirrored must give the same answer mirrored"
    assert mirrored.turn < 0


def test_an_echo_beyond_the_planning_reach_is_a_direction_and_not_a_brake():
    """The other half of the far map, which is what the rewrite bought.

    `planning_reach` is 1.26 m for the default clearance and brake lead, so a wall 6 m ahead cannot shorten
    any heading's runway — it is not a thing to brake for. It is not nothing either: it is the floor down
    that ray, and 6 m of floor is what puts the heading on the menu at all. A wall 3 m out used to be a vote
    in the field; here it is a sentence about where the hall is.
    """
    assert planning_reach(0.40, 0.80) == pytest.approx(hypot(1.2, 0.4))
    decided = avoid(a_scan([(0, 6.0)]), INCREMENT, RANGE_MAX)
    assert decided.state == CLEAR and decided.direction == pytest.approx(0.0)
    assert decided.forward == pytest.approx(0.35), "6 m of floor ahead is driven at full speed"
    assert floor_down(a_scan([(0, 6.0)]), INCREMENT, 0.0, RANGE_MAX) == pytest.approx(6.0)
    assert floor_down(a_scan(), INCREMENT, 0.0, RANGE_MAX) == inf, "nothing back is the most open answer"


def test_a_nearer_wall_ahead_means_a_smaller_forward_command():
    """The braking term on its own: `speed` scaled by the run the driven direction has, over `brake_lead`.

    Both halves steer with the same wall on the right, so what is left to change is how far the direction
    actually driven has to travel before an echo enters the clearance ring.
    """
    steer_by = (radians(25), 0.90), (radians(-25), 0.90)
    far = avoid(a_scan([(25, 0.90), (-25, 0.90)]), INCREMENT, RANGE_MAX, strafe=True)
    near = avoid(a_scan([(25, 0.62), (-25, 0.62)]), INCREMENT, RANGE_MAX, strafe=True)
    assert far.direction == pytest.approx(near.direction) == pytest.approx(0.0), \
        "the way chosen must not be what changed"
    assert far.forward > near.forward > 0.0, "the same walls, nearer, must not ask for more speed"


def test_a_corridor_that_closes_stops_the_wheels_and_turns_to_the_widest_gap():
    """The state the todo's third principle is about.

    This test used to assert `(forward, sideways, turn) == (0, 0, 0)` — a standing stop with no turn, on the
    reasoning that a stopped field which still turns is a robot pivoting into the wall. That is what the
    measured spin was: a robot with nothing in front of it and no turn is a robot that stays there, which is
    how the old rule ended 45 s of running 3 cm from where it started. The wheels stay still — that part of
    the old assertion was right, and `forward == 0.0` keeps it — but the nose now goes to the widest gap and
    the node says which one it picked.
    """
    decided = avoid(a_scan([(0, 0.40)]), INCREMENT, RANGE_MAX)
    assert decided.state == FIELD_BLOCKED and decided.forward == 0.0
    assert decided.turn != 0.0, "a stopped robot with no turn is a robot that stays stopped"
    assert "wheels stay still" in decided.why, decided.why


def test_the_refusal_to_stand_there_for_ever_says_so_and_stops_trying():
    """`stuck_timeout` on a robot shut in on every side: after 20 s of turning with nothing offering
    `free_travel`, the answer is `nowhere to go` with the seconds attached, because "back out and try the
    other side of the hall" is a program with a map in it, and this is not that program.
    """
    shut_in = a_scan([(0, 0.40), (90, 0.40), (180, 0.40), (-90, 0.40)])
    still_turning = avoid(shut_in, INCREMENT, RANGE_MAX, spent=19.0)
    assert still_turning.state == FIELD_BLOCKED and still_turning.turn != 0.0
    given_up = avoid(shut_in, INCREMENT, RANGE_MAX, spent=20.0)
    assert given_up.state == NOWHERE and (given_up.forward, given_up.turn) == (0.0, 0.0)
    assert "20 s" in given_up.why, given_up.why


def test_the_candidate_grid_is_as_coarse_as_the_wheels_and_no_coarser():
    """5° steps, because one 0.05 s cycle at the turn limit moves the nose 3.2°: a heading between the
    samples is a heading the robot cannot reach before it is asked again. This is the assertion that caught
    the rewrite sampling every 25° while its own docstring said 5° — 14 headings around a robot instead of
    72, which makes a 30° gap read as a wall.

    It is also the reason the old test about "straight ahead" is gone: there is no nose cone any more. A wall
    60 cm to the flank is a fact about the gap, and the heading down the corridor keeps its runway whatever
    the flank says.
    """
    corridor = a_scan([(-90, 0.60), (90, 0.60)])
    every = ways(corridor, INCREMENT, RANGE_MAX, 0.40, 0.80)
    assert len(every) == 72, "one candidate per 5 beams of a 360 beam scan"
    assert wrap(every[1].angle - every[0].angle) == pytest.approx(radians(5))
    ahead = [w for w in every if abs(w.angle) < radians(3)]
    assert ahead and all(w.runway > 1.0 for w in ahead), "the flank walls do not brake the corridor"
    assert avoid(corridor, INCREMENT, RANGE_MAX).state == CLEAR


def test_which_side_an_obstacle_is_gone_round_is_committed_and_not_re_chosen_every_cycle():
    """The bug whose measurement opened this whole round: 13.97 m of path for 0.03 m of net, `wz` pinned at
    ±1.1 flipping sign every 50 ms, because a fresh scan answers "which way round?" two different ways on two
    neighbouring cycles. `held` is the answer coming back in, and it survives while the committed side stays
    open — the two numbers it carries are the whole of what this rule remembers.
    """
    wall_dead_ahead = a_scan([(0, 1.00), (-50, 2.50), (50, 2.50)])
    committed_right = avoid(wall_dead_ahead, INCREMENT, RANGE_MAX, held=radians(-35)).direction
    committed_left = avoid(wall_dead_ahead, INCREMENT, RANGE_MAX, held=radians(35)).direction
    assert committed_right < 0.0 < committed_left, (
        f"held right chose {degrees(committed_right):+.0f}°, held left chose {degrees(committed_left):+.0f}°: "
        "the side committed to on an earlier cycle is what has to decide, once there is one")


def test_the_default_drives_where_the_nose_points_and_leaves_strafe_alone():
    """Principle two, asserted on the wheel message rather than in prose: with the switch off, `vy` is 0.0 in
    every state including the one that has stopped, and the chosen heading is reached by turning to it. With
    the switch on the same scan produces a sideways command, which is the comparison the demo is for.
    """
    blocked_off_ahead = a_scan([(0, 0.90)])
    car = avoid(blocked_off_ahead, INCREMENT, RANGE_MAX)
    assert car.direction != 0.0, "the nose is pointed at a dead end, so a side is chosen"
    assert car.sideways == 0.0, "the default never asks for a sideways wheel, in any state"
    assert car.forward >= 0.0
    mecanum = avoid(blocked_off_ahead, INCREMENT, RANGE_MAX, strafe=True)
    assert mecanum.sideways != 0.0, "and it is the switch that strafes, not the scan"


def test_the_turn_to_a_wanted_heading_goes_the_short_way_round():
    """170° to -170° is 20°, not 340° — the most common first bug in a heading controller.

    Both files carry their own `wrap` on purpose, so both are checked rather than one being assumed.
    """
    assert wrap(radians(340)) == pytest.approx(-radians(20))
    assert obstacle_avoidance.wrap(pi * 3) == turn_and_move.wrap(pi * 3)
    off_a_small_angle = turn_towards(radians(170), radians(-170), gain=1.8, limit=1.2)
    assert off_a_small_angle.turn > 0, "the short way from 170° to -170° is through 180°"
    assert off_a_small_angle.turn == pytest.approx(1.8 * radians(20))


# ------------------------------------------------------------------------------- the three primitives


def test_turn_ninety_means_ninety_degrees_and_not_ninety_radians():
    assert parse_command("turn 90") == ("turn", pytest.approx(pi / 2))
    assert parse_command("rotate 45")[1] == pytest.approx(pi / 4)
    assert parse_command("drive 1.5") == ("drive", 1.5)
    assert parse_command("stop") == ("stop", 0.0)


def test_a_command_that_is_not_a_command_comes_back_with_the_reason():
    """The lecturer types this at a whiteboard, so the answer has to be readable in the log line."""
    for typed in ("", "turn", "dance 3"):
        what, why = parse_command(typed)
        assert what is None, f"{typed!r} was taken as a command"
        assert "turn" in why or "stop" in why, f"{typed!r} answered with {why!r}"


def test_a_turn_is_proportional_and_capped_at_what_the_wheels_can_do():
    half_a_radian = turn_towards(0.0, 0.2, gain=1.8, tolerance=0.05)
    assert half_a_radian.running and half_a_radian.turn == pytest.approx(0.36)
    assert half_a_radian.forward == 0.0, "a turn is a turn, not a curve"
    # Which way round at exactly 180 degrees is not asserted and cannot be: `wrap` answers -pi there, and
    # both ways are the same distance. What this test is about is the ceiling, and the ceiling is a magnitude.
    assert abs(turn_towards(0.0, pi, gain=1.8, limit=1.2).turn) == pytest.approx(1.2), \
        "1.8 * pi rad/s is a command the wheels ignore; a limited one is the only one worth sending"


def test_a_turn_is_finished_by_its_tolerance_rather_than_by_arriving():
    """A P controller gets asymptotically closer and never arrives, so the tolerance declares it done."""
    assert not turn_towards(1.0, 1.02, tolerance=0.05).running
    assert turn_towards(1.0, 1.06, tolerance=0.05).running


def test_driving_straight_stops_at_its_distance():
    assert drive_straight(1.0, 1.0) == drive_straight(1.2, 1.0)                     # both done
    assert not drive_straight(0.99, 1.0).running, "inside the tolerance of the distance is arrived"
    assert drive_straight(0.0, 1.0).forward == pytest.approx(0.25)


def test_the_last_centimetres_of_a_drive_are_driven_slowly():
    """`remaining / 0.15` is the deceleration phase; without it the stop overshoots by one cycle."""
    nearly_there = drive_straight(0.0, 0.05, speed=1.0)
    assert nearly_there.running and nearly_there.forward == pytest.approx(0.05 / 0.15)
    assert nearly_there.forward < 1.0


def test_holding_the_line_pushes_sideways_only_when_a_mecanum_base_is_asked_to():
    """ADAPTED, because the default of `drive_straight` changed and this is the line that pinned it.

    What was asserted was that a cross-track error is answered by `vy`. That is still true and still tested —
    with `strafe=True`, which is what the switch is for. What changed is the default: car-like, `vx` and `wz`,
    because the measured 6 m run behind that decision put 1.36 m of its 8.51 m of path in the sideways
    direction. The forward speed is untouched either way, which is the half of the test the name keeps.
    """
    off_the_line = drive_straight(0.0, 1.0, cross_track=0.20, strafe=True)
    assert off_the_line.sideways < 0, "the line is 20 cm to the right, so the correction goes right"
    assert off_the_line.forward == pytest.approx(0.25)
    assert drive_straight(0.0, 1.0, 5.0, strafe=True).sideways == pytest.approx(-0.12), \
        "a correction beyond the wheels' reach is a command the wheels ignore"
    assert drive_straight(0.0, 1.0, cross_track=0.20).sideways == 0.0, \
        "the default is a car: the drift is measured and not corrected, because a car cannot hold a line " \
                                        "with its bumper and leaning the nose would make `drive 3.0` a curve"


def test_driving_to_a_place_walks_diagonally_at_it_only_with_strafe_and_still_arrives_without():
    """ADAPTED, for the same reason as the test above: `strafe` now defaults to off.

    The diagonal — forward, left, and turning left, all at once — is what `strafe=True` does and is still
    asserted. The default answer to the same pose is the car's: forward and a turn, no sideways at all.
    """
    decided = drive_to((0.0, 0.0, 0.0), (1.0, 0.5))
    assert decided.running and decided.forward > 0 and decided.turn > 0
    assert decided.sideways == 0.0, "the default is vx and wz"
    diagonal = drive_to((0.0, 0.0, 0.0), (1.0, 0.5), strafe=True)
    assert diagonal.sideways > 0, "the goal is ahead and to the left: on a mecanum base it goes left as well"
    assert diagonal.forward == pytest.approx(decided.forward) and diagonal.turn == pytest.approx(decided.turn), \
        "the switch adds a wheel, it does not retune the controller it is bolted to"


def test_a_goal_within_arrive_distance_is_arrived_rather_than_approached():
    arrived = drive_to((0.0, 0.0, 0.0), (0.05, 0.0), arrive_distance=0.12)
    assert not arrived.running
    assert (arrived.forward, arrived.sideways, arrived.turn) == (0.0, 0.0, 0.0)


def follow(step, pose, goal, seconds=60.0, dt=0.05):
    """Close one of these controllers on an ideal odometry and return (poses, commands).

    The odometry here is the truth: the pose is the command integrated, midpoint heading, no slip and no
    noise. That is deliberate — the claim under test is that the *rule* reaches a place, which is a fact about
    the arithmetic and not about `robot.slip` or `sensors.odom.sigma_xy`. What the same rule does against a
    wall that zeroes its turn and a pose that keeps counting metres is a different claim, belongs to the hall,
    and is measured by `tools/try_demo.sh`.
    """
    poses, commands, error_sum = [pose], [], 0.0
    for _ in range(int(seconds / dt)):
        forward, sideways, turn, running, error_sum = step(pose, goal, error_sum, dt)
        commands.append((forward, sideways, turn))
        x, y, heading = pose
        mid = heading + 0.5 * turn * dt                       # the same midpoint the simulator integrates
        pose = (x + (forward * cos(mid) - sideways * sin(mid)) * dt,
                y + (forward * sin(mid) + sideways * cos(mid)) * dt, heading + turn * dt)
        poses.append(pose)
        if not running:
            break
    return poses, commands


def drive_to_step(pose, goal, error_sum, dt):
    """`drive_to` at the node's own published defaults, spelled out so a changed default shows up here."""
    decided = drive_to(pose, goal, error_sum, speed=0.3, turn_limit=1.2, arrive_distance=0.12, dt=dt)
    return decided.forward, decided.sideways, decided.turn, decided.running, decided.error_sum


def test_the_car_like_drive_to_a_place_6_m_ahead_arrives_without_stopping_to_aim():
    """The whole acceptance of the default, as arithmetic: 6.0 m, from a spawn facing 90° away.

    The same pair of numbers `tools/try_demo.sh` prints, on an odometry that cannot lie. Before the `strafe`
    default was changed, this run measured 20.4 s of driving and 1.36 m of sideways travel; with the switch
    off it is 21.0 s and 6.02 m of path for 5.88 m of net — 4 % over the straight line, and the extra 0.6 s is
    the 90° turn the car has to make and the mecanum base does not.

    The assertion that keeps the two controllers different is the last one: a forward command in every cycle
    but the 26 that the place is behind it, and exactly one stop in the run — the arrival. A sequential
    controller would stop to aim, which is `move_to_point.py`, tested in its own file.
    """
    spawn, goal = (2.25, 6.25, 0.0), (2.25, 12.25)            # the `open` spawn and a place 6.0 m due north
    poses, commands = follow(drive_to_step, spawn, goal, seconds=60.0)
    path = sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))
    net = hypot(poses[-1][0] - spawn[0], poses[-1][1] - spawn[1])

    assert hypot(poses[-1][0] - goal[0], poses[-1][1] - goal[1]) <= 0.12, "ends inside its own tolerance"
    assert len(poses) * 0.05 < 30.0, f"arrived in {len(poses) * 0.05:.1f} s, not after a lecture"
    assert path / net < 1.5, f"path {path:.2f} m for net {net:.2f} m is driving, not circling"
    assert max(abs(sideways) for _, sideways, _ in commands) == 0.0, "the default never strafes"

    behind = [c for c, p in zip(commands, poses[:-1])
              if c[0] == 0.0 and hypot(goal[0] - p[0], goal[1] - p[1]) > 0.12]
    assert len(behind) <= 30, f"{len(behind)} cycles with no forward command: once the place is ahead, a " \
                                                       "car-like drive_to keeps driving and turns while it does"
    stops = sum(1 for a, b in zip(commands, commands[1:])
                if max(map(abs, a)) > 0.01 and max(map(abs, b)) <= 0.01)
    assert stops == 1, "one stop in the whole run, at the end of it"


def test_the_two_terms_of_the_mecanum_version_saturate_at_metres_nobody_tuned_them_for():
    """Where the wall follower's bug would live here, in the two numbers that show it.

    `gain_side * side` is a metre gain like the one that spun that robot for 45 s: with a place 5 m to the
    side it asks for 3.5 m/s of strafe and `side_limit` answers 0.15 m/s, so the rule stops being
    proportional at 0.15 / 0.7 = 0.21 m of cross-track error and every error worth correcting is inside the
    clamp. That term is why `strafe` exists and why it is off.

    The turn term clamps too, at 1.2 / 1.8 = 0.67 rad = 38° off the bearing, and that one is honest: 1.8 * pi
    is a rate these wheels ignore. The two together are the sentence the file needs — an angle out of the
    default, a rate ceiling on it, and no metres multiplied by anything.
    """
    far_to_the_left = drive_to((0.0, 0.0, 0.0), (0.5, 5.0), strafe=True)
    assert far_to_the_left.sideways == pytest.approx(0.15), "asked 3.5 m/s, got the ceiling"
    just_off_line = drive_to((0.0, 0.0, 0.0), (1.0, 0.20), strafe=True)
    assert just_off_line.sideways == pytest.approx(0.7 * 0.20, abs=1e-9), "20 cm is still proportional"
    assert 0.15 / 0.7 == pytest.approx(0.214, abs=0.001), "and that is where it stops being proportional"

    at_30 = drive_to((0.0, 0.0, 0.0), (cos(radians(30)), sin(radians(30))))
    at_40 = drive_to((0.0, 0.0, 0.0), (cos(radians(40)), sin(radians(40))))
    assert at_30.turn == pytest.approx(1.8 * radians(30), abs=0.02), "inside the ceiling: proportional"
    assert at_40.turn == pytest.approx(1.2), "40° off the bearing is past 38°: the ceiling, and no more"
    assert abs(drive_to((0.0, 0.0, 0.0), (cos(radians(90)), sin(radians(90)))).forward) < 1e-9, \
        "and at 90° there is nothing along the nose to drive at, which is the only stop this controller makes"


def test_the_heading_integral_cannot_grow_while_a_robot_is_held_against_a_wall():
    """ADAPTED: the bound on `error_sum` moved from a clamp to a condition, and this pinned the clamp.

    What was asserted was that the sum stops at `integral_limit` (1.0) after 10 s of a robot held from
    turning. It now stops earlier and for a better reason: while `gain_turn * off` is itself past
    `turn_limit`, the wheels are already turning as fast as they can towards the bearing, so the cycle has
    nothing left to act with and adds nothing. Measured on that held case — a robot 90° off the bearing for
    10 s — the sum used to reach its whole 1.0 rad·s clamp in 1.0 / (1.571 × 0.05) = 13 cycles, 0.65 s; now it
    stays at zero, which is the difference between a wound-up integral that has to be spent afterwards and
    one that never existed.

    The other half is what keeps this from being a test of a constant zero: inside the 0.67 rad = 38° where
    the proportional term is not clamped, the sum still grows, which is the only reason the term is there.
    """
    error_sum = 0.0
    for _ in range(200):                                    # 10 s of a robot held from turning at 20 Hz
        error_sum = drive_to((0.0, 0.0, 0.0), (0.0, 1.0), error_sum, dt=0.05).error_sum
    assert error_sum == 0.0, "held with the turn clamped, the sum must not move at all"

    error_sum = 0.0
    for _ in range(20):                                     # 1 s at 17° off: the turn is proportional here
        error_sum = drive_to((0.0, 0.0, 0.0), (1.0, 0.3), error_sum, dt=0.05).error_sum
    assert error_sum == pytest.approx(0.291, abs=0.01), "0.29 rad·s after a second of a 17° error"
    assert abs(error_sum) <= 1.0, "and integral_limit still stands behind it as the second bound"


def test_the_integral_changes_nothing_about_arrival_in_a_hall_that_gives_it_no_work():
    """Measured, because the module docstring sells this term and the number says it is a lecture prop.

    On the 6.0 m closed loop, P-only and the shipped PI arrive at the same instant (20.95 s) and the same
    distance from the place (0.117 m), and adding a 0.05 rad/s heading bias — `sensors.odom.bias_omega`, which
    this simulator sets to 0.0 by default — changes that to 0.116 m for both. The reason is the geometry: a
    heading loop that closes on the *bearing to the place* has no steady-state error to integrate away, which
    is what an integral is for. The trace shows what it does instead: the sum peaks at 0.31 rad·s during the
    turn at the start, spends itself over the next 6 s, and holds the 3 m line to within 0.03 rad.

    So the term is kept for the trade it demonstrates — the module docstring's "add the I term and watch the
    overshoot appear" — and not for a benefit claimed here, and the one thing it measurably cost, 13 cycles of
    windup against a wall that refuses the turn, is what the test above now bounds.
    """
    def closed(gain_integral, bias=0.0, dt=0.05):
        pose, error_sum = (0.0, 0.0, 0.0), 0.0
        for _ in range(int(60.0 / dt)):
            decided = drive_to((pose[0], pose[1], pose[2]), (0.0, 6.0), error_sum,
                               gain_integral=gain_integral, dt=dt)
            error_sum = decided.error_sum
            turn = decided.turn + bias
            mid = pose[2] + 0.5 * turn * dt
            pose = (pose[0] + decided.forward * cos(mid) * dt, pose[1] + decided.forward * sin(mid) * dt,
                    pose[2] + turn * dt)
            if not decided.running:
                return hypot(pose[0], pose[1] - 6.0)
        return hypot(pose[0], pose[1] - 6.0)

    assert closed(0.0) == pytest.approx(closed(0.25), abs=1e-3), "no bias: the same arrival either way"
    assert closed(0.0, bias=0.05) == pytest.approx(closed(0.25, bias=0.05), abs=1e-3), \
        "a bias the size of a dragging motor: still the same, because the bearing loop is the feedback"
    assert closed(0.25) < 0.12, "and both of them arrive"


def test_the_arrival_line_names_the_place_and_says_whether_the_tolerance_was_met():
    """A stop that prints nothing is a silent freeze with a pose attached — see the three principles.

    `drive_to` finishes on `arrive_distance`, not on the place, so the residue is the number a lecture should
    see, and 0.11 m and 0.30 m look the same as a bare coordinate. The words are the judgement, and they are
    a function so that the judgement can be tested without a graph.
    """
    assert "inside the 0.12 m tolerance" in arrive_words(0.108, 0.12)
    assert "outside its own 0.12 m tolerance" in arrive_words(0.30, 0.12), \
        "the sentence has to be able to admit that the controller missed"


# ------------------------------------------------------------------------------- the split itself


def test_the_maths_of_every_reactive_module_imports_without_ros(monkeypatch):
    """`test_reactive.py` is the reason for the guarded imports, so the guard is checked, not assumed.

    With the ROS modules unavailable, the files must still import — that is what `field` and `steer` being
    plain functions is for — and it has to be `main` that refuses, in words that say what to source.

    `move_to_point.py` is in this list because it wants the same property, and the two message modules added
    below are in it because its overlay imports and `view_markers`' are the newest guarded ones in the
    package: a marker type imported outside a `try` would break the one thing this file is for.
    """
    from importlib import reload

    modules = (wall_following, obstacle_avoidance, turn_and_move, move_to_point)
    for name in ("rclpy", "rclpy.node", "rclpy.action", "geometry_msgs", "geometry_msgs.msg",
                 "sensor_msgs", "sensor_msgs.msg", "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg",
                 "visualization_msgs", "visualization_msgs.msg", "builtin_interfaces",
                 "builtin_interfaces.msg"):
        monkeypatch.setitem(sys.modules, name, None)            # `import x` on a None entry raises

    for module in modules:
        without_ros = reload(module)
        assert without_ros.rclpy is None, f"{module.__name__} imported rclpy anyway"
        with pytest.raises(SystemExit) as refused:
            without_ros.main()
        assert "rclpy" in str(refused.value), f"{module.__name__}.main() said {refused.value!r}"

    assert wall_following.steer(a_scan(), INCREMENT, RANGE_MAX).state == SEARCHING, \
        "the rule itself is supposed to work with no ROS in the process at all"

    monkeypatch.undo()                                          # the real ROS, for whichever test is next
    for module in modules:
        reload(module)


def test_with_ros_present_the_guard_imports_every_message_name_it_uses():
    """The mirror image of the test above, and the one that has earned its keep once already.

    A guarded import that puts a whole block under one `except ImportError:` cannot tell "this machine has no
    ROS" from "this file asked for a message that is not where it said" — and answers both with the same hint
    about sourcing a setup file. `move_to_point.py` did exactly that for a day: it wanted `Odometry` from
    `nav_msgs` and asked `sensor_msgs` for it, and the guard told the author to install ROS on a machine that
    had it. So on a machine that does have ROS, every name behind every guard in the package must be a class.
    """
    names = {wall_following: ("Twist", "LaserScan", "MarkerArray"),
             obstacle_avoidance: ("Twist", "LaserScan", "MarkerArray"),
             turn_and_move: ("Twist", "PoseStamped", "Odometry", "String"),
             move_to_point: ("Twist", "PoseStamped", "Odometry", "String", "MarkerArray"),
             view_markers: ("Marker", "MarkerArray", "Point", "Duration")}
    for module, given in names.items():
        missing = [name for name in given if getattr(module, name, None) is None]
        assert not missing, f"{module.__name__} has {missing} as None: either this machine has no ROS, or the " \
                            "name is on the wrong module and the guard has hidden that"
    assert hasattr(turn_and_move, "Node") and turn_and_move.Node is not object, \
        "`Node = object` in a guard means the same lie about rclpy"
    assert view_markers.available(), "the guards are fine but the overlay says it is not there anyway"


def test_a_text_marker_is_placed_by_its_pose_and_a_list_marker_by_its_points_because_rviz_reads_two_places():
    """The one asymmetry in `visualization_msgs/msg/Marker` that silently moves the whole overlay.

    Every kind this package draws is a list marker — dots, arrows, lines — and reads `points`. Text is not: a
    `TEXT_VIEW_FACING` marker is drawn at its `pose`, and a label whose `points` are set and whose `pose` is left
    alone is a label at the origin of the frame, which for an odometry-frame overlay is where the robot was
    booted and not where it is. Nothing complains: the topic carries the marker, the display lists the
    namespace, and the screen has no sentence on it. Found by rendering the view and looking.
    """
    from builtin_interfaces.msg import Time

    stamp = Time(sec=1000, nanosec=0)
    label, = view_markers.labels("muster/odom", stamp, "numbers", [((3.0, 6.0), "arrived · 0.04 m left")])
    assert label.type == label.TEXT_VIEW_FACING
    assert label.text.startswith("arrived"), "the words are the point of the marker"
    assert (label.pose.position.x, label.pose.position.y) == (3.0, 6.0), \
        "a label over the robot, not at the origin of the frame it is drawn in"
    assert label.scale.z > 0.1, "`scale.z` is the height of the text in metres; at 0 RViz draws nothing at all"

    line = view_markers.lines("muster/odom", stamp, "goal", [((0.0, 0.0), (3.0, 6.0))])
    assert len(line.points) == 2 and line.pose.position.x == 0.0, \
        "and the geometry kinds are the other way round, which is exactly why one line of this is worth " \
        "a test: the same field name means nothing across marker types"


def test_every_executable_a_launch_file_starts_is_installed_and_every_one_installed_is_launched():
    """`Node(package=..., executable=...)` is a string: a missing console_script is a launch-time error and
    the kind of typo that no import catches, so the two lists are compared here.

    Compared by walking `launch/*.py` rather than a list of file names written here, because the list was the
    weak part: a new launch file simply was not checked, and the one failure mode of this test is exactly a
    file nobody thought to add. Checked both ways for the same reason — an entry point no launch file starts
    is an executable that is installed, documented and never run, which is how a demo rots.
    """
    installed = set(re.findall(r"([\w-]+)\s*=\s*ohm_frontier\.\w+:main", (PACKAGE / "setup.py").read_text()))
    assert {"frontier_node", "wall_following", "obstacle_avoidance", "turn_and_move",
            "move_to_point"} <= installed, installed

    # `package=` and `executable=` are adjacent in every launch file of this package, which is what lets one
    # regex ask "which executables of *this* package" without dragging in the nav2 and slam_toolbox ones.
    starts = re.compile(r'package="ohm_frontier",\s*\n?\s*executable="([\w-]+)"')
    launched = set()
    for launch in sorted((PACKAGE / "launch").glob("*.py")):
        started = set(starts.findall(launch.read_text()))
        launched |= started
        assert started <= installed, f"{launch.name} starts {started - installed}, which setup.py does not build"
    assert launched == installed, f"installed but launched by nothing: {installed - launched}"
