"""The one angle this package has to get right, in one place.

Each reactive example in this package used to carry its own copy of `wrap`, on the reasoning that a student
should be able to read one file alone. That held at two files. A third copy of the line where a heading
controller is most often wrong is not readability, it is three places for one bug to live, so it is here now —
and the import line at the top of each example still tells a reader of that one file where the answer comes
from, which is the part of the old reasoning worth keeping.
"""
from math import pi


def wrap(angle: float) -> float:
    """An angle in radians into -π … +π, so a turn goes the short way round.

    Without it, "from 170° to -170°" is a 340° turn instead of a 20° one — the most common first bug in a
    heading controller, and the reason one line is worth reading twice. `atan2` already answers inside this
    range, so the wrap matters wherever two headings are *subtracted*: `obstacle_avoidance.field` between the
    field's direction and the wanted one, `turn_and_move.turn_towards` between where the robot is and where it
    means to be, `move_to_point.drive_to_in_order` between the robot and the place it was sent to.

    The end points are not symmetric, and no controller here cares: `wrap(π)` comes back as -π, because the
    modulo puts the answer in [-π, +π). Due east of a robot that is facing due west is a coin toss, and a test
    that asserts which way the coin lands would be testing this line rather than the robot.
    """
    return (angle + pi) % (2 * pi) - pi
