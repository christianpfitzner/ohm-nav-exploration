"""The numbers a node decided, drawn where the robot can see them.

RViz can show a map, a scan and a goal. It cannot show *why* — and the teaching value of this package is in
the why: which frontier won and by how much, which way the vector field pointed, how many degrees are left
before a turn is finished. Those are quantities, and a quantity on a slide is worth one photograph, while a
quantity next to a moving robot is worth the whole hour, because a student can watch it change and predict
the next one.

So: one module that turns numbers into markers, used by every node in this package that has a view. It exists
so the four of them do not each carry twenty lines of `visualization_msgs` field-setting, which is how a
marker ends up drawn in a frame nobody else uses and invisible for an hour of debugging.

Three rules the whole package keeps, written down once here:

* **The frame is the one the map message carried**, never `"map"` spelled literally. The mapper anchors its
  own frame and the simulator calls the hall's coordinates `hall`; a marker in a frame that does not exist is
  not an error, it is simply absent, which is worse.
* **Everything of one kind in one marker, by namespace.** Namespaces are what RViz's checkbox switches, so a
  lecturer hides the scores and keeps the dots: a marker per dot would mean a checkbox per dot. Text is the
  exception the message type forces on this — ROS 2 has `TEXT_VIEW_FACING`, one string per marker, and no list
  of them — so a label is one marker each and `publish` flattens the lists back for the caller.
* **A fresh stamp on every cycle, and an empty marker to remove one.** RViz keeps what it was told, so a
  stale orange dot in the middle of a hall reads as "that is where it is going".

`visualization_msgs` is optional, as everywhere else here: the rules in this package are testable without a
graph, so `available()` answers for the import and a node says once what it is not drawing.
"""
from math import cos, pi, sin

try:                                        # so the maths of this package imports in a plain python
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    Duration = Point = Marker = MarkerArray = None

#: heights above the floor, in metres. Different kinds of overlay at different heights, so they do not
#: overwrite one another when the view is looked at from above — which is how a lecture sees it.
DOT_HEIGHT, LINE_HEIGHT, LABEL_HEIGHT = 0.10, 0.06, 0.50

#: The palette, and what each colour means across the package: blue is a result (a chosen goal, a velocity
#: that goes to the wheels), orange is a decision or a command, green is what the robot wants, red is what it
#: is avoiding, yellow is what it merely reads, cyan is raw sensor data, white is context. Six names, because
#: more than six colours on a projector is a lecture about the palette instead of about the robot.
BLUE, ORANGE, GREEN, RED, WHITE = (0.1, 0.3, 0.9, 0.9), (0.95, 0.5, 0.0, 1.0), \
    (0.1, 0.8, 0.3, 0.9), (0.9, 0.15, 0.15, 0.9), (1.0, 1.0, 1.0, 0.9)
CYAN, YELLOW = (0.1, 0.75, 0.85, 0.75), (0.9, 0.85, 0.2, 0.8)


def available() -> bool:
    """Whether anything in here can be built at all, for the node that has to say so out loud."""
    return Marker is not None


def dots(frame, stamp, namespace: str, places, scale: float = 0.18, colour=BLUE, action=None) -> "Marker":
    """One sphere per place in `places`, which are `(x, y)` or `(x, y, z)` pairs — how this package passes a
    position about everywhere, from a frontier to an odometry pose. `action=DELETEALL` empties the namespace."""
    marker = _marker(Marker.SPHERE_LIST, frame, stamp, namespace, colour)
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.action = Marker.ADD if action is None else action
    marker.points = [_at(place, DOT_HEIGHT) for place in places]
    return marker


def arrows(frame, stamp, namespace: str, pairs, shaft: float = 0.035, colour=ORANGE) -> "Marker":
    """One arrow per `(from, to)` pair. The arrow *is* the vector: metres per second read as a command,
    metres read as a direction to drive. Its length is the value, which is the point of drawing it."""
    marker = _marker(Marker.ARROW, frame, stamp, namespace, colour)
    marker.scale.x, marker.scale.y, marker.scale.z = shaft, shaft * 2.4, 0.0
    for start, end in pairs:
        marker.points += [_at(start, LINE_HEIGHT), _at(end, LINE_HEIGHT)]
    return marker


def lines(frame, stamp, namespace: str, pairs, width: float = 0.02, colour=GREEN) -> "Marker":
    """Unarrowed segments, for geometry rather than decision: the line a controller is trying to stay on, the
    beam a rule happened to read, the way a field points."""
    marker = _marker(Marker.LINE_LIST, frame, stamp, namespace, colour)
    marker.scale.x, marker.action = width, Marker.ADD
    for start, end in pairs:
        marker.points += [_at(start, LINE_HEIGHT), _at(end, LINE_HEIGHT)]
    return marker


def labels(frame, stamp, namespace: str, notes, height: float = 0.30, colour=WHITE) -> list:
    """Text in the scene, as one marker per line: `notes` is `(where, text)` per label.

    One marker each because ROS 2 has no text-list marker — `TEXT_VIEW_FACING` carries a single string, so a
    list of them is a list of markers, numbered 0 upwards within the namespace, which is how RViz tells two
    labels apart. `publish` takes the list back apart, so a caller adds it to the others and thinks about
    nothing.

    Written out with units rather than as `1.82`, because a number a student has to look up in a table is a
    number they will not read while a robot is moving. `cells 34 · 1.9 m · score 1.82` reads in a second and
    says which of the three weights is deciding this one.

    `height` is the height of the text in metres, and 0.30 is not a taste: the view in `config/explore.rviz`
    orbits at 20 m because the hall it is opened on is 30 × 20 m, and text at the 0.11 m this defaulted to is a
    grey smear at that distance — measured in a screenshot of a real run, where the geometry was legible and the
    arithmetic written over it was not. It is the height of the robot, which is also the size at which a lecture
    hall reads it.

    Each carries a one-second lifetime, which is how a text overlay disappears when a node stops having things
    to say: the labels are the one kind of marker that would otherwise be left hanging over a hall that has
    moved on, and unlike a dot a stale sentence is not recognisable as stale.
    """
    if Marker is None:
        return []
    out = []
    for index, (where, text) in enumerate(notes):
        marker = _marker(Marker.TEXT_VIEW_FACING, frame, stamp, namespace, colour)
        marker.id, marker.text = index, text
        marker.scale.z = height
        marker.lifetime = Duration(sec=1) if Duration is not None else marker.lifetime
        marker.points = [_at(where, LABEL_HEIGHT)]
        out.append(marker)
    return out


def ring(frame, stamp, namespace: str, centre, radius: float, colour=GREEN, spokes: int = 48) -> "Marker":
    """A circle of `radius` about `centre`: the distance a reactive rule is trying to hold, drawn.

    `spokes` segments look round at the distance a lecture hall sees it and are cheap enough to rebuild from
    scratch every scan, which matters: the alternative is a marker that lags the robot until somebody notices
    it is stale. The last segment joins back to the first, so it reads as a circle and not as a C.
    """
    points = [(centre[0] + radius * cos(2 * pi * i / spokes),
               centre[1] + radius * sin(2 * pi * i / spokes)) for i in range(spokes)]
    return lines(frame, stamp, namespace, list(zip(points, points[1:] + points[:1])), 0.015, colour)


def publish(view, markers) -> None:
    """One `MarkerArray` per cycle — everything this node decided, in one message, namespaces switchable.

    Markers that could not be built are dropped rather than sent as holes in the picture, and a node without
    `visualization_msgs` has no publisher to speak of, which `view is None` covers.

    A marker stamped at the epoch is dropped too, and that is not tidiness. With `use_sim_time` a node's clock
    reads zero until the first `/clock` message arrives, so the first cycles of every run are stamped at 0 while
    the transform cache already holds tens of seconds of simulation. RViz answers with `Message Filter dropping
    message: frame '<robot>/odom' at time 0.040 for reason 'the timestamp on the message is earlier than all the
    data in the transform cache'` — seen on the first run of this code, and it sends a student to the tf tree for
    a problem that fixes itself one cycle later. Dropping it costs nothing, because the next timer tick draws the
    same picture with a real time on it.
    """
    if view is None or MarkerArray is None:
        return
    sent = [marker for kind in markers for marker in (kind if isinstance(kind, list) else [kind])
            if marker is not None and not _at_the_epoch(marker)]
    if sent:
        view.publish(MarkerArray(markers=sent))


def _at_the_epoch(marker) -> bool:
    """Is this stamped at second zero, which is a clock that has not been told what time it is.

    A real timestamp on this machine is a billion-something in seconds whether it is the wall clock or the
    simulator's, so a zero here means exactly one thing: `use_sim_time` is on and `/clock` has not arrived yet.
    """
    stamp = marker.header.stamp
    return stamp.sec == 0 and stamp.nanosec == 0


def _marker(kind: int, frame, stamp, namespace: str, colour: tuple) -> "Marker":
    """The fields every marker needs and nobody ever reads: type, frame, stamp, id, orientation."""
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = frame, stamp
    marker.type, marker.ns, marker.id = kind, namespace, 0
    marker.pose.orientation.w = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
    return marker


def _at(place, height: float):
    """A `Point` at `place`, raised to `height` unless the place carries its own height."""
    return Point(x=float(place[0]), y=float(place[1]),
                 z=float(place[2]) if len(place) > 2 else height)
