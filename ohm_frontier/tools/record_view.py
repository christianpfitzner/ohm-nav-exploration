#!/usr/bin/env python3
"""Record a running stack and turn it into the picture on the front page.

    ros2 launch ohm_frontier explore.launch.py headless:=true rviz:=false      # one terminal
    python3 ohm_frontier/tools/record_view.py muster 120 /tmp/explore.gif      # another

RViz cannot be screenshotted on the machine this package is developed on — there is no display, and `rviz2`
aborts with exit -6 when asked for one — so the animation of an exploration run is made the way the
explorer's own view is made: from `/map`, `/<robot>/odom`, `/tf` and the `/frontiers` marker array. Nothing here
knows what a frontier is. It draws what the node published, in the frame the node published it in, which is why
the dots land where the ranking put them and not where this file guessed.

Three things it does that a photograph of a window cannot: it runs headless, it keeps a whole run in one file (a
two-minute exploration at one frame a second is a few hundred kilobytes), and it draws the map as the planner
sees it — unknown dark, free light, occupied black — which is the half of frontier exploration a screen full of
laser dots hides.

The drawing is separate from the ROS side so it can be tested with a hall made up on the spot: `map_image`,
`frame` and `write_gif` are pure functions over tuples and `PIL.Image`s, and `test/test_record_view.py` proves
the picture still comes out when no robot is alive.
"""
import argparse
import os
import sys
from math import atan2, cos, sin

try:                                        # the recording needs ROS; the drawing above does not, and is
    import rclpy                            # tested without it — the same arrangement the nodes use for their
    from nav_msgs.msg import Odometry, OccupancyGrid                # own rules
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:                         # so the picture code is importable (and testable) anywhere
    rclpy = Odometry = OccupancyGrid = Marker = MarkerArray = None
    QoSProfile = DurabilityPolicy = HistoryPolicy = None
    Buffer = TransformListener = object
    Node = object

from PIL import Image, ImageDraw

#: what the three states of an occupancy grid look like: dark unknown, light free, black walls. The ordering
#: RViz's map display uses, so a recording looks like the run looked.
UNKNOWN, FREE, WALL = (28, 30, 40), (226, 228, 232), (16, 18, 22)
#: the track and the robot. The track is *not* orange: the node draws the goal it chose as an orange dot
#: (`GOAL_COLOUR = (1.0, 0.55, 0.1)`), and the first version of this file drew the driven track in almost the
#: same orange — so the picture had two of the three things a reader is trying to tell apart in one colour.
PATH, ROBOT = (238, 70, 150), (60, 200, 120)


def map_image(info, data, pixels_per_metre=8.0):
    """The hall as one RGB image, from `(width, height, resolution)` and a flat list of cell values.

    Row 0 of a `nav_msgs/OccupancyGrid` is the *bottom* row of the map, and PIL starts at the top, so the rows
    go in upside down. Each cell becomes exactly one pixel and the whole thing is then blown up with
    `NEAREST`: the obvious version — a filled rectangle per cell — overlaps its neighbour by a pixel on every
    side, because PIL's `rectangle` includes both corners, and a map drawn that way has a half-cell grey bleed
    along every wall. A grid shorter than its own header (a mapper mid-update) loses its tail rather than the
    recording.
    """
    width, height, resolution = info
    cells = max(1, int(round(resolution * pixels_per_metre)))   # one pixel per cell, then blown up
    img = Image.new("RGB", (width, height))
    img.putdata([FREE if v == 0 else WALL if v > 50 else UNKNOWN for row in range(height - 1, -1, -1)
                 for v in data[row * width:(row + 1) * width]])
    if cells == 1:
        return img
    return img.resize((width * cells, height * cells), Image.NEAREST)


def frame(base, path, dots, pose, note="", pixels_per_metre=8.0):
    """One frame: the hall, the track driven so far, the frontiers as the node drew them, the robot on top.

    `path` is `(x, y)` per sample in the map's frame, `dots` is `(x, y, radius_metres, rgb)` per marker point,
    and `pose` is `(x, y, theta)` — the triangle points where the robot is looking, which is the one thing in the
    picture that says whether it is driving or only sitting in a corner.
    """
    img = base.copy()
    draw = ImageDraw.Draw(img)

    def here(p):                                # y is up in ROS and down in a bitmap
        return (p[0] * pixels_per_metre, img.height - p[1] * pixels_per_metre)

    if len(path) > 1:
        draw.line([here(p) for p in path], fill=PATH, width=3)
    for x, y, r, colour in dots:
        # The marker's own size, and never less than a few pixels: a frontier is `dots(scale=0.18)` — 0.18 m,
        # which at the ~14 pixels per metre that fits a hall into a README is a pixel and a half. The node's
        # marker is the truth about where the frontier is; the floor is so a person can see it.
        cx, cy = here((x, y))
        r = max(5.0, r * pixels_per_metre)
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=colour, width=2)
    if pose:
        cx, cy = here(pose[:2])
        draw.polygon([(cx + 10 * cos(pose[2] + a), cy - 10 * sin(pose[2] + a)) for a in (0, 2.5, -2.5)],
                     fill=ROBOT)
    if note:
        draw.text((8, 6), note, fill=(240, 240, 240))
    return img


def write_gif(images, out, fps=8.0):
    """Write the frames as a looping GIF at `fps` of them a second and return how many were written.

    The frames arrive at the recording rate (one per second by default), so the GIF is a time-lapse by however
    much `fps` is above it: a 120 s run at 8 fps plays in 15 s, which is the length that survives being watched
    by a lecture. PIL quantises each frame to its own 256-colour palette; a whole exploration costs a few
    hundred kB, small enough for a README.
    """
    if not images:
        return 0
    wide, high = max(im.width for im in images), max(im.height for im in images)
    # One canvas for every frame, because a GIF is one image repeated and PIL says `images do not match` to
    # anything else. That is not a detail here: the map of an exploration run grows as the robot walks into new
    # ground, so the frames of a two-minute recording really are different sizes, and the first version of this
    # function died at the end of every run that worked. The canvas is the size of the biggest frame and each
    # frame is pasted at the **bottom left**, which is where the map's own origin is: the hall stays put and the
    # new rooms appear at the edge, instead of the whole picture jumping every time slam_toolbox extends it.
    # What was never mapped yet is filled with the unknown colour, which is what it was.
    sheets = []
    for im in images:
        sheet = Image.new("RGB", (wide, high), UNKNOWN)
        sheet.paste(im, (0, high - im.height))
        sheets.append(sheet)
    sheets[0].save(out, save_all=True, append_images=sheets[1:], duration=int(1000 / fps), loop=0,
                   optimize=True)
    return len(sheets)


def render(samples, out, fps=8.0, still=None):
    """Assemble the recorded snapshots into the GIF, plus one PNG of the last frame for the README.

    The map grows during a run, so the picture is rebuilt whenever the grid changes and every frame is drawn
    over a copy of the map it was recorded against — otherwise the robot walks off the edge of its own hall
    somewhere around the third room.
    """
    images, cache, path = [], {}, []
    for s in samples:
        # ONE scale, used for the map and for everything drawn on it. `map_image` has to spend a whole number of
        # pixels per cell — a fraction of a cell is not a thing you can paint — so the picture's true scale is
        # `pixels per cell ÷ cell size`, and on a 5 cm grid asking for 18 px/m gets 1 px per cell, which is
        # 20 px/m. Drawing the track at the asked-for 18 while the map is at 20 puts every dot 10 % of the
        # hall away from the wall it was found against: exactly the error this file exists not to make.
        cells = max(1, int(round(s["map"][2] * s["ppm"])))
        ppm = cells / s["map"][2]
        key = (s["map"], tuple(s["data"][-2048:]))       # the tail of a grid is what changes, and an
        #                                                 # unknown cell is -1, which is not a byte
        if key not in cache:
            cache[key] = map_image(s["map"], s["data"], ppm)
        path = path + [s["pose"]]                          # the track is the run, whatever the map is doing
        images.append(frame(cache[key], path, s["dots"], s["pose"],
                            "t = {:.0f} s".format(s["t"]), ppm))
    written = write_gif(images, out, fps)
    if written and still:
        images[-1].save(still)
    return written


if QoSProfile is not None:                    # the latched `/map` of a slam_toolbox that published it once
    MAP_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Recorder(Node):
    """Collect one snapshot a second of everything the view is being told.

    The map arrives latched and only when it changes; the marker array arrives twice a second and says nothing
    about the past; and the transform `map → <robot>/odom` is the only thing that puts the two in one picture —
    the same lookup `frontier_node.pose_on_map` makes, for the same reason, with the same fallback if it is not
    there.
    """

    def __init__(self, robot: str, every: float, ppm: float = None):
        super().__init__("record_view")
        self.robot, self.every, self.ppm = robot, every, ppm
        self.frames = Buffer()
        self.listener = TransformListener(self.frames, self)        # kept on self, or it is collected
        self.map, self.pose = None, None
        self.dots = []
        self.samples, self.started = [], None
        self.create_subscription(OccupancyGrid, "/map", self.on_map, MAP_QOS)
        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 10)
        self.create_subscription(MarkerArray, "/frontiers", self.on_markers, 10)
        self.create_timer(every, self.snapshot)

    def on_map(self, msg):
        self.map = msg

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def on_markers(self, array):
        """Every marker that has points, in the colour and size the node gave it.

        Text is left out on purpose: the labels are 0.30 m tall, which is the same reason `show_scores` is off
        by default — a frame of the GIF would be a cloud of words over the hall.
        """
        dots = []
        for m in array.markers:
            # `TEXT_VIEW_FACING` and not `TEXT`: ROS 2 renamed the marker types, and the name a ROS 1 mind
            # reaches for is a crash out of the recording at the first label (`type object 'Marker' has no
            # attribute 'TEXT'`). Labels are skipped here for the same reason `show_scores` is off by default:
            # they are 0.30 m tall, so at GIF resolution they are a cloud of words over the hall.
            if m.action == Marker.DELETE or not m.points or m.type == Marker.TEXT_VIEW_FACING:
                continue
            colour = tuple(int(255 * c) for c in (m.color.r, m.color.g, m.color.b))
            scale = m.scale.x or 0.18
            dots += [(p.x, p.y, scale, colour) for p in m.points]
        self.dots = dots

    def place_on_map(self):
        """The robot in the frame of the map, or None until the mapper has published both a map and a tf."""
        if not (self.map and self.pose):
            return None
        try:
            seen = self.frames.lookup_transform(self.map.header.frame_id, f"{self.robot}/odom",
                                                rclpy.time.Time())
        except Exception:                                        # noqa: BLE001 — tf says whatever tf says
            return None
        t, q = seen.transform.translation, seen.transform.rotation
        return (self.pose[0] + t.x, self.pose[1] + t.y,
                self.pose[2] + atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def snapshot(self):
        now = self.get_clock().now()
        self.started = self.started or now
        place = self.place_on_map()
        if place is None or self.map is None:
            self.get_logger().info("no map or no map → odom transform yet")
            return
        info = (self.map.info.width, self.map.info.height, self.map.info.resolution)
        self.samples.append({"t": (now - self.started).nanoseconds / 1e9, "pose": place, "map": info,
                             "data": list(self.map.data), "dots": list(self.dots),
                             "ppm": self.ppm or max(6.0, min(18.0, 720.0 / max(1.0, info[0] * info[2])))})
        print(f"frame {len(self.samples)}: t={self.samples[-1]['t']:.0f} s, {len(self.dots)} markers, "
              f"{info[0] * info[2]:.0f} × {info[1] * info[2]:.0f} m of map")


def main():
    ap = argparse.ArgumentParser(description="record a running stack into a GIF and a still PNG")
    ap.add_argument("robot")
    ap.add_argument("seconds", type=float, help="how long to record")
    ap.add_argument("out", help="where to write the .gif")
    ap.add_argument("--every", type=float, default=1.0, help="seconds between frames (default 1)")
    ap.add_argument("--fps", type=float, default=8.0, help="playback rate of the GIF (default 8)")
    ap.add_argument("--ppm", type=float, default=None, help="pixels per metre (default: fit the hall to ~620 px)")
    ap.add_argument("--still", default=None, help="also write this PNG of the last frame")
    a = ap.parse_args()
    if QoSProfile is None:
        sys.exit("recording needs a sourced ROS 2 (rclpy, tf2_ros, nav_msgs, visualization_msgs) — the drawing "
                 "functions in this file are importable without it, which is how they get tested")

    rclpy.init()
    node = Recorder(a.robot, a.every, a.ppm)
    timer = node.create_timer(a.seconds, lambda: rclpy.shutdown())     # the recording stops itself
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):     # a shutdown raises "rclpy has shut down" out of the spin
        pass
    written = render(node.samples, a.out, a.fps, a.still)
    if not written:
        print("nothing recorded — is the stack running, and is the mapper publishing /map?")
        return 1
    print(f"{a.out}: {written} frames, {node.samples[-1]['map'][0]} × {node.samples[-1]['map'][1]} cells, "
          f"{os.path.getsize(a.out) / 1024:.0f} kB" + (f" · {a.still} written too" if a.still else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
