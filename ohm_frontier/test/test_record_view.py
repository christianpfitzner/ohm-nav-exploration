"""The recorder's drawing, drawn without a robot.

`tools/record_view.py` makes the picture on the front page out of the topics a live run publishes. Half of it —
the part that turns a grid, a track and some markers into pixels — needs no ROS and no hall, which is the only
reason it is worth claiming the GIF is the same picture the RViz view shows: the map is drawn from an occupancy
grid's cells in the same three states, the frontiers are drawn from the marker array's points at the scale the
node gave them, and neither of those is a guess about what a frontier is.

So the hall below is made up: a 10 × 6 m room with a wall down the middle and a doorway in it, which is enough
to say whether the rows are upside down, whether the scale is the scale, and whether a robot that moved appears
to have moved.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

from record_view import FREE, UNKNOWN, WALL, frame, map_image, render, write_gif   # noqa: E402

CELL = 0.5
TEN_BY_SIX = (20, 12, CELL)                       # 20 × 12 cells at 0.5 m = the 10 × 6 m hall


def hall():
    """A room with a wall across it and one doorway: free on the left, unknown on the right, wall between."""
    data = []
    for y in range(TEN_BY_SIX[1]):
        for x in range(TEN_BY_SIX[0]):
            if x == 10 and 4 <= y <= 7:
                data.append(0)                    # the doorway
            elif x == 10:
                data.append(100)                  # the wall
            elif x < 10:
                data.append(0)                    # mapped and free
            else:
                data.append(-1)                   # never looked at
    return data


def test_the_three_states_of_the_map_are_three_different_colours():
    """Unknown is the whole subject of this package, so the picture has to be able to say it."""
    img = map_image(TEN_BY_SIX, hall(), 8.0)
    px = CELL * 8                                 # a cell is `CELL` metres at 8 pixels per metre: 4 px here
    centre = lambda x, y: (x * px + px // 2, img.height - (y * px + px // 2))   # noqa: E731  cell centres
    assert img.getpixel(centre(2, 2)) == FREE, "a mapped free cell"
    assert img.getpixel(centre(10, 2)) == WALL, "the wall across it"
    assert img.getpixel(centre(16, 2)) == UNKNOWN, "the half nobody has looked at"
    assert img.getpixel(centre(10, 6)) == FREE, "and the doorway in it is free"


def test_up_is_up():
    """Row 0 of a grid is the bottom of the map, and the way to get this wrong is to draw it at the top: the
    robot then appears to drive upside down, in a picture nobody can check without knowing the hall."""
    data = [0] * (TEN_BY_SIX[0] * TEN_BY_SIX[1])
    data[0] = 100                                             # cell 0 is (0, 0): the bottom-left corner
    img = map_image(TEN_BY_SIX, data, 8.0)
    assert img.getpixel((2, img.height - 2)) == WALL, "cell (0,0) is the bottom-left of the picture"
    assert img.getpixel((2, 2)) == FREE, "and so the top-left is cell (0, height-1)"


def test_a_shorter_grid_than_the_header_promises_is_skipped_not_fatal():
    """A mapper mid-update can send a grid shorter than its own header. Losing the tail of one frame of a
    two-minute recording is right; raising out of the recording is not."""
    img = map_image(TEN_BY_SIX, hall()[:120], 8.0)            # 240 cells promised, 120 delivered
    assert img.size == (20 * 4, 12 * 4), "the picture is the size the hall is, not the size that arrived"


def test_a_moving_robot_leaves_a_track_and_a_heading():
    """The track and the triangle are the two things in the animation that separate driving from sitting, so
    both have to visibly depend on the samples rather than being drawn unconditionally."""
    base = map_image(TEN_BY_SIX, hall(), 8.0)
    still = frame(base, [(3.0, 3.0)], [], (3.0, 3.0, 0.0), "t = 0 s", 8.0)
    moved = frame(base, [(3.0, 3.0), (4.0, 3.0), (5.0, 3.2)], [(7.0, 3.0, 0.18, (60, 120, 240))],
                  (5.0, 3.2, 0.2), "t = 2 s", 8.0)
    assert still.tobytes() != moved.tobytes(), "two different runs cannot draw the same picture"
    on_the_track = (int(3.4 * 8), base.height - int(3.0 * 8))        # between two samples, short of the robot
    assert moved.getpixel(on_the_track) != still.getpixel(on_the_track), \
        "the track is drawn where the robot has been"
    turned = frame(base, [(3.0, 3.0)], [], (3.0, 3.0, 1.57), "", 8.0)
    assert turned.tobytes() != still.tobytes(), "the triangle has to point where the robot is looking"


def test_the_gif_is_a_loop_of_the_frames_it_was_given(tmp_path):
    """`write_gif` is the reason there is a picture on the front page at all: it has to write a file, report the
    frames it wrote, and say nil for none rather than writing a one-frame file that looks like a run."""
    out = str(tmp_path / "run.gif")
    base = map_image(TEN_BY_SIX, hall(), 8.0)
    frames_ = [frame(base, [(2.0 + i * 0.4, 3.0)], [], (2.0 + i * 0.4, 3.0, 0.0), "", 8.0) for i in range(9)]
    assert write_gif(frames_, out) == 9
    assert os.path.getsize(out) > 200, "a GIF of a run is not a stub"
    assert open(out, "rb").read(6) in (b"GIF87a", b"GIF89a")
    assert write_gif([], out) == 0


def test_render_rebuilds_the_picture_when_the_map_grows(tmp_path):
    """The map of an exploration run is not one size: the mapper adds the room the robot walked into. Every frame
    after that has to be drawn over the bigger hall, because the alternative is a robot that walks off the edge
    of its own animation halfway through."""
    small = {"t": 0.0, "pose": (2.0, 2.0, 0.0), "map": (20, 12, CELL), "data": hall(), "dots": [], "ppm": 8.0}
    grown = dict(small, t=1.0, pose=(6.0, 2.0, 0.0), map=(24, 12, CELL), data=hall() + [0] * 48)
    out = str(tmp_path / "grown.gif")
    assert render([small, grown], out, fps=8.0) == 2
    import PIL.Image                                            # noqa: PLC0415 — only here, only to look
    frames = PIL.Image.open(out)
    assert frames.n_frames == 2, "one frame per snapshot, or the time-lapse lies about how long the run was"
