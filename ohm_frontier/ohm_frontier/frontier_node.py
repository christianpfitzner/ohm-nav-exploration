"""Pick where to explore next in a hall nobody has mapped, and tell the navigation stack about it.

Listens:
    /map                                      slam_toolbox's map: free, occupied and unknown cells
    <robot>/odom                              where the robot thinks it is
    /tf                                       how far that is from where the map says it is
Says:
    /navigate_to_pose                         the frontier to drive to, as a nav2 action goal
    /frontier_goal                            the same goal as a pose, for a display and for a run without nav2
    /frontiers                                every candidate the map offers, and the chosen one, as markers

Only deciding is in here. Mapping is slam_toolbox's job, driving is nav2's, so this node keeps no map of
its own and publishes none: two mappers in one graph means two answers to "what does the hall look like",
and the one RViz shows is never the one the planner used.

The map is the interesting input, because a cell there has three states — free, occupied and **unknown** —
and unknown is neither of the others. A frontier is free floor next to unknown floor: the next room. A rule
of "free next to not-free" cannot tell unknown from a wall, and stops finding rooms after the first room is
mapped — the classic way this algorithm ends up doing nothing.

The interface to the stack is an **action**, not a topic. `bt_navigator` has no subscriber that takes a pose
as a goal — `/goal_pose` is what RViz's *button* publishes and nothing in nav2 listens to it, checked
against the installed `bt_navigator` — so this node is a `NavigateToPose` client. Where nav2's messages are
not installed the pose still goes out on `/frontier_goal`, which is the same decision written down for
anything else that drives: a test, or a student's own follower.

Five more things are in here because running it showed them:

* **A goal is kept.** The first version chose the goal in the same tick that looked at the map, so the timer
  period *was* the re-decision period: at 0.5 s a map growing under the robot's own wheels handed over a new
  winner several times a minute, the robot drove a metre towards one frontier and then towards the next, and
  arrived at none of them. A goal now stands until it is reached, until the robot has stopped getting nearer
  to it, or until the stack says no; a better-looking candidate has to beat it by `reselect_margin` before it
  is allowed to take over. See `keep_or_switch`.
* **A goal is given up on.** A frontier is a direction, not a reachable pose, and the planner may refuse one.
  A goal whose robot never moves towards, or never arrives at, writes off its surroundings and the next
  frontier is asked. Without that the node asks one impossible place forever. The clock runs on progress
  rather than on the calendar, so that a frontier across a hall is not written off for being far away. See
  `watch_the_current_goal`.
* **A word from the stack is answered to the goal it was about.** The `result` of a goal the node had already
  dropped used to write off the place the *following* goal had been chosen for, because both callbacks asked
  the node what the current goal was and were told the newer answer. Every goal now carries an attempt
  number, a late word about an older one is ignored, and a dropped goal is cancelled rather than forgotten —
  `bt_navigator` takes one goal at a time, and a stack left driving to a place the node has moved on from
  reports the failure of the drive nobody wanted.
* **Empty is said once.** With nothing left to map, that message would otherwise come twice a second.
* **Every candidate is drawn, not only the chosen one.** RViz gets the whole ranking every tick, which is the
  question a lecture actually asks: not where is it going, but why that one and not the other four. See
  `publish_markers`.
"""
from math import atan2, cos, hypot, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, FloatingPointRange
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

try:                                    # the action type exists only where nav2's messages are installed
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

try:                                    # the markers exist only where the visualisation messages are installed
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    Marker = MarkerArray = None

try:                                    # to ask the mapper where its map frame starts, see `pose_on_map`
    from tf2_ros import Buffer, TransformListener
except ImportError:
    Buffer = TransformListener = None

from . import view_markers as view
from .frontiers import DEFAULT_WEIGHTS, Grid, Weights, to_frame

STATUS = {4: "reached", 5: "canceled", 6: "aborted — the stack could not get there"}

SUCCEEDED = 4                                 # action_msgs/GoalStatus: STATUS_SUCCEEDED

REFUSALS_BEFORE_WRITTEN_OFF = 3               # see `give_up`

#: The namespaces RViz sorts the picture into — one per kind of thing this node knows: the candidates, the one
#: chosen, the arithmetic behind the ranking, the clock the chosen one is running against, and the way it
#: intends to travel. Namespaces rather than markers because a namespace is what RViz's checkbox switches: the
#: lecturer keeps the dots and hides the numbers, or the other way round, without restarting anything.
FRONTIERS_NAMESPACE, SELECTED_NAMESPACE = "frontiers", "selected"
SCORES_NAMESPACE, CLOCK_NAMESPACE, APPROACH_NAMESPACE = "scores", "clock", "approach"
CANDIDATE_SCALE, GOAL_SCALE = 0.2, 0.35
CANDIDATE_COLOUR, GOAL_COLOUR = (0.2, 0.4, 1.0, 0.8), (1.0, 0.55, 0.1, 0.9)

#: The three the ranking trades off, as parameters so they can be turned while the robot is driving.
WEIGHTS = ("weight_size", "weight_orientation", "weight_distance")


class FrontierNode(Node):
    def __init__(self):
        super().__init__("frontier_node")
        for name, value in {
            "robot": "muster",
            "map_topic": "/map",
            "goal_topic": "/frontier_goal",
            "markers_topic": "/frontiers",
            "action_topic": "/navigate_to_pose",
            "period": 0.5,                    # how often the map is looked over — and now only that
            "min_frontier_cells": 12,         # below this a clump is a crack between two beams
            # The edge of a pocket one metre across is 0.9 m away, which is a goal; the cell under the
            # laser is not. Below ~0.5 m there is a third problem: the mapper only takes a scan into the
            # graph once the robot has moved 0.2 m (`minimum_travel_distance`), so a robot that shuffles
            # 0.2 m at a time never gives it the reason to. Measured with 1.2 here: three goals of 0.2 m
            # and 0.3 m, each reached, and a map that stayed at 1 425 free cells the whole time.
            "min_goal_distance": 0.7,
            "walk_out_reach": 4.0,            # how far the first goal may be when nothing is a frontier yet
            "avoid_radius": 0.75,             # written off around a goal that did not work out
            "reached_distance": 0.35,
            "progress_distance": 0.25,        # this much nearer than ever before counts as making progress
            "goal_timeout_s": 10.0,           # this long without it counts as not working out
            "reselect_margin": 1.5,           # how much better a candidate must be to take a live goal
        }.items():
            self.declare_parameter(name, value)

        # The weights are declared with a range and a sentence, because that is what a tuning panel builds a
        # slider out of: rqt_reconfigure in ROS 2 reads the descriptors of the parameters a node declares,
        # so a described bounded double is the panel — no dynamic_reconfigure server, and nothing in this
        # ament_python package has to become a CMake package for the knobs to be turnable. The range is also
        # the guard: a negative weight is refused by rclpy itself, and a ranking that rewards a frontier for
        # being far away is not a ranking.
        for name, default, sentence in (
            ("weight_size", DEFAULT_WEIGHTS.size,
             "how much a wide opening is worth, against the widest one on the map"),
            ("weight_orientation", DEFAULT_WEIGHTS.orientation,
             "how much entering nose-first, with little wall around the goal, is worth; 0 ranks without it"),
            ("weight_distance", DEFAULT_WEIGHTS.distance,
             "how much nearness is worth, against the nearest candidate on the map"),
        ):
            self.declare_parameter(name, default, ParameterDescriptor(
                description=sentence,
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.05)]))
        self.add_on_set_parameters_callback(self.weights_changed)

        self.robot = str(self.get_parameter("robot").value)
        self.grid = None                      # slam_toolbox's map, as cells
        self.map_frame = ""
        self.pose = None                      # (x, y, theta) in the odometry frame
        self.goal = None                      # the Frontier under way
        self.attempt = 0                      # which goal the stack is answering about, see the module header
        # The nav2 goal handle of that attempt, to cancel it with. Named `goal_handle` rather than `handle`
        # because `handle` belongs to rclpy: `Node.handle` is a property whose setter raises
        # `handle cannot be modified after node creation`, so a node that stores its own handle under that
        # name dies in its own constructor — measured here, where the node could not be created at all.
        self.goal_handle = None
        self.goal_since = 0.0
        self.closest = 1e9                    # nearest approach to it so far
        self.progress_since = 0.0             # when it was last `progress_distance` nearer than ever
        self.recovered = 0                    # recoveries the stack reported for this goal
        self.avoid = []                       # centres reached or given up on
        self.refused = {}                     # (x, y) → how often the stack has said no to this one
        self.said_empty = False
        self.said_frames = False              # the missing-transform warning, once

        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, 10)
        self.marker_pub = None
        if MarkerArray is not None:
            self.marker_pub = self.create_publisher(MarkerArray,
                                                    self.get_parameter("markers_topic").value, 10)
        else:
            self.get_logger().info(f"no visualization_msgs here — nothing will be drawn on "
                                   f"{self.get_parameter('markers_topic').value}")
        self.frames = None                          # the tf buffer, where it can be had
        if Buffer is not None:
            self.frames = Buffer()
            self.watcher = TransformListener(self.frames, self)      # kept: it lives on this node's timers
        else:
            self.get_logger().info("no tf2_ros here — the odometry will be taken as the map frame")
        self.navigate = ActionClient(self, NavigateToPose, self.get_parameter("action_topic").value) \
            if NavigateToPose is not None else None
        if self.navigate is None:
            self.get_logger().info(f"no nav2_msgs here — the goal stays on "
                                   f"{self.get_parameter('goal_topic').value} for something else to drive")
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.on_map, 10)
        self.create_subscription(Odometry, f"/{self.robot}/odom", self.on_odom, 10)
        self.create_timer(float(self.get_parameter("period").value), self.on_timer)

    # ------------------------------------------------------------------------------- what comes in

    def on_map(self, msg: OccupancyGrid):
        incoming = Grid.from_map(msg)
        if self.grid is not None and incoming.checksum() == self.grid.checksum():
            return              # slam_toolbox republishes a map that has not changed. Keeping the grid keeps
                                # the survey of it: the clumps and the wall counts of a map that is the same
                                # map are the same clumps, and re-walking them at 2 Hz costs 60 ms a period
                                # for no information.
        self.grid = incoming
        if not self.map_frame:
            self.map_frame = msg.header.frame_id
            self.get_logger().info(
                f"map on frame {msg.header.frame_id}: {msg.info.width} × {msg.info.height} cells at "
                f"{msg.info.resolution:g} m per cell, origin "
                f"({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})")

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     2.0 * atan2(q.z, q.w))            # yaw of a quaternion; the world is flat

    # ------------------------------------------------------------------------------- what goes out

    def on_timer(self):
        """Look the map over, keep or replace what is being driven to, and draw the whole ranking.

        The frontier search runs on every tick now, not only when there is no goal. That is the price of the
        two rules that replaced "decide whenever the timer comes round": a better candidate can only be
        noticed by looking for it, and RViz is asked for every frontier while the robot is on the way to one.
        On the maps here — a few tens of thousands of cells at 2 Hz — it is a price easily paid.
        """
        if self.grid is None or self.pose is None:
            return
        if self.goal is not None:
            self.watch_the_current_goal()                     # may drop the goal, nothing else
        ranked = self.candidates()
        if self.goal is None:
            self.take(ranked)
        else:
            self.keep_or_switch(ranked)
        self.publish_markers(ranked)

    def pose_on_map(self) -> tuple:
        """The robot's pose in the frame of the map, which is not the frame `<robot>/odom` is in.

        The gap between them is the mapper's correction for the odometry drifting — centimetres while the
        odometry is good, metres in the exercises that ruin it on purpose. Measured here on the first run:
        0.18 m, which was already enough to put a goal under the robot's own wheels.
        """
        if not (self.frames and self.map_frame and self.pose):
            return self.pose
        try:
            seen = self.frames.lookup_transform(self.map_frame, f"{self.robot}/odom", Time())
        except Exception:                             # not up yet, expired, or a tree without that edge
            if not self.said_frames:
                self.get_logger().warning(f"no transform {self.map_frame} → {self.robot}/odom on /tf — using "
                                       "the odometry as if it were the map frame")
                self.said_frames = True
            return self.pose
        return to_frame(self.pose, seen.transform)

    # --------------------------------------------------------------------- the goal, and its whole life

    def weights(self) -> Weights:
        """The three weights, read as they are *now* — a value set while the robot drives is in the next
        ranking without anything having to be updated here.

        The parameters carry a `weight_` prefix and the fields of `Weights` do not: the prefix is there for
        whoever opens the tuning panel, which lists a node's parameters by name among all the others, and is a
        nuisance inside `frontiers.py`. Stripping it is the whole translation, and it is a line that has to be
        read as a pair with `WEIGHTS` — a parameter named there without a matching field in `Weights` answers
        with a `TypeError` from the first decision, which is why the test names the parameters rather than
        building a `Weights` by hand.
        """
        return Weights(**{name.removeprefix("weight_"): float(self.get_parameter(name).value)
                          for name in WEIGHTS})

    def candidates(self) -> list:
        """Everything worth driving to on this map, best first — including the filler goal when nothing is.

        With nothing worth driving to that is far enough away — which at the start of a run means the whole
        map, because the map is a metre across — the answer is the farthest cell the mapper calls floor, as
        one candidate. It comes back in the same shape as a frontier so there is one thing to handle below,
        and its score is 0.0, which is what lets the first real frontier take it over (see
        `keep_or_switch`).
        """
        where = self.pose_on_map()
        found = self.grid.frontiers(
            robot=where,
            min_cells=int(self.get_parameter("min_frontier_cells").value),
            min_distance=float(self.get_parameter("min_goal_distance").value),
            avoid=self.avoid, avoid_radius=float(self.get_parameter("avoid_radius").value),
            weights=self.weights())
        if found:
            self.said_empty = False
            return found
        stroll = self.grid.walk_out(where, reach=float(self.get_parameter("walk_out_reach").value),
                                   avoid=self.avoid,
                                   avoid_radius=float(self.get_parameter("avoid_radius").value))
        if stroll is None:
            if not self.said_empty:
                self.get_logger().info("no frontier on this map — everything around has been reached or "
                                       "refused")
                self.said_empty = True
            return []
        return [stroll]

    def take(self, ranked: list):
        """Start driving to the best of the candidates, and count this as a fresh attempt.

        The attempt number goes up before the goal is sent, so every callback the stack answers with can be
        recognised as being about this drive or about one that is already history.
        """
        if not ranked:
            return
        chosen = ranked[0]
        self.attempt += 1
        self.goal, self.goal_since, self.closest, self.recovered = chosen, self.now(), 1e9, 0
        self.progress_since = self.now()
        self.publish(chosen)
        if chosen.cells:
            self.get_logger().info(f"goal ({chosen.x:.2f}, {chosen.y:.2f}) — {chosen.cells} frontier cells, "
                                   f"{chosen.distance:.1f} m away, score {chosen.score:.2f}")
        else:
            self.get_logger().info(f"walking out to ({chosen.x:.2f}, {chosen.y:.2f}) — the farthest floor "
                                   f"known, {chosen.distance:.1f} m away, no frontier beyond reach yet")

    def keep_or_switch(self, ranked: list):
        """The rule that was missing: a live goal is kept, and only a much better candidate takes it.

        `reselect_margin` is the price of changing one's mind, multiplied by the score the goal had when it
        was taken — so the candidate has to be better than the best of that earlier map by that factor, not
        merely better than whatever the map offers now. Two things follow, and both are wanted:

        * The same place seen again is not news. `frontiers` re-aims at a clump as the map grows, so without
          the distance check below a clump whose best cell moves by a cell would look like a new candidate
          and the robot would be sent after its own frontier.
        * A walk-out goal has score 0.0, so any real frontier beats it, which is right: that goal is a filler
          whose only job is to give the mapper a reason to integrate a scan.

        A margin of 1.0 would bring the original bug straight back — every tick a fresh winner, a goal every
        half second — so the parameter is not allowed below 1.0 rather than being trusted with it.
        """
        if not ranked:
            return                                              # nothing to compare against: keep driving
        best = ranked[0]
        if hypot(best.x - self.goal.x, best.y - self.goal.y) < self.grid.res:
            return                                              # the same doorway, seen again
        margin = max(1.0, float(self.get_parameter("reselect_margin").value))
        if best.score < margin * self.goal.score:
            return
        why = ("the walk-out goal was only a filler" if self.goal.score <= 0.0 else
               f"score {best.score:.2f} against {self.goal.score:.2f}")
        self.get_logger().info(f"switching to ({best.x:.2f}, {best.y:.2f}) — {why}. The drive to "
                               f"({self.goal.x:.2f}, {self.goal.y:.2f}) is dropped, not written off: it was "
                               "never tried.")
        self.drop_goal()                                        # no blacklist: nobody failed here
        self.take(ranked)

    def watch_the_current_goal(self):
        """Arrived, or not arriving. The clock that matters runs on progress, not on the calendar.

        `goal_timeout_s` — 10 s — is how long the robot may fail to come nearer before the goal is written
        off. Ten seconds is the shortest wait that still separates the two things it can be waiting for: a
        robot whose velocity command never arrives (the stack can be whole, the plan computed and the commands
        flowing on a topic nothing here subscribes to — a TwistStamped in front of a base that wants a Twist,
        see the fourth pit in the README), and a frontier on the far side of a wall, which looks the same from
        here and is not worth another hour.

        It is deliberately not a deadline for the whole drive. A frontier at the far end of a hall is
        perfectly reachable and takes far longer than 10 s, and a rule that wrote those off would blacklist a
        whole hall inside a minute; that is what `stall_s`/`patience_s` amounted to, a 25 s and a 120 s
        calendar deadline, and they are gone. What restarts the clock instead is `progress_distance` metres of
        being closer to the goal than the robot has ever been to it.
        """
        x, y, _ = self.pose_on_map()
        gap = hypot(x - self.goal.x, y - self.goal.y)
        timeout = float(self.get_parameter("goal_timeout_s").value)
        step = float(self.get_parameter("progress_distance").value)
        if gap < self.closest - step or self.closest > 1e8:     # first sight of the gap counts as progress
            self.closest, self.progress_since = gap, self.now()
        if gap < float(self.get_parameter("reached_distance").value):
            self.get_logger().info(f"reached ({self.goal.x:.2f}, {self.goal.y:.2f})")
            # Seen from here, the floor around it is no longer news; and a goal the node forgets without
            # writing off is picked again on the next tick, which looks from the outside like a robot
            # arriving at the same doorway twice a second for ever.
            self.avoid.append((self.goal.x, self.goal.y))
            self.drop_goal()
            return
        stalled = self.now() - self.progress_since
        if stalled > timeout:
            self.get_logger().warning(f"({self.goal.x:.2f}, {self.goal.y:.2f}) given up on — no nearer in "
                                   f"{stalled:.0f} s (nearest approach {self.closest:.2f} m, still "
                                   f"{gap:.2f} m away)")
            self.avoid.append((self.goal.x, self.goal.y))
            self.drop_goal()

    def drop_goal(self):
        """Stop working on this goal, and stop the stack working on it either.

        Cancelling is not decoration: `bt_navigator` serves one goal at a time, so a node that only forgets
        its goal leaves the stack driving to the place it forgot — and about to answer, some seconds later,
        with a result that the node would otherwise read as news about the goal it has in the meantime. The
        attempt number goes up first, which makes that answer stale, and the cancel makes it unnecessary.
        """
        self.attempt += 1
        handle, self.goal_handle = self.goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        self.goal = None

    # ------------------------------------------------------------------------------- what goes out

    def publish(self, frontier):
        """The pose for a display and for a run without nav2, the action goal for the stack."""
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame or "map"         # the map's own frame, never assumed
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(frontier.x), float(frontier.y)
        msg.pose.orientation.z = sin(frontier.heading / 2.0)  # nose-first: the first new scan then looks
        msg.pose.orientation.w = cos(frontier.heading / 2.0)  # into the unknown instead of behind
        self.goal_pub.publish(msg)
        if self.navigate is None:
            return
        if not self.navigate.server_is_ready():
            self.get_logger().warning(f"no {self.get_parameter('action_topic').value} server — the navigation"
                                   " stack is not up, so nothing will drive to this")
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        attempt = self.attempt
        self.navigate.send_goal_async(goal, feedback_callback=lambda feedback:
                                       self.on_feedback(feedback, attempt)) \
            .add_done_callback(lambda response: self.on_response(response, attempt))

    def publish_markers(self, ranked: list):
        """Every candidate with the numbers that ranked it, the one being driven to, and the clock on it.

        The whole ranking goes out on every tick whether or not anything changed, because the picture worth
        looking at from a lecture chair is the one showing what was *not* chosen, and why. So every candidate
        carries its own arithmetic as a label — cells, metres, score, the three the weights trade off — and the
        goal carries how long it has left to be worth keeping, which is the number `goal_timeout_s` is about
        and the one a student asks about first when the robot refuses to move.

        What can go stale is removed rather than left standing: an orange dot in the middle of a hall reads as
        "that is where it is going", and that is the one thing in this view that must never be a lie. `DELETE`
        and not `DELETEALL`, because `DELETEALL` would take the candidate dots published earlier in the same
        array along with it; the labels need neither, each carrying a one-second lifetime (`view.labels`).
        """
        if self.marker_pub is None:
            return
        frame = self.map_frame or "map"                   # the map's own frame, never assumed
        stamp = self.get_clock().now().to_msg()
        markers = [view.dots(frame, stamp, FRONTIERS_NAMESPACE, [(f.x, f.y) for f in ranked],
                             CANDIDATE_SCALE, CANDIDATE_COLOUR)] + view.labels(
            frame, stamp, SCORES_NAMESPACE,
            [((f.x, f.y), f"cells {f.cells} · {f.distance:.1f} m · score {f.score:.2f}") for f in ranked])
        if self.goal is None:
            markers += [view.dots(frame, stamp, SELECTED_NAMESPACE, [], GOAL_SCALE, GOAL_COLOUR,
                                  action=Marker.DELETE),
                        view.arrows(frame, stamp, APPROACH_NAMESPACE, [], colour=view.GREEN)]
            view.publish(self.marker_pub, markers)
            return

        x, y, _ = self.pose_on_map()
        left = float(self.get_parameter("goal_timeout_s").value) - (self.now() - self.progress_since)
        markers += [view.dots(frame, stamp, SELECTED_NAMESPACE, [(self.goal.x, self.goal.y)],
                              GOAL_SCALE, GOAL_COLOUR),
                    view.arrows(frame, stamp, APPROACH_NAMESPACE, [((x, y), (self.goal.x, self.goal.y))],
                                colour=view.GREEN)] + view.labels(
            frame, stamp, CLOCK_NAMESPACE,
            [((self.goal.x, self.goal.y),
              f"score {self.goal.score:.2f} · {max(left, 0.0):.0f} s left to get nearer")])
        view.publish(self.marker_pub, markers)

    def on_response(self, future, attempt: int):
        """The stack's first word: whether it takes the goal at all, before any driving."""
        if attempt != self.attempt:
            self.get_logger().debug(f"a refusal for drive {attempt} arrived while drive {self.attempt} is "
                                    "the current one — ignored")
            return
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning("the navigation stack refused this goal outright")
            self.give_up(attempt)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda outcome: self.on_result(outcome, attempt))

    def on_result(self, future, attempt: int):
        """The stack's last word on the goal.

        `future.result().status` is a `GoalStatus` **message**, not the number — comparing it to an int is
        true for every goal including a successfully arrived one, which would write off every place the robot
        ever reached. The number is one level deeper.
        """
        if attempt != self.attempt:
            self.get_logger().debug(f"a result for drive {attempt} arrived while drive {self.attempt} is "
                                    "the current one — ignored")
            return
        outcome = future.result()
        status = getattr(outcome.status, "status", outcome.status)   # a number here, a message there
        why = getattr(outcome.result, "error_msg", "") or ""
        self.get_logger().info(f"the stack ended this goal: {STATUS.get(status, status)}"
                               + (f" — {why}" if why else ""))
        self.goal_handle = None
        if self.goal is not None and status != SUCCEEDED:
            self.give_up(attempt)

    def on_feedback(self, feedback, attempt: int):
        """The stack's own account of the drive, once a second. Only the recoveries are worth a line, because
        they are what a robot that cannot get through looks like. The field is `number_of_recoveries` on this
        ROS release (`number_recovered_behaviors` on the one before) — measured the hard way, by the node
        dying inside the callback.
        """
        if attempt != self.attempt:
            return
        done = feedback.feedback.number_of_recoveries
        if done and done != self.recovered:
            self.recovered = done
            where = f"({self.goal.x:.2f}, {self.goal.y:.2f})" if self.goal else "a goal already forgotten"
            self.get_logger().info(f"{where}: the stack is recovering ({done} behaviours so far)")

    def give_up(self, attempt: int = None):
        """The stack stopped working on this goal.

        One no is not a verdict about the place. A goal that leaves while nav2 is still coming up is refused
        within a millisecond of being sent — measured: 1 ms — and a robot started together with its stack
        always does exactly that, because `bt_navigator` is the last node to activate. The third no writes a
        place off; the two before it only send the explorer back to the same frontier once the stack is up.
        """
        if self.goal is None or (attempt is not None and attempt != self.attempt):
            return
        place = (round(self.goal.x, 1), round(self.goal.y, 1))
        self.refused[place] = self.refused.get(place, 0) + 1
        if self.refused[place] >= REFUSALS_BEFORE_WRITTEN_OFF:
            self.avoid.append(place)
            self.get_logger().info(f"({place[0]}, {place[1]}) written off after {self.refused[place]} refusals")
        self.drop_goal()

    # ------------------------------------------------------------------------------- the tuning panel

    def weights_changed(self, parameters) -> SetParametersResult:
        """Refuse a set of weights that would leave nothing to rank by, and say out loud what the ranking now
        values, so a slider has an answer.

        Nothing is stored here: the weights are read on every decision, so a value set while the robot is
        driving is in the next ranking on its own. What is done here is the one check that cannot be made by a
        declared range, because it is not about one weight but about the three together.

        With all three at zero every candidate scores 0.0, so `keep_or_switch` compares 0.0 against
        `reselect_margin` times 0.0, finds the candidate not worse, and takes it — a new goal on every tick,
        which is the failure the margin exists to prevent. A walk-out goal already has score 0.0 on purpose
        (see `candidates`), and the rule that lets any real frontier take that filler over is the same rule
        that would let every frontier take over every other one once nothing carries a weight. So the sum is
        refused rather than trusted, and the reason goes back with the refusal for whoever is at the panel to
        read.
        """
        moved = {p.name: float(p.value) for p in parameters if p.name in WEIGHTS}
        if not moved:
            return SetParametersResult(successful=True)
        # The callback runs before the values are applied, so the state to judge is the current one with these
        # few values laid over it — a `ros2 param set` of one weight has to be judged against the two that
        # stayed where they were, not against zero.
        after = {name: moved.get(name, float(self.get_parameter(name).value)) for name in WEIGHTS}
        if not any(after.values()):
            return SetParametersResult(
                successful=False,
                reason="all three weights at zero leaves every candidate on the same score, and the node "
                       "would choose a new goal every tick again — leave at least one of them above zero")
        for name, value in moved.items():
            self.get_logger().info(f"{name} is now {value:g} — the next decision will be ranked with it")
        return SetParametersResult(successful=True)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
