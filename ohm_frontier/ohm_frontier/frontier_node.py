"""Pick where to explore next in a hall nobody has mapped, and tell the navigation stack about it.

Listens:
    /map                                      slam_toolbox's map: free, occupied and unknown cells
    <robot>/odom                              where the robot thinks it is
    /tf                                       how far that is from where the map says it is
Says:
    /navigate_to_pose                         the frontier to drive to, as a nav2 action goal
    /frontier_goal                            the same goal as a pose, for a display and for a run without nav2

Only deciding is in here. Mapping is slam_toolbox's job, driving is nav2's, so this node keeps no map of
its own and publishes none: two mappers in one graph means two answers to "what does the hall look like",
and the one RViz shows is never the one the planner used.

The map is the interesting input, because a cell there has three states — free, occupied, **unknown** —
and unknown is neither of the others. A frontier is free floor next to unknown floor: the next room. A rule
of "free next to not-free" cannot tell unknown from a wall, and stops finding rooms after the first room is
mapped — the classic way this algorithm ends up doing nothing.

The interface to the stack is an **action**, not a topic. `bt_navigator` has no subscriber that takes a pose
as a goal — `/goal_pose` is what RViz's *button* publishes and nothing in nav2 listens to it, checked
against the installed `bt_navigator` — so this node is a `NavigateToPose` client. Where nav2's messages are
not installed the pose still goes out on `/frontier_goal`, which is the same decision written down for
anything else that drives: a test, or a student's own follower.

Two more things are in here because running it showed them:

* **A goal is given up on.** A frontier is a direction, not a reachable pose, and the planner may refuse
  one. A goal whose robot never moves towards, or never arrives at, writes off its surroundings and the next
  frontier is asked. Without that the node asks one impossible place forever.
* **Empty is said once.** With nothing left to map, that message would otherwise come twice a second.
"""
from math import atan2, cos, hypot, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

try:                                    # the action type exists only where nav2's messages are installed
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

try:                                    # to ask the mapper where its map frame starts, see `pose_on_map`
    from tf2_ros import Buffer, TransformListener
except ImportError:
    Buffer = TransformListener = None

from .frontiers import Grid, to_frame

STATUS = {4: "reached", 5: "canceled", 6: "aborted — the stack could not get there"}

SUCCEEDED = 4                                 # action_msgs/GoalStatus: STATUS_SUCCEEDED

REFUSALS_BEFORE_WRITTEN_OFF = 3               # see `give_up`


class FrontierNode(Node):
    def __init__(self):
        super().__init__("frontier_node")
        for name, value in {
            "robot": "muster",
            "map_topic": "/map",
            "goal_topic": "/frontier_goal",
            "action_topic": "/navigate_to_pose",
            "period": 0.5,                    # how often the map is looked over
            "min_frontier_cells": 12,         # below this a clump is a crack between two beams
            # The edge of a pocket one metre across is 0.9 m away, which is a goal; the cell under the
            # laser is not. Below ~0.5 m there is a third problem: the mapper only takes a scan into the
            # graph once the robot has moved 0.2 m (`minimum_travel_distance`), so a robot that shuffles
            # 0.2 m at a time never gives it the reason to. Measured with 1.2 here: three goals of 0.2 m
            # and 0.3 m, each reached, and a map that stayed at 1 425 free cells the whole time.
            "min_goal_distance": 0.7,
            "walk_out_reach": 4.0,          # how far the first goal may be when nothing is a frontier yet
            "avoid_radius": 0.75,             # written off around a goal that did not work out
            "reached_distance": 0.35,
            "stall_distance": 0.25,           # less approach than this in `stall_s` is not moving
            "stall_s": 25.0,
            "patience_s": 120.0,              # after this a goal is given up on
        }.items():
            self.declare_parameter(name, value)

        self.robot = str(self.get_parameter("robot").value)
        self.grid = None                      # slam_toolbox's map, as cells
        self.map_frame = ""
        self.pose = None                      # (x, y, theta) in the odometry frame
        self.goal = None                      # the Frontier under way
        self.goal_since = 0.0
        self.closest = 1e9                    # nearest approach to it so far
        self.recovered = 0                    # recoveries the stack reported for this goal
        self.avoid = []                       # centres reached or given up on
        self.refused = {}                     # (x, y) → how often the stack has said no to this one
        self.said_empty = False
        self.said_frames = False              # the missing-transform warning, once

        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, 10)
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
        self.grid = Grid.from_map(msg)
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
        if self.grid is None or self.pose is None:
            return
        if self.goal is not None:
            self.watch_the_current_goal()
        if self.goal is None:
            self.seek_goal()

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
                self.get_logger().warn(f"no transform {self.map_frame} → {self.robot}/odom on /tf — using "
                                       "the odometry as if it were the map frame")
                self.said_frames = True
            return self.pose
        return to_frame(self.pose, seen.transform)

    def watch_the_current_goal(self):
        x, y, _ = self.pose_on_map()
        gap = hypot(x - self.goal.x, y - self.goal.y)
        self.closest = min(self.closest, gap)
        age = self.now() - self.goal_since
        if gap < float(self.get_parameter("reached_distance").value):
            self.get_logger().info(f"reached ({self.goal.x:.2f}, {self.goal.y:.2f})")
            self.goal = None
            return
        stalled = age > float(self.get_parameter("stall_s").value) \
            and self.closest > gap - float(self.get_parameter("stall_distance").value)
        if stalled or age > float(self.get_parameter("patience_s").value):
            self.get_logger().warn(f"({self.goal.x:.2f}, {self.goal.y:.2f}) given up on after {age:.0f} s"
                                   f" — nearest approach {self.closest:.2f} m")
            self.avoid.append((self.goal.x, self.goal.y))
            self.goal = None

    def seek_goal(self):
        candidates = self.grid.frontiers(
            robot=self.pose_on_map(),
            min_cells=int(self.get_parameter("min_frontier_cells").value),
            min_distance=float(self.get_parameter("min_goal_distance").value),
            avoid=self.avoid, avoid_radius=float(self.get_parameter("avoid_radius").value))
        if not candidates:
            # Nothing worth driving to is far enough away — which at the start of a run means the whole map,
            # because the map is a metre across. Walk to the edge of what is known instead of standing still.
            stroll = self.grid.walk_out(self.pose_on_map(),
                                        reach=float(self.get_parameter("walk_out_reach").value),
                                        avoid=self.avoid,
                                        avoid_radius=float(self.get_parameter("avoid_radius").value))
            if stroll is None:
                if not self.said_empty:
                    self.get_logger().info("no frontier on this map — everything around has been reached or "
                                           "refused")
                    self.said_empty = True
                return
            candidates = [stroll]
        self.said_empty = False
        best = candidates[0]
        self.goal, self.goal_since, self.closest = best, self.now(), 1e9
        self.publish(best)
        if best.cells:
            self.get_logger().info(f"goal ({best.x:.2f}, {best.y:.2f}) — {best.cells} frontier cells, "
                                   f"{best.distance:.1f} m away")
        else:
            self.get_logger().info(f"walking out to ({best.x:.2f}, {best.y:.2f}) — the farthest floor known, "
                                   f"{best.distance:.1f} m away, no frontier beyond reach yet")

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
            self.get_logger().warn(f"no {self.get_parameter('action_topic').value} server — the navigation"
                                   " stack is not up, so nothing will drive to this")
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.navigate.send_goal_async(goal, feedback_callback=self.on_feedback) \
            .add_done_callback(self.on_response)

    def on_response(self, future):
        """The stack's first word: whether it takes the goal at all, before any driving."""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("the navigation stack refused this goal outright")
            self.give_up()
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        """The stack's last word on the goal.

        `future.result().status` is a `GoalStatus` **message**, not the number — comparing it to an int is
        true for every goal including a successfully arrived one, which would write off every place the robot
        ever reached. The number is one level deeper.
        """
        outcome = future.result()
        status = getattr(outcome.status, "status", outcome.status)   # a number here, a message there
        why = getattr(outcome.result, "error_msg", "") or ""
        self.get_logger().info(f"the stack ended this goal: {STATUS.get(status, status)}"
                               + (f" — {why}" if why else ""))
        if self.goal is not None and status != SUCCEEDED:
            self.give_up()

    def on_feedback(self, feedback):
        """The stack's own account of the drive, once a second. Only the recoveries are worth a line, because
        they are what a robot that cannot get through looks like. The field is `number_of_recoveries` on this
        ROS release (`number_recovered_behaviors` on the one before) — measured the hard way, by the node
        dying inside the callback.
        """
        done = feedback.feedback.number_of_recoveries
        if done and done != self.recovered:
            self.recovered = done
            where = f"({self.goal.x:.2f}, {self.goal.y:.2f})" if self.goal else "a goal already forgotten"
            self.get_logger().info(f"{where}: the stack is recovering ({done} behaviours so far)")

    def give_up(self):
        """The stack stopped working on this goal.

        One no is not a verdict about the place. A goal that leaves while nav2 is still coming up is refused
        within a millisecond of being sent — measured: 1 ms — and a robot started together with its stack
        always does exactly that, because `bt_navigator` is the last node to activate. The third no writes a
        place off; the two before it only send the explorer back to the same frontier once the stack is up.
        """
        if self.goal is None:
            return
        place = (round(self.goal.x, 1), round(self.goal.y, 1))
        self.refused[place] = self.refused.get(place, 0) + 1
        if self.refused[place] >= REFUSALS_BEFORE_WRITTEN_OFF:
            self.avoid.append(place)
            self.get_logger().info(f"({place[0]}, {place[1]}) written off after {self.refused[place]} refusals")
        self.goal = None

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
