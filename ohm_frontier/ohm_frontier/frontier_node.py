"""One node: lidar in, map and navigation goal out.

Reads the robot's lidar and odometry out of the simulator, keeps a `frontiers.Grid`, publishes that grid
as `/map` — for nav2's global costmap and for rviz — and publishes the best frontier as a goal on
`/goal_pose`. The navigation stack is not in here: it runs beside this node and is told where to drive.

    ros2 run ohm_frontier frontier_node --ros-args -p robot:=muster
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .frontiers import FREE, UNKNOWN, Frontier, Grid

try:                                        # nav2's goal message. Without nav2 installed the plain
    from nav2_msgs.msg import GoalPoseStamped as Goal, PoseWithStatus      # geometry message is used
except ImportError:
    Goal, PoseWithStatus = PoseStamped, None


class FrontierNode(Node):

    def __init__(self):
        super().__init__("frontier_node")
        p = self.declare_parameter
        p("robot", "muster")
        p("resolution", 0.10)                   # metres per cell of the map built here
        p("min_frontier_cells", 12)             # below this a clump is a crack between two beams
        p("min_distance", 0.6)                  # do not aim at your own doorstep
        p("avoid_radius", 0.9)                  # how far a failed goal keeps a clump out
        p("replan_s", 0.5)                      # how often the map goes out and the next goal is weighed
        p("goal_tol", 0.40)                     # this close counts as arrived
        p("stall_s", 25.0)                      # this long without this much motion counts as refused
        p("stall_distance", 0.25)
        p("patience_s", 120.0)                  # never arriving counts as refused too
        p("map_topic", "/map")
        p("goal_topic", "/goal_pose")

        robot = self.get_parameter("robot").value
        self.grid = None                        # built once /sim/world says how big the hall is
        self.pose = None
        self.mount = (0.0, 0.0, 0.0)            # base_link -> laser, from /tf_static
        self.goal = None                        # (frontier, second sent, pose when sent)
        self.avoid = []                         # centres already reached or failed
        self.said_empty = False
        self.painted = False                    # an empty grid has no frontiers either, and saying so

        self.map_pub = self.create_publisher(OccupancyGrid, self.get_parameter("map_topic").value, 1)
        self.goal_pub = self.create_publisher(Goal, self.get_parameter("goal_topic").value, 1)
        self.create_subscription(LaserScan, f"{robot}/scan", self.on_scan, 10)
        self.create_subscription(Odometry, f"{robot}/odom", self.on_odom, 10)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, 10)
        self.create_subscription(String, "/sim/world", self.on_world, 10)
        self.create_timer(float(self.get_parameter("replan_s").value), self.on_timer)

    # ---------------------------------------------------------------------------- in
    def on_world(self, msg: String) -> None:
        if self.grid is not None:
            return
        hall = json.loads(msg.data)
        self.grid = Grid(hall["size"][0], hall["size"][1],
                         float(self.get_parameter("resolution").value))
        self.get_logger().info(f"mapping {hall['name']}: {hall['size'][0]} × {hall['size'][1]} m "
                               f"at {self.grid.res} m per cell")

    def on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.child_frame_id.endswith("laser"):
                x, y = t.transform.translation.x, t.transform.translation.y
                q = t.transform.rotation
                self.mount = (x, y, 2.0 * math.atan2(q.z, q.w))

    def on_odom(self, msg: Odometry) -> None:
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (pos.x, pos.y, 2.0 * math.atan2(q.z, q.w))

    def on_scan(self, msg: LaserScan) -> None:
        if self.grid is None or self.pose is None:
            return
        cos, sin = math.cos(self.pose[2]), math.sin(self.pose[2])       # beams leave the laser, not
        x = self.pose[0] + cos * self.mount[0] - sin * self.mount[1]    # the middle of the robot
        y = self.pose[1] + sin * self.mount[0] + cos * self.mount[1]
        angles = np.arange(len(msg.ranges), dtype=float) * msg.angle_increment + msg.angle_min
        self.grid.scan((x, y, self.pose[2] + self.mount[2]), angles,
                       np.asarray(msg.ranges, dtype=float), msg.range_max)
        self.painted = True

    # ---------------------------------------------------------------------------- out
    def on_timer(self) -> None:
        if self.grid is None:
            return
        self.publish_map()
        if self.pose is not None and not self.busy():
            self.seek_goal()

    def busy(self) -> bool:
        """True while the goal already sent is worth waiting for; a refused one is put on the list."""
        if self.goal is None:
            return False
        chosen, sent, when = self.goal
        now = self.get_clock().now().nanoseconds * 1e-9
        if math.dist(self.pose[:2], (chosen.x, chosen.y)) <= self.get_parameter("goal_tol").value:
            self.get_logger().info(f"reached ({chosen.x:.2f}, {chosen.y:.2f})")
        elif now - sent > self.get_parameter("patience_s").value:
            self.get_logger().warn(f"({chosen.x:.2f}, {chosen.y:.2f}): never arrived")
        elif now - sent > self.get_parameter("stall_s").value \
                and math.dist(self.pose[:2], when[:2]) < self.get_parameter("stall_distance").value:
            self.get_logger().warn(f"({chosen.x:.2f}, {chosen.y:.2f}): the robot never moved")
        else:
            return True
        self.avoid.append((chosen.x, chosen.y))     # reached or impossible, both are behind us
        self.goal = None
        return False

    def seek_goal(self) -> None:
        best = self.grid.frontiers(robot=self.pose,
                                   min_cells=int(self.get_parameter("min_frontier_cells").value),
                                   min_distance=float(self.get_parameter("min_distance").value),
                                   avoid=self.avoid,
                                   avoid_radius=float(self.get_parameter("avoid_radius").value))
        if not best:
            if self.painted and not self.said_empty:
                self.get_logger().info("no frontier left — mapped as far as this lidar can see")
                self.said_empty = True
            return
        self.said_empty = False
        self.goal = (best[0], self.get_clock().now().nanoseconds * 1e-9, self.pose)
        self.publish_goal(best[0])

    def publish_goal(self, chosen: Frontier) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"                  # the grid is anchored at the hall's own origin
        pose.pose.position.x, pose.pose.position.y = float(chosen.x), float(chosen.y)
        yaw = math.atan2(chosen.y - self.pose[1], chosen.x - self.pose[0])     # face where it is going
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if PoseWithStatus is not None:                # what nav2's bt_navigator listens for
            wrapped = Goal()
            wrapped.pose.header, wrapped.pose.pose = pose.header, pose.pose
            pose = wrapped
        self.goal_pub.publish(pose)
        self.get_logger().info(f"goal ({chosen.x:.2f}, {chosen.y:.2f}) — {chosen.cells} frontier cells")

    def publish_map(self) -> None:
        cells = self.grid.cells
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = MapMetaData(resolution=self.grid.res, height=cells.shape[0], width=cells.shape[1],
                               position=Pose())
        msg.data = np.where(cells == UNKNOWN, -1, np.where(cells == FREE, 0, 100)).ravel().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(FrontierNode())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
