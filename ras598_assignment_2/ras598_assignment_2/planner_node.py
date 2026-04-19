"""
planner_node.py
===============
Main ROS2 navigation node for RAS 598 Assignment 2.

Confirmed from grading_scout.py:
  - /get_task service type: example_interfaces.srv.Trigger
  - Response fields: success (bool), message (string "sx,sy,gx,gy")
  - Start: (-7.0, -7.0)   Goal: (7.0, 2.5)
  - Goal acceptance radius: 0.5m (scout uses math.dist < 0.5)
  - Startup tax: 0.6 units, triggered when v goes from <=0.04 to >0.04
  - Tick rate: 0.1s (10 Hz scout timer)
  - Energy per tick = (0.05 + v*0.06 + w*0.15 + startup_tax) * 0.5

Energy Optimization (based on exact coefficients above):
  - angular_coeff (0.15) >> linear_coeff (0.06): minimise turns
  - startup_tax (0.6) >> motion cost: NEVER stop mid-path
  - time_penalty (0.05/tick): faster paths save energy
  - angular.z = exactly 0.0 while driving: zero angular drain
  - LOS pruning: minimise waypoints -> minimise turns -> minimise taxes
"""

import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg       import Twist, Point
from nav_msgs.msg            import Odometry
from std_msgs.msg            import Float32
from visualization_msgs.msg  import Marker, MarkerArray
from example_interfaces.srv  import Trigger   # confirmed from grading_scout.py
from ras598_assignment_2.grid_map   import GridMap
from ras598_assignment_2.a_star      import astar
from ras598_assignment_2.prune_los import prune_path
from ament_index_python.packages import get_package_share_directory

STATE_IDLE   = 'IDLE'
STATE_ROTATE = 'ROTATE'
STATE_DRIVE  = 'DRIVE'
STATE_DONE   = 'DONE'


class PlannerNode(Node):

    # Params
    CELL_RESOLUTION   = 0.032     #(0.2) m/cell  — fixed by spec
    INFLATION_RADIUS  = 0.60 # 0.65    # m       — > 0.6 required

    # The map image is cloned alongside the package
    # MAP_IMAGE = os.path.join(
    #     os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    #     'map', 'cave_filled.png'
    # )
    MAP_IMAGE = os.path.join(
        get_package_share_directory('ras598_assignment_2'),
        'cave_filled.png'
    )

    # Controller thresholds
    HEADING_THRESH  = 0.08    # rad — switch ROTATE -> DRIVE
    REDRIVE_THRESH  = 0.20    # rad — switch DRIVE -> ROTATE (avoid jitter)
    WAYPOINT_DIST   = 0.70 # 0.70 # 0.30    # m   — accept intermediate waypoint
    GOAL_DIST       = 0.48 # 0.45    # m   — accept final goal (scout uses 0.5m)
    MAX_LINEAR      = 1.00    # m/s
    MAX_ANGULAR     = 0.65 # 0.65    # rad/s
    KP_ANGULAR      = 0.6 # 1.8     # proportional gain
    CTRL_HZ         = 20      # Hz

    def __init__(self):
        super().__init__('planner_node')

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',         10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planner_markers', 10)

        # Subscribers
        self.create_subscription(Odometry, '/ground_truth',    self._odom_cb,   10)
        self.create_subscription(Float32,  '/energy_consumed', self._energy_cb, 10)

        # State
        self.pose              = None
        self.energy            = 0.0
        self.raw_path_world    = []
        self.pruned_path_world = []
        self.waypoints         = []
        self.wp_idx            = 0
        self.state             = STATE_IDLE
        self._task_called      = False

        # Control loop
        self.create_timer(1.0 / self.CTRL_HZ, self._control_loop)

        # Delayed task init (give scout time to come up)
        self.create_timer(2.5, self._init_task)

        self.get_logger().info('PlannerNode ready.')

    # ==============================================================
    # Task initialisation
    # ==============================================================

    # def _init_task(self):
    #     if self._task_called:
    #         return
    #     self._task_called = True

    #     client = self.create_client(Trigger, '/get_task')
    #     self.get_logger().info('Waiting for /get_task …')
    #     if not client.wait_for_service(timeout_sec=15.0):
    #         self.get_logger().error('/get_task not available!')
    #         return

    #     future = client.call_async(Trigger.Request())
    #     rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

    #     if future.result() is None:
    #         self.get_logger().error('/get_task call failed!')
    #         return

    #     self.get_logger().info(f'/get_task response: {future.result().message}')
    #     self._run_planning(future.result().message)

    def _init_task(self):
        if self._task_called:
            return
        self._task_called = True

        self._get_task_client = self.create_client(Trigger, '/get_task')
        self.get_logger().info('Waiting for /get_task ...')
        if not self._get_task_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('/get_task not available!')
            return

        future = self._get_task_client.call_async(Trigger.Request())
        future.add_done_callback(self._tr_callback)

    def _tr_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Task: {result.message}')
            self._run_planning(result.message)
        except Exception as e:
            self.get_logger().error(f'/get_task failed: {e}')

    def _run_planning(self, task_str: str):
        """Parse task, build map, run A*, prune, set up controller."""
        try:
            sx, sy, gx, gy = [float(v) for v in task_str.split(',')]
        except Exception as e:
            self.get_logger().error(f'Bad task string "{task_str}": {e}')
            return

        self.get_logger().info(
            f'Planning: start=({sx},{sy}) goal=({gx},{gy})'
        )

        # Load & inflate map
        self.get_logger().info(f'Loading map: {self.MAP_IMAGE}')
        self.gmap = GridMap(
            self.MAP_IMAGE,
            cell_resolution=self.CELL_RESOLUTION,
            inflation_radius_m=self.INFLATION_RADIUS,
        )

        # World -> cell
        start_cell = self.gmap.world_to_cell(sx, sy)
        goal_cell  = self.gmap.world_to_cell(gx, gy)
        self.get_logger().info(f'Start cell: {start_cell}  Goal cell: {goal_cell}')

        # Validate cells are free
        # if not self.gmap.is_free_cell(*start_cell):
        #     self.get_logger().warn('Start cell is inside obstacle! Nudging …')
        # if not self.gmap.is_free_cell(*goal_cell):
        #     self.get_logger().warn('Goal cell is inside obstacle! Nudging …')
        if not self.gmap.is_free_cell(*start_cell):
            self.get_logger().warn('Start inside obstacle → nudging')
            start_cell = self.gmap.find_nearest_free(*start_cell)

        if not self.gmap.is_free_cell(*goal_cell):
            self.get_logger().warn('Goal inside obstacle → nudging')
            goal_cell = self.gmap.find_nearest_free(*goal_cell)

        if start_cell is None or goal_cell is None:
            self.get_logger().error("Could not find valid start/goal!")
            return

        self.get_logger().info(f'Nudged Start: {start_cell}, Goal: {goal_cell}')



        # A*
        self.get_logger().info('Running A* …')
        raw_cells = astar(self.gmap.grid, start_cell, goal_cell)
        if not raw_cells:
            self.get_logger().error('A* found no path!')
            return
        self.get_logger().info(f'A* raw path: {len(raw_cells)} cells')
        self.get_logger().info(f'Grid obstacle ratio: {self.gmap.grid.mean():.3f}')

        # Convert to world coords
        self.raw_path_world = [
            self.gmap.cell_to_world(c, r) for (c, r) in raw_cells
        ]

        # LOS prune
        pruned_cells = prune_path(self.gmap.grid, raw_cells)
        self.pruned_path_world = [
            self.gmap.cell_to_world(c, r) for (c, r) in pruned_cells
        ]
        self.get_logger().info(
            f'Pruned: {len(self.pruned_path_world)} waypoints '
            f'(from {len(raw_cells)})'
        )

        # Replace last waypoint with exact goal world coords
        # so the scout's 0.5m acceptance circle is guaranteed to trigger
        self.pruned_path_world[-1] = (gx, gy)

        # Set up controller (skip waypoint[0] = start, robot is already there)
        self.waypoints = self.pruned_path_world[1:]
        self.wp_idx    = 0
        self.state     = STATE_ROTATE
        self.get_logger().info('Executing path …')

    # ==============================================================
    # Callbacks
    # ==============================================================

    def _odom_cb(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.pose = (x, y, yaw)

    def _energy_cb(self, msg: Float32):
        self.energy = msg.data

    # ==============================================================
    # Control loop
    # ==============================================================

    # def _control_loop(self):
    #     if self.state in (STATE_IDLE, STATE_DONE):
    #         return
    #     if self.pose is None:
    #         return
    #     if self.wp_idx >= len(self.waypoints):
    #         self._finish()
    #         return

    #     rx, ry, ryaw = self.pose
    #     gx, gy       = self.waypoints[self.wp_idx]
    #     dx, dy       = gx - rx, gy - ry
    #     dist         = math.hypot(dx, dy)
    #     angle        = math.atan2(dy, dx)
    #     err          = self._wrap(angle - ryaw)

    #     # Is this the final waypoint?
    #     is_last = (self.wp_idx == len(self.waypoints) - 1)
    #     accept  = self.GOAL_DIST if is_last else self.WAYPOINT_DIST

    #     # Waypoint acceptance — try NOT to stop
    #     if dist < accept:
    #         self.wp_idx += 1
    #         if self.wp_idx >= len(self.waypoints):
    #             self._finish()
    #             return
    #         # Recompute for new waypoint
    #         gx, gy   = self.waypoints[self.wp_idx]
    #         dx, dy   = gx - rx, gy - ry
    #         angle    = math.atan2(dy, dx)
    #         err      = self._wrap(angle - ryaw)
    #         if abs(err) > self.HEADING_THRESH:
    #             self.state = STATE_ROTATE
    #         # else keep driving — no stop, no startup tax!

    #     cmd = Twist()

    #     if self.state == STATE_ROTATE:
    #         if abs(err) <= self.HEADING_THRESH:
    #             self.state = STATE_DRIVE
    #         else:
    #             # Pure rotation — linear stays 0
    #             omega = self.KP_ANGULAR * err
    #             omega = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, omega))
    #             cmd.angular.z = omega
    #             # cmd.linear.x = 0.0  (default)

    #     if self.state == STATE_DRIVE:
    #         if abs(err) > self.REDRIVE_THRESH:
    #             self.state = STATE_ROTATE
    #         else:
    #             cmd.linear.x  = self.MAX_LINEAR
    #             cmd.angular.z = 0.0   # exactly zero — no angular energy drain

    #     self.cmd_pub.publish(cmd)
    #     self._publish_markers()

    def _control_loop(self):
        if self.state in (STATE_IDLE, STATE_DONE) or self.pose is None:
            return
        if self.wp_idx >= len(self.waypoints):
            self._finish()
            return

        rx, ry, ryaw = self.pose
        gx, gy       = self.waypoints[self.wp_idx]
        dx, dy       = gx - rx, gy - ry
        dist         = math.hypot(dx, dy)
        angle        = math.atan2(dy, dx)
        err          = self._wrap(angle - ryaw)

        is_last = (self.wp_idx == len(self.waypoints) - 1)
        accept  = self.GOAL_DIST if is_last else self.WAYPOINT_DIST

        # Advance waypoint index without stopping
        # if dist < accept:
        #     self.wp_idx += 1
        if dist < accept and abs(err) < 0.3:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self._finish()
                return
            gx, gy = self.waypoints[self.wp_idx]
            dx, dy = gx - rx, gy - ry
            angle  = math.atan2(dy, dx)
            err    = self._wrap(angle - ryaw)

        cmd = Twist()

        if self.state == STATE_ROTATE:
            if abs(err) <= self.HEADING_THRESH:
                self.state = STATE_DRIVE
                omega = 0.0
            else:
                # Rotate BUT keep a small forward velocity to avoid full stop
                p_omega = self.KP_ANGULAR * err
                p_omega = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, p_omega))
                #cmd.angular.z = omega
                omega = 0.8 * err + 0.2 * (p_omega)
                cmd.angular.z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, omega))
                # Keep velocity just above startup threshold so scout never sees v=0
                #cmd.linear.x  = 0.05
                if cmd.linear.x < 0.06:
                    cmd.linear.x = 0.06

        if self.state == STATE_DRIVE:
            if abs(err) > self.REDRIVE_THRESH:
                self.state = STATE_ROTATE
            else:
                cmd.linear.x  = self.MAX_LINEAR
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self._publish_markers()

    def _finish(self):
        self.state = STATE_DONE
        self.cmd_pub.publish(Twist())
        self.get_logger().info(
            f'DONE! Final energy: {self.energy:.4f}  '
            f'Pose: {self.pose}'
        )
        self._publish_markers()

    # ==============================================================
    # Markers
    # ==============================================================

    def _publish_markers(self):
        ma = MarkerArray()
        ma.markers.append(self._line_strip(0, self.raw_path_world,
                                           r=0.0, g=1.0, b=0.0,
                                           w=0.05, ns='raw_path'))
        ma.markers.append(self._line_strip(1, self.pruned_path_world,
                                           r=0.0, g=0.0, b=1.0,
                                           w=0.10, ns='pruned_path'))
        if self.wp_idx < len(self.waypoints):
            wx, wy = self.waypoints[self.wp_idx]
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = 'goal'; m.id = 2
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 1.0; m.color.a = 1.0
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def _line_strip(self, mid, points, r, g, b, w, ns):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = ns; m.id = mid
        m.type   = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = w
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        m.pose.orientation.w = 1.0
        for (wx, wy) in points:
            p = Point(); p.x = wx; p.y = wy; p.z = 0.05
            m.points.append(p)
        return m

    # ==============================================================
    # Helpers
    # ==============================================================

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
