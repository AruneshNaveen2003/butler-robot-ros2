#!/usr/bin/env python3
"""
butler_node.py
==============
Café Butler Robot — all 7 milestone scenarios.

MAP FRAME COORDINATES (verified against map bounds -6.24 to 9.91 x, -5.25 to 6.25 y):
  home    : (-0.812,  4.70)   ← robot spawn/home position
  kitchen : (-4.04,   3.99)
  table1  : (-0.7981, 1.44)
  table2  : ( 6.87,   1.54)
  table3  : ( 7.57,  -2.41)

Milestone behaviours:
  1. Single delivery, no confirmation needed  → home→kitchen→table→home
  2. Timeout at any stop                      → return home
  3a. Kitchen timeout                         → home
  3b. Table timeout (food collected)          → kitchen→home
  4. Cancel during navigation
       cancel going to kitchen               → home
       cancel going to table                 → kitchen→home
  5. Multi-table, all confirmed               → home→kitchen→t1→t2→t3→home
  6. Multi-table, table timeout               → skip table→…→kitchen→home
  7. Multi-table, table cancelled             → skip table→…→kitchen→home

Topics:
  SUB  /butler/orders   std_msgs/String  {"tables":["table1"]}
  SUB  /butler/confirm  std_msgs/String  any string
  SUB  /butler/cancel   std_msgs/String  {"table":"table2"} or {"table":"all"}
  PUB  /butler/status   std_msgs/String  current state string

Author : Arunesh
Package: turtlebot3_gazebo (placed in turtlebot3_gazebo/scripts/)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion

import threading
import math
import json
from enum import Enum, auto
from dataclasses import dataclass
from typing import List, Optional


# ──────────────────────────────────────────────────────────────────────────────
# Enumerations
# ──────────────────────────────────────────────────────────────────────────────

class NavResult(Enum):
    SUCCESS   = auto()
    FAILED    = auto()
    CANCELLED = auto()


class ConfResult(Enum):
    CONFIRMED = auto()
    TIMEOUT   = auto()
    CANCELLED = auto()


# ──────────────────────────────────────────────────────────────────────────────
# Data classes
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Waypoint:
    x:   float
    y:   float
    yaw: float = 0.0


@dataclass
class Order:
    table_id:  str
    cancelled: bool = False


# ──────────────────────────────────────────────────────────────────────────────
# Butler Node
# ──────────────────────────────────────────────────────────────────────────────

class ButlerNode(Node):
    """
    Core butler robot.
    Uses Nav2 NavigateToPose action for all movement.
    All 7 scenarios handled by one generic _mission() function.
    """

    # ── Café waypoints in MAP frame ───────────────────────────────────────────
    WAYPOINTS: dict = {
        'home'    : Waypoint(-0.812,   4.70),   # robot home / spawn position
        'kitchen' : Waypoint(-4.04,    3.99),
        'table1'  : Waypoint(-0.7981,  1.44),
        'table2'  : Waypoint( 6.87,    1.54),
        'table3'  : Waypoint( 7.57,   -2.41),
    }

    # Seconds to wait for human confirmation before timeout
    CONFIRM_TIMEOUT: float = 15.0

    def __init__(self):
        super().__init__('butler_node')

        self._cb_group = ReentrantCallbackGroup()

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, '/butler/orders',
            self._on_order, 10, callback_group=self._cb_group)

        self.create_subscription(
            String, '/butler/confirm',
            self._on_confirm, 10, callback_group=self._cb_group)

        self.create_subscription(
            String, '/butler/cancel',
            self._on_cancel, 10, callback_group=self._cb_group)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/butler/status', 10)

        # ── Thread-safe state ─────────────────────────────────────────────────
        self._busy      = False
        self._busy_lock = threading.Lock()

        # Signaling events
        self._confirm_event = threading.Event()   # set on /butler/confirm
        self._cancel_event  = threading.Event()   # set on /butler/cancel
        self._cancel_target = 'all'               # which table was cancelled
        self._nav_abort     = threading.Event()   # abort current nav goal

        # Current goal handle (for cancellation)
        self._goal_handle      = None
        self._goal_handle_lock = threading.Lock()

        # ── Wait for Nav2 ─────────────────────────────────────────────────────
        self.get_logger().info('🤖 Butler Node started — waiting for Nav2...')
        self._wait_nav2()
        self.get_logger().info('✅ Nav2 ready. Butler is ONLINE!')
        self._pub_status('IDLE')

    # ─────────────────────────────────────────────────────────────────────────
    # Startup helper
    # ─────────────────────────────────────────────────────────────────────────

    def _wait_nav2(self):
        """Block until navigate_to_pose action server is up."""
        while not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('⏳ Waiting for navigate_to_pose server...')

    # ─────────────────────────────────────────────────────────────────────────
    # ROS topic callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _on_order(self, msg: String):
        """
        Receive order and start mission thread.
        JSON: {"tables": ["table1"]}
              {"tables": ["table1", "table2", "table3"]}
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('⚠️  Robot busy — order ignored.')
                return
            self._busy = True

        try:
            data   = json.loads(msg.data)
            tables = data.get('tables', [])

            if not tables:
                self.get_logger().warn('⚠️  Order has no tables.')
                with self._busy_lock:
                    self._busy = False
                return

            orders = [Order(table_id=t) for t in tables]
            self.get_logger().info(
                f'📦 Order received → {[o.table_id for o in orders]}')

            threading.Thread(
                target=self._mission,
                args=(orders,),
                daemon=True
            ).start()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Bad order JSON: {e}')
            with self._busy_lock:
                self._busy = False

    def _on_confirm(self, msg: String):
        """Human confirmed at kitchen or table."""
        self.get_logger().info(f'✅ Confirmation received: "{msg.data}"')
        self._confirm_event.set()

    def _on_cancel(self, msg: String):
        """
        Cancel a specific table or whole mission.
        JSON: {"table": "table2"}  or  {"table": "all"}
        """
        try:
            data = json.loads(msg.data)
            self._cancel_target = data.get('table', 'all')
        except Exception:
            self._cancel_target = 'all'

        self.get_logger().info(f'🚫 Cancel received → target: {self._cancel_target}')
        self._cancel_event.set()
        self._nav_abort.set()   # abort any running navigation immediately

    # ─────────────────────────────────────────────────────────────────────────
    # Navigation
    # ─────────────────────────────────────────────────────────────────────────

    def _make_pose(self, wp: Waypoint) -> PoseStamped:
        """Convert a Waypoint to a PoseStamped in the map frame."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        pose.pose.position.z = 0.0
        qz = math.sin(wp.yaw / 2.0)
        qw = math.cos(wp.yaw / 2.0)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        return pose

    def _navigate(self, name: str) -> NavResult:
        """
        Navigate to a named waypoint.
        Blocks until SUCCESS, FAILED, or CANCELLED.
        Uses threading.Event callbacks — no polling deadlock.
        """
        wp = self.WAYPOINTS.get(name)
        if wp is None:
            self.get_logger().error(f'❌ Unknown waypoint: {name}')
            return NavResult.FAILED

        self.get_logger().info(
            f'🧭 Navigating → {name}  ({wp.x:.4f}, {wp.y:.4f})')
        self._pub_status(f'GOING_TO_{name.upper()}')
        self._nav_abort.clear()

        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(wp)

        # Send goal asynchronously
        send_future  = self._nav_client.send_goal_async(goal_msg)

        # Wait for acceptance via callback + Event
        accepted_event  = threading.Event()
        accepted_holder = [None]

        def _on_goal_response(future):
            accepted_holder[0] = future.result()
            accepted_event.set()

        send_future.add_done_callback(_on_goal_response)

        if not accepted_event.wait(timeout=10.0):
            self.get_logger().error('❌ Goal acceptance timed out.')
            return NavResult.FAILED

        handle = accepted_holder[0]
        if handle is None or not handle.accepted:
            self.get_logger().error(f'❌ Goal to "{name}" was rejected.')
            return NavResult.FAILED

        with self._goal_handle_lock:
            self._goal_handle = handle

        # Wait for result via callback + Event
        result_future  = handle.get_result_async()
        done_event     = threading.Event()
        result_holder  = [None]

        def _on_result(future):
            result_holder[0] = future.result()
            done_event.set()

        result_future.add_done_callback(_on_result)

        # Wait, but check abort flag every 100 ms
        while not done_event.wait(timeout=0.1):
            if self._nav_abort.is_set():
                self.get_logger().info(f'🛑 Aborting nav to "{name}"')
                handle.cancel_goal_async()
                done_event.wait(timeout=3.0)
                with self._goal_handle_lock:
                    self._goal_handle = None
                return NavResult.CANCELLED

        with self._goal_handle_lock:
            self._goal_handle = None

        result = result_holder[0]
        if result is None:
            return NavResult.FAILED

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ Reached "{name}"')
            return NavResult.SUCCESS
        elif result.status in (GoalStatus.STATUS_CANCELED,
                               GoalStatus.STATUS_CANCELING):
            return NavResult.CANCELLED
        else:
            self.get_logger().warn(
                f'⚠️  Nav to "{name}" ended with status {result.status}')
            return NavResult.FAILED

    # ─────────────────────────────────────────────────────────────────────────
    # Confirmation wait
    # ─────────────────────────────────────────────────────────────────────────

    def _wait_confirm(self, location: str) -> ConfResult:
        """
        Wait at a location for human confirmation.
        Returns CONFIRMED, TIMEOUT, or CANCELLED.
        """
        self._confirm_event.clear()
        self._pub_status(f'WAITING_AT_{location.upper()}')
        self.get_logger().info(
            f'⏳ Waiting {self.CONFIRM_TIMEOUT:.0f}s for confirmation at {location}...')

        confirmed = self._confirm_event.wait(timeout=self.CONFIRM_TIMEOUT)

        if self._cancel_event.is_set():
            return ConfResult.CANCELLED
        if confirmed:
            self._confirm_event.clear()
            return ConfResult.CONFIRMED
        return ConfResult.TIMEOUT

    # ─────────────────────────────────────────────────────────────────────────
    # Mission — handles all 7 milestones generically
    # ─────────────────────────────────────────────────────────────────────────

    def _mission(self, orders: List[Order]):
        """
        Single function that implements all 7 scenarios.

        Key logic:
          • multi = True  when more than one table ordered
          • undelivered   tracks if any table was skipped/timed-out
            → if True after last table, robot returns to kitchen first

        Decision at KITCHEN:
          TIMEOUT / CANCEL → go home (milestones 2, 3a, 4)

        Decision at TABLE (single):
          TIMEOUT / CANCEL → kitchen → home (milestones 3b, 4)

        Decision at TABLE (multi):
          TIMEOUT / CANCEL → skip; after last table → kitchen → home
                             (milestones 6, 7)
        """
        try:
            self._cancel_event.clear()
            self._nav_abort.clear()
            multi       = len(orders) > 1
            undelivered = False

            # ═══════════════════════════════════════════════════════════════
            # PHASE 1 — Travel to kitchen
            # ═══════════════════════════════════════════════════════════════
            self.get_logger().info('══ PHASE 1: Going to kitchen ══')

            nav = self._navigate('kitchen')

            if nav != NavResult.SUCCESS:
                # Cancelled or failed going to kitchen → straight home
                self.get_logger().info(
                    '🔁 Nav to kitchen interrupted → going home')
                self._cancel_event.clear()
                self._nav_abort.clear()
                self._go_home()
                return

            # ═══════════════════════════════════════════════════════════════
            # PHASE 2 — Wait at kitchen for food pickup
            # ═══════════════════════════════════════════════════════════════
            conf = self._wait_confirm('kitchen')

            if conf == ConfResult.TIMEOUT:
                self.get_logger().info('⏰ Kitchen timeout → going home')
                self._go_home()
                return

            if conf == ConfResult.CANCELLED:
                self.get_logger().info('🚫 Cancelled at kitchen → going home')
                self._cancel_event.clear()
                self._nav_abort.clear()
                self._go_home()
                return

            self.get_logger().info('🍽️  Food collected! Moving to table(s).')

            # ═══════════════════════════════════════════════════════════════
            # PHASE 3 — Deliver to each table
            # ═══════════════════════════════════════════════════════════════
            active_orders = [o for o in orders if not o.cancelled]

            for idx, order in enumerate(active_orders):
                table = order.table_id

                # ── Pre-arrival cancel check ──────────────────────────────
                if self._cancel_event.is_set():
                    target = self._cancel_target
                    if target == table or target == 'all':
                        order.cancelled = True
                        self._cancel_event.clear()
                        self._nav_abort.clear()
                        if not multi:
                            # Milestone 4: cancelled before reaching table
                            self.get_logger().info(
                                f'🚫 {table} cancelled before arrival → kitchen → home')
                            self._navigate('kitchen')
                            self._go_home()
                            return
                        else:
                            self.get_logger().info(f'⏭️  {table} cancelled → skip')
                            undelivered = True
                            continue

                # ── Navigate to table ────────────────────────────────────
                self.get_logger().info(f'══ Delivering to {table} ══')
                nav = self._navigate(table)

                if nav == NavResult.CANCELLED:
                    self._nav_abort.clear()
                    self._cancel_event.clear()
                    if not multi:
                        # Milestone 4: nav cancelled mid-route to table
                        self.get_logger().info(
                            f'🚫 Nav to {table} cancelled → kitchen → home')
                        self._navigate('kitchen')
                        self._go_home()
                        return
                    else:
                        self.get_logger().info(
                            f'⏭️  Nav to {table} cancelled → skip')
                        undelivered = True
                        continue

                if nav == NavResult.FAILED:
                    self.get_logger().warn(f'⚠️  Failed to reach {table} → skip')
                    undelivered = True
                    continue

                # ── Wait at table for delivery confirmation ────────────────
                conf = self._wait_confirm(table)

                if conf == ConfResult.CONFIRMED:
                    self.get_logger().info(f'✅ Delivered to {table}!')
                    # Continue to next table

                elif conf == ConfResult.TIMEOUT:
                    if not multi:
                        # Milestone 3b: table timeout → kitchen → home
                        self.get_logger().info(
                            f'⏰ {table} timeout → returning food to kitchen')
                        self._navigate('kitchen')
                        self._go_home()
                        return
                    else:
                        # Milestone 6: skip this table, continue
                        self.get_logger().info(
                            f'⏰ {table} timeout → skip, next table')
                        undelivered = True

                elif conf == ConfResult.CANCELLED:
                    self._cancel_event.clear()
                    self._nav_abort.clear()
                    if not multi:
                        # Milestone 4 (at table): → kitchen → home
                        self.get_logger().info(
                            f'🚫 {table} cancelled at delivery → kitchen → home')
                        self._navigate('kitchen')
                        self._go_home()
                        return
                    else:
                        # Milestone 7: skip this table
                        self.get_logger().info(
                            f'🚫 {table} cancelled → skip')
                        undelivered = True

            # ═══════════════════════════════════════════════════════════════
            # PHASE 4 — Post-delivery
            # ═══════════════════════════════════════════════════════════════
            # Multi-table only: if any table was skipped → return food to kitchen
            if multi and undelivered:
                self.get_logger().info(
                    '🔙 Undelivered food exists → returning to kitchen')
                self._navigate('kitchen')

            self._go_home()

        except Exception as exc:
            self.get_logger().error(f'💥 Mission error: {exc}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self._go_home()

        finally:
            with self._busy_lock:
                self._busy = False

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _go_home(self):
        """Go to home position and set IDLE."""
        self.get_logger().info('🏠 Returning home...')
        self._navigate('home')
        self._pub_status('IDLE')
        self.get_logger().info('🟢 IDLE — ready for next order.')

    def _pub_status(self, status: str):
        """Publish current robot status."""
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node     = ButlerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
