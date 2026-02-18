#!/usr/bin/env python3
"""
order_publisher.py
==================
Interactive CLI to test all 7 butler robot milestones.

Usage:
  ros2 run butler_robot order_publisher

Author: Arunesh
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading


class OrderPublisher(Node):

    MENU = """
╔════════════════════════════════════════════════════════╗
║          BUTLER ROBOT — Order & Control Console        ║
╠════════════════════════════════════════════════════════╣
║  SEND ORDERS                                           ║
║   1   → Order for table1 only                         ║
║   2   → Order for table2 only                         ║
║   3   → Order for table3 only                         ║
║   m   → Multi-order: table1, table2, table3           ║
║                                                        ║
║  CONFIRMATION  (press when robot arrives)             ║
║   c   → Confirm (at kitchen OR at table)              ║
║                                                        ║
║  CANCEL                                                ║
║   x1  → Cancel table1                                 ║
║   x2  → Cancel table2                                 ║
║   x3  → Cancel table3                                 ║
║   xa  → Cancel entire mission                         ║
║                                                        ║
║  MILESTONE QUICK-TEST GUIDES                          ║
║   ms1 → Guide: Milestone 1 (basic delivery)           ║
║   ms2 → Guide: Milestone 2 (timeout)                  ║
║   ms3 → Guide: Milestone 3 (split timeout)            ║
║   ms4 → Guide: Milestone 4 (cancel)                   ║
║   ms5 → Guide: Milestone 5 (multi-table)              ║
║   ms6 → Guide: Milestone 6 (multi timeout)            ║
║   ms7 → Guide: Milestone 7 (multi cancel)             ║
║                                                        ║
║   q   → Quit                                          ║
╚════════════════════════════════════════════════════════╝
"""

    GUIDES = {
        'ms1': """
── Milestone 1: Basic single delivery ──────────────────
  1. Type '1' → sends order for table1
  2. Type 'c' → confirm when robot reaches kitchen
  3. Type 'c' → confirm when robot reaches table1
  Expected: home → kitchen → table1 → home
  (Confirmation is optional for milestone 1)
""",
        'ms2': """
── Milestone 2: Timeout at any stop ────────────────────
  1. Type '1' → sends order for table1
  2. DO NOT type 'c' — just wait 15 seconds
  Expected: robot waits at kitchen → timeout → home
""",
        'ms3': """
── Milestone 3: Split timeout scenarios ────────────────
  3a. Kitchen timeout:
      Type '1', then wait 15s without confirming
      Expected: home → kitchen → TIMEOUT → home

  3b. Table timeout (food collected, then no one at table):
      Type '1', type 'c' (confirm kitchen), then wait 15s
      Expected: home → kitchen → table1 → TIMEOUT → kitchen → home
""",
        'ms4': """
── Milestone 4: Cancel during navigation ───────────────
  Cancel going to kitchen:
      Type '1', then immediately type 'xa'
      Expected: home → (cancel) → home

  Cancel going to table:
      Type '1', type 'c' (confirm kitchen),
      then quickly type 'x1' before robot reaches table1
      Expected: home → kitchen → (cancel to table) → kitchen → home
""",
        'ms5': """
── Milestone 5: Multi-table all confirmed ──────────────
  1. Type 'm'
  2. Type 'c' at kitchen
  3. Type 'c' at table1
  4. Type 'c' at table2
  5. Type 'c' at table3
  Expected: home → kitchen → t1 → t2 → t3 → home
""",
        'ms6': """
── Milestone 6: Multi-table with timeout ───────────────
  1. Type 'm'
  2. Type 'c' at kitchen
  3. DO NOT confirm at table1 → wait 15s (timeout)
  4. Type 'c' at table2
  5. Type 'c' at table3
  Expected: home → kitchen → t1(TIMEOUT) → t2 → t3 → kitchen → home
""",
        'ms7': """
── Milestone 7: Multi-table with cancel ────────────────
  1. Type 'm'
  2. Type 'c' at kitchen
  3. Type 'c' at table1
  4. Type 'x2' to cancel table2 (before or after robot arrives at t2)
  5. Type 'c' at table3
  Expected: home → kitchen → t1 → t2(SKIP) → t3 → kitchen → home
""",
    }

    def __init__(self):
        super().__init__('order_publisher')

        self._order_pub   = self.create_publisher(String, '/butler/orders',  10)
        self._confirm_pub = self.create_publisher(String, '/butler/confirm', 10)
        self._cancel_pub  = self.create_publisher(String, '/butler/cancel',  10)

        self._status_sub = self.create_subscription(
            String, '/butler/status', self._on_status, 10)

        self._last_status = ''

    def _on_status(self, msg: String):
        if msg.data != self._last_status:
            self._last_status = msg.data
            print(f'\n  🤖 Robot status → {msg.data}')

    def _send_order(self, tables: list):
        msg      = String()
        msg.data = json.dumps({'tables': tables})
        self._order_pub.publish(msg)
        print(f'  📦 Order sent → {tables}')

    def _send_confirm(self):
        msg      = String()
        msg.data = 'confirmed'
        self._confirm_pub.publish(msg)
        print('  ✅ Confirmation sent')

    def _send_cancel(self, table: str):
        msg      = String()
        msg.data = json.dumps({'table': table})
        self._cancel_pub.publish(msg)
        print(f'  🚫 Cancel sent → {table}')

    def run(self):
        print(self.MENU)
        while rclpy.ok():
            try:
                cmd = input('cmd> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if   cmd == 'q':   break
            elif cmd == '1':   self._send_order(['table1'])
            elif cmd == '2':   self._send_order(['table2'])
            elif cmd == '3':   self._send_order(['table3'])
            elif cmd == 'm':   self._send_order(['table1', 'table2', 'table3'])
            elif cmd == 'c':   self._send_confirm()
            elif cmd == 'x1':  self._send_cancel('table1')
            elif cmd == 'x2':  self._send_cancel('table2')
            elif cmd == 'x3':  self._send_cancel('table3')
            elif cmd == 'xa':  self._send_cancel('all')
            elif cmd in self.GUIDES:
                print(self.GUIDES[cmd])
            else:
                print(f'  ❓ Unknown command: "{cmd}"  (type ms1..ms7 for guides)')

        print('Goodbye!')


def main(args=None):
    rclpy.init(args=args)
    node = OrderPublisher()

    # Spin in background so status updates arrive while waiting for input
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Only shutdown if context is still valid
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
