"""Stop Nav2 before dropping its goal handle, then clear transient navigation state."""
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap


class NavigationStop:
    def __init__(self, nav, velocity):
        self.nav = nav
        self.velocity = velocity
        self.smoother_input = nav.create_publisher(Twist, "cmd_vel_nav", 10)

    def brake(self):
        self.smoother_input.publish(Twist())
        self.velocity.publish(Twist())

    def wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            self.brake()
            rclpy.spin_once(self.nav, timeout_sec=.05)
        return future.done()

    def stop(self, timeout=5.0):
        self.brake()
        result = self.nav.result_future
        if result is not None and not result.done():
            # cancelTask() waits for an acknowledgement without a timeout and
            # does not wait for FollowPath / a recovery to actually terminate.
            cancel = self.nav.goal_handle.cancel_goal_async()
            if not self.wait(cancel, timeout):
                raise RuntimeError("Nav2 cancellation acknowledgement timed out")
            cancel.result()  # Propagate an action transport error.
            if not self.wait(result, timeout):
                raise RuntimeError("Nav2 did not finish the canceled goal")
            result.result()
        self.nav.goal_handle = None
        self.nav.result_future = None
        self.nav.feedback = None
        self.nav.status = None

        # Costmap clearing does not reset AMCL or alter the robot's pose.
        for client in (self.nav.clear_costmap_local_srv, self.nav.clear_costmap_global_srv):
            if not client.wait_for_service(timeout_sec=.5):
                self.nav.get_logger().warning(f"Stop: cleanup service unavailable: {client.srv_name}")
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            if not self.wait(future, 2.0):
                client.remove_pending_request(future)
                future.cancel()
                self.nav.get_logger().warning(f"Stop: cleanup timed out: {client.srv_name}")
            else:
                future.result()
        # Flush the velocity smoother's previous nonzero input after canceling.
        deadline = time.monotonic() + .3
        while rclpy.ok() and time.monotonic() < deadline:
            self.brake()
            rclpy.spin_once(self.nav, timeout_sec=.05)
        self.brake()
