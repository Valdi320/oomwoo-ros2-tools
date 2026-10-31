# Copyright 2026 Valdi320
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""DockDrive input freshness with real ROS2 clocks/messages; no robot or simulator."""

import math
import time
import unittest

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from oomwoo_dock import dock_harness as harness
from oomwoo_dock.dock_detector_node import DockDetector
from oomwoo_dock.dock_drive_node import DockDrive
from oomwoo_dock.dock_template import inv, mul, wrap
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time


class DockPoseFreshnessTest(unittest.TestCase):
    """Check source-time admission on the native DockDrive node."""

    @classmethod
    def setUpClass(cls):
        """Initialize an isolated ROS context with motor output remapped."""
        rclpy.init(args=['--ros-args', '-p', 'auto_start:=false',
                         '-p', 'exit_when_done:=false',
                         '-r', '__ns:=/dock_freshness_test',
                         '-r', '~/dock_pose:=/dock_freshness_test/shared_pose',
                         '-r', 'cmd_vel:=/dock_freshness_test/cmd_vel'])

    @classmethod
    def tearDownClass(cls):
        """Close the test ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create the actual driver with automatic motion disabled."""
        self.node = DockDrive()

    def tearDown(self):
        """Destroy the native node and its middleware endpoints."""
        self.node.destroy_node()

    def pose(self, age=0.01, frame='base_scan'):
        """Build a scan-frame observation with its original measurement time."""
        msg = PoseStamped()
        ns = self.node.get_clock().now().nanoseconds - round(age * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        msg.header.frame_id = frame
        msg.pose.position.x = 0.65
        msg.pose.orientation.w = 1.0
        return msg

    def assert_holds(self):
        """Check that an active approach emits a zero command."""
        self.node.enabled = True
        self.node.state = 'GOTO'
        self.node._on_odom(self.odometry(age=0.0, velocity=0.0))
        self.node._tick()
        self.assertEqual(self.node.cmd.linear.x, 0.0)
        self.assertEqual(self.node.cmd.angular.z, 0.0)

    def test_stale_pose_cannot_start_approach(self):
        """Hold when the only received pose is already too old."""
        self.node._on_pose(self.pose(age=10.0))
        self.assert_holds()
        self.assertEqual(self.node.confirms, 0)

    def test_future_pose_cannot_start_approach(self):
        """Reject a pose measured in the future."""
        self.node._on_pose(self.pose(age=-10.0))
        self.assert_holds()

    def test_wrong_frame_cannot_start_approach(self):
        """Reject map coordinates where a scan-frame pose is required."""
        self.node._on_pose(self.pose(frame='map'))
        self.assert_holds()

    def test_duplicate_does_not_count_as_second_confirmation(self):
        """Require distinct measurements for repeated confirmation."""
        msg = self.pose()
        self.node._on_pose(msg)
        self.node._on_pose(msg)
        self.assertEqual(self.node.confirms, 1)

    def test_older_pose_does_not_replace_newer_estimate(self):
        """Preserve the current estimate on reordered delivery."""
        self.node._on_pose(self.pose())
        estimate = self.node.est
        older = self.pose(age=0.5)
        older.pose.position.x = 1.25
        self.node._on_pose(older)
        self.assertEqual(self.node.est, estimate)
        self.assertEqual(self.node.confirms, 1)

    def test_nonfinite_pose_is_not_an_accepted_fix(self):
        """Reject a nonfinite detector position."""
        msg = self.pose()
        msg.pose.position.x = math.nan
        self.node._on_pose(msg)
        self.assertIsNone(self.node.est)
        self.assertEqual(self.node.confirms, 0)

    def test_nonunit_rotation_is_not_an_accepted_fix(self):
        """Reject a missing orientation quaternion."""
        msg = self.pose()
        msg.pose.orientation.w = 0.0
        self.node._on_pose(msg)
        self.assert_holds()

    def test_source_time_is_preserved(self):
        """Retain observation age rather than its arrival time."""
        msg = self.pose(age=0.5)
        self.node._on_pose(msg)
        source = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.assertEqual(self.node.t_fix.nanoseconds, source)

    def test_valid_pose_and_odometry_allow_approach(self):
        """Keep ordinary approach behavior with fresh pose and wheel feedback."""
        self.node._on_pose(self.pose())
        self.node.enabled = True
        self.node.state = 'GOTO'
        self.node._on_odom(self.odometry(age=0.0, velocity=0.0))
        self.node._tick()
        self.assertGreater(self.node.cmd.linear.x, 0.0)
        self.assertEqual(self.node.confirms, 1)

    def test_dds_delivery_preserves_stale_and_fresh_behavior(self):
        """Exercise the real subscription over DDS without hardware or Gazebo."""
        topic = self.node.resolve_topic_name('~/dock_pose')
        subscription = next(s for s in self.node.subscriptions if s.topic_name == topic)
        original = subscription.callback
        received = []

        def record(msg):
            """Record delivery before invoking the unmodified input callback."""
            received.append(msg.header.stamp.sec)
            original(msg)

        subscription.callback = record
        publisher = self.node.create_publisher(PoseStamped, '~/dock_pose', 10)
        deadline = time.monotonic() + 3.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertGreater(publisher.get_subscription_count(), 0)
        self.node.enabled = True
        self.node.state = 'GOTO'
        publisher.publish(self.pose(age=10.0))
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(received, 'stale message must actually reach the subscription')
        self.assert_holds()
        self.assertEqual(self.node.confirms, 0)
        publisher.publish(self.pose())
        while len(received) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(len(received), 2)
        self.node._on_odom(self.odometry(age=0.0, velocity=0.0))
        self.node._tick()
        self.assertGreater(self.node.cmd.linear.x, 0.0)

    def odometry(self, age=0.01, velocity=0.1):
        """Build native body velocity with a source observation timestamp."""
        msg = Odometry()
        ns = self.node.get_clock().now().nanoseconds - round(age * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.twist.twist.linear.x = velocity
        return msg

    def test_nonfinite_odometry_cannot_corrupt_estimate(self):
        """Keep a valid detector estimate finite after a bad velocity sample."""
        self.node._on_pose(self.pose())
        self.node._on_odom(self.odometry())
        self.node._on_odom(self.odometry(age=0.0, velocity=math.nan))
        self.assertTrue(all(math.isfinite(v) for v in self.node.est))

    def test_stale_odometry_does_not_replace_source_time(self):
        """Refuse stale body motion instead of rejuvenating it on arrival."""
        self.node._on_odom(self.odometry(age=10.0))
        self.assertIsNone(self.node.odom)

    def test_duplicate_odometry_does_not_extrapolate_twice(self):
        """Do not carry the estimate twice using a repeated measurement."""
        self.node._on_pose(self.pose(age=0.2))
        msg = self.odometry(age=0.05)
        self.node._on_odom(msg)
        estimate = self.node.est
        self.node._on_odom(msg)
        self.assertEqual(self.node.est, estimate)

    def test_wrong_odometry_frame_is_not_used(self):
        """Reject a twist whose body-reference contract does not match."""
        msg = self.odometry()
        msg.child_frame_id = 'map'
        self.node._on_odom(msg)
        self.assertIsNone(self.node.odom)

    def test_odometry_uses_source_interval_after_latest_fix(self):
        """Use the measured interval after a dock fix instead of callback spacing."""
        msg = self.pose(age=0.2)
        source = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.node._on_pose(msg)
        estimate = self.node.est
        odom = self.odometry()
        ns = source + 100_000_000
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        self.node._on_odom(odom)
        self.assertAlmostEqual(self.node.est[0], estimate[0] - 0.01, places=6)
        self.assertAlmostEqual(self.node.est[1], estimate[1], places=6)

    def test_unobserved_motion_gap_holds_until_new_detector_fix(self):
        """Hold rather than extrapolate over an unobserved odometry interval."""
        self.node._on_pose(self.pose(age=0.7))
        self.node._on_odom(self.odometry())
        self.assertIsNone(self.node.est)
        self.assert_holds()
        self.node._on_pose(self.pose())
        self.node._on_odom(self.odometry(age=0.0, velocity=0.0))
        self.node._tick()
        self.assertGreater(self.node.cmd.linear.x, 0.0)

    def test_delayed_fix_does_not_drop_already_received_motion_interval(self):
        """Carry a delayed fix through the entire known constant-speed interval."""
        msg = self.pose(age=0.2)
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        earlier = self.odometry()
        earlier.header.stamp.sec, earlier.header.stamp.nanosec = divmod(
            stamp + 180_000_000, 1_000_000_000)
        self.node._on_odom(earlier)
        self.node._on_pose(msg)
        estimate = self.node.est
        later = self.odometry()
        later.header.stamp.sec, later.header.stamp.nanosec = divmod(
            stamp + 190_000_000, 1_000_000_000)
        self.node._on_odom(later)
        self.assertAlmostEqual(self.node.est[0], estimate[0] - 0.019, places=6)

    def test_detector_publisher_and_driver_preserve_measurement_contract(self):
        """Connect the actual detector publisher to the actual driver over DDS."""
        detector = DockDetector()
        received = []
        topic = self.node.resolve_topic_name('~/dock_pose')
        subscription = next(s for s in self.node.subscriptions if s.topic_name == topic)
        original = subscription.callback

        def record(msg):
            """Observe delivery while retaining the real consumer callback."""
            received.append(msg)
            original(msg)

        subscription.callback = record
        try:
            self.assertEqual(detector.pose_pub.topic_name, topic)
            deadline = time.monotonic() + 3.0
            while (detector.pose_pub.get_subscription_count() == 0
                   and time.monotonic() < deadline):
                rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertGreater(detector.pose_pub.get_subscription_count(), 0)
            self.node.enabled = True
            self.node.state = 'GOTO'
            stale = self.pose(age=10.0)
            detector._publish_pose((0.65, 0.0, 0.0), stale.header.stamp)
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertEqual(len(received), 1)
            self.assertEqual(received[0].header.frame_id, 'base_scan')
            self.assert_holds()
            self.assertEqual(self.node.confirms, 0)
            fresh = self.pose(age=0.05)
            detector._publish_pose((0.65, 0.0, 0.0), fresh.header.stamp)
            while len(received) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertEqual(len(received), 2)
            self.assertEqual(self.node.confirms, 1)
            stamp = fresh.header.stamp.sec * 1_000_000_000 + fresh.header.stamp.nanosec
            self.assertEqual(self.node.t_fix.nanoseconds, stamp)
            self.node._on_odom(self.odometry(age=0.0, velocity=0.0))
            self.node._tick()
            self.assertGreater(self.node.cmd.linear.x, 0.0)
        finally:
            detector.destroy_node()

    def test_native_driver_completes_existing_nominal_dock_model(self):
        """Drive the existing 2D dock geometry with the actual ROS controller."""
        self.node.set_parameters([Parameter('use_sim_time', value=True)])
        clock = self.node.get_clock()
        clock.set_ros_time_override(Time(seconds=10))
        body = (-0.75, 0.0, 0.0)
        dt = 0.05
        previous = (0.0, 0.0)
        phases = set()
        self.node.enabled = True
        self.node._set_state('SEARCH')
        for step in range(1800):
            ns = 10_000_000_000 + step * 50_000_000
            clock.set_ros_time_override(Time(nanoseconds=ns))
            odom = self.odometry(age=0.0, velocity=previous[0])
            odom.twist.twist.angular.z = previous[1]
            self.node._on_odom(odom)
            if step % 10 == 0:
                # Perfect synthetic detector fix at 2 Hz. Reuse the original
                # harness frame algebra and dock geometry, not its controller.
                dock_in_scan = mul(inv(harness.T_BL), inv(body))
                msg = self.pose(age=0.0)
                msg.pose.position.x, msg.pose.position.y = dock_in_scan[:2]
                msg.pose.orientation.z = math.sin(dock_in_scan[2] / 2.0)
                msg.pose.orientation.w = math.cos(dock_in_scan[2] / 2.0)
                self.node._on_pose(msg)
            self.node._tick()
            phases.add(self.node.state)
            velocity = (self.node.cmd.linear.x, self.node.cmd.angular.z)
            self.assertTrue(all(math.isfinite(v) for v in velocity))
            self.assertLessEqual(abs(velocity[0]), 0.14)
            self.assertLessEqual(abs(velocity[1]), 0.8)
            body = mul(body, (velocity[0] * dt, 0.0, velocity[1] * dt))
            self.assertFalse(harness.dock_contact(body), 'model body contacted the dock')
            self.assertLessEqual(body[0] + harness.BODY_RADIUS_M,
                                 harness.BAY_DEPTH_M + harness.OVERSHOOT_M)
            if self.node.state == 'DONE':
                break
            previous = velocity
        self.assertEqual(self.node.state, 'DONE')
        self.assertTrue(harness.seated(body))
        self.assertLess(abs(body[1]), harness.BAY_HALF_M - harness.BODY_RADIUS_M)
        self.assertLess(abs(wrap(body[2] - math.pi)), math.radians(5.0))
        self.assertTrue({'GOTO', 'TURN', 'BACK', 'DONE'} <= phases)
        self.assertEqual((self.node.cmd.linear.x, self.node.cmd.angular.z), (0.0, 0.0))

    def test_odometry_loss_with_delayed_fixes_does_not_shove_dock(self):
        """Respect the existing overshoot limit when wheel feedback disappears."""
        self.node.set_parameters([Parameter('use_sim_time', value=True)])
        clock = self.node.get_clock()
        clock.set_ros_time_override(Time(seconds=10))
        body = (0.200 - harness.BODY_RADIUS_M, 0.0, math.pi)
        history = {i: (body[0] + i * 0.003, 0.0, math.pi) for i in (-3, -2, -1)}
        self.node.enabled = True
        self.node._set_state('BACK')
        self.node._on_odom(self.odometry(age=0.0, velocity=-0.06))
        for step in range(40):
            clock.set_ros_time_override(Time(nanoseconds=10_000_000_000 + step * 50_000_000))
            history[step] = body
            if step % 10 == 0:
                dock_in_scan = mul(inv(harness.T_BL), inv(history[step - 3]))
                msg = self.pose(age=0.15)
                msg.pose.position.x, msg.pose.position.y = dock_in_scan[:2]
                msg.pose.orientation.z = math.sin(dock_in_scan[2] / 2.0)
                msg.pose.orientation.w = math.cos(dock_in_scan[2] / 2.0)
                self.node._on_pose(msg)
            # Native input contract: poses at2Hz with150ms delivery delay;
            # no wheel sample after the first one. Reuse harness overshoot.
            self.node._tick()
            if step == 0:
                self.assertEqual(self.node.state, 'BACK')
                self.assertLess(self.node.cmd.linear.x, 0.0)
            body = mul(body, (self.node.cmd.linear.x * 0.05,
                              0.0, self.node.cmd.angular.z * 0.05))
            self.assertFalse(harness.dock_contact(body))
            self.assertLessEqual(body[0] + harness.BODY_RADIUS_M,
                                 harness.BAY_DEPTH_M + harness.OVERSHOOT_M,
                                 'wheel feedback loss allowed shoving the dock')
        self.assertEqual((self.node.cmd.linear.x, self.node.cmd.angular.z), (0.0, 0.0))
        clock.set_ros_time_override(Time(seconds=12))
        self.node._on_odom(self.odometry(age=0.0, velocity=0.0))
        dock_in_scan = mul(inv(harness.T_BL), inv(body))
        msg = self.pose(age=0.0)
        msg.pose.position.x, msg.pose.position.y = dock_in_scan[:2]
        msg.pose.orientation.z = math.sin(dock_in_scan[2] / 2.0)
        msg.pose.orientation.w = math.cos(dock_in_scan[2] / 2.0)
        self.node._on_pose(msg)
        self.node._tick()
        self.assertLess(self.node.cmd.linear.x, 0.0, 'fresh feedback must permit recovery')

    def test_fresh_pose_without_wheel_feedback_holds(self):
        """Require wheel feedback even when the detector fix is fresh."""
        self.node._on_pose(self.pose())
        self.node.enabled = True
        self.node.state = 'GOTO'
        self.node._tick()
        self.assertEqual((self.node.cmd.linear.x, self.node.cmd.angular.z), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
