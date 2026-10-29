#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
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
"""
Autonomous coverage-cleaning regression runner (headless CLI).

Observes the coverage_meter while the coverage_planner sweeps the map, then
scores the run against the cleaning-jobs acceptance metrics:

    * coverage   >= COVERAGE_TARGET   (default 0.90)
    * efficiency >= EFFICIENCY_TARGET (default 0.80)

Reaching the coverage target is the PASS GATE, not a stop condition: the sweep
runs to completion so the coverage number is uncapped. The run ends when the
planner reports the sweep done (cleaning_active latched False -> sweep_complete),
or coverage plateaus for PLATEAU_S of sim time, or MAX_SIM_TIME is hit; two
wall-clock backstops (wall_timeout, clock_dead) also bound it in real time so a
sim that never starts or freezes can't hang the run. Emits a machine-parseable
COVERAGE_SUMMARY line, writes a JSON report, exits 0 iff the run passes.

Exit codes: 0 = pass, 1 = target missed (a real coverage result), 2 = the
measurement is invalid (sim_unstable = ground-truth pose teleported, or
clock_dead = the sim clock stalled). A wall_timeout is exit 1 with
measurement_valid=true: the coverage so far is real, the host was just slow.
Intended to be launched alongside coverage_regression.launch.py.
"""

import json
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool, Float32


class CoverageRunner(Node):
    def __init__(self) -> None:
        super().__init__('coverage_regression_runner')
        self.declare_parameter('coverage_target', 0.90)
        self.declare_parameter('efficiency_target', 0.80)
        # generous: run-to-completion sweeps outlast the old stop-at-90% runs
        self.declare_parameter('max_sim_time_s', 3600.0)
        # wall-clock backstops: with use_sim_time, every other exit condition
        # depends on /clock — a sim that never starts (or freezes mid-run)
        # would otherwise hang this "CI-friendly" runner forever.
        self.declare_parameter('max_wall_time_s', 5400.0)
        self.declare_parameter('clock_dead_wall_s', 120.0)
        self.declare_parameter('plateau_s', 180.0)
        self.declare_parameter('plateau_eps', 0.005)
        self.declare_parameter('report_path', '/root/coverage_report.json')

        self.declare_parameter('motion_audit', os.environ.get('OOMWOO_MOTION_AUDIT') == '1')
        self.motion_audit = self.get_parameter('motion_audit').value
        self.motion_samples = {'status': [], 'feedback': []}
        self.motion_graph = {}
        self._motion_fault = False
        self.motion_samples['phase'] = []
        self._motion_graph_wall = 0.0
        self._motion_status_count = 0
        self.declare_parameter('sim_battery_fraction',
                               float(os.environ.get('OOMWOO_SIM_BATTERY_FRACTION', '-1')))
        fraction = self.get_parameter('sim_battery_fraction').value
        self._sim_battery_pub = None
        self._sim_battery_last = 0.0
        if fraction != -1:
            if not 0.0 <= fraction <= 1.0:
                raise ValueError('simulation battery fixture requires fraction0..1 or -1 disabled')
            from sensor_msgs.msg import BatteryState
            self._sim_battery_message = BatteryState()
            self._sim_battery_message.percentage = fraction
            self._sim_battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        if self.motion_audit:
            from action_msgs.msg import GoalStatusArray
            from nav2_msgs.action import NavigateToPose
            from std_msgs.msg import String
            self.create_subscription(String, '/coverage_planner/motion_state',
                                     self._audit_phase, 10)
            self.create_subscription(
                GoalStatusArray, '/navigate_to_pose/_action/status',
                self._audit_status, 10)
            self.create_subscription(
                NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
                self._audit_feedback, 10)
        self.cov_target = self.get_parameter('coverage_target').value
        self.eff_target = self.get_parameter('efficiency_target').value
        self.max_t = self.get_parameter('max_sim_time_s').value
        self.max_wall = self.get_parameter('max_wall_time_s').value
        self.clock_dead_s = self.get_parameter('clock_dead_wall_s').value
        self.plateau_s = self.get_parameter('plateau_s').value
        self.plateau_eps = self.get_parameter('plateau_eps').value
        self.report_path = self.get_parameter('report_path').value

        self.coverage = 0.0
        self.efficiency = 0.0
        self.revisit_ratio = 0.0
        self.best = 0.0
        self.last_gain_sim_t = None
        self.sim_unstable = False
        self.measurement_invalid = False
        self.target_crossed = False
        self.eff_at_target = 0.0
        self.t_to_target = 0.0

        # sim_unstable is published latched; a latched sub can't miss it even if
        # the flag was raised before this runner finished starting up.
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float32, '/coverage_meter/ratio', self._on_cov, 10)
        self.create_subscription(
            Float32, '/coverage_meter/efficiency', self._on_eff, 10)
        self.create_subscription(
            Float32, '/coverage_meter/revisit_ratio', self._on_revisit, 10)
        self.create_subscription(
            Bool, '/coverage_meter/sim_unstable', self._on_unstable, latched)
        # planner publishes cleaning_active False when the sweep + gap-fill
        # passes are exhausted — the honest end-of-job signal (latched)
        self.sweep_started = False
        self.sweep_complete = False
        self.create_subscription(
            Bool, '/coverage_planner/cleaning_active', self._on_active, latched)

    def _audit_phase(self, msg):
        if msg.data == 'FAULT':
            self._motion_fault = True
        phases = self.motion_samples['phase']
        if not phases or phases[-1]['phase'] != msg.data:
            if len(phases) >= 128:
                phases.pop(0)
            phases.append({'phase': msg.data, 'wall': time.monotonic(), 'sim': self._sim_now()})

    def _audit_status(self, msg):
        self._motion_status_count += 1
        if len(self.motion_samples['status']) >= 256:
            self.motion_samples['status'].pop(0)
        self.motion_samples['status'].append({
            'wall': time.monotonic(), 'sim': self._sim_now(),
            'states': [(bytes(s.goal_info.goal_id.uuid).hex(), s.status)
                       for s in msg.status_list]})

    def _audit_feedback(self, msg):
        # Retain bounded aggregate cadence, not an unbounded pose trace.
        samples = self.motion_samples['feedback']
        now = time.monotonic()
        if not samples:
            samples.append({'first_wall': now, 'last_wall': now, 'count': 1,
                            'max_gap_wall': 0.0})
        else:
            summary = samples[0]
            summary['max_gap_wall'] = max(summary['max_gap_wall'], now - summary['last_wall'])
            summary['last_wall'] = now
            summary['count'] += 1

    def _audit_graph(self):
        now = time.monotonic()
        if now - self._motion_graph_wall < 0.5:
            return
        self._motion_graph_wall = now
        for topic in ('/cmd_vel', '/cmd_vel_nav', '/navigate_to_pose/_action/status',
                      '/valdi/nav2_cmd_vel', '/valdi/sweep_cmd_vel',
                      '/valdi/dock_drive_cmd_vel', '/valdi/lab_safe_cmd_vel_output'):
            publishers = self.get_publishers_info_by_topic(topic)
            snapshot = sorted((p.node_name, p.node_namespace, p.topic_type,
                               bytes(p.endpoint_gid).hex()) for p in publishers)
            self.motion_graph[topic] = snapshot
        self.motion_graph['safe_output_subscribers'] = sorted(
            (p.node_name, p.node_namespace, p.topic_type)
            for p in self.get_subscriptions_info_by_topic('/valdi/lab_safe_cmd_vel_output'))

    def _on_active(self, msg):
        if msg.data:
            self.sweep_started = True
        elif self.sweep_started:
            self.sweep_complete = True

    def _on_cov(self, msg):
        self.coverage = float(msg.data)

    def _on_eff(self, msg):
        self.efficiency = float(msg.data)

    def _on_revisit(self, msg):
        self.revisit_ratio = float(msg.data)

    def _on_unstable(self, msg):
        self.sim_unstable = self.sim_unstable or bool(msg.data)

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def run(self) -> int:
        self.get_logger().info(
            f'watching coverage; target={self.cov_target:.0%} '
            f'efficiency>={self.eff_target:.0%}')
        # Bound startup too: a missing clock/first coverage sample must not
        # add an unaccounted five minutes before either wall-clock backstop.
        start_sim = self._sim_now()
        self.last_gain_sim_t = start_sim
        measuring = False

        reason = 'max_time'
        wall_start = time.monotonic()
        last_sim = self._sim_now()
        last_sim_advance_wall = wall_start
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            # wall-clock backstops: a dead/frozen sim clock stops every
            # sim-time exit condition, so bound the run in real time too
            now_wall = time.monotonic()
            if getattr(self, '_sim_battery_pub', None) is not None \
                    and now_wall - self._sim_battery_last >= 0.5:
                self._sim_battery_pub.publish(self._sim_battery_message)
                self._sim_battery_last = now_wall
            if getattr(self, 'motion_audit', False):
                self._audit_graph()
            if self._sim_now() > last_sim:
                last_sim = self._sim_now()
                last_sim_advance_wall = now_wall
            elif now_wall - last_sim_advance_wall >= self.clock_dead_s:
                # the sim clock stopped advancing — the measurement IS invalid
                reason = 'clock_dead'
                self.measurement_invalid = True
                break
            if now_wall - wall_start >= self.max_wall:
                # ran out of real time. The coverage number so far is VALID
                # (nothing teleported) — this is a slow host / incomplete run,
                # NOT sim instability. Report it as its own reason.
                reason = 'wall_timeout'
                break
            if getattr(self, '_motion_fault', False):
                reason = 'motion_fault'
                break
            if self.sim_unstable:
                # no point measuring further — the numbers are already invalid
                break
            if not measuring:
                # Keep startup out of the simulated sweep/plateau duration,
                # while both wall-clock safeguards above remain in force.
                if self.coverage == 0.0 and not self.sweep_started:
                    continue
                start_sim = self._sim_now()
                self.last_gain_sim_t = start_sim
                measuring = True
            sim_t = self._sim_now() - start_sim
            if self.coverage > self.best + self.plateau_eps:
                self.best = self.coverage
                self.last_gain_sim_t = self._sim_now()
            # The contract's two gates are ONE condition: reach >=90% coverage
            # at >=80% efficiency. Efficiency is judged the moment coverage
            # first crosses the target; the sweep then continues to completion
            # so the coverage number is uncapped. Chasing the final few percent
            # costs extra path (diminishing returns) — that thoroughness tax is
            # reported (efficiency_final) but doesn't retroactively fail a
            # gate that was already met.
            if not self.target_crossed and self.coverage >= self.cov_target:
                self.target_crossed = True
                self.eff_at_target = self.efficiency
                self.t_to_target = sim_t
            # No break at coverage_target: the target is the PASS GATE, not a
            # stop switch. The run ends when the sweep genuinely finishes (or
            # stalls), so the reported number is uncapped and informative —
            # 97% and 90.2% no longer read the same.
            if self.sweep_complete:
                reason = 'sweep_complete'
                # let the meter's last 1 Hz update land before scoring
                grace_end = min(time.monotonic() + 3.0,
                                wall_start + self.max_wall)
                while rclpy.ok() and time.monotonic() < grace_end:
                    rclpy.spin_once(self, timeout_sec=0.2)
                # a late coverage update during the grace can cross the target;
                # re-evaluate so eff_at_target isn't left null on a passing run
                if not self.target_crossed and self.coverage >= self.cov_target:
                    self.target_crossed = True
                    self.eff_at_target = self.efficiency
                    self.t_to_target = self._sim_now() - start_sim
                break
            if self._sim_now() - self.last_gain_sim_t >= self.plateau_s:
                reason = 'plateau'
                break
            if sim_t >= self.max_t:
                reason = 'max_time'
                break

        if self.sim_unstable:
            reason = 'sim_unstable'
        # a dead clock or a pose teleport both mean the numbers can't be
        # trusted; a wall timeout does not (the coverage so far is real)
        invalid = self.sim_unstable or getattr(self, 'measurement_invalid', False)
        # gate efficiency = at the target crossing (the contract condition);
        # if the target was never crossed the final value is all there is.
        gate_eff = self.eff_at_target if self.target_crossed else self.efficiency
        result = {
            'coverage': round(self.coverage, 4),          # final, uncapped
            'coverage_target': self.cov_target,
            'target_crossed': self.target_crossed,
            # null when the target was never crossed — the gate then falls
            # back to the final figure, and there is no crossing to time
            'efficiency_at_target': (round(gate_eff, 4)
                                     if self.target_crossed else None),
            'efficiency_final': round(self.efficiency, 4),  # incl. thoroughness tax
            'revisit_ratio': round(self.revisit_ratio, 4),
            'efficiency_target': self.eff_target,
            'time_to_target_s': (round(self.t_to_target, 1)
                                 if self.target_crossed else None),
            'end_reason': reason,
            'sim_unstable': self.sim_unstable,
            'measurement_valid': not invalid,
            'pass': bool(not invalid and reason != 'motion_fault'
                         and self.coverage >= self.cov_target
                         and gate_eff >= self.eff_target),
        }
        if getattr(self, 'motion_audit', False):
            result['motion_audit'] = {
                'graph': self.motion_graph, 'samples': self.motion_samples,
                'status_packets_total': self._motion_status_count}
        try:
            with open(self.report_path, 'w') as f:
                json.dump(result, f, indent=2)
        except OSError as e:
            self.get_logger().warn(f'could not write report: {e}')
        if invalid:
            # Invalid measurement, NOT a behavior failure: distinct exit code
            # so CI/users don't read a physics glitch as a coverage regression.
            why = ('sim unstable: ground-truth pose teleported'
                   if self.sim_unstable else f'sim clock stalled ({reason})')
            self.get_logger().error(
                f'COVERAGE_SUMMARY MEASUREMENT INVALID ({why}). This host '
                'cannot run the sim faithfully — use a native x86-64 Linux '
                'machine or CI runner. '
                f'coverage={result["coverage"]:.4f} (informational only)')
            return 2
        eat = result['efficiency_at_target']
        self.get_logger().info(
            f'COVERAGE_SUMMARY coverage={result["coverage"]:.4f} '
            f'eff_at_target={f"{eat:.4f}" if eat is not None else "n/a"} '
            f'eff_final={result["efficiency_final"]:.4f} '
            f'reason={reason} pass={result["pass"]}')
        return 0 if result['pass'] else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageRunner()
    code = 1
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
