#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Headless coverage-cleaning regression, CLI, CI-friendly.
# Launches the sim + Nav2 + coverage planner/meter, runs the scoring node, and
# exits 0 iff coverage >= 90%, efficiency >= 80%, and owned processes exit cleanly.
# No GUI/display required.
#
#   RUNS=3 ./run_coverage_regression.sh    # repeat 3x and report the spread
#
# Exit codes: 0 = all runs pass, 1 = a run missed its targets,
#             2 = measurement invalid (sim unstable on this host),
#             3 = launch or owned-process failure (score remains separate).
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash
[ -f "$HOME/oomwoo-dev/install/setup.bash" ] && source "$HOME/oomwoo-dev/install/setup.bash"
# any extra CLI args (e.g. `./run_coverage_regression.sh gui:=true`) are
# forwarded to the launch, on top of whatever LAUNCH_ARGS carries
LAUNCH_ARGS="${LAUNCH_ARGS:-} $*"
# software GL is only forced headless; a gui:=true run keeps the host's GL
case " $LAUNCH_ARGS " in *" gui:=true"*|*" gui:=True"*) ;;
  *) export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe ;;
esac
# isolate DDS discovery so a co-running ROS graph can't interfere
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_LOCALHOST_ONLY=1

RUNS=${RUNS:-1}
LOG=${LOG:-/tmp/coverage_regression.log}
REPORT_DIR=$(mktemp -d /tmp/coverage_runs.XXXX)
WORST=0

for i in $(seq 1 "$RUNS"); do
  echo "[run] ($i/$RUNS) launching headless coverage stack -> $LOG.$i"
  # One launch owns both the stack and its existing scorer. Its process-exit
  # event shuts down only this launch's children; never search for other gz
  # processes (another simulation or CI job may be running on the host).
  set +e
  python3 - "$REPORT_DIR/run$i.json" "$LAUNCH_ARGS" > "$LOG.$i" 2>&1 <<'PY'
import os
import shlex
import subprocess
import sys
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

arguments = {}
for argument in shlex.split(sys.argv[2]):
    name, separator, value = argument.partition(':=')
    if not separator or not name:
        raise ValueError(f'expected name:=value launch argument: {argument!r}')
    arguments[name] = value

# The scorer owns one meter; do not launch a competing measurement node.
arguments['with_meter'] = 'false'

# A private Gazebo transport partition makes the native server-stop RPC belong
# to this launch, even when another simulation is running on the same host.
os.environ['GZ_PARTITION'] = 'oomwoo-coverage-' + uuid.uuid4().hex

scorer = Node(
    package='oomwoo_sim_support', executable='coverage_regression_runner',
    output='screen', parameters=[{
        'use_sim_time': True, 'report_path': sys.argv[1],
        'cleaning_radius': 0.20, 'robot_radius': 0.1745,
        'max_wall_time_s': float(os.environ.get('MAX_WALL_TIME_S', '5400')),
        'clock_dead_wall_s': float(os.environ.get('CLOCK_DEAD_WALL_S', '120')),
    }])
result = [3]  # no completed scorer result is an execution failure
execution_failed = [False]


def scored(event, context):
    # The existing exit observer also retains failures from the owned stack.
    # A required-server Shutdown may leave LaunchService returning zero even
    # when Gazebo exits139 after a positive stop ACK. Never hide that failure.
    if event.action is not scorer:
        # Launch sends SIGINT to its children after the scorer has finished.
        # Only that already-requested shutdown explains POSIX returncode -2.
        # Preserve every other nonzero exit, including crashes after stop ACK.
        if event.returncode == -2 and context.is_shutdown and result[0] in (0, 1, 2):
            print('COVERAGE_EXPECTED_SHUTDOWN_INTERRUPT', event.process_name,
                  event.returncode, flush=True)
        elif event.returncode != 0:
            execution_failed[0] = True
            print('COVERAGE_PROCESS_FAILURE', event.process_name,
                  event.returncode, flush=True)
        return []
    result[0] = event.returncode if event.returncode in (0, 1, 2) else 3
    # Stop the owned server through Gazebo before Launch signals its children.
    # The CLI bounds discovery/request time; this callback has a final bound too.
    # An accepted request does not prove server exit: always request the
    # existing Launch shutdown, retaining the scored result and child failures.
    native_stopped = False
    try:
        stopped = subprocess.run([
            'gz', 'service', '-s', '/server_control',
            '--reqtype', 'gz.msgs.ServerControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', 'stop: true'],
            capture_output=True, text=True, timeout=4)
        print('GZ_NATIVE_STOP', stopped.returncode, stopped.stdout.strip(),
              stopped.stderr.strip(), flush=True)
        native_stopped = stopped.returncode == 0 and stopped.stdout.strip() == 'data: true'
    except (OSError, subprocess.TimeoutExpired) as exc:
        print('GZ_NATIVE_STOP unavailable:', exc, flush=True)
    if native_stopped:
        from signal import SIGINT
        from launch.events.process import SignalProcess

        # The native RPC already starts Gazebo's stop thread. Keep Launch's
        # existing escalation timers for EVERY child, but do not send that
        # server a second stop through SIGINT. Other owned processes still
        # receive their framework signal; every nonzero failure stays visible.
        return [
            EmitEvent(event=Shutdown(reason='coverage scorer finished', due_to_sigint=True)),
            EmitEvent(event=SignalProcess(
                signal_number=SIGINT,
                process_matcher=lambda action: not (action.process_details or {}).get(
                    'name', '').startswith('gazebo-'))),
        ]
    return [EmitEvent(event=Shutdown(reason='coverage scorer finished'))]


stack = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('oomwoo_sim_support'),
        'launch', 'coverage_regression.launch.py')),
    launch_arguments=arguments.items())
service = LaunchService()
service.include_launch_description(LaunchDescription([
    RegisterEventHandler(OnProcessExit(on_exit=scored)),
    stack, scorer,
]))
launch_code = service.run()
sys.exit(3 if launch_code != 0 or execution_failed[0] else result[0])
PY
  CODE=$?
  set -e
  [ "$CODE" -gt "$WORST" ] && WORST=$CODE

  echo "[run] ($i/$RUNS) report:"
  cat "$REPORT_DIR/run$i.json" 2>/dev/null || true
  cp -f "$REPORT_DIR/run$i.json" /root/coverage_report.json 2>/dev/null || true
  echo
  echo "[run] ($i/$RUNS) exit code: $CODE"

done

if [ "$RUNS" -gt 1 ]; then
  echo "[run] ===== VARIANCE over $RUNS runs ====="
  python3 - "$REPORT_DIR" <<'PY'
import json, glob, statistics as st, sys
allr = [json.load(open(p)) for p in sorted(glob.glob(sys.argv[1] + '/run*.json'))]
# only valid measurements enter the spread; a clock_dead/unstable run's partial
# coverage is meaningless and must not pollute min/max/mean
rs = [r for r in allr if r.get('measurement_valid', True)]
invalid = len(allr) - len(rs)
for k in ('coverage', 'efficiency_at_target', 'efficiency_final'):
    v = [x for x in (r.get(k, r.get('efficiency')) for r in rs) if x is not None]
    if not v:
        continue
    print(f"  {k:11}: min={min(v):.4f} max={max(v):.4f} mean={st.mean(v):.4f}"
          + (f" stdev={st.stdev(v):.4f}" if len(v) > 1 else ""))
print(f"  score passes: {sum(r['pass'] for r in allr)}/{len(allr)}"
      f"  invalid(unstable/clock): {invalid}")
PY
fi
echo "[run] overall exit: $WORST  (0=PASS, 1=target missed — check end_reason:" \
     "wall_timeout means the host ran out of real time, coverage is still valid;" \
     "2=measurement invalid, sim unstable/clock stalled; 3=launch/process failure)"
exit $WORST
