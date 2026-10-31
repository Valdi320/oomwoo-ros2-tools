# Reactive row executor — delivery

## Native acceptance — 31 October 2026

The opt-in masked `test_room` sweep now finishes with **95.38% coverage**,
**80.47% efficiency at the first 90% crossing**, `sweep_complete` and exit 0.
The strict ground-truth disk/chord audit is valid: **zero forbidden crossings**
over 48,996 poses / 48,995 segments, maximum sample gap 20 ms, unchanged
0.18 m audit radius and 220-cell mask. All 20 owned processes close cleanly.
Final-sweep efficiency is **66.26%**, reported separately; it is not the 80% gate.
The first crossing is at 789.7 simulated seconds / 106.38 m. Original metrics,
mask, safety checks and deadlines remain in effect.

Acceptance snapshots: ROS code `a4d0bc666554270523794ae0ee7498bf5fbc3bfe`,
cleaning-jobs code `9feaf3fcc660c7ac22ca074a09be420f6f37cd2e`.
The ROS snapshot with the final receipt is
[`9f1720f`](https://github.com/Valdi320/oomwoo-ros2-tools/commit/9f1720f9c21af05e1b5b57d44f4c95ef5be39dff).

- [Acceptance audit, all source hashes and limits](https://github.com/Valdi320/oomwoo-ros2-tools/blob/9f1720f9c21af05e1b5b57d44f4c95ef5be39dff/src/oomwoo_sim_support/docs/evidence/native_90_80_delivery_20261031.json)
- [Full unmodified native receipt and logs](https://github.com/Valdi320/oomwoo-ros2-tools/blob/9f1720f9c21af05e1b5b57d44f4c95ef5be39dff/src/oomwoo_sim_support/docs/evidence/gapfill_final_full_native_20261031.json)
- [Focused gap-fill causal/regression evidence: 20 cases and style](https://github.com/Valdi320/oomwoo-ros2-tools/blob/9f1720f9c21af05e1b5b57d44f4c95ef5be39dff/src/oomwoo_sim_support/docs/evidence/gapfill_terminal_delivery_20261031.json)
- [Owned shutdown causal/regression evidence: 17 cases and native prefix](https://github.com/Valdi320/oomwoo-ros2-tools/blob/9f1720f9c21af05e1b5b57d44f4c95ef5be39dff/src/oomwoo_sim_support/docs/evidence/coordinated_shutdown_delivery_20261031.json)

### Delivered behavior

Rows use the reactive executor; Nav2 owns only transit goals in this profile.
The optional private cleaning-jobs adapter arbitrates NAV/SWEEP/DOCK through
one output, with exact UUID/durable-result checks and fail-closed health.
Keepout uses both Nav2 costmaps plus the reactive-motion checks. The existing
0.25 m lookahead avoids the 0.5 m experimental override; RPP rechecks distance
instead of retaining a final rotation after localization corrections.
AMCL uses the probabilistic likelihood-field model in simulation only.
Global ordinary-obstacle inflation is 0.24 m; local inflation stays 0.10 m.
The cleaning-meter denominator is not reduced to improve the score.

Gap-fill now retains an uncredited target after Nav2 reports arrival, releases
navigation normally, then lets the existing reactive driver finish the short
approach. Four residual targets finish in one pass. The scorer buffers bounded
future-stamped truth until its original clock catches up, keeping original
stamps and all chord checks. Gazebo receives one native stop request; ROS children
receive their normal interrupt, with the same shutdown deadline and strict
nonzero-exit classification. SIGTERM and crashes still fail the run.

This is one configured scene/profile, not multi-room or repeated-seed reliability,
a complete cleaning-jobs RFC, an agreed public API, physical cleaning/charging,
or MCU safety validation. The margin over the gate is 0.47 percentage points.
Earlier short docking evidence uses another frozen snapshot and does not prove
that this final full-room run included a dock-service interruption.

### Reproduce the accepted profile

Use native x86-64 Linux and Docker or Podman. Check out the delivery source
commit linked in PR #79; the branch below includes the historical trace and
byte-exact diagnostic input archive. The cleaning-jobs branch contains the
paired relay fixes. No hardware or GPU is required. The battery input is
synthetic (0.92). Respect the host's existing CI/resource reservations.

```bash
git clone --branch delivery/reactive-90-80-20261031   https://github.com/Valdi320/oomwoo-ros2-tools.git ros2-tools
git clone --branch delivery/reactive-90-80-20261031   https://github.com/Valdi320/oomwoo-cleaning-jobs.git cleaning-jobs
```

The following uses the recorded image digest (cached image ID
`65bec2e9c5122c286ef4f34e9e99e94f90e3583d8a9f960cc3422c624e61452f`,
ROS 2 Jazzy, Nav2 1.3.13, Gazebo 8.15.0). It runs one focal room integration,
not all suites. Diagnostic mounts preserve the actual scorer and relay bytes;
they are archives, not promoted runtime features. The scorer's control/grading
methods are unchanged by those observers. See the archive manifest for the
canonical/observed hashes and the correction of historical observer metadata.

```bash
mkdir -p oomwoo-acceptance-output
docker run --rm -i --network none --cap-drop ALL \
  --security-opt no-new-privileges --security-opt label=disable \
  --memory 3g --memory-swap 3584m --cpus 2 --pids-limit 512 --ulimit core=0 \
  --tmpfs /tmp:rw,exec,size=1024m,mode=1777 \
  -v "$PWD/ros2-tools:/work:ro" -v "$PWD/cleaning-jobs:/contrib:ro" \
  -v "$PWD/oomwoo-acceptance-output:/results:rw" \
  -v "$PWD/ros2-tools/src/oomwoo_sim_support/launch/sim_bringup.launch.py:/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/launch/sim_bringup.launch.py:ro" \
  -v "$PWD/ros2-tools/src/oomwoo_sim_support/launch/coverage_regression.launch.py:/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/launch/coverage_regression.launch.py:ro" \
  -v "$PWD/ros2-tools/src/oomwoo_sim_support/config/nav2_params.yaml:/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/config/nav2_params.yaml:ro" \
  -v "$PWD/ros2-tools/src/oomwoo_sim_support/docs/evidence/native_90_80_runtime_20261031/coverage_regression_runner.py.txt:/work/src/oomwoo_sim_support/oomwoo_sim_support/coverage_regression_runner.py:ro" \
  -v "$PWD/ros2-tools/src/oomwoo_sim_support/docs/evidence/native_90_80_runtime_20261031/ros_dock_velocity_relay.py.txt:/contrib/src/oomwoo_cleaning_jobs/ros_dock_velocity_relay.py:ro" \
  --entrypoint /bin/bash \
  makerspet/oomwoo@sha256:b97aaa649ff91b6a92a0a5ddbe741ff3be590ca3658e5fd5fc8618e5e0f762cd \
  -s <<'RUN'
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/roslogs
export FASTRTPS_DEFAULT_PROFILES_FILE=/work/src/oomwoo_sim_support/config/nav2_lifecycle_replies.xml
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LP_NUM_THREADS=2
export PYTHONPATH=/work/src/oomwoo_sim_support:/work/src/oomwoo_coverage:/work/src/oomwoo_dock:/contrib/src:${PYTHONPATH:-}
export OOMWOO_MOTION_AUDIT=1 OOMWOO_SIM_BATTERY_FRACTION=0.92
export MAX_WALL_TIME_S=1800 CLOCK_DEAD_WALL_S=45 RUNS=1
mkdir -p /tmp/valdi_ros_ws/src
cp -r /contrib/ros2_lab_interfaces /tmp/valdi_ros_ws/src/oomwoo_cleaning_jobs_lab_interfaces
cd /tmp/valdi_ros_ws
colcon build --packages-select oomwoo_cleaning_jobs_lab_interfaces --parallel-workers 2
source /tmp/valdi_ros_ws/install/setup.bash
python3 /contrib/integration/generate_keepout_lab.py --out /tmp/keepout-fusion \
  --test-room /work/src/oomwoo_sim_support/maps/test_room.yaml \
  --nav-params /work/src/oomwoo_sim_support/config/nav2_params.yaml
# Only the mask below is consumed; the generator's legacy Nav2 overlay is not.
bank_code=0
bash /work/deploy/run_coverage_regression.sh executor:=reactive \
  v_cruise:=0.25 rotate_speed:=0.8 nav_linear_speed:=0.14 nav_angular_speed:=0.7 \
  motion_relay_journal:=/tmp/oomwoo-owned-motion.json \
  sweep_cmd_topic:=/valdi/sweep_cmd_vel nav_cmd_topic:=/valdi/nav2_cmd_vel \
  motor_cmd_topic:=/valdi/lab_safe_cmd_vel_output \
  keepout_mask:=/tmp/keepout-fusion/keepout_test_room.yaml || bank_code=$?
cp /tmp/coverage_regression.log.1 /results/coverage_regression.log
cp /tmp/coverage_runs.*/run1.json /results/coverage_report.json
cp /tmp/oomwoo-owned-motion.json /results/motion_journal.json
exit "$bank_code"
RUN
```

Expected acceptance is coverage >=90%, first-crossing efficiency >=80%, valid
zero-crossing audit, `sweep_complete` and exit 0, not identical timing on every
machine. The archived native record is one successful observation. Reports, launch log and the final journal are retained in
`oomwoo-acceptance-output/`; the command preserves the original bank exit code.

## Original design rationale

# Reactive row executor — design note

*Status: design / scoping. No code yet — this is the shape to decide on.*

## Why

The coverage **planner** decides *what* to clean and in *what order* (cells →
serpentine passes → gap-fill). Today it also **executes** motion by firing one
Nav2 `NavigateToPose` goal per waypoint. Measured on the living_room sim, that
execution path is the wrong tool for the job:

- **~3 s of overhead per waypoint** — drive ~0.4 m, reach the goal tolerance,
  wait for `bt_navigator` to declare success, send the next goal. At ~190
  waypoints that is **~10 minutes for one clean even when nothing goes wrong**,
  most of it goal-handshake, not cleaning.
- **Costmap phantom** — RPP's forward collision check stopped the robot short of
  obstacles (fixed by disabling it, but the machinery is still fighting us).
- **Corner-cutting** — pure pursuit arcs through row-end U-turns.
- **Collision-averse recoveries** — Nav2 `spin`/`backup` refuse to move in a
  wedge (footprint in lethal cells), so they stall exactly when needed.

None of this is a bug to patch; it is Nav2 being a *point-to-point, costmap-aware*
navigator asked to do *coverage*, whose motion is really **drive-straight →
turn → drive-straight → (bump) edge-follow**. That motion wants a small reactive
controller, not a global planning stack.

## Principle (subsumption / hybrid)

Split *planning* from *executing*, and use each tool where it is strong:

| Concern | Owner |
|---|---|
| What to clean, pass order, gap-fill | **coverage planner** (done) |
| Driving a sweep pass (straight line + row-end turn) | **reactive row executor** (new) |
| Contact reflex (peel off / edge-follow) | **bumper escape** (built) — overrides the row drive while in contact |
| Longer inter-cell transit, routing around furniture, go-to-dock | **Nav2** `NavigateToPose` (kept) |
| Localization | **Nav2 AMCL** (kept) |

Coverage plans the open floor; the executor drives it; the bumper reflex handles
the last few centimetres; Nav2 is demoted to the few jobs it is actually good at.
This is the reactive-control layer the recovery-safety RFC anticipated.

## How it slots in — as a mode of the coverage planner (at first)

The `coverage_planner` node **already has everything the executor needs**: a
`cmd_vel` publisher (used by the escape), `bumper_left|right/contact`
subscriptions, the `amcl_pose` subscription, the 10 Hz `_tick` loop, and the
contact-aware peel-off. So the executor starts life as a **RowDriver mode inside
the existing node**, not a new process — minimal new infrastructure, and it
reuses the escape reflex directly. If it grows, it splits into its own
`reactive-control` node later (same plan the recovery-safety RFC sketches).

The plan the planner already produces is a list of `(x, y)` waypoints. We tag
each segment as one of:

- **`row`** — a straight run of collinear waypoints (a sweep pass, and the short
  intra-cell connector to the next pass). Driven reactively.
- **`transit`** — the hop between two cells (or robot → first cell), which may
  need to route around furniture. Dispatched to Nav2 as one `NavigateToPose`.

`_tick` looks at the current segment's tag and calls the row driver or the Nav2
dispatcher. Rows collapse ~190 discrete goals into ~a few dozen continuous
drives; transits stay ~one Nav2 goal per cell boundary (≈10), where Nav2's
overhead is negligible and its furniture-avoidance is worth having.

## The row-drive control law (the one genuinely new piece)

Given a segment `(p_start → p_end)` and the robot pose from localization:

1. **Follow the line.** `heading_err = wrap(atan2(p_end − pos) − yaw)`;
   `cross_track = signed perpendicular distance from the p_start→p_end line`.
   `cmd.angular.z = −k_h·heading_err − k_ct·cross_track`;
   `cmd.linear.x = v_cruise`, tapered down as `|heading_err|` grows or near p_end.
2. **Turn at the row end.** Within `row_end_tol` of `p_end`, stop linear and
   rotate in place to the next segment's heading (`angular.z = k·heading_err`)
   until aligned, then start the next segment. (Crisp 90° corners — the thing
   pure pursuit couldn't give us — for the price of a short in-place rotate,
   which is cheap because it happens per *pass*, not per waypoint.)
3. **Feedback.** Use `odom` for the high-rate, smooth heading/velocity loop and
   `amcl_pose` to correct odometry drift periodically (a slow vacuum tolerates
   10 Hz control fine). No costmap in the loop — the robot is *meant* to touch
   things; safety is the bumper.

A pass ends on: **reached `p_end`** (→ next segment), **bumper contact** (→
escape reflex, below), **no-progress** (→ skip the pass, same watchdog we just
added), or **off-map / lost localization** (→ stop, hand to safety).

## How it shares the bumper escape

The contact-aware peel-off already built (held-bumper → rotate away from the
pressed side, record a no-go pocket) **becomes the executor's contact reflex**,
unchanged in spirit:

- While driving a row, the executor watches `bumper_*/contact`. Sustained
  contact **overrides** the row drive (subsumption): run the peel-off open-loop
  on `cmd_vel`, then resume the row (or skip it if the pocket is now no-go).
- **Edge-follow (floor-care)** is the natural next behavior on the *same* signal:
  instead of only peeling off, hug the contour (back a hair → turn out → arc in →
  re-contact) to clean right up to the object. Shared contact plumbing means the
  executor, the wedge escape, and edge cleaning are one reflex layer, not three.

## Interfaces

- **Consumes:** the internal plan (already in-node); `amcl_pose` + `odom` + TF
  `map→base_link`; `bumper_left|right/contact`; `coverage_meter/ratio` (skip
  already-clean passes).
- **Produces:** `cmd_vel` (rows + escape); `~/plan` Path (done, for RViz);
  optionally `~/status` (current segment, mode: row/transit/escape) for
  observability. Keep the topic/behaviour contract aligned with
  `SOFTWARE_INTERFACES.md`.
- **New params:** `v_cruise`, `k_heading`, `k_crosstrack`, `row_end_tol`,
  `rotate_speed`, `min_transit_len` (how long a hop must be to go via Nav2 vs.
  drive reactively). All tunable, all with sane defaults.

## Phasing (no big-bang rewrite)

1. **Row driver behind a flag.** Add the RowDriver + segment tagging; `executor:=reactive`
   drives rows reactively and transits via Nav2, `executor:=nav2` keeps today's
   all-Nav2 path. A/B them: **time-to-coverage** should drop sharply (the ~3 s/wp
   overhead disappears), coverage and turns hold or improve.
2. **Fold in the reflex.** Route the bumper escape through the executor; confirm
   peel-off still frees wedges. Add edge-follow as an opt-in.
3. **Retire per-row Nav2.** Once the reactive path wins the A/B, rows never touch
   Nav2; Nav2 is transits + dock + localization only.

The `executor:=nav2` fallback stays as the regression baseline and a safety net.

## Risks / open questions

- **Open-loop straightness.** How well can it hold a line on `odom`+`amcl` alone,
  with no costmap? (Mitigation: the cross-track term + amcl drift correction; a
  bump just triggers the reflex — contact is acceptable.)
- **Dynamic obstacles mid-row** (a foot). No costmap means we rely on the bumper
  and, optionally, **Nav2 Collision Monitor** (`nav2_collision_monitor`) as a
  reactive slow/stop zone from the LiDAR — cheap and composes with this. (Ties
  into Deepak's dynamic-obstacle-yielding work — coordinate.)
- **Transit boundary.** Deciding which connectors are "rows" vs "transits" — a
  length + free-corridor test; err toward Nav2 when a hop crosses unknown space.
- **Localization dropout.** If `map→odom` goes stale mid-row, stop and re-localize
  rather than dead-reckon into a wall.

## Testing / acceptance

Headless regression, `executor:=reactive` vs `executor:=nav2` on living_room
(and a tall room, and a simple rectangle):

- **Time-to-coverage** (the headline win) — expect a large drop.
- **Coverage %** and **turns** — hold or improve.
- **Stuck-freedom** — no wedge lasts more than a couple of seconds (reflex +
  no-progress skip).
- **Regression gate** stays green with the Nav2 fallback so we never lose the
  known-good baseline.

## Related

- `recovery-safety` RFC — this *is* the reactive-control layer it anticipated;
  the wedge escape and edge-follow live here.
- `floor-care` — edge cleaning rides the same bumper reflex.
- `clean-and-map` (Deepak) — coordinate the executor boundary with his SLAM +
  coverage work and the dynamic-obstacle yielding.
- `health-monitor` — stack-liveness / MCU watchdog is orthogonal but shares the
  "reactive, contact-tolerant vacuum" worldview.
