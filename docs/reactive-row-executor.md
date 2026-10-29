# Reactive row executor — design note

*Status: experimental mode implemented in `coverage_planner_node.py` behind
`executor:=reactive`; `executor:=nav2` remains the default baseline. This
note retains the design rationale and outstanding acceptance work.*

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

## Observed test_room result — 29 October 2026

One headless run of `executor:=reactive` on local commit `75762a6` completed
with **PASS**: **98.00% final coverage**, **90.34% efficiency at the first 90%
coverage crossing**, and **74.43% final efficiency**. The existing acceptance
condition uses efficiency at that first crossing; finishing the remaining
floor adds travel. The scorer records 439.0 s to target; total wall time was
562.71 s including startup and teardown. The meter recorded 115.01 m of travel
and no pose jumps. These are a single test_room result, not an A/B speedup or a
reliability estimate.

The planner/scorer use local source. Both launch files, the world, map and Nav2
configuration match local files by hash; a read-only input bind supplies the
local sim launch to the cached Jazzy image. The run stayed offline with no
hardware devices, a 3 GiB container inside the 4 GiB host scope, and a 512
PID/thread cap. Observed peaks: 281 tasks and 890,155,008 bytes; these do not
establish physical robot or ARM hardware requirements.

The meter excludes its configured 0.15 m edge band for separate floor-care.
No Keepout mask was enabled, and the three-way relay, single-writer ownership,
fault/dropout handling, docking and motor watchdog remain unvalidated by this
run. Two reactive no-progress skips shortly after successful Nav2 goals, plus
three immediately skipped gap-fill passes, remain diagnostic follow-ups.
Exact inputs, hashes, counters, report and limits are in
[`reactive-test-room-evidence-20261029.json`](reactive-test-room-evidence-20261029.json).

## Row-progress context follow-up — 29 October 2026

The completed test_room log showed two `stuck (reactive)` skips immediately
after successful Nav2 goals. A focused reproduction confirms a defect that can
explain that pattern: the row's closest-distance and last-progress timestamp
survived a transit or target replacement. Time in a previous target/controller
could therefore exhaust the new target's watchdog before it had a chance to
move.

The existing fields now reset at Nav2 dispatch, covered-waypoint advancement,
and gap-fill plan replacement. No threshold, timer or controller was added.
Six new transition cases fail before the correction; the complete single
planner test file passes **25/25** afterward. Positive controls confirm a new
row held without progress for seven seconds still stops and skips under the
unchanged six-second budget. These are synthetic unit boundaries using actual
planner methods and ROS message/time classes, not DDS or physical proof.
A subsequent full test_room run of corrected source `95e85c6` also passes:
98.25% final coverage, 90.86% efficiency at the first 90% crossing, and 74.50%
final efficiency. It completes in 563.23 s wall time including startup/teardown,
with 14 successful Nav2 transit goals and no `stuck (reactive)` warnings (the
prior run had two). This single comparison does not establish a statistical
performance improvement. Three gap-fill passes still immediately skip three
uncovered spots; that candidate/skipping policy remains a separate follow-up.
Exact unit and Gazebo reports, inputs, provenance and limits:
[`reactive-progress-context-evidence-20261029.json`](reactive-progress-context-evidence-20261029.json).

## Gap-fill admission follow-up — 29 October 2026

The next focused reproduction confirms why a tiny remaining hole could consume
all gap-fill passes without a visit. Gap generation selected an uncovered
centre cell, but the shared sweep shortcut then discarded it because its
neighbourhood was mostly covered. The gap-fill phase now credits that target
only when its selected centre cell is covered; the main sweep shortcut and
known-wedge exclusion stay unchanged.

Three hole cases fail before the correction, while four controls pass.
Afterward the single planner test file passes **32/32**, including prior
progress and zero-command cases. A subsequent full Gazebo run of `9283378`
passes at 99.09% coverage, with 90.84% efficiency at the first 90% crossing and
70.14% final efficiency. The first gap-fill now starts a real Nav2 goal and runs
for about 32 s rather than consuming its pass immediately. The last report
before gap-fill is 98.76% at 115.76 m; final is 99.09% at 122.05 m. This exposes
the extra-travel cost of finishing small holes and is not an A/B gain claim.
The second pass's last candidate coincides with delayed coverage updates; the
log does not prove a separate visit to every candidate. This remains one
simulation, without active Keepout, arbitration, hardware or dirt-removal proof.
Exact unit/Gazebo source, commands, provenance and limits:
[`gapfill-uncovered-center-evidence-20261029.json`](gapfill-uncovered-center-evidence-20261029.json).

## Related

- `recovery-safety` RFC — this *is* the reactive-control layer it anticipated;
  the wedge escape and edge-follow live here.
- `floor-care` — edge cleaning rides the same bumper reflex.
- `clean-and-map` (Deepak) — coordinate the executor boundary with his SLAM +
  coverage work and the dynamic-obstacle yielding.
- `health-monitor` — stack-liveness / MCU watchdog is orthogonal but shares the
  "reactive, contact-tolerant vacuum" worldview.

## Explicit row-command stop follow-up (2026-10-29)

The row tick now publishes a zero Twist when its pose helper reports no
pose, no target remains, the target is reached or skipped for no progress,
and before requesting a long Nav2 transit (including server-not-ready).
Those returns previously left the last reactive cruise command in place.
The already-active Nav2 branch remains silent so it does not inject a
competing stop into the transit command stream.

Baseline: six transition cases failed after a preceding 0.35 m/s row
command; the active-transit silence case passed. After the fix, all 19
tests in `test_planner_geometry.py` pass in the cached Jazzy image, using
the real imported `_tick_reactive` method and Twist type with a recording
publisher and synthetic pose/clock/target sources. This is focused unit
evidence, not a running ROS2 graph, Gazebo movement or physical braking.
The unchanged upstream source blob was verified as
`747aae3be08a9c91b0a2d3b10df5ac7451700100` before the correction.

Reproduce inside the existing isolated Jazzy image with this source mounted
read-only at `/work` and `/opt/ros/jazzy/setup.bash` sourced:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/work/src/oomwoo_coverage:$PYTHONPATH \
  python3 -m pytest -p no:cacheprovider \
  /work/src/oomwoo_coverage/test/test_planner_geometry.py -v
```

Full command/results: `reactive-row-stop-evidence-20261029.json` beside this
note. No new node, watchdog, timer or tool was introduced. A missing-pose
stop only applies when `_robot_pose()` returns None; freshness of its TF or
cached-AMCL fallback is a separate unresolved contract. Exclusive command
ownership, Keepout during direct drive, actuator timeout and stopping
distance still require integrated simulation and hardware validation.


## Velocity-source compatibility audit — 29 October 2026

The three-way policy is still not connected to the runtime. Reading the actual
launches and drivers establishes these distinct paths before any joint test:

| Component | Current command path / lifecycle |
| --- | --- |
| Reactive CoveragePlanner | Publishes `cmd_vel`; hands long transits to Nav2. Defaults are 0.35 m/s and 1.0 rad/s. |
| Minimal simulation Nav2 | Controller publishes `/cmd_vel` directly; no velocity smoother or collision monitor in this trimmed path. |
| Composed navigation launch | Controller/behaviors feed `cmd_vel_nav`, followed by the velocity smoother/collision-monitor chain. It excludes Nav2's docking server. |
| OOMWOO DockDrive | A separate LiDAR docking driver publishes `cmd_vel`; `auto_start=true` begins SEARCH. Its launch sends parent `Shutdown` when the driver exits. |
| Experimental contributor relay | Two sources only, NAV/DOCK, resolved inputs under `/valdi`; output restricted to `/valdi/lab_safe_cmd_vel_output`. No SWEEP adapter. |

Therefore adding the existing docking launch to the coverage launch would not
establish exclusive motor ownership. A joint isolated test must first remap
all real producers, reconcile phase/UUID handoffs and launch shutdown scope,
and obtain fresh evidence for the existing policy's health inputs. Its
0.25 m/s / 0.8 rad/s defaults also reject some unmodified coverage commands;
raising limits simply to make the test pass is not a resolution.

One concrete incompatibility is corrected here. The contributor relay requires
DockDrive's IDLE state to be received within three seconds, but DockDrive only
announced transitions. With `auto_start=false`, initial IDLE was not announced
at all; with any stable phase, late subscribers missed it and freshness expired.
DockDrive now announces initial IDLE (including latched `docked=false`) and
refreshes its current state on the existing command tick. No timer, node, gate,
watchdog or source-arbitration tool was added; transition logs remain event-only.
A stopped command tick also stops state refresh. The message asserts current
software phase, not sensor validity, exclusive output ownership or electrical
charging/contact evidence. In particular the driver's DONE/docked signal is
computed from estimated docking geometry, not an independent contact sensor.

Two real ROS2/DDS cases reproduce the late-subscriber defect before the fix,
both with no received state. Both pass afterward: IDLE and SEARCH persist as a
fresh state stream beyond the relay's three-second budget; a real enable=false
message yields IDLE and zero Twist. The initial false docked value is available
to a late transient-local subscriber. The isolated graph has one lab velocity
publisher and zero `/cmd_vel` publishers. This test uses the actual driver and
ROS2 messages but no relay, Nav2 server, detector, Gazebo or physical robot;
it is not a three-way handoff validation. Flake8 checks both changed Python
files successfully. Exact commands, results and runtime bounds are in
[`dock-state-freshness-evidence-20261029.json`](dock-state-freshness-evidence-20261029.json).

Reproduce only this focused ROS2 case in the existing isolated Jazzy image,
with the contributor source mounted read-only at `/work` and Jazzy sourced:

```bash
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/work/src/oomwoo_dock:$PYTHONPATH \
  python3 -m pytest -p no:cacheprovider \
  /work/src/oomwoo_dock/test/test_dock.py -k ros_dock_state -v
```


## Reactive row localization follow-up — 29 October 2026

The latest TF lookup previously returned any cached map-to-base pose regardless
of age, and missing TF fell back to a stored AMCL position indefinitely. The
row controller could continue emitting 0.35 m/s after localization stopped.
It now uses only a latest common TF timestamp with age between zero and
500 ms, inclusive, and a positive timestamp. A missing, older, future or
unstamped transform returns None through the existing zero-command row path.
The latched AMCL position remains available for initial planning and existing
Nav2 progress accounting; it no longer revives direct row steering. This is a
software experiment budget in ROS time, not a certified physical limit.

Loss of row pose also clears the existing row-progress context, so time spent
waiting for localization cannot falsely exhaust the new row's stall budget.
Timely TF resumes the same target; seven further seconds without real progress
still trigger the unchanged stall/skip behavior. No timer, node, watchdog,
tool or test bank was introduced.

Before the correction, six of nine focal cases fail and three controls pass.
The final single planner file passes **42/42** with ten added cases. Tests use
the actual tf2 Buffer and ROS message/time classes with a synthetic clock and
recording publisher. They cover stale/missing/future/zero-stamp TF, cached-AMCL
fallback, the inclusive 500 ms boundary, dropout after actual prior nonzero
output, long-interruption recovery, a genuine subsequent stall, and a timely
composed transform despite AMCL's five-second future map-to-odom tolerance.
Flake8 and pep257 pass for both changed Python files. Evidence and full commands:
[`reactive-pose-freshness-evidence-20261029.json`](reactive-pose-freshness-evidence-20261029.json).

This fixes row-pose admission only. The separate open-loop escape branches,
active Nav2 ownership, full health attestation, direct-drive Keepout, stopped
process/clock handling and physical stopping remain separate unresolved
integration work. The owner confirms there is no hardware available: all
results are design/software/simulation.

A subsequent nominal full test_room run of corrected source `a68b048` passes:
**98.80% final coverage**, **90.51% efficiency at the first 90% crossing**,
69.65% final efficiency, and `sweep_complete`. The scorer records 441.0 s to
target; wall time including startup/teardown is 610.15 s. There are 19 successful
Nav2 transit goals, no reactive-stuck warnings, and no reported pose jumps or
simulation instability. The final meter reports 122.90 m and 13,696 reachable
cells; the existing deferred edge band remains outside that denominator.
This confirms nominal completion without the AMCL motion fallback. It does
not inject localization loss into Gazebo or prove the three-way ownership
policy. Current 98.80% is not replaced with the earlier source's 99.09%; a single
unseeded comparison cannot attribute coverage or timing differences to this fix.
The unchanged contract evaluates efficiency at the target crossing; final
finishing efficiency is reported separately. Exact inputs and results are in
the evidence JSON above. No further production change was made for this run.


## Private single-output integration — 29 October 2026, local WIP

An empty motion_relay_journal keeps the existing regression behavior. A
nonempty disposable absolute path enables coordination with the contributor's
existing three-way relay. This private dependency is not part of upstream or
installed by the default launch. The planner publishes its actual phase from
the existing 5Hz tick. It stops the sweep source, releases its ownership,
persists a preallocated exact UUID and map digest, requests NAV permission,
and only then sends that UUID to the actual NavigateToPose server. The actual
accepted/terminal callbacks update the journal. A successful or canceled
terminal permits NAV release and a fresh quiet interval before SWEEP. Faults
latch, request STOP and require reconciliation; aborted goals do not silently
resume the sweep. A status reply requested before a permission transition
cannot overwrite the new acknowledged owner.

The controlled launch reuses the actual DockDrive with auto_start=false;
it is alive and IDLE in this coverage scene. It is not an executed docking
trajectory or charging proof. Nav2 and sweep publishers go to isolated inputs;
only the existing relay output feeds ros_gz_bridge. Nav2's RViz goal_pose
alternative ingress is quarantined in this mode. The managed scene removes
behavior_server and the unused NavigateThroughPoses plugin, using Nav2's
stock navigate_w_replanning_only_if_goal_is_updated.xml tree. This leaves one
controller_server publisher and retains strict publication sequence checking:
Fast DDS has no supported reception sequence to order different publishers.
A failed transit requiring recovery remains a fault; no recovery equivalence
with the normal Nav2 launch is claimed. Controller speed/lookahead arguments
allow bounded NAV commands without increasing the relay limits.
A constant explicitly selected simulated BatteryState supplies only the
resource fixture; no physical battery is inferred.

The scorer's opt-in motion audit records actual status events, feedback
cadence, phase transitions and command-topic graph (including empty public
inputs and the final bridge subscriber). Producer FAULT forces a failed
result even after a coverage target crossing. Audit records are bounded and
reuse the existing scorer loop. A measurement-valid partial result means
only that the partial simulated coverage is measurable, not that the run
passed. There is no hardware in this project; all results are digital.

The initial focal planner module passed 45 cases; the scorer bounds module passes
8 cases including fault-after-target. Actual-controller Gazebo acceptance
and complete handoffs remain under investigation; the earlier 98.80% result
is the unarbitrated reactive run and must not be reused for this mode.


The terminal-ordering trial subsequently executed two successful actual Nav2
transits, each released to SWEEP, and drove reactive rows. It stopped during
held-bumper escape entry (5.91% partial coverage): that scheduling branch
returned without a command for one tick, leaving a 0.4 s gap in a 5Hz producer
against the unchanged 0.30 s command expiry. Escape scheduling now emits zero
immediately, both for held contact and skip/budget entry. Three focal actual
method cases cover these exits; the planner module now passes 48 cases.
This partial run is failed acceptance evidence, not a whole-room result.


### Bounded actual-controller observation

The final 180-second diagnostic ended normally at the scorer wall-time bound,
with 22.55% partial measured coverage, no recorded relay fault, four acknowledged
NAV releases and five SWEEP claims. Actual status records contain three
successful goals and one canceled goal after the existing waypoint timeout.
The command graph has one controller_server NAV publisher, one coverage_planner
SWEEP publisher, one IDLE DockDrive publisher and one final relay writer consumed
by ros_gz_bridge; /cmd_vel and /cmd_vel_nav have no publishers. There were no
pose jumps or instability. Total collection wall time including setup/teardown:
201.98 s. This is **not a whole-room PASS** (bank exit 1, wall_timeout).
No escape entry occurred in this final scene, so its zero-on-entry correction
is supported by the three focal method cases, not by this trajectory.

Four separate focused modules pass 49 +31 +13 +8 cases; three native managed
constructor/remap cases pass. Flake8 and pep257 pass on the six changed ROS2
files. No full suite was run. See [exact scoped evidence and retained failures](evidence/managed_nav2_gazebo_codex_20261029.json).
The managed boot deadline now also covers the wait before plan_started, so a
lost Nav2 activation response cannot leave this mode waiting indefinitely.
Remaining acceptance: full-room performance, actual escape exercise, docking
trajectory/phase coordination and injected faults with the actual controllers.
All work remains local; the owner has no hardware.

The existing basic three-way DDS scenario also passes its 14 checks again
on this relay source; its generated motion/health/status inputs remain synthetic.


## Held contact during NAV — follow-up in progress

The first whole-room attempt on ec34c0e/c53a6d0 stopped at 75.31% coverage
(bank exit1, motion_fault, 561.21s including setup/teardown). Nine Nav2 goals
succeeded and ten releases completed before controller_server reported Failed
to make progress; the eleventh goal ended ABORTED. The managed adapter correctly
latched the existing fault. This is failed whole-room acceptance, not PASS.

Source inspection found that managed NAV returned before the existing held
bumper reflex could run. The follow-up asks cancellation once on held contact,
publishes TRANSIT_CANCEL to mute the existing relay NAV output, waits for the
actual result/journal and ordinary terminal/quiet/permission handoff, and only
then schedules the already existing contact reflex as SWEEP owner. It preserves
the observed direction/target through asynchronous acceptance and handoff.
An actual ABORTED result still faults; it never becomes a docking or recovery
permission. No output timer, implementation node, tool or worker is added.

CancelGoal must acknowledge exactly the current UUID with ERROR_NONE. Its
ERROR_GOAL_TERMINATED race grants nothing and only waits, at zero, for the actual
result. Rejected, foreign/duplicate UUID, transport and three-second unresolved
cancellation paths latch fault. Fault cancellation is bounded to the first fault
rather than repeated every tick; STOP transport errors cannot create a request
loop. Primary contract: [CancelGoal.srv](https://github.com/ros2/rcl_interfaces/blob/jazzy/action_msgs/srv/CancelGoal.srv),
blob 86afbdbf4eacb2c55c1faf8da3a17946c5f0fa9b.

Current focal evidence: 59 planner cases and33 real-method/metadata cases PASS,
including no escape before terminal plus SWEEP acknowledgement, cancel before
acceptance, cancellation/result race, bounded failures, continued ABORTED fault,
and actual relay output muting while retaining NAV ownership. The health phase
is accepted only in managed mode. Actual Gazebo confirmation of this follow-up
and whole-room acceptance remain pending at this note's entry.


### Complete test_room follow-up: scored PASS

The complete follow-up run finished sweep_complete with **98.66% coverage**,
**91.06% efficiency at the first90% crossing**, and70.49% final efficiency.
The existing90%/80% acceptance criteria were unchanged. The scorer reports605s
to target and total collection time857.12s including setup/teardown. Geometry
measurement remained valid with no pose jumps or simulator instability.

Actual status history contains21 distinct terminal UUIDs:17 SUCCEEDED and4
CANCELED. All21 NAV releases and22 SWEEP claims/releases were acknowledged;
the final producer phase was IDLE, with its final release before scoring ended.
Two actual held-contact episodes during NAV caused cancellation, exact terminal
release, quiet handoff and the existing reactive escape under SWEEP ownership.
No synthetic contact publisher or substitute controller was used in this run.
The final command graph still has one source per input and one final relay
publisher consumed by ros_gz_bridge,with no /cmd_vel publishers. No relay fault
was logged during the measured scene.

The first75.31% failed attempt remains preserved. This is one successful
follow-up,not a controlled A/B or a reliability estimate. See [full scoped
result,contact sequences,source hashes and retained errors](evidence/reactive_contact_full_room_codex_20261029.json).

**Shutdown limit:** the scored scene passed,but process shutdown was not clean.
After the final summary,the relay logged KeyboardInterrupt,the Gazebo bridge
exitedSIGABRT(-6),and Gazebo required the existing launchSIGTERM escalation.
The bridge abort cause remains open;the owned --rm container is terminal.
The existing sub-inscribed-inflation warnings and inner FollowPath halt-result
errors are also retained. Do not turn a scored PASS into a clean-log or physical
safety claim. Actual docking,combined active Keepout,managed fault injection,
shutdown diagnosis and broader-scene validation remain open.


### Managed simulation shutdown: UDP workaround and native Gazebo exit

The core dump from the98.66% run places heap-corruption detection in Fast DDS
2.14.7 shared-memory teardown. It does not identify the original corrupting
write. The managed launch now defaults to builtin UDPv4, retaining an explicit
parent transport choice and leaving the legacy launch mode unchanged. The
optional motion audit records that choice. This is a scoped workaround, not an
upstream memory repair.

The relay catches normal KeyboardInterrupt/ExternalShutdownException and still
cleans up. The existing regression launch gives its Gazebo transport a private
partition, then requests native /server_control stop when the scorer exits.
After a positive reply, the existing required-Gazebo exit closes the launch.
Sending Launch shutdown immediately after the reply interrupted native teardown;
that failed intermediate observation is retained. Missing/failed replies retain
the existing immediate shutdown. No new worker, node or timer was introduced.

The final bounded30s scene returned a positive native reply and every non-scorer
process exited cleanly, including Gazebo, the bridge and the relay. The scorer
correctly returned1 for wall_timeout; this partial scene did not pass coverage.
The previous bounded90s observation closed the bridge/relay cleanly but still
needed Gazebo SIGTERM. Four actual LaunchContext cases and lint on the two
changed ROS2 Python files pass. The existing14 DDS checks passed with UDP before
the relay exception-only change. See [source versions, all intermediate results,
exact exits and limitations](evidence/managed_shutdown_udp_codex_20261029.json).

The full post-change test_room passes: **98.51% coverage**, **91.33%
efficiency at the first 90% crossing**, 70.32% final efficiency, and 599.0 s to
target by the scorer. Total collection time was 855.96 s. The existing 90%/80%
criteria, geometry and motion limits were unchanged; no pose jumps or simulator
instability occurred. Status history contains 21 terminal UUIDs: 15 SUCCEEDED and
6 CANCELED. All 21 NAV releases and 22 SWEEP releases were acknowledged; final
phase was IDLE. One actual held-contact episode caused verified cancellation,
quiet handoff and the existing reactive escape. DockDrive remained IDLE.

The native stop returned data:true and **all 18 owned processes exited 0**,
including Gazebo, ros_gz_bridge and the relay. No SIGTERM escalation, bridge
SIGABRT or relay KeyboardInterrupt appeared. The graph still shows one input
publisher per role, one final relay output consumed by the bridge, and no
/cmd_vel publishers. This is one full successful observation after the change;
it does not prove an upstream memory fix, reliability, or controlled A/B gains.
The two sub-inscribed inflation errors and four inner FollowPath halt-result
errors remain recorded. A successful close is not a clean-log claim.

This consolidates the scoped shutdown milestone. Runtime source fingerprints
and the terminal/output/handoff evidence match. Repeating the green scene or
further transport polishing has low demonstrated return without new
evidence. The next required integration is active Keepout at the existing
final output: Nav2's mask topic is /oomwoo/keepout_mask, the planner currently
subscribes to keepout_filter_mask, and the relay does not yet consume the
mask/costmaps. The synthetic envelope also assumes a sequenced mux and 0.3 s map
freshness, while the current Nav2 costmaps publish at 2 Hz; it cannot simply be
attached or refreshed with invented sample times. Fuse existing components
and verify real masked movement before claiming that combination. Actual
docking remains pending.

The earlier 98.66% result stays tied to its original hashes. A positive native
ACK followed by a permanently hung Gazebo destructor has not been fault
injected; this uses the existing required-server exit without a new watchdog.
All evidence concerns software and simulation; no physical prototype exists.
