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


### Keepout fusion in progress: direct command and transport evidence

The opt-in integration connects the native /oomwoo/keepout_mask to the planner
and folds the existing arc/swept-disk projection into the relay's sole output.
The synthetic envelope retains its original three-map checks. The live mask
path attests immutable aligned content, stable publishers and a recent original
post-filter global costmap; receiving static bytes does not refresh their age.
Masked coverage requires the existing relay/journal mode because a Nav2 sweep
can still issue a direct bumper/wedge escape. No extra output node or timer is
introduced. Ordinary obstacle/contact clearance retains its previous behavior;
keepout dilation now includes corners and its explicit margin.

The projection22, relay42 and planner70 focused cases pass. One existing DDS
probe run passes17 checks with required mask evidence, including direct virtual
intersection, stale post-filter map and fresh-but-unenforced map. It uses
synthetic clock/maps/commands, not an actual Nav2 server or Gazebo. A queued
zero from before a new lease is ignored without acquiring command/release proof;
old nonzero commands still fault. The first DDS rearm failure remains recorded
without a confirmed original cause. A separate diagnostic assertion was updated
because the output pose limit stops earlier than the common health witness.

The scorer's final13 cases and five-file lint/admission contexts pass. The
first masked Gazebo scene observed5181 poses with no disk/chord crossings, but
failed to release a goal canceled immediately after acceptance. A deterministic
case reproduced the lost ACTIVE observation between50ms output ticks. The
existing Nav2-status callback now passes its observed health to the mux, keeping
exact UUID, journal and common-health requirements. Its43-case module and17 DDS
checks passed. A later44-case module also checks that the first lease revocation
survives subsequent stopped commands in the existing bounded log/status record.

The next native run failed early with unexplained ownership loss and an invalid
or stale truth sample. Its original first cause was not retained; no clock or
transport cause is established. The first run also had Gazebo SIGSEGV139 after
a positive native stop reply. Two later clean closes do not repair that cause.
Both failed runs remain part of the evidence.

A final bounded180s scene observed8858 original ground-truth poses,8857 chords,
maximum20ms spacing and zero crossings for the0.18m circular proxy. Observed
SWEEP distance was9.74m, TRANSIT10.42m and ESCAPE0.28m. Native mask/info servers
and the post-filter global costmap were connected; the graph retained one final
output. Five Nav2 UUIDs succeeded and one was canceled. All20 auxiliary processes
closed cleanly. The scorer correctly exited1 for wall_timeout:16.37% coverage is
partial, not the full90%/80% acceptance. Final observed phase wasSWEEP, not a
completed sweep or an IDLE handoff.

Minimum sampled center-to-mask-bbox distance was0.206m. This supports the
reported circular-proxy observation, not a ground-truth0.06m margin, complete
URDF-footprint certificate, hardware stop or reliability claim. Eight poses
before mask arrival were counted but excluded from this observation. Full masked
coverage, intermittent-failure diagnosis, actual docking and physical validation
remain open. No hardware exists. See [exact source versions, all observations
and retained failures](evidence/reactive_keepout_fusion_codex_20261030.json).


The subsequent full masked acceptance **failed** at53.84% coverage after423.92s
wall time. Its19975 poses and19974 chords had20ms maximum spacing and zero
proxy-footprint crossings. Six Nav2 UUIDs succeeded and three were canceled;
eight of nine NAV releases were acknowledged. The first revocation was
"Nav2 goal journal missing/corrupted" during another immediate cancellation.
The final valid canceled journal does not erase that observed failure. A prior
rapid cancellation in the same scene did release successfully and resumed
SWEEP/escape, providing native evidence for the between-ticks callback repair.

The existing journal can independently reproduce a legitimate atomic-replace
race: after opening the old file but before fstat, accepted() replaces it; the
old descriptor has nlink0 and is rejected while the new named record is valid.
The native witness discards its exact read exception, so this remains a
reproduced candidate cause, not a proven explanation of the native failure.
At preparation, the bounded-reopen proposal and three focal cases were unapplied
pending explicit owner authorization under machine rule7. The subsequent
authorized application and its verification status are recorded below. Full masked acceptance
remains open. Gazebo also exited139 after its positive native ACK in this full
attempt;19 other auxiliary processes closed cleanly. This is distinct from the
previous FastDDS bridge issue and has no established repair. No hardware exists.


Offline inspection of the existing Gazebo core now localizes the shutdown
failure to iteration of the `events::Stop` callback map. RTTI confirms the
StopTag event; the main thread is in `std::thread::join` under `Server::~Server`.
Both owned failures have the same relative library offsets. The installed
Event.hh cleans pending connections under its mutex and then iterates the map
without that mutex. [Gazebo8.15.0 ServerPrivate](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/ServerPrivate.cc)
creates a Stop RPC thread and also calls Stop from destruction. This supports a
concurrent event-cleanup/lifetime hypothesis; the exact invalidating operation
and a repair remain unproven. No new simulation or runtime change was needed.

The first offline GDB trace could not unwind the Ubuntu libraries until its
analysis-only `/lib -> usr/lib` mapping was corrected. The final selected
frames and event RTTI are available, although219 other library inputs and
libthread_db remain absent. Nearest-export addr2line labels are misleading here:
SimulationRunner::Stop is only13bytes; the two failure offsets lie elsewhere.
The old CET-bounded metadata query also missed the UTC event; both core records
exist. Fingerprints, raw traces, limits and the existing sparse offline core are
referenced in the evidence. This is independent of the prior FastDDS bridge
workaround and does not establish a fix. Persistence was unchanged at that offline diagnosis. See the subsequent
authorized application below.


The owner subsequently answered "Autorizo" for the prepared journal patch.
It is now applied in the owned contribution copy. A read of a detached regular
inode closes that descriptor and reopens the named path once; a second detached
inode is refused. All original file, permission, size, schema and UUID checks
and the serialized writer remain in place. Three causal/security cases were
added to the existing inode module; they have NOT executed yet.

The first verification attempt did not launch Python unittest: the machine
guard reported that the owner was playing. This is an applied, unverified WIP,
not a tested fix or a successful native Keepout acceptance. The race is proven
independently; its identity with the native generic journal error is still
unproven. The next work is the listed focal modules, individually after LIBRE,
then a discriminating integration. No simulation or new scheduler was started.


### Owned follow-up, 30 October: completed masked sweep and coherent grading

This update supersedes the verification-pending journal status above. In the
owned contribution copy, the bounded journal reopen and the subsequent missing
path refusal have focused POSIX/UUID and DDS evidence. The later relay change
retains at most one admitted and one pending original TF: a future-dated sample
cannot authorize motion, and a pending sample becomes usable only after the
clock reaches its stamp. The final focused module passes 52 cases; the existing
ROS DDS probe passes 18 checks, including repeated 5 ms TF lead. These are
synthetic inputs through actual ROS interfaces, not physical validation. The
contribution source used by the native run is pinned to fa8b7af in its receipt.

The [completed native masked-room receipt](../src/oomwoo_sim_support/docs/evidence/gazebo_keepout_tf_fixed_delivery_20261030.json)
records 95.60% coverage with sweep_complete, zero sampled disk/chord Keepout
crossings, all 20 auxiliary processes exiting cleanly, and 24 actual Nav2 UUIDs
(20 SUCCEEDED, four CANCELED). The scorer exits 1: the efficiency criterion
still fails. The first meter report above 90% has 78.72% efficiency; the old
scorer captured 78.88% from the preceding report. Both values fail the unchanged
80% criterion. This single successful sweep observation does not establish
shutdown reliability, active docking, a physical footprint certificate, or
performance across homes. DockDrive stayed IDLE.

The [focused pre-change reproduction](../src/oomwoo_sim_support/docs/evidence/metric_pairing_false_pass_before_20261030.json)
also demonstrates a false pass: new 90.1% coverage paired with previous 80.5%
efficiency, when the matching efficiency is 79.9%. The implementation now
fuses the existing meter and scorer into one process and takes their three
metrics from one calculation. It removes the scalar metric inputs and duplicate
truth/active subscriptions. The regression wrapper disables the separate meter;
the standalone launch keeps its original default. Measurement geometry remains
0.1745 m robot radius and 0.20 m cleaning radius, independent of planner margin.
The first target crossing is retained during the existing completion grace too.
Own previously validated phase snapshots and child-failure exit code 3 are
carried forward; a positive score must not hide a failed owned process.

The [final coherent-grading receipt](../src/oomwoo_sim_support/docs/evidence/metric_pairing_fused_delivery_20261030.json)
records 29 focused cases passing, four changed Python files passing style, and
valid wrapper shell syntax. Cases cover actual ROS construction/report calls,
completion-grace crossings in both directions, real launch-condition evaluation,
and the existing clock/wall/child-exit contracts. A further causal case exposed
a motion fault arriving during grace being reported as a pass; the final result
now retains that fault too. Original red evidence is retained. These are
synthetic inputs through actual methods, ROS objects and Launch events, not a
new Gazebo acceptance or hardware result. The owner explicitly authorized
compatible small tests at .25 CPU/512 MB without GPU, and native tests through
the existing FIFO. Actual occupied lock results were retained, not bypassed.

The [existing-run transit analysis](../src/oomwoo_sim_support/docs/evidence/transit_timeout_review_20261030.json)
identifies the next candidate bottleneck without another simulation. Even after
subtracting the 0.20 m Nav2 goal tolerance and 0.02 m log-rounding allowance,
three of four canceled routes need nominal straight-line times of 31.84, 40.64
and 37.14 seconds at the configured 0.14 m/s, exceeding the 30-second budget.
All four recorded net ground-truth motion; the shorter case is not proven stuck.
No timeout or route was changed. Increasing all timeouts would also change
handoff waiting and is not justified by this evidence.

Decision: coherent grading is checked at component level. The
[static cell-entry comparison](../src/oomwoo_sim_support/docs/evidence/cell_entry_cost_comparison_20261030.json)
found identical 326 targets, 130.198 m path and 149.698 m path/turn cost for the
actual-entry candidate and current heuristic. No route code was changed: zero
benefit in this scene does not justify its extra work. Next address the existing
30-second transit timeout using the configured Nav2 speed and a per-goal budget,
while retaining the handoff deadline and no-progress cancellation. Any benefit
must be checked by native acceptance through the existing FIFO. Its benefit remains
unproven; the global useful milestone still needs masked efficiency acceptance
and an active NAV/SWEEP/DOCK integration. Further variants before this evidence
would add review and maintenance cost without demonstrated utility. The goal
is not complete and its overall sweet spot has not been established. Everything
here is local; no upstream push, PR, merge or comment was made.


The [per-goal transit-budget component receipt](../src/oomwoo_sim_support/docs/evidence/transit_budget_delivery_20261030.json)
now records the causal moving-goal cancellation before the change, 82 planner
cases passing after it, 30 scorer/launch cases passing, four-file style and shell
syntax checks. A known positive nav_linear_speed allows the existing watchdog
min(2 * goal_timeout, max(goal_timeout, distance / speed + no_progress_sec))
seconds for that goal. Zero, the planner default, preserves the fixed deadline;
the regression launch reuses the same speed argument as Nav2. Each dispatch
resets its budget. Negative/nonfinite speed settings are rejected. Boot/handoff
still use the original fixed deadline, and the existing no-progress check,
near-goal exception and ABORTED policy remain intact. No timer or node was added.

This demonstrates component behavior, not improved native efficiency. The
shorter canceled goal still gets the original 30 seconds; its cause remains
unproven. Nominal straight-line time does not guarantee enough time for detours.
Next: one original masked-room acceptance under the existing FIFO, immutable
sources, coherent scoring and unchanged 90%/80% criteria. Actual docking remains
pending. Do not keep tuning a passing component before measuring integration.


A [read-only installed-scene inventory](../src/oomwoo_sim_support/docs/evidence/stock_dock_scene_inventory_20261030.json)
clarifies the next active-docking integration input. The installed kitchen_dining
world contains vacuum_dock and charging_dock_basic; living_room has no dock-named
model or dock include in the inspected XML. This project's maps directory has
test_room, living_room and contour_torture, with no kitchen map. This is not a
claim about every installed map location. Before a joint real-controller cycle,
verify an existing world-aligned kitchen map or reuse the current map generator
with checked extent/collision geometry. That generator already accepts a world
argument, but fixes its grid extent and output basename; blindly passing the
kitchen world is insufficient. No map, world or runtime was modified, and this
inventory is not a docking or charging test. The native transit acceptance
continues independently with its frozen test_room inputs.


The [kitchen map input search](../src/oomwoo_sim_support/docs/evidence/kitchen_map_inputs_20261030.json)
found five map YAMLs across the inspected ROS install/source trees, none for
kitchen_dining. All referenced kitchen model SDF and mesh files were present.
The [generator preflight](../src/oomwoo_sim_support/docs/evidence/kitchen_map_static_candidate_20261030.json)
then declined generation: trimesh is absent in the inspected image, and several
actual table/chair collision poses include roll/pitch that the existing yaw-only
T_of/add_box path does not represent. A pressure-cooker cylinder is also an
unsupported primitive, although it may be above the robot band. No kitchen map
was produced or admitted, and no dependency was installed. Metadata collection
exiting zero does not mean map generation passed. Resolve the relevant geometry
in the existing generator or use an existing mapping path before the joint
cycle; the pending native transit test is unchanged.


The [oriented-box correction](../src/oomwoo_sim_support/docs/evidence/map_tilted_box_delivery_20261030.json)
now resolves that primitive transform issue in the existing generator. T_of
uses full roll/pitch/yaw. add_box intersects a cell center's vertical body-band
segment with all three local box constraints, retaining the existing grid-center
convention. The existing mesh import is deferred until mesh loading; no library
was installed. Eleven geometry cases pass, including a causal shifted-foot
failure before the fix, composed rotations, planar symmetry and height exclusion.

Five original kitchen table/chair models were also run through collisions_of.
Eight cell centers become occupied and six stop being occupied. One newly
occupied chair-foot point is independently inside the original SDF box using
SciPy's RPY transform. This is static geometry evidence, not eight avoided
physical collisions or a complete-map result. No kitchen map was generated or
admitted; mesh loading/projection and cylinder handling are unchanged. The
native transit run remains on immutable c67a7fc sources. Consolidate this fix
and wait for that native result before expanding generator work.


The [native transit-budget acceptance](../src/oomwoo_sim_support/docs/evidence/gazebo_transit_budget_delivery_20261030.json)
**failed** at 56.25% coverage after 483.676 seconds. Nine actual Nav2 UUIDs were
observed: eight SUCCEEDED and one ABORTED. One long transit remained observed
ACTIVE for more than 35 seconds before SUCCEEDED, beyond the original 30-second
budget. This supports that scoped change; it is not full-room improvement.

The final goal, from approximately (2.99, 0.05) to (4.23, 0.03) after a bumper
escape, failed with controller code 105 and the original log "Failed to make
progress". The exact command/pose cause remains unproven. The relay requested
STOP and the coherent scorer returned motion_fault/exit1. The 90% target was
not crossed, so there is no efficiency-at-target measurement; final clipped
1.0 is not a passing efficiency claim. All eight input fingerprints were
unchanged. The sampled Keepout audit has 23,056 poses, 23,055 chords, zero disk
crossings and 20 ms maximum spacing; 39 pre-mask poses were excluded. All 19
auxiliary processes closed cleanly, with only the scorer's expected exit1. No
active docking or hardware result is implied.

The existing FIFO granted ticket657 and the owned scope was restored from its
waiting .25 CPU cap to 2 CPU after admission. Early startup before restoration
was not fully profiled; no wall-time A/B gain is claimed. The wrapper buffers
child logs in the container while running, so the outer collector's null meter
heartbeat did not mean no motion. Both the FIFO and container are now terminal.
Next inspect the existing controller-progress and relay command-projection
paths, then make the smallest focused observation that distinguishes command
source, filtered output and physical-simulation motion. Preserve ABORTED/STOP;
changing success criteria or blindly rerunning the room would not diagnose it.


A single-goal diagnostic now reuses already-consumed relay status, AMCL,
truth and Nav2 feedback in private frozen copies. The production sources and
ABORTED/STOP behavior are unchanged. The [three focused fixture cases](../src/oomwoo_sim_support/docs/evidence/nav_progress_focal_fixture_20261030.json)
pass, covering selected goal, bounded samples, original source stamps and
continued status updates after the trace limit. These are fixture checks,
not a native navigation result or a coverage gain.

The native case is queued as FIFO666, with .25 CPU while waiting and an
automatic switch to 2 CPU only after verifying its own FIFO admission. It
starts near the prior final truth position and attempts the failed target
once. Fresh localization does not replay the prior trajectory. The unchanged
90/80 grader is expected to miss its target in this intentionally short
diagnostic; command/pose observations and actual action outcome must be
assessed separately. No second full-room run is justified before that contrast.


A complementary temporal-frame check, retained in the same fixture receipt,
uses the installed tf2 library with synthetic map-to-odom transforms. A
0.4 m frame correction makes the same map goal appear 0.4 m from the robot
at the original goal stamp and coincident at the current stamp; the 0.2 m
arrival decisions disagree. A stable-frame control agrees at both stamps.
The original 10/20-second input stamps are retained. The first harness
attempt failed on NumPy boolean identity; explicit boolean conversion fixes
that representation issue without changing transforms or thresholds.

The installed Nav2 packages identify as 1.3.13. That tag's ControllerServer
transforms the stored endpoint stamp, while RPP's PathHandler uses the
current robot stamp. This is a candidate mechanism to investigate if the
native traces show healthy pure rotation without translation. The small
check does not execute those controllers or establish the cause of the
original abort105. Keep production and arrival/progress tolerances unchanged
until the queued native case provides the next discriminating evidence.


The [near-end native diagnostic](../src/oomwoo_sim_support/docs/evidence/nav_progress_focal_delivery_20261030.json)
finished: its single actual Nav2 UUID transitioned ACTIVE to SUCCEEDED in
2.05 simulated seconds. Ten existing-status samples include nine with that
UUID active; their navigation input remains nonzero and the output changes
from pure rotation to translation. Sampled truth travels 0.1162 m during
TRANSIT. All eight frozen input hashes match; the Keepout audit records
840 poses, 839 chords and zero sampled disk crossings. All 19 auxiliary
processes close cleanly and there is no host timeout or motion fault.

The unchanged full-room grader returns exit1 at 0.51% coverage, as expected
for this deliberate one-goal diagnostic. It is not a 90/80 pass. Fresh
localization near the previous final position does not reproduce abort105,
so neither a general controller fix nor a global coverage improvement is
established. The command path works in this specific context.

A second single-goal case is queued as FIFO668, starting from the original
segment's initial truth XY (3.04384, 0.07799). Its yaw0 is approximate, since
the previous starting yaw/history was not retained. It reuses the exact
private frozen sources, map, mask and controls; only spawn and case identity
change. This compares the approach path with the successful near-end start.
No repeat of the complete room or tolerance change is justified yet.


The [fault-progress correction](../src/oomwoo_sim_support/docs/evidence/fault_progress_delivery_20261030.json)
resolves a bookkeeping defect visible in the abort105 native log: after
latching a relay fault, the result callback still ran legacy retry/skip
updates. A later success/cancel callback could also replace the fault stage
with release_nav. The actual terminal outcome remains in the journal and
the completed handle/deadline are cleared, while the current waypoint,
retry/skip counters, cooldown and fault stage remain pending reconciliation.

The strengthened existing abort case fails on the original retry update.
The initial harness attempt lacked next_send and failed before exercising
that behavior; both attempts are retained. After the correction, all 85
cases in the existing planner test file pass, including late success, cancel
and abort after an earlier fault, plus legacy/nonfatal transition controls.
Lint passes on the two changed Python files. A line-length correction after
the functional run has identical test AST; no functional retest was needed.

This preserves actual ABORTED/STOP policy and accurate failed-work state;
it does not fix the source of Nav2's missing progress or establish improved
coverage. Consolidate it and collect the already-queued approach case668.
That case uses its earlier immutable sources and does not include this
bookkeeping change. Further recovery expansion awaits causal native data.


The [approach-start native case](../src/oomwoo_sim_support/docs/evidence/nav_progress_approach_delivery_20261030.json)
stopped before sending any Nav2 goal. Starting in contact with the table
triggered SWEEP to ESCAPE at 9.0 simulated seconds, then FAULT at 9.6.
The relay's first revocation records nav2_idle=false with no active UUID,
while its other general-health fields are true. Its empty-journal
managed-idle policy excluded ESCAPE, although its sweep-health policy
already accepted that phase. Thus this run does not reproduce abort105.

All eight source hashes match and there is no host timeout. Gazebo later
segfaults139 during native-stop shutdown, and the wrapper correctly returns
exit3. The sampled Keepout audit has zero crossings. These facts establish
neither successful navigation nor a clean full bank. The conditional
full-history preparation was halted before creating snapshots when this
new result appeared; the actual contact-start failure takes priority.

The jobs branch now includes b9ae33e: only ESCAPE is added to the existing
empty-journal non-navigation phases. All 55 cases in its sequenced-ingress
file pass, including nine negative proof subcases and prior-phase controls.
A native contrast is queued as FIFO671 with the exact same spawn/goal and
seven unchanged input hashes. Only the relay phase-consistency source
changes. Waiting is capped at .05 CPU, automatically restoring 2 CPU after
verified own admission. Collect that case before claiming native escape
continuity or preparing a complete-room replay. Original abort105 and the
90/80 masked-room acceptance remain unresolved.


A [private full-history observer fixture](../src/oomwoo_sim_support/docs/evidence/history_trace_fixture_20261030.json)
is prepared and passes three focused cases. Original full-room planning,
gap-fill and all non-observer methods remain AST-identical to ROS7896286
and jobsb9ae33e. Existing status replies log at most 192 samples for the
selected target (4.225, 0.025); other targets do not consume that budget.
The scorer retains one initial feedback/truth pair for up to 32 recent
UUIDs plus its final 192-pair ring. The existing status RPC also returns
pose and source ages already in cache, without another TF lookup.

Checks cover selection, bounded immutable original sources, continued status
updates after the cap, and exactly one existing health evaluation per status
reply. Source patches are retained with the receipt for review. No new
ROS nodes, subscriptions, clients or timers were added. This is preparation
in frozen private copies, with no native result or production interface
change. The fixture is now queued as673 after initial-escape671 supplied the
next decision. Original abort105 and full90/80 acceptance remain pending.


The [native initial-escape contrast](../src/oomwoo_sim_support/docs/evidence/initial_escape_delivery_20261030.json)
now validates b9ae33e in Gazebo: the escape traverses 0.21634 m over
130 truth poses and returns to SWEEP without revoking its lease or entering
FAULT. Seven other source fingerprints match the previous failing case.
All eight stay unchanged during this run. Nineteen auxiliary exits are
clean and native-stop acknowledges. The scorer's unchanged90/80 gate
returns its expected exit1 for this intentionally one-goal fixture.

No Nav2 UUID is sent: existing contact recovery marks the sole target as
a wedge pocket and the plan completes after pruning. Thus this is escape
continuity evidence, not a successful approach, an abort105 fix or a
full-room gain. A clean close here does not resolve the separate Gazebo
shutdown race. Consolidate the phase correction. Full-history673 now
reuses the original room plan, current functional fixes and tested bounded
observations to obtain the next integration evidence.


The [contact-pocket correction](../src/oomwoo_sim_support/docs/evidence/contact_pocket_delivery_20261030.json)
addresses a second behavior exposed by native671: a bumper contact near
(3.04384, 0.07799) excluded the unvisited target (4.225, 0.025), 1.182 m
away. Held-contact recovery now records its pocket at the existing fresh
map-frame robot pose. Missing or nonfinite pose does not fabricate a
pocket. The radius, reflex command, escape budget and ownership handshake
are unchanged. The separate repeated-failed-goal wedge policy is unchanged.

The original method fails an actual waypoint-filter causal case. All 91
cases in the existing planner file now pass, including distant/near
targets, missing/nonfinite positions, contact deduplication and the existing
relay cancellation/release/claim ordering. Lint passes on the two changed
Python files. The first causal harness attempt lacked the existing logger's
max_escapes field; that fixture initialization was corrected before the
causal retry. No new node, subscription, client or timer was added; the
contact recorder reuses one existing fresh-pose lookup per bounded reflex.

This establishes accurate contact/waypoint bookkeeping in focused cases,
not native navigation or a full-room gain. Reserve a short same-spawn
contrast behind full-history673, changing only this contact method from
the previous case671. The existing full-history source is frozen and
excludes this new correction, preserving its original diagnostic context.


The [full-history native result](../src/oomwoo_sim_support/docs/evidence/nav_progress_history_delivery_20261030.json)
ends at 54.43% coverage: nine navigation goals succeed and the tenth is
canceled after the original map-freshness check revokes the lease. Its
post-filter ROS age is exactly 1 s and original DDS source age is
1.001946663 s, beyond the existing 1 s limit. Other health proofs remain
valid. Nineteen auxiliary processes close cleanly; all eight source hashes
remain unchanged. The selected focal target is never reached, and abort105
is not reproduced. The sampled Keepout audit records 22,759 poses and
22,758 chords with zero footprint crossings. Full90/80 remains unmet.

The separate [native contact-pocket contrast](../src/oomwoo_sim_support/docs/evidence/contact_pocket_native_delivery_20261030.json)
confirms that the previously discarded target is retained after contact
escape and an actual Nav2 UUID is sent. Nav2 then aborts immediately with
error208; explicit STOP is preserved. This establishes accurate target
bookkeeping and an attempted transit, not successful navigation. All eight
source hashes remain unchanged and nineteen auxiliary exits are clean.
The full-history run excludes the pocket correction, preserving isolation.

Prioritize the observed map-publication cadence before another full-room
run. Compare the installed costmap binary at update2/publish2 and
update2/publish4 with original source timestamps and identical geometry.
Keep the 1 s freshness limit and fatal-navigation behavior unchanged.


The [actual-binary cadence contrast](../src/oomwoo_sim_support/docs/evidence/costmap_cadence_delivery_20261030.json)
reproduces a 1 s publication interval with global update2/publish2.
Changing only publish_frequency to4 yields 0.5 s intervals over fifteen
measured gaps, with identical map geometry and original ROS/DDS stamps.
This is consistent with Nav2 1.3.13's strict publication-time comparison
inside its existing update loop. Global update_frequency remains2; no
additional update loop or timer is introduced. The original1s freshness
limit remains unchanged. Serialization may occur on each existing update;
end-to-end resource cost is not yet measured.

The standalone probe is NOT an overall pass: both measurements finish,
but the target exits-11 during shutdown, including after successful
deactivate/cleanup transitions. Preserve those receipts and stop that
cleanup line after two identical outcomes. The initial attempt also
records the corrected Jazzy metadata/await transport mistakes. Validate
the configuration independently through the existing normal Gazebo bank,
whose preceding run closed nineteen auxiliary processes cleanly. Native
coverage90/efficiency80 and the earlier navigation aborts remain open.

The [configuration check](../src/oomwoo_sim_support/docs/evidence/costmap_cadence_config_20261030.json)
confirms that only global publish_frequency changes semantically. Both
existing Jazzy launch-graph cases execute and pass, including opt-in
Keepout overlay generation. This is not the pending native room result.


The [normal native cadence contrast](../src/oomwoo_sim_support/docs/evidence/costmap_cadence_native_delivery_20261030.json)
finishes at95.50% coverage, sweep_complete, without lease revocation or
motion fault. Twenty-two actual goals succeed and one is canceled by the
existing timeout. The selected goal(4.225,0.025) succeeds in10.45simsec;
its52 existing-status observations keep map_current true, with maximum
original DDS costmap age472231652ns. All nineteen auxiliary processes
close cleanly; the scorer alone returns its proper exit1. All nine
frozen input hashes remain unchanged; the previous eight code hashes
match baseline673 and only global publication cadence changes. The
sampled Keepout audit has47140poses,47139chords and zero disk crossings.

The full90/80 gate still FAILS: efficiency at the first90% crossing is
78.19%, although coverage subsequently reaches95.50%. The first crossing
meter record has path109.48m versus ideal85.60m: this run would need
approximately2.48m less path to reach80%, with the same measured coverage.
That arithmetic is not a predicted optimization gain. Latest contact-
pocket9d is deliberately excluded; no universal abort105 repair, joint
docking or physical-robot result is claimed.

Consolidate the source-cadence correction. The next small diagnostic will
rank path length and coverage gain of already-recorded pre90 transits,
retaining original time sources and stating the1Hz alignment uncertainty.
Implement another route change only if that evidence identifies useful
savings; do not repeat this complete room without a discriminating change.

Existing1Hz meter samples aligned to original status events identify
pre90 transits of approximately5.67m/0.95percentage-points new coverage,
4.49m/0.17points and3.44m/0.30points. Nearest-sample boundary error is
at most0.452s and log-clock alignment spread0.000405s. These are ranked
cost observations, not exact transit lengths or evidence those transits
can be removed. The earlier actual-entry-only static candidate saved0
and should not be repeated unchanged. Assess a distinct entry/exit
ordering hypothesis against the same map and waypoint set before any
production route edit or another native room.


Two bounded static alternatives are rejected without changing production:
[fixed-order entry/exit optimization](../src/oomwoo_sim_support/docs/evidence/cell_entry_exit_comparison_20261030.json)
saves only0.325m of a130.198m plan;
[a single pass of whole-cell reversals](../src/oomwoo_sim_support/docs/evidence/cell_reversal_comparison_20261030.json)
saves0. Both retain the same326 free targets. These benefits do not justify
extra planner code or another room simulation. Do not repeat static
ordering variants without new evidence.

Native679's remaining goal290 cancellation is not proven to need more
time: its final feedback reports0.285m remaining, but path length is
almost constant in the last seconds. Extending its budget could hide
a different problem. The managed scene selects the installed Nav2 tree
that plans only when the goal is updated. A private follow-up may instead
reuse installed navigate_w_replanning_distance.xml, whose DistanceController
recomputes after1m on the existing tree tick. It adds neither recovery
actions nor a RateController, and retains the same planner/controller
writers. This is a hypothesis to check, not a proven stall repair.


The [private distance-replanning graph check](../src/oomwoo_sim_support/docs/evidence/distance_replan_graph_final_20261030.json)
passes four baseline/candidate and normal/managed combinations using
the actual Jazzy Node constructor and extracted navigation_setup function.
Installed XML/plugin checks confirm1m DistanceController, one
ComputePathToPose and one FollowPath, with no RateController, RecoveryNode,
Spin, BackUp, Wait or ClearEntireCostmap. Managed lifecycle servers and
quarantined external goal remapping are retained; normal-mode selection
is unchanged. No ROS process is executed by this graph check.

The initial check failed because ROS_LOG_DIR was not assigned inside a
read-only container. Its [failure receipt](../src/oomwoo_sim_support/docs/evidence/distance_replan_graph_20261030.json)
is preserved; assigning the existing bounded/tmp log convention corrects
that fixture setup. No application fix is attributed to this correction.

A single full-room contrast is reserved as FIFO688, waiting behind687,
with .05CPU while queued and2CPU/3GiB only after verified own admission.
The frozen copy changes only the managed stock-tree filename functionally.
The existing bounded planner observer alone is retargeted to(1.525,-2.525);
its other methods are AST-identical and eight other prior source hashes
match679. sim_bringup is a tenth immutable input. Production still uses
the previously validated configuration. Collect the same native handle;
this graph result proves neither renewed runtime paths nor a stall fix,
efficiency gain or complete90/80 acceptance.


The [private distance-replanning result](../src/oomwoo_sim_support/docs/evidence/distance_replan_native_delivery_20261030.json)
is rejected for adoption. The scorer finishes at95.41% coverage and
78.36% efficiency at the first90% crossing, still below80%. Twenty-one
goals succeed and one is canceled. Thirty-five controller path updates
confirm the installed distance tree actually executes, versus0 in679,
but the same focal target still times out after30.02simsec. Its150
status observations retain map_current; the last33 observations span
6.4s with zero translational relay output and a small angular output.
The exact goal-frame/controller cause is unproven. All ten input hashes
stay unchanged. Production still uses its original managed tree.

The final bank is NOT a clean exit. Gazebo acknowledges native stop but
remains live; scorer/report/logs are preserved in a separate snapshot
before stopping the exact owned container. Podman escalates after15s,
explaining controller rc137. No OOM or quota cause is inferred. Only two
early auxiliary exits are clean before that intervention, not nineteen.
FIFO688 and its scope are terminal. The independent Keepout audit also
has valid:false because of an invalid/stale truth sample. Its0 counted
crossings must not be presented as complete Keepout validation, despite
the separate coverage meter emitting measurement_valid:true.

Prioritize a small causal grading check before another physics run:
can an invalid independent audit, or a detected forbidden crossing,
coexist with a positive coverage/efficiency result? Reuse the existing
audit/scorer file and real methods. If confirmed, fuse that existing
proof into the existing invalid/fault outcome, preserving unmasked
behavior and existing thresholds. No new policy switch or validation
process is proposed. Shutdown-ACK liveness and exact near-goal behavior
remain separate unresolved issues.
