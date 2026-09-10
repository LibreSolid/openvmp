# Restate Don1's cycle-2 joint sites in the declaring body's own frame

## Why

solid-node ADR-097 (`joint-frame-follows-declarer`, integrated into main at
91c0b2a) changed how a class-body joint (`Revolute`, `Prismatic`, `Orbit`,
`Free`) reads its `axis`, `at` and `carries`: they are now read in the
declaring body's OWN rest frame, not the parent's, and `at` defaults to the
body's own origin. An `at` that restated the parent's placement now applies
that offset a second time (once in the parent's `render()`, once again
inside the joint's own frame) and is wrong until deleted; an axis on a body
its parent rotates must be rewritten in the body's own frame.

This project is broken on main today for exactly that reason. All seven
`Revolute` joints in `simulation/don1/robot.py` (`Wheel.spin`, `Foot.knee`,
`Leg.turn`, `CameraArm.tilt`, `Camera.pan`, `Hip.roll`, `Side.yaw`) declare
an `at=` that restates a parent `render()` placement, and two of them
(`Wheel.spin`, `Leg.turn`) sit on a body their parent also rotates, so
their `axis=` needs re-expression too. Running the faceted contract suite
on the unmodified tree confirms the break: 14 of 30 tests in
`simulation/don1/test_robot.py` fail — every pose-dependent joint test
(wheel spin, knee bend, thigh turn, hip roll, camera pan/tilt, yaw, the
motion-clearance sweep) — while the placement-only tests (blueprint
tracking, part counts, rest overlaps) stay green, exactly as expected from
a doubled joint offset rather than a broken import or a bad placement.

## What changes

All seven joints in `simulation/don1/robot.py`:

- **`Wheel.spin`**: `at=WHEEL_OFFSET` restated `Foot.render()`'s
  `self.wheel.rotate(90, X).translate(WHEEL_OFFSET)` exactly — dropped
  (defaults to the wheel's own placed origin). `axis=(0, -1, 0)` was
  parent-frame; because `Foot.render()` also turns the wheel 90 degrees
  about X before placing it (not just translating it), the axis itself
  must be re-expressed too: `R_rest^-1 @ (0,-1,0) = (0, 0, 1)`. New:
  `axis=(0, 0, 1)`.
- **`Foot.knee`**: `at=KNEE_SHAFT` was a genuine parent-frame offset under
  `Leg.render()`'s `self.foot.rotate(FOOT_REST_BEND, X).translate(FOOT_OFFSET)`
  (`FOOT_REST_BEND = 39.0`, a real non-identity rotation). The module
  constant `KNEE_SHAFT` is redefined in place — its one use site
  (`at=KNEE_SHAFT`) is untouched — from the leg-frame value
  `[0.0, -316.4, 108.8]` to the foot's own rest frame:
  `R_rest^-1 @ (KNEE_SHAFT - FOOT_OFFSET) = [0.0, -8.905712537202675,
  7.276041795145978]`, computed with `Rx(39°)` and the actual `FOOT_OFFSET`/
  `KNEE_SHAFT` literals (verified with numpy; matches the archived
  framework change's own estimate of `~(0, -8.906, 7.276)`). `axis=(1, 0,
  0)` is the rest rotation's own axis, unchanged by construction.
- **`Leg.turn`**: `at=SIDE_OFFSET` restated `Hip.render()`'s
  `leg.rotate(90 + 90*side, Z).translate(SIDE_OFFSET)` at both
  `Leg(side=1)`/`Leg(side=-1)` sites — dropped. `axis=(0, -1, 0)` was one
  literal shared by both legs in the hip's frame, but `Hip.render()`
  rotates the two legs 180 degrees apart (`Rz(180°)` left, `Rz(0°)` right),
  so the own-frame axis genuinely differs by leg: `(0, 1, 0)` for
  `side=1`, `(0, -1, 0)` for `side=-1`. `Leg` already declares its own
  `side = Count(1, min=-1, max=1)`, so the axis becomes a callable of the
  realized node: `axis=lambda node: (0, node.side, 0)`. This is the
  bridge cycle 2 offers for a per-site axis; a later framework cycle
  ("site-keyword" joints) would let `Hip` state this at the two
  `Leg(side=...)` instantiation sites instead of a callable inside `Leg`
  itself.
- **`CameraArm.tilt`**: both `axis=lambda node: (0.0, -node._hand, 0.0)`
  and `at=lambda node: (...)` existed only to restate `Camera.render()`'s
  per-corner placement
  (`self.camera.rotate(self.side * -90, Z).translate([self.dir * self.side
  * x, y, z])`, four corners: `dir` and `side` each `±1`). Computed for
  all four corners: the anchor restates the translation exactly at every
  corner (dropped); the axis collapses to the SAME literal, `(1, 0, 0)`,
  at every corner — `dir` only ever affects the translation, and the two
  possible `±90°` Z rotations each exactly cancel their own `side`-driven
  sign flip of the old axis. New: `tilt = Revolute(axis=(1, 0, 0),
  range=SERVO_RANGE, unit='deg')`, no callable needed. The now-unused
  `_end`/`_hand` fields (set only to feed the two lambdas) are deleted
  from `CameraArm.__init__`.
- **`Camera.pan`**: `at=(...)` restated `Hip.render()`'s
  `camera.translate(...).rotate(90 + 90*side, Z).translate(SIDE_OFFSET)`
  at both `side=±1` sites — dropped. `axis=(0, 0, 1)` is Z, and both of
  `Hip.render()`'s possible rest rotations (`Rz(180°)`, `Rz(0°)`) leave Z
  unchanged — no axis rewrite.
- **`Hip.roll`**: `at=HIP_OFFSET` restated `Side.render()`'s pure
  `self.hip.translate(HIP_OFFSET)` (no rotation) — dropped. `axis=(1, 0,
  0)` unchanged (identity rest rotation).
- **`Side.yaw`**: `at=(dir * END_OFFSET, 0, END_HEIGHT)` restated
  `Don1.render()`'s placement of both `front` (`dir=1`, identity
  rotation) and `rear` (`dir=-1`, `Rz(180°)`) — dropped at both sites.
  `axis=(0, 0, 1)` is Z, unchanged under either rest rotation.

No `at`/`axis` needed re-derivation beyond what is listed above; every
other joint's axis was already the rest rotation's own axis or a line Z
invariant under it.

### Confirmed out of scope, not touched

- `simulation/don1/link.py`'s eight `spin()` sites on the `.assy`-built
  drive-train parts (worms, worm gears, sprockets, shafts): these are
  hand-written helpers, not `Revolute`/`Prismatic`/`Orbit`/`Free`
  declarations, and never read a joint's `axis=`/`at=`/`carries=`
  argument. They wait for a future site-keyword joint (cycle 3), as the
  assignment specifies; the project clearance engine is likewise
  untouched.
- Every other placement (`render()` translate/rotate calls), driver,
  relation (`front_yaw.drives(front.yaw)` etc.), instruction, and test
  assertion.

### One deviation from the survey, verified empirically

The assignment's per-project briefing (drawing on the earlier framework
survey) also asked to remove `Leg.simulate()`'s hand-written handedness
correction at line 129 (`-self.turn.value * self.side`), on the theory
that the callable axis now carries the handedness itself. Pose evidence
says otherwise: removing the `* self.side` term (tested, then reverted)
introduces a **47.88 mm** deviation on every right-leg pose (`coupler`,
`motion-collar_clamping_8mmREX`, `rls-shaft` under `hip.right_leg.thigh`,
both ends) — a real break, not noise. The formula is unchanged by this
migration: `Leg`'s own `render()` and internal structure are entirely
side-independent (only `Hip.render()`'s placement of the whole `Leg`
differs by side), so the physical rotation a fixed `turn.value` produces,
expressed in the thigh's own local frame — the frame `axis_of(thigh,
'rls-shaft', ...)` is read in — is provably identical before and after
ADR-097 (that identity is exactly what makes the migration pose-preserving
in the first place: the framework composes the joint's local rotation at
the same point in the chain, with a numerically identical local axis,
either way). `Leg.simulate()`'s dressing is left untouched. See "What the
rule could not state" below.

## What does not change

- Every driver, relation (`front_yaw.drives(front.yaw)`,
  `front_left_knee.drives(front.hip.left_leg.foot.knee,
  offset=-FOOT_REST_BEND)`, etc.), instruction (`Rest`, `Stand`, `Crouch`,
  `Hug`, `Walk`, `Duct`, `TurnLeft`, `TurnRight`, `Roll`, `Look`), and
  guard in `check()`.
- Every `render()` placement (`translate()`/`rotate()` calls unchanged)
  and every hand-turned `simulate()` drive-train dressing
  (`Leg.simulate()`, `Side.simulate()`, `Don1.simulate()`), including the
  `Leg.simulate()` handedness term discussed above.
- `WHEEL_OFFSET`, `SIDE_OFFSET`, `HIP_OFFSET`, `CAMERA_OFFSET`,
  `END_OFFSET`/`END_HEIGHT`: still used by the `render()` placements they
  always described; only `KNEE_SHAFT` (used solely by the joint's own
  `at=`) is recomputed into the new frame.
- `simulation/don1/link.py`, `clearance.py`, every test assertion.

## Evidence

- **Poses.** BEFORE reused from the framework cycle's own capture on the
  pre-ADR-097 source (`<scratchpad>/before2/openvmp-don1-before.json`, 53
  poses, 499 leaves). AFTER captured on solid-node main (91c0b2a) from
  this project's refactored source. `capture_poses.py compare`: **max
  deviation 0.000e+00 over 53 poses.**
- **Suite, before (unmodified tree, on main — the broken state this
  change fixes):** `solid test --faceted don1`: **16 passed, 14 failed**.
  Every failure is pose-dependent: `test_a_spun_wheel_stays_put`,
  `test_the_foot_bends_about_the_knee_shaft`,
  `test_the_thigh_bearings_stay_on_the_thigh_axis`,
  `test_the_four_legs_turn_in_the_same_sense`,
  `test_the_hip_hub_stays_on_the_roll_axis`,
  `test_roll_turns_the_hip_about_the_turntable_bearings`,
  `test_pan_turns_the_base_about_the_vision_servo`, `test_tilt_follows_pan`,
  `test_yaw_turns_the_table_about_the_base_bearings`,
  `test_a_half_turn_of_the_thigh_brings_the_foot_under_the_body`,
  `test_stand_lifts_the_body_onto_the_feet`, `test_hug_folds_the_feet_back`,
  `test_a_pose_that_must_collide_is_caught` and
  `Don1MotionTest.test_no_new_overlap_along_the_moves` — consistent with
  the doubled joint offset pushing every posed body away from where its
  own tests, or the clearance engine, expect it.
- **Suite, after (this change):** `solid test --faceted don1`: **30
  passed, 0 failed** — the exact pre-ADR-097 baseline recorded in the
  tracker's `move-onto-motion` row (`30/30 faceted green`); no test
  changed state.

## What the rule could not state

Nothing new about the rule itself. Two findings, both already recorded by
the framework survey and confirmed here, not reopened:

- `Leg.turn`'s axis needs a callable of the realized node because the
  parent (`Hip`) places two instances of the same class 180 degrees apart
  by a parameter (`side`) the class itself owns; a future site-keyword
  joint (cycle 3) would let `Hip` state the axis at the `Leg(side=...)`
  site instead. Not blocking here — the callable is a correct, verified
  bridge.
- The eight `link.py` `spin()` sites remain hand-written for the same
  reason (drive-train parts placed by blueprint data, not declared as
  jointed nodes) — unaffected by this cycle, waiting on the same future
  primitive.

One correction to the assignment's own briefing, evidenced above: the
`Leg.simulate()` handedness term (`* self.side`) does NOT go away under
this rule; pose evidence shows removing it breaks the right leg by 47.88
mm. This is reported for the orchestrator, not filed to
`solid-node/workflow/warts.md` directly.
