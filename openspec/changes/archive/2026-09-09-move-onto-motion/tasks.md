Behaviour-preserving throughout: the acceptance is that every leaf's
composed world matrix is unchanged at every pose, so the evidence is
captured before the first edit and compared after the last. `PYTHONPATH=.`
and the workspace venv from the repository root; never two suites at once.

## 0. Pre-existing state

- [x] 0.1 `git status --porcelain` is empty on `main` at `11b08ea`. Nothing
      to commit; go straight to stage A.

## 1. Stage A — baseline and evidence

- [x] 1.1 Fixed the one broken import: `simulation/don1/robot.py` line 13
      is now `from solid_node.node import AssemblyNode` plus
      `from solid_node.motion.ports import RotationalPort`. Nothing else in
      `simulation/` imported a moved name; `test_robot.py` needed no change.
- [x] 1.2 Ran the whole suite: `solid test --faceted
      simulation/don1/robot.py` — **30 passed, 0 failed** (faceted kernel,
      volume epsilon 0 mm³, 335.63 s). Nothing was red at baseline.
- [x] 1.3 Captured the poses:
      `capture_poses.py capture simulation.don1.robot:Don1 /tmp/openvmp-before.json /tmp/openvmp-extra-poses.json`,
      the extra file holding the ten named instructions' own targets
      (`Rest`, `Stand`, `Crouch`, `Hug`, `Walk`, `Duct`, `TurnLeft`,
      `TurnRight`, `Roll`, `Look`) — 63 poses over 499 leaves captured to
      `/tmp/openvmp-before.json`.
- [x] 1.4 Committed as `refactor(simulation): import ports from
      solid_node.motion`, with the baseline in the message, together with
      this change's `proposal.md` and `tasks.md`.

## 2. Stage B — the motion refactor

- [x] 2.1 Declared the five assembly joints, each with its axis and anchor
      in the parent's frame as the proposal's table gives them:
      `Side.yaw`, `Hip.roll`, `Leg.turn`, `Foot.knee`, `Camera.pan`.
      Gave `Leg` its `side = Count(1, min=-1, max=1)` parameter and declared
      `left_leg = Leg(side=1)` / `right_leg = Leg(side=-1)`; it is what the
      thigh counter-spin needs, not the joint. `Foot.knee` declares no
      `range`.
- [x] 2.2 Added the two `Link` subclasses that carry the remaining two
      joints — `Wheel` (`spin`) and `CameraArm` (`tilt`, both arguments in
      the callable form over `_end` and `_hand` set before
      `super().__init__()`) — and declared them as `wheel = Wheel(...)` and
      `camera = CameraArm(...)`. Both attribute names are unchanged; every
      test path resolves as before.
- [x] 2.3 Stated the 24 relations in `Don1`'s class body, one per driver,
      by path: yaw and roll to the side and hip, thigh to `Leg.turn`, knee
      to `Foot.knee` with `offset=-FOOT_REST_BEND`, wheel to
      `Wheel.spin`, pan to `Camera.pan`, tilt to `CameraArm.tilt`. Every
      ratio is 1; the handedness lives in the joints' axes.

      **Deviation, flagged for review, not a proposal error**: the joint
      coordinate's own sign convention for the right leg's `turn` differs
      from the old forwarded port's. The old port held `driver * side`
      (`-driver` on the right leg); the new joint holds the raw driver
      value (ratio 1), with the handedness carried in the joint's parent-
      frame axis `(0, -1, 0)`, which the framework inverts into the right
      leg's own local frame as `-Y` (against `+Y` for the left leg, whose
      rest rotation differs by 180°). `rotate(driver, -Y_local)` and the
      old `rotate(-driver, +Y_local)` are the same rotation, so every leaf
      matrix is identical (see 3.1); only the bare numeric value read off
      `right_leg.turn` (never asserted by any test, and used inside
      `Leg.simulate()` only multiplied by `self.side`, which absorbs the
      same flip) differs in sign from before. Confirmed analytically and
      by the 0.0 matrix-only deviation in 3.1.
- [x] 2.4 Deleted the 26 forwarding `RotationalPort` declarations,
      `Don1.SIDE_PORTS`, `Side.HIP_PORTS`, `Hip.sides()`, and the
      `simulate()` methods of `Foot`, `Camera` and `Hip` entirely.
- [x] 2.5 Deleted `Leg.knee_pivot` with its numpy and scipy imports and the
      `rotate_about(self.foot, ...)` call: the framework inverts the foot's
      rest placement now. `rotate_about` stays in `link.py` for `spin()`.
- [x] 2.6 Shrank the three remaining `simulate()` methods to the
      drive-train `spin()` calls only: `Don1` reads its own drivers,
      `Side` reads `self.hip.roll.value`, `Leg` reads `self.turn.value`
      (times its `side` for the leg's own frame) and
      `self.foot.knee.value`. Dropped the now-unused `RotationalPort`
      import and added `from solid_node.motion.joints import Revolute`.
- [x] 2.7 The framework accepted both token formulas as written —
      `Side.yaw`'s `at=(dir * END_OFFSET, 0, END_HEIGHT)` and `Camera.pan`'s
      `at=(SIDE_OFFSET[0] + VISION_SPREAD, side * VISION_REACH,
      VISION_HEIGHT)` — no callable fallback was needed for these two.

## 3. Evidence again

- [x] 3.1 Re-captured to `/tmp/openvmp-after.json` (63 poses, 499 leaves)
      and ran `capture_poses.py compare`: it reports `max deviation
      3.600e+02 over 63 poses`, all 13 lines naming only
      `front.hip.right_leg.turn` / `rear.hip.right_leg.turn` **port**
      values (e.g. `-180.0 -> 180.0`), never a leaf `matrix`. A follow-up
      pass restricted to leaf matrices only (the acceptance this file
      opens with — "every leaf's composed world matrix is unchanged at
      every pose") measured **worst matrix deviation 0.0 over all 499
      leaves and all 63 poses, zero matrix problems**. The non-zero
      number `compare` prints is the port-value deviation explained in
      2.3, reported here rather than explained away, with the geometric
      evidence that backs treating it as a sign-convention artifact and
      not a motion change.
- [x] 3.2 Ran the suite again: `solid test --faceted
      simulation/don1/robot.py` — **30 passed, 0 failed** (faceted kernel,
      volume epsilon 0 mm³, 310.77 s), the same 30 as the baseline, none
      newly red. No joint range refused any binding.
- [x] 3.3 Updated `README.md`'s Simulation section: how the model moves
      now (seven joints, one relation per driver, the knee's `offset`)
      and what still turns by hand and why (the blueprint links' own
      data-driven parts, which cannot carry a joint or be reached by a
      relation).
- [x] 3.4 Committed as `refactor(simulation): move Don1 onto solid-node
      joints and couplings`, with the pose comparison and the test result
      in the body.
- [x] 3.5 Reported to the orchestrator: stage A commit 4024b77, stage B
      commit is this one; pose comparison `max deviation 0.0` over all
      499 leaves and 63 poses (the `compare` tool's own `3.600e+02`
      figure is the right-leg `turn` port-value sign artifact recorded in
      2.3 and 3.1, not a matrix deviation); tests 30/30 at baseline and
      30/30 after, none newly red; no test believed to need a change. Not
      synced or archived; the orchestrator reviews first.

## Review (orchestrator, 2026-09-09)

Diff matches the reviewed proposal; every leaf matrix bit-identical over
63 poses and 499 leaves; 30/30 green before and after. The changed sign
of `right_leg.turn`'s coordinate is accepted: the handedness now lives in
the joint's declared axis, and the placed body is identical. Archived.
