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

- [ ] 2.1 Declare the five assembly joints, each with its axis and anchor
      in the parent's frame as the proposal's table gives them:
      `Side.yaw`, `Hip.roll`, `Leg.turn`, `Foot.knee`, `Camera.pan`.
      Give `Leg` its `side = Count(1, min=-1, max=1)` parameter and declare
      `left_leg = Leg(side=1)` / `right_leg = Leg(side=-1)`; it is what the
      thigh counter-spin needs, not the joint. `Foot.knee` declares no
      `range`.
- [ ] 2.2 Add the two `Link` subclasses that carry the remaining two
      joints — `Wheel` (`spin`) and `CameraArm` (`tilt`, both arguments in
      the callable form over `_end` and `_hand` set before
      `super().__init__()`) — and declare them as `wheel = Wheel(...)` and
      `camera = CameraArm(...)`. **Keep both attribute names**: every test
      path is built from child attribute names.
- [ ] 2.3 State the 24 relations in `Don1`'s class body, one per driver,
      by path: yaw and roll to the side and hip, thigh to `Leg.turn`, knee
      to `Foot.knee` with `offset=-FOOT_REST_BEND`, wheel to
      `Wheel.spin`, pan to `Camera.pan`, tilt to `CameraArm.tilt`. Every
      ratio is 1; the handedness lives in the joints' axes.
- [ ] 2.4 Delete the 26 forwarding `RotationalPort` declarations,
      `Don1.SIDE_PORTS`, `Side.HIP_PORTS`, `Hip.sides()`, and the
      `simulate()` methods of `Foot`, `Camera` and `Hip` entirely.
- [ ] 2.5 Delete `Leg.knee_pivot` with its numpy and scipy imports and the
      `rotate_about(self.foot, ...)` call: the framework inverts the foot's
      rest placement now. `rotate_about` stays in `link.py` for `spin()`.
- [ ] 2.6 Shrink the three remaining `simulate()` methods to the
      drive-train `spin()` calls only: `Don1` reads its own drivers,
      `Side` reads `self.hip.roll.value`, `Leg` reads `self.turn.value`
      (times its `side` for the leg's own frame) and
      `self.foot.knee.value`. Drop the now-unused `RotationalPort` import
      and add `from solid_node.motion.joints import Revolute`.
- [ ] 2.7 If the framework refuses a token formula in a joint argument
      (`dir * END_OFFSET`, `side * VISION_REACH`), use the callable form
      and record the deviation here.

## 3. Evidence again

- [ ] 3.1 Re-capture to `/tmp/openvmp-after.json` and
      `capture_poses.py compare`: expected maximum deviation 0 over 503
      leaves and every pose. Any non-zero deviation stops the change and
      is reported, not explained away.
- [ ] 3.2 Run the suite again: the same tests green as the baseline, none
      newly red. Report the counts before and after. If a joint range
      refuses a binding, drop that joint's `range=` — never edit a test.
- [ ] 3.3 Update `README.md`'s Simulation section to say how the model
      moves now — seven joints, one sentence per driver — and what still
      turns by hand and why (the blueprint links' parts).
- [ ] 3.4 Commit as `refactor(simulation): move Don1 onto solid-node
      joints and couplings`, with the pose comparison and the test result
      in the body.
- [ ] 3.5 Report the two commit hashes, the pose line, the test counts,
      every deviation from the proposal, and every test believed to need a
      change. Do not sync or archive; the orchestrator reviews first.
