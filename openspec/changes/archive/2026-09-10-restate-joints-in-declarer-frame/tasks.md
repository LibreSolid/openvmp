Behaviour-preserving: the acceptance is that every leaf's world matrix is
unchanged at every pose, so evidence is captured before the first edit and
compared after the last (BEFORE reused from the framework cycle's own
capture on the pre-ADR-097 source). No test is edited; none needs to
change.

## 0. Confirm the break and the repository state

- [x] 0.1 Confirm the repository is clean (`git status --porcelain` empty
      before any edit).
- [x] 0.2 Confirm the project imports on solid-node main (91c0b2a): it
      does — the break is wrong poses, not an import failure.
- [x] 0.3 Run the suite on the unmodified tree to record the broken
      baseline: `solid test --faceted don1` — 30 tests, **16 passed, 14
      failed** (every pose-dependent joint test, listed in `proposal.md`).

## 1. Restate the seven joints in their own declaring frame

- [x] 1.1 `Wheel.spin` (`simulation/don1/robot.py`): `at=WHEEL_OFFSET`
      dropped (RESTATES `Foot.render()`'s translation exactly); `axis`
      recomputed `(0, -1, 0)` → `(0, 0, 1)` (Foot's 90° X rest rotation on
      the wheel is a real, non-identity rotation).
- [x] 1.2 `Foot.knee`: module constant `KNEE_SHAFT` recomputed in place
      from `[0.0, -316.4, 108.8]` (leg frame) to
      `[0.0, -8.905712537202675, 7.276041795145978]` (foot's own rest
      frame), by inverting `Leg.render()`'s `rotate(FOOT_REST_BEND,
      X).translate(FOOT_OFFSET)` — verified with numpy:
      `Rx(39°)^-1 @ (KNEE_SHAFT_old - FOOT_OFFSET)`. `axis=(1, 0, 0)`
      unchanged (it is the rest rotation's own axis). The joint's own
      `at=KNEE_SHAFT` line is untouched; only the constant's value and
      its comment changed.
- [x] 1.3 `Leg.turn`: `at=SIDE_OFFSET` dropped (RESTATES at both
      `side=1`/`side=-1` sites). `axis` becomes
      `lambda node: (0, node.side, 0)` — `Hip.render()` places the two
      legs 180° apart (`Rz(180°)` left, `Rz(0°)` right), so the one
      physical bearing line reads oppositely in each leg's own frame;
      `Leg`'s own `side = Count(1, min=-1, max=1)` is the distinguishing
      parameter (ADR-088 callable-of-realized-node).
- [x] 1.4 `CameraArm.tilt`: both the `axis=lambda node: (0.0,
      -node._hand, 0.0)` and `at=lambda node: (...)` deleted. Verified
      numerically for all four `(dir, side)` corners that the anchor
      restates `Camera.render()`'s translation exactly at every corner,
      and the axis collapses to the SAME literal, `(1, 0, 0)`, at every
      corner (the two possible ±90° Z rest rotations each exactly cancel
      their own side-driven sign flip; `dir` never enters the rotation).
      New: `tilt = Revolute(axis=(1, 0, 0), range=SERVO_RANGE,
      unit='deg')`. The now-dead `self._end, self._hand = dir, side`
      line is deleted from `CameraArm.__init__`; the rest of that
      `__init__` (`super().__init__(assembly, name=name, dir=side)`) is
      KEPT — it is not joint machinery, it forwards this arm's `side` as
      the blueprint's own `dir` parameter (the mirrored STEP variant the
      `link-camera.assy` file understands), which is unrelated to
      ADR-097 and still needed (see "Deviations" below).
- [x] 1.5 `Camera.pan`: `at=(...)` dropped (RESTATES at both `side=±1`
      sites, verified numerically). `axis=(0, 0, 1)` unchanged (Z is
      invariant under both of `Hip.render()`'s possible rest rotations).
- [x] 1.6 `Hip.roll`: `at=HIP_OFFSET` dropped (RESTATES
      `Side.render()`'s pure translation). `axis=(1, 0, 0)` unchanged
      (identity rest rotation).
- [x] 1.7 `Side.yaw`: `at=(dir * END_OFFSET, 0, END_HEIGHT)` dropped
      (RESTATES `Don1.render()`'s placement at both `front`/`rear`
      sites). `axis=(0, 0, 1)` unchanged (Z is invariant under both
      identity and `Rz(180°)`).
- [x] 1.8 Confirm `simulation/don1/link.py`'s eight `spin()` sites are
      not `Revolute`/`Prismatic`/`Orbit`/`Free` declarations and read no
      joint's `axis=`/`at=`/`carries=` argument — out of scope per the
      assignment, left untouched.
- [x] 1.9 Confirm the edited file still parses (`ast.parse`) and the
      project still imports.

## 2. Deviation from the survey, checked empirically

- [x] 2.1 The assignment's briefing (from the framework survey) also
      asked to delete `Leg.simulate()`'s hand-written handedness
      correction (`-self.turn.value * self.side` → `-self.turn.value`)
      at `robot.py` line 129, on the theory that the callable axis now
      carries the handedness. Applied the edit as a TEST (not committed),
      captured AFTER poses, and compared: **max deviation 4.788e+01 mm**,
      confined to `hip.right_leg.thigh.{rls-shaft, coupler,
      motion-collar_clamping_8mmREX}` at both `front` and `rear`, at
      every pose that moves `right_thigh` away from 0. Reverted the edit
      immediately.
- [x] 2.2 Re-derived why: `Leg`'s own `render()`/structure is entirely
      side-independent (only `Hip.render()`'s placement of the whole
      `Leg` differs by side), so the physical rotation a fixed
      `turn.value` produces, expressed in the thigh's own local frame
      (the frame `axis_of(thigh, 'rls-shaft', ...)` reads), is
      numerically identical before and after ADR-097 — that identity is
      exactly what makes this migration pose-preserving at all: the
      framework composes the joint's local rotation at the same point in
      the chain, with a numerically identical local axis vector, either
      way (`new_local = R_rest^-1 @ old_local`, so a fixed `.value`
      produces the identical local-frame transform regardless of which
      convention declared the axis). The hand correction's job is
      unrelated to which frame `axis=` is written in; it maps the
      physical (already-fixed) rotation onto a SEPARATE fixed local axis
      that `axis_of()` reads straight off the mesh, and that mapping does
      not change. `Leg.simulate()` line 129 is therefore left exactly as
      it was.
- [x] 2.3 Also confirmed the `__init__` override on `CameraArm` (item 1.4
      above) is not purely joint machinery, against the assignment's
      claim that "`_end`, `_hand` and the `__init__` override exist only
      to restate the placement": `super().__init__(assembly, name=name,
      dir=side)` forwards `side` as the blueprint's `dir` parameter
      (confirmed against `blueprints.py`'s docstring — "`dir` for the
      handed camera links" — and grepped: no assembly parameter named
      `side` exists), unrelated to either joint lambda. Deleted only the
      dead `_end`/`_hand` assignment; kept the `__init__` override and its
      `dir=side` forwarding.

## 3. Evidence

- [x] 3.1 AFTER poses captured with `capture_poses.py capture
      simulation.don1.robot:Don1 <out.json>`: 53 poses, 499 leaves.
- [x] 3.2 `capture_poses.py compare` against the framework cycle's BEFORE
      capture (pre-ADR-097 source, `<scratchpad>/before2/openvmp-don1-before.json`,
      53 poses, 499 leaves): **max deviation 0.000e+00 over 53 poses.**
- [x] 3.3 Re-run the suite: `solid test --faceted don1` — 30 tests, **30
      passed, 0 failed** — the exact pre-ADR-097 baseline recorded in the
      tracker's `move-onto-motion` row; no test changed state.

## 4. Commit

- [x] 4.1 Commit the `simulation/don1/robot.py` edit as one commit naming
      ADR-097 and the pose/test evidence. Do not sync or archive this
      change — the orchestrator reviews first.

## Findings for the framework (orchestrator, not the implementer)

1. **The assignment's briefing was wrong about `Leg.simulate()`'s
   handedness term.** It predicted `-self.turn.value * self.side` "goes"
   once `turn`'s axis becomes a per-side callable. Pose evidence
   (section 2 above) shows removing it breaks the right leg by 47.88 mm;
   the term is independent of the frame convention and must stay. Not a
   framework gap — a survey misjudgment on this one project, corrected
   here with the numeric proof.
2. **`Leg.turn`'s axis needed a callable of the realized node**
   (`side`) because `Hip` places two `Leg` instances 180° apart by a
   parameter the class itself owns — confirms the same
   declaration-site-joint gap already on file (a future site-keyword
   joint would let `Hip` state this at the `Leg(side=...)` site instead
   of inside `Leg`). Not new; not re-filed.
3. **The `link.py` `spin()` sites remain hand-written**, confirmed again
   not to be joint declarations at all (they read no `axis=`/`at=`/
   `carries=`) — same pre-existing gap, not re-filed.

No other sentence this project wanted was unreachable under the rule as
specified.
