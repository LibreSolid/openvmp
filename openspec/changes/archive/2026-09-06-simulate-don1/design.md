## Context

Don1 is a quadruped-with-wheels: a base carrying batteries, computers and
four steppers; a turntable at each end that the base yaws; a hip on each
turntable that the turntable rolls; two legs per hip, each a thigh that
turns about its own length and an L-shaped foot bent at a knee, with a
rubber wheel at the L's corner and a hook at its tip; a pan-tilt stereo
camera at each corner of each hip. The blueprint repository draws all of
it as PartCAD assemblies over the public PartCAD index, and this change
reads those drawings into a machine that moves.

The model is a solid-node project on the declarative node API. Every
placed part is a `CadQueryNode` that imports a STEP file; every `.assy`
file is one `Link` assembly that realizes the file's entries at the
file's locations; the robot file's nesting — which is the kinematic tree —
is written by hand in `robot.py` with the same numbers, because that is
where the joints are declared and the blueprint has no notation for a
joint.

## Coordinates

The base frame is the blueprint's: origin at the top board's centre, X
along the body toward the front, Z up. Each end's turntable group sits at
(±272.5, 0, -7) mm, the rear one turned 180° about Z so that its own frame
points outward; inside it the hip group is at (149.5, 0, -30) mm; inside
that each side's frame is at (90, 0, 0) mm, the left turned 180° about Z
so that both sides read the same leg file; the thigh at (0, -11.5, 0) mm
in the side frame, the foot at (0, -304.9, 108.75) mm bent 39° about X,
the wheel at (0, -293.5, -75.2) mm in the foot turned 90° about X, and the
vision assembly at (-side·32.5, -152, 86) mm in the side frame — all as
`robot.assy` writes them.

## Goals / Non-Goals

**Goals:**
- The `.assy` files are the machine. The layer places nothing by its own
  numbers except the joint axes it names and the poses it declares; every
  part sits where the blueprint puts it, at rest.
- The sourced parts are the index's own STEP files, not redrawings; the
  custom parts are the blueprint's STEP files.
- One slider per motor, the joint angle in degrees, on the root; the
  drive-train parts that visibly turn (worms, worm gears, sprockets)
  turn with it at their ratios.
- Poses that mean something: the blueprint's rest, the upstream motion
  scripts' stand, crouch and hug, a walk stance, a turn, a roll, a look.
- Every part coloured by what it is made of, chosen for contrast.

**Non-Goals:**
- Editing the blueprints or the ROS description. Disagreements are
  recorded, not fixed.
- Chains between sprockets, wiring, the brushless motors the two BLD-510S
  drivers serve (no motor part is placed), the four hip side servos (no
  horn or driven part is placed), the passive centre wheel (its axle and
  bearings are placed, the wheel is not).
- Ground contact, gait dynamics, body pose from the leg pose: the base
  frame is fixed and the floor is wherever the lowest part is.
- The other OpenVMP robots.

## Decisions

**The index's STEP files are the parts, fetched rather than committed.**
The 67 distinct parts total 105 MB of STEP, 96 MB of it vendor geometry
(the Intel NUC alone is 28 MB, the EGO battery 21 MB). Committing that
into a repository whose blueprint submodule is already optional would be
out of proportion; redrawing fifty goBILDA parts from catalogue dimensions
would be the redesign this layer exists not to do. So `catalogue.py` pins
each of the eight vendor packages to the commit inspected here and
fetches only the files Don1 names, over raw GitHub, into an ignored
directory; a missing file fails the build naming the fetch command. The
alternative of running PartCAD itself was rejected: the index now wants
PartCAD ≥ 0.8 where the blueprints say ≥ 0.6, and a build that needs a
second package manager and the network every time is not a layer.

**One `Link` class per assembly file, not one class per link.** A link's
identity is the file it reads plus its PartCAD parameters (`dir` for the
camera links), so `Link` keeps the constructor form: `Link('link-hip')`
hashes to one artifact per file and parameter set, the children are
created in `__init__` from `blueprints.load()` in file order and named by
the file's own `name` fields (duplicated names get an index suffix), and
`render()` applies each placement. Nested `assembly` entries become
nested `Link`s (the worm-gear sub-assembly appears three times); nested
unnamed groups are flattened into their parts' placements by composing
the group's location into each, since a group without a name has no
identity of its own. Rejected: generating one class per file — the same
artifacts with more code.

**`StepPart` is the one leaf class, keyed by part id.** A part's identity
is its file; `StepPart(part=...)` in the constructor form hashes the id.
`render()` imports the STEP; for a file that holds only faces it sews
them (tolerance 0.05 mm) and makes a solid of every closed shell — the
hook becomes one solid of 170.6 cm³, the battery 27. The STEP file joins
the node's tracked source set, so replacing it rebuilds. Multi-body
vendor parts (bearings' races and balls, servos' cases, the NUC's 71
bodies) stay multi-body: they are bought, not printed, and no connectivity
is asserted on them.

**The side channel is drawn as its script draws it.** `don1-side-channel`
is a CadQuery script that fetches the 9-hole channel through PartCAD and
cuts two holes; `SideChannel` imports the same channel STEP and applies
the same two cuts at the same faces and positions, so the part is the
script's without the script's dependency.

**Joints are where the bearings are, and `robot.py` says so.** Each axis
was read off the placed parts, not assumed: the base's two 8 mm REX
flanged bearings at (272.64, 0.02, 1.8) and (272.64, 0.02, 47) mm put the
front yaw axis vertical through (272.5, 0), 0.14 mm from the turntable
disc's centre (the blueprint's tolerance, kept); the turntable's bearings
at (371, 0, -37) and (350, 0, -37) mm put the roll axis along X through
the hip group's origin exactly; the thigh's radial-load bearings at
(512, 66.5, -37) and (512, 45.5, -37) mm put the thigh axis along the
side frame's Y through its origin; the knee bearings at (492 and 532,
316.4, 71.8) mm, the knee worm gear at (512.14, 316.48, 71.78) mm and the
knee shaft's own axis at (·, 316.2, 71.7) mm agree on the knee axis; the
foot's 6 mm bearings and D-shaft on the wheel axis; the vision servo's
spline on the pan axis and the base servo's spline on the tilt axis.

**The knee bends about the shaft, and the 11.5 mm is recorded.**
`robot.assy` nests the foot at (0, -304.9, 108.75) mm in the side frame,
the thigh's knee shaft is at (0, -316.4, 108.8) mm — the thigh sits at
-11.5 mm in the same frame and the file forgot it — and the foot's two
sonic hubs are centred on the nesting point. The blueprint carries a
TODO about refactoring this end of the thigh. The simulation rotates the
foot about the shaft, because the shaft in the thigh's bearings is the
thing that physically turns, and the foot's hubs therefore orbit it at
11.5 mm; the alternative, rotating about the hubs, would have the shaft
orbit inside the foot. Either shows the blueprint's error; the shaft is
the honest axis. The rest pose is unchanged: at the default bend the
foot sits exactly where `robot.assy` composes it.

**Drivers live on the root and reach the joints through ports.** The
pilot asked for one slider per motor, so the 24 drivers are declared on
`Don1` and the viewer shows them at the root. Each side, hip, leg and
camera declares a `RotationalPort` per joint it owns and applies the
motion in its own `simulate()`; the parent binds them by assignment
after reading its own. A joint's motion is applied to the moving child
node about the child's own frame (`rotate` in `simulate()` composes
inside the rest placement), so no extra wrapper assemblies exist: the
turntable group node is rotated by its parent about its Z, the hip node
about its X, the leg node about its Y, the foot node about its X, the
wheel about its Z, the camera base about its Z and the camera link about
its X. Default angles are the blueprint's rest (0 everywhere, 39° at the
knee); ranges are the ROS description's limits where it has them and the
servo's travel for the cameras.

**Drive-train parts turn about their own axes by their ratios.** The
worm gear, its hub and the shaft it is on belong to the moving link and
need nothing. The worm spins about its own axis, which is not through its
origin: the goBILDA STEP puts it at local (-5.25, ·, 33.94) mm, so the
motion is a translate–rotate–translate about that line, and likewise for
the hubs. A right-hand single-start worm on a 28-tooth gear gives 28
turns of worm per turn of gear; the 16-tooth worm sprocket and 14-tooth
motor sprockets give 16/14 on the base and 16/16 at the knee. Sense is
chosen so that the worm's thread advances the gear in the driver's
direction; the sprockets turn with the worm's sense (a chain does not
reverse). The motors themselves do not move.

**Poses from the upstream scripts, with their outcomes as the contract.**
`stand.py`, `stand2.py`, `hug.py` and `walk.py` in
`openvmp_motion_control_py` command the ROS joints; the ROS description's
knee zero coincides with the blueprint's unbent foot (its hook sits at
the blueprint's hook position) and its `stand2` knee of 0.7 rad is the
blueprint's 39° rest bend, so the scripts' angles carry over as absolute
joint angles here with the sign each side's frame gives. What each pose
must achieve — Stand lifting the body onto vertical feet, Rest on the
hooks, Hug with the feet folded back — is asserted on the meshes, so a
sign error shows as a failed outcome rather than a wrong number that
looks right.

**No pose in this blueprint rolls on level wheels; `Duct` is the
nearest.** With the hips level no bend of the knees puts the wheels
under the robot with their axes horizontal: the wheel sits at the corner
of the L-shaped foot, so its axis is level only when the foot's long
segment is level, and then the wheel hangs 37 mm above the base's
underside (thighs 180°, knees 0°: rims at -131 mm, base at -168 mm). The
upstream Stand (knees -90°) has rims and hook tips together at -454 mm
with the wheels flat. Rolling a hip a quarter turn points one thigh down
and the other up, and a scan of the knee from -180° to 180° on the down
leg finds the wheel lowest only between 0° and 75° of bend, cambered by
90° minus the bend: at 90°, where the axis is level, the thigh's own
knee-end channel reaches 48 mm below the rim. And the clearance sweep
finds the foot's main channel fouling the thigh's knee motor from 50° of
bend on (the ROS description allows 126°), so the usable bend ends at
45°. `Duct` therefore rolls the front hip -90° and the rear +90° with
every knee at 45°: one wheel per end presses the floor at -560 mm and
the other a ceiling, all cambered 45°, front thighs at 0° and rear at
180° so the two floor wheels sit on opposite sides of the centre line.
The design says this rather than pretending a level four-wheel stance
exists.

**Clearance is measured between links, by the project, because the
framework cannot ingest these parts.** The framework's spatial
assertions refuse any part whose STL is not one closed volume, even on
the exact kernel, and twenty-one of the sixty-seven pieces are not:
where a bearing's balls touch its races or a board's components touch
the board the merged mesh is non-manifold, and the stepper, the servo,
the wheel, the worm, the REX shafts, the standoff, the gearbox housing
and a few NUC bodies tessellate with degenerate or unshared triangles.
No B-rep repair tried (ShapeFix, sewing at 0.01–0.5 mm, unifying
domains, fusing the bodies) closed them, and a convex envelope would
fill every bore and fake every shaft fit. So `clearance.py` measures the
motion contract itself: for the check only, each artifact's mesh is
cleaned (degenerate and duplicate triangles dropped, vertices merged),
split into bodies along manifold edges so touching shells come apart,
and a body that still is not closed is replaced by its convex hull; the
bodies become Manifolds and every pair of parts from *different* links
with overlapping boxes is intersected. Parts of one link never move
relative to each other, so what they share is the blueprint's static
placement, not a question about motion. Nine artifacts have hulled
bodies: the servo, the wheel, the worm, the 6 mm set collar, the hyper
hub, the gearbox, the NUC, the brushless driver and the battery.

**The blueprint's rest overlaps are recorded and asserted.** Across
links, at rest, 44 pairs share volume, all of them a part on a joint
shaft or the knee offset above: each turntable's REX shaft, its last
spacer and the gear shaft's bearing, clip and shaft in the hub, gearbox,
mount and attachment across the turntable joint (6 per end); each
thigh's radial-load shaft and last spacer in the hip's hyper hub, each
knee shaft in the foot's two sonic hubs and two flat brackets it misses
by 11.5 mm, and each camera's servo spline in its hub and its arm (8 per
side). `test_robot.py` asserts exactly that set at rest, and the
scenario sweep asserts that no pair outside it appears at any sample
along the instructions, sampled every 0.5 s; a pose that must collide
(the front hip rolled 90° with the left foot folded back into the hip
channel) is asserted to be caught. Within links the blueprint overlaps
379 pairs, the largest being two stepper drivers placed through the
battery (9.7 and 9.4 cm³), each thigh's channel through its motor mount
(1.8 cm³), the eight drivers through the bottom board (1.1 cm³ each) and
every servo through its frame (0.85 cm³, partly the servo's hull); they
are the blueprint's and are reported, not asserted.

**Parts carry a coarser angular tessellation.** At the framework's
0.1 mm and 0.1 rad the robot's STLs came to 202 MB; storing a
triangulation on each shape at 0.1 mm and 0.5 rad, which the export
reuses, keeps the linear precision and cuts the size to a quarter.

## Evidence

Mutation checks run against the built model, each mutation in node code:

- The knee pivoted at the nesting origin instead of the shaft: caught by
  the knee-bend contract (the wheel's origin misses its predicted point);
  the default-bend contract is blind to it by design, since at the rest
  bend both pivots give the blueprint's pose.
- The roll sense flipped: caught by the knee-shaft-axis contract (669 mm
  off); the hub-on-axis contract is blind, since a hub centred on the
  axis stays put in either sense.
- The yaw applied about a point 30 mm off the disc centre: caught by the
  yaw contract.
- A 14-tooth worm gear instead of 28: caught by both worm-spin contracts.
- The camera tilted about Y instead of its own X: caught by the
  tilt-follows-pan contract.

## Risks / Trade-offs

- [The NUC's 258 k-triangle tessellation, twice] → it is inside the
  enclosure; kept as the index has it, and reported with the build's
  triangle count so the pilot can decide on a lighter board later.
- [Exact-kernel tests over 503 exact parts] → the faceted kernel is the
  loop; the exact run is attempted at the end and its cost reported. If
  it does not finish in reasonable time, that is recorded, not hidden.
- [Sewn shells that are not closed] → the battery's 27 shells are checked
  for closure; an open one is reported and the part admitted as-is,
  since it is a bought part whose only contract is placement.
- [Vendor pins go stale] → the pins are commits, not branches; a re-pin
  is an edit to one table with the inspection repeated.
- [`openvmp-models` is an untracked clone beside the project, not the
  `models` submodule] → the layer reads `<project>/openvmp-models/`, the
  path this workspace uses; `.gitignore` lists it. Pointing at the
  submodule path instead is a one-line change if the pilot initializes
  it.

## Findings in the blueprint, for OpenVMP

- `robot.assy` nests each foot 11.5 mm short of the thigh's knee shaft
  (the knee decision above); at rest the shaft passes through the foot's
  sonic hubs and flat brackets.
- The base's `motion-*-worm-collar` is placed on the gear shaft (its bore
  on the yaw axis), not on the worm shaft its name says; simulated as a
  gear-shaft part.
- The foot fouls the thigh's knee motor from 50° of bend; nothing puts a
  level wheel on the floor (see `Duct`).
- A direct ramp from `Crouch` (thighs 180°) to `Hug` (thighs 0°) swings
  the front and rear feet through the body's centre, where the hooks
  meet; the scenario routes such moves through `Rest`.
- The camera channel fouls the hip's vision beams when panned outward
  and tilted down, and the servo arm fouls the base's servo frame beyond
  about 30° of tilt; `Look` pans the cameras 45° inward and tilts them
  30° up, which is clear.
- Within links, 379 overlapping placements, the drivers through the
  battery and the motor mounts through the thigh channels the largest.
- Two stepper drivers are placed through the battery (9.7 and 9.4 cm³).

## Findings for the framework

Recorded here for the shop to file as warts, not fixed in this change:

- The spatial assertions' watertight gate rejects bought parts as
  imported, and there is no project-level hook to repair a mesh on an
  exact leaf; `StlNode.adjust()` exists but takes one body only.
- The exact leaf's STL export keeps degenerate triangles (the fusion
  path removes them), so a valid solid can fail the gate on its own.
- Tessellation precision is not declarable per node; a bought part
  cannot ask for less than the framework's angular deflection.
- `solid test` on a bare file path builds every node class the file
  defines, so a sub-assembly that only works under its parent (ports
  bound by the parent) makes the whole file untestable by path; the
  class must be named in the reference.

## Open Questions

- Whether the pilot wants the blueprint's knee offset fixed in
  `openvmp-models` (a change to `robot.assy`'s foot placement) rather than
  only recorded.
- Whether the four hip side servos should get sliders once their horns
  are known.
