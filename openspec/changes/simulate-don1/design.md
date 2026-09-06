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
must achieve — Stand on the wheels with level axes, Rest on the hooks,
Hug with the feet folded back — is asserted on the meshes, so a sign
error shows as a failed outcome rather than a wrong number that looks
right.

**Interference is asserted with the blueprint's own overlaps listed.**
The blueprint places spacers, bearings and hubs by hand and some of them
share volume at rest; `test_robot.py` asserts no interference over the
whole robot along every instruction with the faceted kernel, and lists
the rest overlaps found, asserting each is present so a corrected
blueprint is noticed. The list is populated by the first build and
recorded here in the implementation commit.

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

## Open Questions

- Whether the pilot wants the blueprint's knee offset fixed in
  `openvmp-models` (a change to `robot.assy`'s foot placement) rather than
  only recorded.
- Whether the four hip side servos should get sliders once their horns
  are known.
