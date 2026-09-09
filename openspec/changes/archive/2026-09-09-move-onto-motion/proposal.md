# Move the Don1 simulation onto solid-node's motion layer

## Why

solid-node moved ports and the time base out of `solid_node.node` into
`solid_node.motion.ports` with no re-export, and added `joints` and
`couplings` beside them (ADR-087..089). `simulation/don1/robot.py` line 13
still writes `from solid_node.node import AssemblyNode, RotationalPort`, so
the model does not import:

    ImportError: module 'solid_node.node' has no attribute 'RotationalPort':
    ports and the declared time base moved to 'solid_node.motion.ports'.

Fixing that import would make the model run again, but it would leave the
machine stated the way it was stated before joints existed. Today Don1's 24
drivers reach their bodies through **26 forwarding `RotationalPort`
declarations** and five `simulate()` methods that re-read and re-assign
them level by level:

- `Don1.simulate()` rotates each `Side` about `Z` by hand and copies its 11
  drivers onto `Side`'s 11 ports with `setattr` over `SIDE_PORTS`;
- `Side.simulate()` rotates `Hip` about `X` by hand and copies 10 ports
  onto `Hip` over `HIP_PORTS`;
- `Hip.simulate()` loops over its two sides, multiplies each thigh driver
  by the side's `±1` because the left leg's frame is turned 180°, rotates
  the `Leg` and the `Camera` by hand, and copies four more ports;
- `Leg.simulate()` subtracts `FOOT_REST_BEND` from the knee driver and
  turns the foot with `rotate_about(self.foot, bend, X, self.knee_pivot)`,
  where `knee_pivot` is a **hand inversion of the foot's rest placement
  written with numpy and scipy** — exactly the arithmetic a `Revolute`
  declaration replaces;
- `Foot.simulate()` and `Camera.simulate()` exist only to turn one child
  about one axis from one forwarded port.

Every one of those is now expressible: seven freedoms as seven joint
declarations on the seven bodies that have them, and 24 `drives` sentences
from the 24 drivers on the root straight to the coordinate each one moves,
by path. Nothing about the machine changes — the same 503 parts at the same
places in every pose — but the class bodies say what moves instead of
plumbing values down to code that moves it.

## What changes

### The seven joints

Every moving body has **exactly one freedom against its parent**, so no
body needs two joints and the composition-order gap recorded in the
framework's `workflow/warts.md` does not bite here. Nothing in the project
uses `.repeat()`, so the fan-out gap does not bite either.

Axis and anchor are stated in the **parent's frame**, as the joints spec
requires. The numbers are the ones `robot.py` already holds as module
constants, which came from `robot.assy` and from the bearing probes
recorded in `openspec/changes/archive/2026-09-06-simulate-don1/design.md`
("Joints are where the bearings are, and `robot.py` says so").

| # | joint | declared on | frame | axis | anchor | range | source of the numbers |
|---|---|---|---|---|---|---|---|
| 1 | `yaw` | `Side` | `Don1` | `(0, 0, 1)` | `(dir * END_OFFSET, 0, END_HEIGHT)` = `(±272.5, 0, -7)` | `YAW_RANGE` | `robot.assy` places each end group there; the base's two 8 mm REX flanged bearings put the axis vertical through (272.5, 0) |
| 2 | `roll` | `Hip` | `Side` | `(1, 0, 0)` | `HIP_OFFSET` = `(149.5, 0, -30)` | `ROLL_RANGE` | `robot.assy`; the turntable's bearings at (371, 0, -37) and (350, 0, -37) put the roll axis along X through the hip group's origin exactly |
| 3 | `turn` | `Leg` | `Hip` | `(0, -1, 0)` | `SIDE_OFFSET` = `(90, 0, 0)` | `THIGH_RANGE` | `robot.assy`; the thigh's radial-load bearings at (512, 66.5, -37) and (512, 45.5, -37) put the axis along the side frame's Y through its origin |
| 4 | `knee` | `Foot` | `Leg` | `(1, 0, 0)` | `KNEE_SHAFT` = `(0, -316.4, 108.8)` | none — see below | the knee bearings at (492 / 532, 316.4, 71.8), the worm gear at (512.14, 316.48, 71.78) and the shaft's own axis all agree; this is **not** the foot's placed origin (`FOOT_OFFSET`), which is the blueprint's recorded 11.5 mm error |
| 5 | `spin` | `Wheel` (new `Link` subclass) | `Foot` | `(0, -1, 0)` | `WHEEL_OFFSET` = `(0, -293.5, -75.2)` | `WHEEL_RANGE` | `robot.assy` places the wheel link there turned 90° about X, so its own Z — the 6 mm bearings' and D-shaft's axis — is the foot frame's -Y |
| 6 | `pan` | `Camera` | `Hip` | `(0, 0, 1)` | `(SIDE_OFFSET[0] + VISION_SPREAD, side * VISION_REACH, VISION_HEIGHT)` = `(122.5, ±152, 86)` | `SERVO_RANGE` | the vision assembly's placement composed into the hip frame; the vision servo's spline is on the pan axis (its world position, (544.5, 152, 49) front-left, is what `test_pan_turns_the_base_about_the_vision_servo` already asserts) |
| 7 | `tilt` | `CameraArm` (new `Link` subclass) | `Camera` | `(0, -side, 0)` | `(dir * side * CAMERA_OFFSET[0], CAMERA_OFFSET[1], CAMERA_OFFSET[2])` = `(±4.125, 13.5, 24.5)` | `SERVO_RANGE` | `robot.assy` places the camera link turned `side * -90°` about Z at `CAMERA_OFFSET`, so its own X — the base servo's spline — is the camera frame's `∓Y`; the world axis, (540.375, ·, 73.5) front-left, is what `test_tilt_follows_pan` already asserts |

Joints 1, 3, 6 and 7 are handed. `Side` and `Camera` already declare the
`Count` parameters (`dir`, `side`) the formulas need. **`Leg` gains one
declared parameter**, `side = Count(1, min=-1, max=1)`, declared
`left_leg = Leg(side=1)` / `right_leg = Leg(side=-1)` — not for the joint,
whose axis `(0, -1, 0)` in the hip's frame is the same for both legs, but
for the thigh counter-spin that stays hand-written (below), which needs the
angle in the leg's own frame.

Joint 4 declares **no range**: the joint's zero is the blueprint's 39° rest
bend, so `KNEE_RANGE` — the ROS description's absolute limits, which the
driver keeps — is not this coordinate's range, and a joint range refuses a
binding rather than clamping it. The other six ranges are the same numbers
the drivers already publish, restated where they belong. Every value the
poses and the tests bind is inside them (checked: `front_roll=±90` and
`-135`, `front_left_thigh=-90` and `180`, `front_yaw=45`, `wheel=360`,
`pan=±60/90`, `tilt=45`).

Where a joint argument is a formula over a declared-parameter token
(`dir * END_OFFSET`, `side * VISION_REACH`), state it as the token formula;
if the framework refuses the arithmetic, fall back to the callable form
`at=lambda node: (...)`, which the joints spec provides for exactly this.

### The two new classes

`Foot.wheel` and `Camera.camera` are `Link` instances, and `Link` is the
one class every `.assy` file is read into — a joint declared on it would be
declared on the base, the hip and every thigh too. Each therefore needs a
subclass whose only content is its joint. `SubAssembly(Link)` is the
project's existing precedent for subclassing `Link` for one reason.

    class Wheel(Link):
        """The wheel link, spinning in the foot's bearings."""
        spin = Revolute(axis=(0, -1, 0), at=WHEEL_OFFSET,
                        range=WHEEL_RANGE, unit='deg')

    class CameraArm(Link):
        """The camera link, tilted by the base's servo."""
        tilt = Revolute(axis=lambda node: (0.0, -node._hand, 0.0),
                        at=lambda node: (node._end * node._hand * CAMERA_OFFSET[0],
                                         CAMERA_OFFSET[1], CAMERA_OFFSET[2]),
                        range=SERVO_RANGE, unit='deg')

        def __init__(self, assembly, dir, side, name=None):
            # read by the joint callables, which resolve at realization
            self._end, self._hand = dir, side
            super().__init__(assembly, name=name, dir=side)

`CameraArm` needs the callable form because its anchor's sign is
`Camera.dir * Camera.side` — the end's handedness times the side's — and a
joint argument resolves against the **declaring** instance only, which
knows just the blueprint parameter `dir=side` it was given. `_end` and
`_hand` are set before `super().__init__()`, which is where the framework
resolves declared joints, and they are plain attributes, so the two mirrored
camera links keep the one artifact each their `dir` parameter gives them (a
joint argument is not identity).

**The attribute names do not change**: `wheel = Wheel('link-wheel')`,
`camera = CameraArm('link-camera', dir=dir, side=side)`. Every test path in
`test_robot.py` is built from child attribute names, so renaming either
would break twelve assertions for no gain.

### The 24 relations

All 24 are stated in `Don1`'s class body, one per driver, each naming the
driver at one end and the coordinate it moves at the other. Written out for
the front end; the rear is the same eight lines with `front` → `rear`, and
the left/right pairs are the same again:

    front_yaw.drives(front.yaw)                                   # deg -> Side.yaw, ratio 1
    front_roll.drives(front.hip.roll)                             # deg -> Hip.roll, ratio 1
    front_left_thigh.drives(front.hip.left_leg.turn)              # deg -> Leg.turn, ratio 1
    front_right_thigh.drives(front.hip.right_leg.turn)            # deg -> Leg.turn, ratio 1
    front_left_knee.drives(front.hip.left_leg.foot.knee,
                           offset=-FOOT_REST_BEND)                # deg -> Foot.knee
    front_right_knee.drives(front.hip.right_leg.foot.knee,
                            offset=-FOOT_REST_BEND)
    front_left_wheel.drives(front.hip.left_leg.foot.wheel.spin)   # deg -> Wheel.spin, ratio 1
    front_right_wheel.drives(front.hip.right_leg.foot.wheel.spin)
    front_left_pan.drives(front.hip.left_camera.pan)              # deg -> Camera.pan, ratio 1
    front_right_pan.drives(front.hip.right_camera.pan)
    front_left_tilt.drives(front.hip.left_camera.camera.tilt)     # deg -> CameraArm.tilt, ratio 1
    front_right_tilt.drives(front.hip.right_camera.camera.tilt)

Every ratio is 1 because the handedness the old code carried as a `± side`
multiplication is now stated once, in the joint's own axis: the left leg's
frame is turned 180° about Z, so the axis that makes both legs swing the
same way in the hip's frame is `(0, -1, 0)` for both, and the driver reaches
the coordinate unchanged. The one law that is not the identity is the
knee's `offset=-FOOT_REST_BEND`: the driver is the ROS description's
absolute knee angle (39° at the blueprint's rest), and the joint's
coordinate is the bend **past** that rest placement, which the framework
composes inside `rotate(FOOT_REST_BEND, X).translate(FOOT_OFFSET)`.

`front_left_thigh.drives(...)` and its three kin cannot be written as one
line each over a repeated child, and are not: `left_leg` and `right_leg`
are separate declarations, as they are today.

**Derived coordinates: none.** No coordinate of this machine is a linear
formula over two others; `bend * WORM_GEAR_TEETH` and the sprocket ratios
feed the hand-written drive-train dressing described below, which is
arithmetic on values, not a coordinate of a body.

### What shrinks or disappears

| method | today | after |
|---|---|---|
| `Foot.simulate()` | rotates the wheel from `spin` | **gone**; `Foot.spin` port gone |
| `Camera.simulate()` | rotates the camera link from `tilt` | **gone**; `Camera.tilt` port gone |
| `Hip.simulate()` | 12 lines: the two-side loop, `±side`, two hand rotations, four `setattr`s | **gone**; all 10 ports gone; `sides()` gone |
| `Side.simulate()` | hand rotation of the hip, 11 `setattr`s, the turntable spin | the turntable `spin()` only, reading `self.hip.roll.value`; all 11 ports and `HIP_PORTS` gone |
| `Leg.simulate()` | `knee_pivot` inversion, `rotate_about`, the foot's port, four `spin()`s | the four `spin()`s only, reading `self.turn.value` and `self.foot.knee.value`; the `knee_pivot` property, its numpy and scipy imports, and all three ports gone |
| `Don1.simulate()` | the per-end loop, hand rotation of each side, 11 `setattr`s, the base drive train | the base drive train only, reading its own drivers; `SIDE_PORTS` gone |

Removed: **26 `RotationalPort` declarations**, three `simulate()` methods
entirely, two class-level port-name lists, one `sides()` helper, and the one
hand-written frame inversion in the project (`Leg.knee_pivot`, with the
scipy `Rotation.from_rotvec` and numpy it needed). `rotate_about` stays in
`link.py`, used by `spin()`.

Removed import: `RotationalPort`. Added: `from solid_node.motion.joints
import Revolute` in `robot.py`.

## What does not change

- **The drivers.** All 24, the same names, defaults, ranges and units,
  declared the same way by `_joint_drivers()`. The viewer shows the same
  sliders at the root.
- **`pose()` and the ten instructions.** Rest, Stand, Crouch, Hug, Walk,
  Duct, TurnLeft, TurnRight, Roll, Look — the same targets, the same
  durations. Every pose must be pixel-identical; that is the acceptance.
- **Every `render()`.** No placement moves. `Don1`, `Side`, `Hip`, `Leg`,
  `Foot`, `Camera` place their children exactly as they do now.
- **Every child attribute name**, so every test path still resolves.
- **`link.py`, `parts.py`, `blueprints.py`, `catalogue.py`,
  `materials.py`, `clearance.py`** — untouched, except the two new `Link`
  subclasses, which live in `robot.py` beside the assemblies that use them.
- **`test_robot.py`** — no assertion changes and no import changes; it
  imports nothing that moved.
- The blueprints, the ROS description, the vendor STEP fetch, the colours,
  `PART_COUNT`, the recorded rest overlaps, `docs/`.

## Known gaps

**A new one, and it is the biggest thing that stays hand-written.** A joint
is class metadata, and a relation's path is checked against declared
children. The blueprint links' children are neither: `Link.__init__` builds
them in a loop from `blueprints.load()`, one generic `StepPart` or
`SubAssembly` per entry of the `.assy` file, named by the file. So none of
the drive-train parts that visibly turn can carry a joint or be reached by a
relation, and the nine `spin()` call sites — 82 placed parts at runtime:
the base's gear shafts, worm gears, hubs, worms, worm shafts and four motor
sprockets; each turntable's shaft, clip, coupler and collar; each thigh's
clamped REX shaft, coupler and collar, its knee shaft, worm gear and hub,
its knee worm, worm shaft, worm hub and worm sprocket, and its knee motor
hub and sprocket — stay hand-written, each with its own frame inversion
inside `spin()` (a part's axis is read off its STEP frame through
`axis_of`, which the framework could not know either). The sentences the
project wants:

    # a joint on a body the blueprint placed, named as the file names it
    front.yaw.drives(base['motion-front-wormgear/worm'].spin,
                     ratio=WORM_GEAR_TEETH)
    front.hip.left_leg.foot.knee.drives(
        front.hip.left_leg.thigh['knee-worm-sprocket'].spin,
        ratio=WORM_GEAR_TEETH * KNEE_WORM_SPROCKET / KNEE_MOTOR_SPROCKET)

that is: a joint declared on a child a parent realized from data rather
than from a class body, and a relation that can reach it. Note the two
ends are a real coordinate each — this is a transmission, not dressing:
28 turns of worm per turn of the gear, 16/14 and 16/16 at the sprockets.

**The first known limit, in its ceremony form** (`workflow/warts.md`,
2026-09-09: a joint's axis and anchor belong to the declaration site as
often as to the class). Two of the seven joints need a subclass whose only
content is that joint, because `Link` is shared by every `.assy` file; and
`CameraArm` additionally needs the callable form and two private attributes
because its anchor's sign is the **parent's** handedness times its own, and
a joint argument resolves against the declaring instance only. The
sentences the project wants:

    wheel = Link('link-wheel',
                 spin=Revolute(axis=(0, -1, 0), at=WHEEL_OFFSET,
                               range=WHEEL_RANGE, unit='deg'))
    camera = Link('link-camera', dir=side,
                  tilt=Revolute(axis=(0, -side, 0),
                                at=(dir * side * CAMERA_OFFSET[0],
                                    CAMERA_OFFSET[1], CAMERA_OFFSET[2]),
                                range=SERVO_RANGE, unit='deg'))

— a joint handed to a child at the declaration site, resolved against the
**declaring parent's** parameters, which is candidate fix (b) already
recorded there. This costs ceremony, not motion: both freedoms are fully
stated either way.

**Not hit here.** No body has more than one freedom against its parent, so
the joints-compose-in-binding-order gap that deferred OpenCycloid and the
hexapod does not arise. Nothing uses `.repeat()`, so no relation fans out
over a repeated child. No node reads its own derived coordinate in its own
`simulate()` — there are no derived coordinates.

**One ordering dependence to verify.** `Side.simulate()` and
`Leg.simulate()` read a coordinate an **ancestor's** relation bound
(`self.hip.roll.value`, `self.turn.value`, `self.foot.knee.value`). The
framework solves a class's relations at the end of that instance's simulate
phase, and a walker recurses into children only after the parent's
`render()` has returned, so the root's 24 relations are solved before any
`Side` or `Leg` renders. This is the same shape Thor and OpenMANIPULATOR-X
already rely on for root-driven joints several levels down, and the pose
comparison is what proves it.

**Recommendation: do not defer.** All seven freedoms of the machine are
statable today, every driver becomes one sentence, and what stays
hand-written is drive-train dressing inside data-driven blueprint links,
which was hand-written before joints existed for a reason the joints
primitive does not address. The new finding above is worth recording in
`solid-node/workflow/warts.md` under "Motion catalogue refactor" either
way.

## Pre-existing state

`git status --porcelain` in `projects/Robots/openvmp` is **empty**: the
tree is clean on branch `main` at `11b08ea refactor(simulation): declare
the tessellation instead of storing it`. Stage 0 has nothing to commit.
The untracked snapshot PNGs the tracker allows to stay untracked are in
fact committed already and unmodified.

## Tests

`simulation/don1/test_robot.py` is the whole suite: `Don1Test` (a
`TestCase` on `Don1`) and `Don1MotionTest` (a `ScenarioTest`). There are no
plain `pytest` files. The suite runs as `solid test --faceted
simulation/don1/robot.py`; `don1` is the only declared model.

What it asserts, by spec area:

- *blueprint-import*: 503 leaves; three nested parts at the blueprint's
  own coordinates; the two camera links mirrored; the hook one closed
  170.6 cm³ solid; custom parts single-solid; four distinct material
  colours; `.assy` and `.step` files tracked as sources.
- *turntable-yaw*: yaw turns the table about (272.5, 0) and leaves the
  base still; exactly two `*_yaw` names on the class and no `*motor*`
  name; the base worm turns 28× and its sprocket 16/14.
- *hip-roll*: roll turns the hip about X through (·, 0, -37) and leaves
  the turntable channel still; the hip hub stays on the roll axis at 60°
  and -135°.
- *thigh-turn*: a half turn brings the foot under the body; the thigh
  bearings stay on the thigh axis at 45° and 180°; the four legs turn in
  one sense.
- *knee-bend*: the default bend is the blueprint's pose; the foot bends
  about the knee shaft; the knee worm turns 28× the bend.
- *wheel-spin*: a spun wheel stays put and turns about its own axis.
- *camera-pan-tilt*: pan turns the base about the vision servo; tilt
  follows pan.
- *robot-poses*: Rest on the hooks; Stand lifts the body; Hug folds the
  feet back; Duct braces level wheels at 45° camber; the recorded rest
  overlaps are exactly the found ones; a pose that must collide is caught;
  and the `ScenarioTest` sweeps all fourteen moves at 0.1 s checking for
  new cross-link overlaps every 0.5 s.

**Tests I expect to need a change: none.** Flagged for the orchestrator,
because they constrain the refactor rather than being changed by it:

1. Every assertion locates parts with `find(node, 'front.hip.left_leg.foot.arm.hook')`
   and its kin, walking **child attribute names**. The refactor must keep
   `front`, `rear`, `turntable`, `hip`, `link`, `left_leg`, `right_leg`,
   `thigh`, `foot`, `arm`, `wheel`, `left_camera`, `right_camera`, `base`
   and `camera` exactly as they are. Changing the class of `wheel` and
   `camera` is invisible to these; renaming the attributes is not.
2. `test_one_yaw_driver_per_end_and_none_per_base_motor` asserts that the
   names on `type(self.node)` ending in `_yaw` are exactly
   `{front_yaw, rear_yaw}`. `Side.yaw` is a name on `Side`, not on `Don1`,
   so it passes — but a joint named `*_yaw` on the root would break it.
3. The six joint ranges are a **new refusal path**: a plain numeric
   binding outside a declared range now raises instead of posing. I have
   checked every value the tests and the ten instructions bind against
   each declared range and all are inside. The knee joint therefore
   declares no range, its coordinate being the bend past the 39° rest
   rather than the ROS absolute angle the driver publishes. If a range
   refusal appears anyway, the right fix is to drop that joint's `range=`,
   not to change a test.
4. No test reads a port, so the 26 removed ports are invisible to it —
   unlike OpenTorque-Actuator, this suite needed no port reads moved onto
   joints.
