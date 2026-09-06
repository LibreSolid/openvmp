## Why

`openvmp-models` describes Don1 as PartCAD assemblies: 503 catalogue
parts placed by coordinates in nine `.assy` files, and nothing that
moves. The joints are only implied — a pair of flanged bearings here, a
worm gear on a shaft there — and the ROS description that does move
(`platform/.../openvmp_robot_don1.urdf`) is a separate hand-simplified
model of boxes and cylinders whose wheel does not sit where the
blueprint's wheel sits. Rendering the blueprint at all needs PartCAD and
the network.

What physically goes wrong today is everything a static placement cannot
say: whether the foot swings about the knee shaft the thigh's bearings
hold, or about the point the robot file nests it at (they differ, and the
file carries a TODO about it); whether a leg folded for hugging clears the
camera beside it; whether the wheels can be brought under the body with
their axes level; what each of the fourteen steppers and eight camera
servos actually turns. None of it has been drawn moving, so none of it can
be checked.

## What Changes

- A solid-node project at the repository root (`pyproject.toml`, model
  `don1`) with the package `simulation/don1/` that reads the blueprint
  assemblies as they are — every part placed by the `.assy` files' own
  locations — and adds only what they do not contain: which links move,
  about which axes, driven by which motors, and the poses the robot takes.
- The catalogue parts come from the same PartCAD index packages the
  blueprints import, as their STEP files, pinned by commit and fetched by
  a script into an ignored directory; the OpenVMP custom parts are read
  from `openvmp-models/parts`. Nothing is redrawn except the one custom
  part that is a CadQuery script depending on PartCAD.
- One driver per motor: two coupled base stepper pairs (turntable yaw),
  two turntable steppers (hip roll), four thigh steppers, four knee
  steppers, four wheel motors, four camera pan servos and four camera tilt
  servos — 24 sliders on the root, named by position.
- Instructions combining them: the blueprint's rest pose, the upstream
  motion scripts' Stand, Crouch and Hug, a Walk step, an articulated
  Turn, a wheel Roll and a camera Look.
- Every part carries a colour by material, chosen to separate structure,
  drive train, printed parts, electronics and rubber at a glance.
- Contracts prove the joints turn about the axes the bearings define, the
  poses reach their stated outcomes, and no two parts share volume along
  any instruction, with the blueprint's own overlaps recorded rather than
  hidden.

## Capabilities

### New Capabilities
- `blueprint-import`: the `.assy` files as the source of every part and
  its rest placement; the vendor parts as the index's STEP files; the
  rebuild when a blueprint or a part file changes; colour by material.
- `turntable-yaw`: the base's coupled stepper pair turning a turntable
  and everything on it about the base's vertical axis through the worm
  drive, including the worm, sprockets and worm gear turning with it.
- `hip-roll`: the turntable's geared stepper rolling the hip about the
  turntable's shaft.
- `thigh-turn`: the thigh's geared stepper turning the leg about the
  thigh's own axis in the hip's radial-load bearings.
- `knee-bend`: the knee stepper bending the foot about the knee shaft
  through the knee worm drive, and the foot's own rest bend.
- `wheel-spin`: the wheel turning on the foot's D-shaft.
- `camera-pan-tilt`: the hip's servo panning the camera base, the base's
  servo tilting the camera.
- `robot-poses`: the instructions, what each pose achieves, and the
  clearance of every part from every other along the moves.

### Modified Capabilities
- (none; this is the project's first design change)

## Impact

- New `pyproject.toml` declaring the model `don1` as
  `simulation.don1.robot:Don1`, new package `simulation/don1/`, `openspec/`
  as the design record, `.gitignore` entries for the build directory, the
  fetched vendor parts and the `openvmp-models` clone; `README.md` gaining
  a section on the simulation.
- Nothing under `openvmp-models/` or `platform/` is edited. Where the
  simulation has to disagree with a blueprint it says so in the design and
  in a contract.
- Out of scope, recorded for later: the hips' four side servos (their
  horns and what they connect are not in the blueprints), the passive
  centre wheel each hip is described with (its axle is in the blueprint,
  the wheel is not), the drive chains between motor sprockets and worm
  sprockets, the two brushless drivers' motors, wiring, and the other
  OpenVMP robots.
- Uses solid-node 0.6 with the declarative node API as published on the
  framework's main branch; no framework change is needed. Needs
  `jinja2`, `pyyaml` and `scipy` (all framework or CadQuery dependencies).
