# blueprint-import Specification

## Purpose
The blueprint assemblies as the source of every part and its rest placement, the index's STEP files as the sourced parts, the rebuild on edits, and the colour of each material.

## Requirements
### Requirement: Every part is placed where the blueprint places it
The simulation SHALL read the Don1 assembly files
(`openvmp-models/robots/don1/*.assy`, Jinja-templated YAML) and realize
every `part` entry of every link, including entries inside nested groups
and nested assemblies, as one node whose rest placement is the entry's
PartCAD location composed through its groups: a rotation of `angle`
degrees about `axis` through the part's origin, then a translation to
`position`. The 503 part instances of `robot.assy` SHALL all be present,
and the world position of each part's origin SHALL match an independent
composition of the same locations within 0.01 mm.

#### Scenario: A part nested three groups deep
- **WHEN** the robot is built at rest
- **THEN** the front turntable's `turntable` part origin stands at (272.5, 0, -4.5) mm, the front hip's `channel-bottom` at (632.5, 0, -15.65) mm, and the front-left foot's `hook` at (512, 318.1, -313.65) mm, each within 0.01 mm of the values composed from `robot.assy`, `link-turn-table.assy`, `link-hip.assy` and `link-upper-arm.assy`

#### Scenario: A handed sub-assembly
- **WHEN** the camera links are realized for the left and right sides
- **THEN** the left and right camera bases mirror each other across the hip's XZ plane within 0.01 mm, as the `dir` parameter of `link-camera-servo.assy` and `link-camera.assy` prescribes

### Requirement: Catalogue parts are the index's own geometry
Every part id of a PartCAD index package SHALL be realized from that
package's STEP file at the commit pinned in `simulation/don1/catalogue.py`,
imported as exact geometry; every part id of the OpenVMP parts package
SHALL be realized from `openvmp-models/parts/<name>.step`, except
`don1-side-channel`, which is drawn as the blueprint's CadQuery script
draws it (a 9-hole U-channel with a 18 mm hole through one end and a
70 mm hole 48 mm from the other) over the imported channel STEP. A STEP
file that holds only faces (the hook, the battery) SHALL be sewn into
closed solids. A vendor part file that is absent SHALL fail the build
naming the part and the fetch command, never substitute a placeholder.

#### Scenario: Vendor parts fetched by the pinned script
- **WHEN** `python -m simulation.don1.catalogue fetch` runs on a checkout without `simulation/don1/vendor/`
- **THEN** every vendor STEP file the Don1 links name is present afterwards and `python -m simulation.don1.catalogue check` exits 0

#### Scenario: The hook is one closed solid
- **WHEN** the hook is built
- **THEN** its artifact is one watertight body whose volume is 170.6 cm³ within 0.5 cm³

### Requirement: Blueprint and part edits rebuild what they place
Each link node SHALL carry every `.assy` file, and each part node its
STEP file, in its tracked source set, so that editing a blueprint or
replacing a part file rebuilds the affected artifacts on the next build.

#### Scenario: Touching a blueprint
- **WHEN** an `.assy` file's modification time changes after a build
- **THEN** the next `solid build` rebuilds the robot rather than reporting it current

### Requirement: Parts are coloured by material
Every part SHALL carry a colour from a material table keyed by the part
id: goBILDA aluminium structure, steel hubs and couplers, stainless
shafts and the custom sheet parts, brass worm gears, black acetal and
rubber, black motors and servos, the printed enclosure and hook, and the
electronics by board, so that the structure, the drive train, the
printed parts, the electronics and the wheels are distinguishable in one
view.

#### Scenario: Materials in the published document
- **WHEN** the model is built
- **THEN** `viewer.json` gives the U-channels, the worm gears, the hook and the wheels four different colours, and every node a colour

