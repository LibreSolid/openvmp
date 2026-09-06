## 1. Layer and catalogue (blueprint-import)

- [x] 1.1 `pyproject.toml` (model `don1`), `.gitignore`, `simulation/don1/` package, `blueprints.py` (assembly loader, location composition, source set), `catalogue.py` (part id to file, vendor pins, `fetch`/`check`), `materials.py`
- [x] 1.2 `parts.py`: `StepPart` (STEP import, shell sewing, tracked file, colour), `SideChannel`; `link.py`: `Link` over `blueprints.load()`
- [x] 1.3 Placement contracts red (a link whose parts ignore their locations), then green: three nested origins, the handed camera links, the hook's volume, part count 503

## 2. Kinematic tree and joints (turntable-yaw, hip-roll, thigh-turn, knee-bend, wheel-spin, camera-pan-tilt)

- [x] 2.1 `robot.py`: `Don1`, `Side`, `Hip`, `Leg`, `Foot`, `Camera` with the robot file's placements; 24 root drivers; ports down the tree; rest pose contract (hook origin) red then green
- [x] 2.2 Joint axis contracts, one per joint, red (motion about the nesting origin) then green: disc concentric, hip hub on axis, knee about the shaft, wheel centre fixed, pan hub concentric, tilt inside pan
- [x] 2.3 Drive-train spins: worm, worm gear hub, sprockets on base and knee; ratio contracts red then green

## 3. Poses (robot-poses)

- [x] 3.1 Instructions `Rest`, `Stand`, `Crouch`, `Hug`, `Walk`, `TurnLeft`, `TurnRight`, `Roll`, `Look`; outcome contracts (wheels lowest and level at Stand, hooks lowest at Rest, feet folded at Hug) red then green
- [x] 3.2 Scenario sweep through every instruction with interference sampled; the blueprint's rest overlaps enumerated in the design and asserted present

## 4. Evidence and record

- [x] 4.1 Mutation check (drop the 11.5 mm knee offset, flip a roll sign, rotate a foot about the nesting origin); note which contract catches each
- [x] 4.2 `solid build`; read `viewer.json` (24 drivers, 9 instructions, pieces, colours); snapshots at rest and at Stand, Hug and Look; the exact run attempted and its cost reported
- [x] 4.3 README section on the simulation; archive the change with `Purpose` filled in every spec; commit
