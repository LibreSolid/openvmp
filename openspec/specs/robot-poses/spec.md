# robot-poses Specification

## Purpose
The instructions the robot takes, what each pose achieves on the meshes, and the clearance between links along the moves.

## Requirements
### Requirement: Instructions pose the robot
The root SHALL declare the instructions `Rest` (every driver to its
default: the blueprint's pose, hooks lowest), `Stand` (thighs 180°,
knees -90°, after the upstream `stand.py`: feet vertical under the
knees, the body lifted on hook tips and wheel rims), `Crouch` (thighs
180°, knees -40°: the same stance lowered onto the wheel rims), `Hug`
(thighs 0°, knees -180°, after `hug.py`: feet folded back along the
thighs), `Walk` (the reaching stance of `walk.py`: knees -83°, the front
turntable -17° and the rear +17°, left thighs 157° and right thighs
-157°), `Duct` (the
front hip rolled -90° and the rear +90°, knees 45°, front thighs 0° and
rear thighs 180°: one wheel per end on the floor and one on the ceiling,
cambered 45°), `TurnLeft` and `TurnRight` (both turntables ±30°, the body
articulated), `Roll` (every wheel one full turn), and `Look` (every
camera panned 45° toward the body's centre line and tilted 30° up, the
quadrant of the pan-tilt range clear of the hip's vision beams and the
base's servo frame). Each SHALL land exactly on
its targets.

#### Scenario: Stand lifts the body onto the feet
- **WHEN** `Stand` has completed
- **THEN** the lowest point of each foot (its hook tip or wheel rim) lies at least 250 mm below the lowest point of the base, the four lowest points are coplanar within 2 mm, and each hook's lowest point is within 2 mm of its wheel's lowest point

#### Scenario: Duct braces the wheels against floor and ceiling
- **WHEN** `Duct` has completed
- **THEN** the front-left and rear-right wheels' lowest points are lower than every other part of the robot and coplanar within 2 mm, the front-right and rear-left wheels' highest points are higher than every other part and coplanar within 2 mm, and every wheel's axis is 45° from horizontal within 1°

#### Scenario: Rest puts the robot on its hooks
- **WHEN** `Rest` has completed
- **THEN** the lowest point of each hook is lower than the lowest point of every wheel

#### Scenario: Hug folds the feet back
- **WHEN** `Hug` has completed
- **THEN** each hook's lowest point is above its knee shaft axis, and each foot's wheel centre is nearer the hip than its knee is

### Requirement: No two parts of different links share volume along the moves
Parts of one link never move relative to each other, so the motion
contract is between links: a simulation triggering every instruction in
turn, sampling every 0.5 s, SHALL find no two parts of different links
sharing volume at any sample beyond the pairs the blueprint itself
overlaps at rest, which SHALL be enumerated in the design and asserted
to be exactly the set found at rest, so that a corrected blueprint is
noticed. The check SHALL be able to fail: a pose that drives a foot
through the hip SHALL be reported.

#### Scenario: Interference-free moves
- **WHEN** a simulation triggers `Stand`, `Crouch`, `Rest`, `Hug`, `Rest`, `Walk`, `Rest`, `Duct`, `Rest`, `TurnLeft`, `TurnRight`, `Look`, `Roll` and `Rest` in turn (a move between thighs at 180° and at 0° is routed through `Rest`, since a direct ramp swings the feet through the body's centre), each given its duration, sampling every 0.5 s
- **THEN** no two parts of different links outside the recorded rest set share volume at any sample

#### Scenario: The recorded overlaps are still there
- **WHEN** the robot is at rest
- **THEN** the set of pairs of parts of different links sharing volume is exactly the recorded set of 44

#### Scenario: A collision is caught
- **WHEN** the front hip is rolled 90° with the front-left thigh at -90° and its knee at 120°
- **THEN** the front-left hook is reported sharing volume with the hip's bottom channel

