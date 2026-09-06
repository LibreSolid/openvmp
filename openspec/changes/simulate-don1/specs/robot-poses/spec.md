## ADDED Requirements

### Requirement: Instructions pose the robot
The root SHALL declare the instructions `Rest` (every driver to its
default: the blueprint's pose, hooks lowest), `Stand` (thighs 180°,
knees -90°, after the upstream `stand.py`: feet under the body, on the
wheels), `Crouch` (thighs 0°, knees 40°, after `stand2.py`), `Hug`
(thighs 0°, knees -180°, after `hug.py`: feet folded back along the
thighs), `Walk` (the first stance of `walk.py`: thighs 180°, knees -90°,
then the front turntable -17° and the rear +17° with the front-left and
rear-right thighs reached 23° forward), `TurnLeft` and `TurnRight`
(both turntables ±30°, the body articulated), `Roll` (every wheel one
full turn), and `Look` (every camera panned 60° outward and tilted 30°
up). Each SHALL land exactly on its targets.

#### Scenario: Stand puts the robot on its wheels
- **WHEN** `Stand` has completed
- **THEN** the lowest point of each of the four wheels is lower than the lowest point of every hook and of the base, the four lowest points are coplanar within 2 mm, and each wheel's axis is horizontal within 1°

#### Scenario: Rest puts the robot on its hooks
- **WHEN** `Rest` has completed
- **THEN** the lowest point of each hook is lower than the lowest point of every wheel

#### Scenario: Hug folds the feet back
- **WHEN** `Hug` has completed
- **THEN** each hook's lowest point is above its knee shaft axis, and each foot's wheel centre is nearer the hip than its knee is

### Requirement: No two parts share volume along the moves
A simulation triggering every instruction in turn, sampling every 0.5 s,
SHALL find no two rigid solids sharing volume at any sample, except the
overlaps the blueprint itself contains at rest, which SHALL be enumerated
in the design and asserted present so that a corrected blueprint is
noticed.

#### Scenario: Interference-free moves
- **WHEN** a simulation triggers `Stand`, `Crouch`, `Hug`, `Walk`, `TurnLeft`, `TurnRight`, `Look`, `Roll` and `Rest` in turn, each given 2 s, sampling every 0.5 s
- **THEN** no two rigid solids outside the recorded rest overlaps share volume at any sample

#### Scenario: The recorded overlaps are still there
- **WHEN** the robot is at rest
- **THEN** each pair of parts the design lists as overlapping in the blueprint does share volume
