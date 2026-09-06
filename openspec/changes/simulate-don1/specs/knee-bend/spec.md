## ADDED Requirements

### Requirement: The foot bends about the knee shaft
The root SHALL declare a driver per leg, `<front|rear>_<left|right>_knee`
(degrees, default 39, range -162 to 126 as the ROS description limits
it), the absolute angle between the foot and the thigh about the knee
shaft the thigh's bearings and worm gear hold: the side frame's X axis
through (0, -316.4, 108.8) mm, which for the front-left leg is the world X
axis through (·, 316.4, 71.8) mm. The default SHALL reproduce the
blueprint's rest pose, in which `robot.assy` nests the foot 39° about X
at (0, -304.9, 108.75) mm — 11.5 mm short of the shaft along the thigh —
and the foot's hubs therefore sit 11.5 mm off the shaft at every angle;
that offset is the blueprint's and SHALL be recorded, not corrected.

#### Scenario: The blueprint's pose at the default
- **WHEN** every driver is at its default
- **THEN** the front-left hook origin stands at (512, 318.1, -313.65) mm within 0.05 mm, exactly where `robot.assy` composes it

#### Scenario: Bending about the shaft
- **WHEN** `front_left_knee` is 0 and every other driver is at its default
- **THEN** the front-left wheel axis has swung about the knee shaft axis so that the wheel's centre lies within 0.1 mm of the point given by rotating its rest centre (512, 485.67, -171.4) mm by -39° about the X axis through (·, 316.4, 71.8) mm

### Requirement: The knee worm drive turns with the bend
The knee worm gear SHALL turn with the foot, the knee worm about its own
axis by the bend times 28, and the knee motor sprocket by the worm's
angle times 16/16.

#### Scenario: The knee worm spins
- **WHEN** `front_left_knee` moves from 39 to 49
- **THEN** the front-left knee worm has turned 280° about its own axis and stays within 0.05 mm of its rest bounding-box centre
