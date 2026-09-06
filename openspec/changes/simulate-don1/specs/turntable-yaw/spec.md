## ADDED Requirements

### Requirement: The base turns each turntable about its vertical axis
The root SHALL declare drivers `front_yaw` and `rear_yaw` (degrees,
default 0, range -90 to 90 as the ROS description limits it). Each SHALL
turn the turntable link and everything it carries — hip, legs, cameras —
about the axis the base's two gear bearings define: vertical, through
(±272.5, 0) mm, with the blueprint's own 0.14 mm offset between the
bearings and the turntable disc recorded and not corrected. The turntable
disc SHALL stay concentric with that axis within 0.2 mm at every angle.

#### Scenario: The disc turns in place
- **WHEN** `front_yaw` is 45
- **THEN** the front turntable disc's centre stays within 0.2 mm of (272.5, 0) mm, the front hip's channel-bottom origin has moved to the point (632.5, 0, -15.65) mm rotated 45° about that axis within 0.05 mm, and the base has not moved

#### Scenario: The coupled pair is one input
- **WHEN** the driver table is read from `viewer.json`
- **THEN** it lists exactly one yaw driver per turntable and no driver per base motor

### Requirement: The worm drive turns with the yaw
The worm gear, its hub and the gear shaft SHALL turn with the turntable;
the worm SHALL turn about its own axis by the yaw times the gear's 28
teeth, and each motor sprocket by the worm's angle times 16/14, in the
sense a right-hand single-start worm gives, so that a moving slider shows
the drive train moving.

#### Scenario: The worm spins faster than the table
- **WHEN** `front_yaw` moves from 0 to 10
- **THEN** the front worm has turned 280° about its own axis and each front motor sprocket 320°, and every spun part stays within 0.05 mm of its rest bounding-box centre
