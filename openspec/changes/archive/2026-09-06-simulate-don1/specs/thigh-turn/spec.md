## ADDED Requirements

### Requirement: The thigh turns about its own length
The root SHALL declare a driver per leg, `<front|rear>_<left|right>_thigh`
(degrees, default 0, range -180 to 180), turning the thigh and the foot
on it about the axis the thigh's radial-load-support bearings define: the
side frame's Y axis through the hip's thigh hub, which for the front-left
leg is the world Y axis through (512, ·, -37) mm. The thigh's bearings
SHALL stay on that axis within 0.2 mm at every angle, and the four legs
SHALL turn in the same sense relative to their own hips: a positive
angle turns both legs of a hip about the hip's Y axis the same way, as
the ROS description's `arm` joints do, so the rear hip, turned 180°,
swings its feet the opposite way along the body.

#### Scenario: One angle, four legs
- **WHEN** every thigh is 60
- **THEN** the two hooks of each hip have moved the same distance along the body within 0.1 mm and risen equally, and the front hip's hooks moved the opposite way to the rear hip's

#### Scenario: A half turn brings the foot under the body
- **WHEN** `front_left_thigh` is 180 and every other driver is at its default
- **THEN** the front-left knee shaft axis, at rest through (512, 316.2, 71.7) mm, passes through (512, 316.2, -145.7) mm within 0.1 mm, and the front-left hook's lowest point is above the front-left wheel's lowest point
