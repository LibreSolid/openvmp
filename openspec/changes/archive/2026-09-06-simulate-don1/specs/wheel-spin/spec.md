## ADDED Requirements

### Requirement: The wheel spins on the foot's D-shaft
The root SHALL declare a driver per leg, `<front|rear>_<left|right>_wheel`
(degrees, default 0, range 0 to 360), turning the rubber wheel about the
axis the foot's two 6 mm flanged bearings and the D-shaft define: the
foot frame's Y axis through (0, ·, -75.2) mm. The wheel's centre SHALL
not move with the spin.

#### Scenario: A spun wheel stays put
- **WHEN** `front_left_wheel` is 137
- **THEN** the front-left wheel's bounding-box centre is within 0.05 mm of its rest value and its mesh vertices have rotated by 137° about the shaft axis
