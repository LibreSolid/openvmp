## ADDED Requirements

### Requirement: The hip servo pans the camera base
The root SHALL declare a driver per camera,
`<front|rear>_<left|right>_pan` (degrees, default 0, range -90 to 90, the
HS-488HB's travel), turning the camera base link and the camera on it
about the axis of the hip's vision servo spline, which the base's 25T
servo hub sits on: vertical through the vision assembly's origin, for the
front-left camera the world Z axis through (544.5, 152) mm. The servo hub
SHALL stay concentric with it within 0.1 mm.

#### Scenario: A panned base
- **WHEN** `front_left_pan` is 60
- **THEN** the front-left camera base's hub centre is within 0.1 mm of (544.5, 152) mm in plan and the camera's channel has swung 60° about that axis within 0.05 mm

### Requirement: The base servo tilts the camera
The root SHALL declare a driver per camera,
`<front|rear>_<left|right>_tilt` (degrees, default 0, range -90 to 90),
turning the camera link about the axis of the base's servo spline, which
the camera's 25T servo arm sits on: the camera link's own X axis through
its origin, for the front-left camera the world Y axis through
(540.4, ·, 73.5) mm at rest. Tilt SHALL compose inside pan: a panned
camera tilts about its panned axis.

#### Scenario: Tilt follows pan
- **WHEN** `front_left_pan` is 90 and `front_left_tilt` is 45
- **THEN** the camera's servo arm hub stays within 0.1 mm of the point (540.4, ·, 73.5) rotated 90° about the pan axis, and the camera channel's far end has risen or fallen by the 45° tilt about the panned axis within 0.1 mm
