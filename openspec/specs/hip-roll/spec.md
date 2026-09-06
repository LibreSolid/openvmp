# hip-roll Specification

## Purpose
The turntable's geared stepper rolling the hip, with its legs and cameras, about the turntable's shaft.

## Requirements
### Requirement: The turntable rolls the hip about its shaft
The root SHALL declare drivers `front_roll` and `rear_roll` (degrees,
default 0, range -210 to 210 as the ROS description limits it). Each
SHALL roll the hip link and everything on it — legs and cameras — about
the axis the turntable's two flanged bearings define: the turntable's X
axis through (149.5, 0, -30) mm in the turntable's frame, which is the
REX shaft the hyper hub `hub-rear` clamps. The hip's rear hub SHALL stay
on that axis within 0.2 mm at every angle.

#### Scenario: A quarter roll
- **WHEN** `front_roll` is 90 and every other driver is at its default
- **THEN** the front-left thigh's knee shaft axis, at rest along X through (512, 316.2, 71.7) mm, now passes through (512, -108.7, 279.2) mm within 0.1 mm, and the front turntable has not moved

