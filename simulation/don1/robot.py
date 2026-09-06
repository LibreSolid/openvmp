"""Don1, as a machine.

``robot.assy`` nests the links of the robot and places them; this module
writes that nesting as assemblies with the same numbers and adds what the
blueprint has no notation for: the joints. Each joint turns about the
axis the blueprint's own bearings define (see the design record), driven
by one driver per motor on the root, reached through ports down the
tree. The drive-train parts that visibly turn -- worms, worm gears,
sprockets, the shafts the joints ride on -- turn with their joints at
their ratios.
"""

from solid_node.node import AssemblyNode, RotationalPort
from solid_node.parameters import Count
from solid_node.simulation import Driver, Instruction

from simulation.don1.link import Link, axis_of, rotate_about, spin

# robot.assy: where the links nest, in millimetres and degrees
END_OFFSET = 272.5          # each turntable group's centre from the base origin
END_HEIGHT = -7.0
HIP_OFFSET = [149.5, 0.0, -30.0]      # the hip group in the turntable frame
SIDE_OFFSET = [90.0, 0.0, 0.0]        # each side's frame in the hip frame
THIGH_OFFSET = [0.0, -11.5, 0.0]      # the thigh in the side frame
FOOT_OFFSET = [0.0, -304.9, 108.75]   # the foot in the side frame ...
FOOT_REST_BEND = 39.0                 # ... bent this much about X
WHEEL_OFFSET = [0.0, -293.5, -75.2]   # the wheel in the foot frame, turned 90 about X
VISION_HEIGHT = 86.0                  # the vision assembly in the side frame:
VISION_REACH = 152.0                  #   (-side * 32.5, -152, 86)
VISION_SPREAD = 32.5
CAMERA_OFFSET = (4.125, 13.5, 24.5)   # the camera link in the vision frame, x times dir * side

# link-lower-arm.assy: the knee shaft the foot really turns on, in the side
# frame -- the knee bearings at y -304.9 in a thigh that sits at -11.5
KNEE_SHAFT = [0.0, -316.4, 108.8]

# The parts' own axes in their STEP frames, read off the cylindrical
# faces of the index files (the design record lists the probe).
WORM_AXIS = ((0, 1, 0), (-5.26, 0.0, 33.93))
REX_SHAFT_AXIS = ((0, 1, 0), (-43.35, 0.0, 6.97))
SPROCKET_AXIS = ((0, 0, 1), (0.0, 0.0, 0.0))

# Gear ratios: a single-start worm on a 28-tooth gear, and the sprockets
# the chains join -- 14T on the base motors, 16T on the worm shafts and
# the knee motor.
WORM_GEAR_TEETH = 28
BASE_MOTOR_SPROCKET = 14
BASE_WORM_SPROCKET = 16
KNEE_MOTOR_SPROCKET = 16
KNEE_WORM_SPROCKET = 16

X, Y, Z = [1, 0, 0], [0, 1, 0], [0, 0, 1]

# ROS description limits (openvmp_robot_don1.urdf) and the servos' travel
YAW_RANGE = (-90.0, 90.0)
ROLL_RANGE = (-210.0, 210.0)
THIGH_RANGE = (-180.0, 180.0)
KNEE_RANGE = (-162.0, 126.0)
WHEEL_RANGE = (0.0, 360.0)
SERVO_RANGE = (-90.0, 90.0)

ENDS = ('front', 'rear')
SIDES = ('left', 'right')


def _handed(value):
    if value not in (-1, 1):
        raise ValueError(f'a hand is 1 or -1, not {value}')


class Foot(AssemblyNode):
    """The L-shaped foot: the upper arm with its hook, and the wheel."""

    arm = Link('link-upper-arm')
    wheel = Link('link-wheel')

    spin = RotationalPort(unit='deg')

    def render(self):
        self.wheel.rotate(90, X).translate(WHEEL_OFFSET)

    def simulate(self):
        self.wheel.rotate(self.spin.value, Z)


class Leg(AssemblyNode):
    """A thigh and the foot on its knee, in the side frame.

    The thigh is turned about its own length by the hip that holds it (the
    ``turn`` port only counter-turns the shaft parts the hip keeps); the
    knee bends the foot about the knee shaft; the wheel spins.
    """

    thigh = Link('link-lower-arm')
    foot = Foot()

    turn = RotationalPort(unit='deg')
    knee = RotationalPort(unit='deg')
    wheel = RotationalPort(unit='deg')

    #: The knee shaft in the foot's unbent frame: the rest placement is
    #: rotate(39, X) then translate(FOOT_OFFSET), and simulate() composes
    #: inside it, so the shaft is carried back through both.
    @property
    def knee_pivot(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        offset = np.asarray(KNEE_SHAFT) - np.asarray(FOOT_OFFSET)
        return Rotation.from_rotvec(np.radians(-FOOT_REST_BEND) * np.asarray(X)).apply(offset).tolist()

    def render(self):
        self.thigh.translate(THIGH_OFFSET)
        self.foot.rotate(FOOT_REST_BEND, X).translate(FOOT_OFFSET)

    def simulate(self):
        bend = self.knee.value - FOOT_REST_BEND
        rotate_about(self.foot, bend, X, self.knee_pivot)
        self.foot.spin = self.wheel

        thigh = self.thigh
        # the shaft the hip clamps, its coupler and collar do not turn with
        # the thigh: counter-turn them about the thigh axis, which is the
        # thigh frame's Y axis (the frame sits on it)
        spin(thigh, ['rls-shaft', 'coupler', 'motion-collar_clamping_8mmREX'],
             -self.turn.value, axis_of(thigh, 'rls-shaft', *REX_SHAFT_AXIS))
        # the knee shaft, its worm gear and hub bend with the foot
        knee_axis = axis_of(thigh, 'knee-shaft', *REX_SHAFT_AXIS)
        spin(thigh, ['knee-shaft', 'assembly-wormgear/worm-gear', 'assembly-wormgear/hub'],
             bend, knee_axis)
        # the worm shaft turns 28 times faster, the motor sprocket with it
        worm = bend * WORM_GEAR_TEETH
        spin(thigh, ['assembly-wormgear/worm', 'knee-worm-shaft', 'knee-worm-hub',
                     'knee-worm-sprocket'],
             worm, axis_of(thigh, 'assembly-wormgear/worm', *WORM_AXIS))
        spin(thigh, ['knee-motor-hub', 'knee-motor-sprocket'],
             worm * KNEE_WORM_SPROCKET / KNEE_MOTOR_SPROCKET,
             axis_of(thigh, 'knee-motor-sprocket', *SPROCKET_AXIS))


class Camera(AssemblyNode):
    """A pan-tilt stereo camera: the base on the hip's servo, the camera on
    the base's servo, in the vision frame whose Z is the pan axis."""

    dir = Count(1, min=-1, max=1)
    side = Count(1, min=-1, max=1)

    base = Link('link-camera-servo', dir=side)
    camera = Link('link-camera', dir=side)

    tilt = RotationalPort(unit='deg')

    def check(self):
        _handed(self.dir)
        _handed(self.side)

    def render(self):
        x, y, z = CAMERA_OFFSET
        self.base.rotate(self.side * -180, Z)
        self.camera.rotate(self.side * -90, Z).translate([self.dir * self.side * x, y, z])

    def simulate(self):
        # the base's servo spline is the camera link's own X axis
        self.camera.rotate(self.tilt.value, X)


class Hip(AssemblyNode):
    """The hip with its two legs and two cameras, in the hip frame.

    Each side's frame is turned so that both legs read one file; the left
    is the side at +Y.
    """

    dir = Count(1, min=-1, max=1)

    link = Link('link-hip')
    left_leg = Leg()
    right_leg = Leg()
    left_camera = Camera(dir=dir, side=1)
    right_camera = Camera(dir=dir, side=-1)

    left_thigh = RotationalPort(unit='deg')
    right_thigh = RotationalPort(unit='deg')
    left_knee = RotationalPort(unit='deg')
    right_knee = RotationalPort(unit='deg')
    left_wheel = RotationalPort(unit='deg')
    right_wheel = RotationalPort(unit='deg')
    left_pan = RotationalPort(unit='deg')
    right_pan = RotationalPort(unit='deg')
    left_tilt = RotationalPort(unit='deg')
    right_tilt = RotationalPort(unit='deg')

    def check(self):
        _handed(self.dir)

    def sides(self):
        return ((1, self.left_leg, self.left_camera), (-1, self.right_leg, self.right_camera))

    def render(self):
        for side, leg, camera in self.sides():
            leg.rotate(90 + 90 * side, Z).translate(SIDE_OFFSET)
            (camera.translate([-side * VISION_SPREAD, -VISION_REACH, VISION_HEIGHT])
             .rotate(90 + 90 * side, Z).translate(SIDE_OFFSET))

    def simulate(self):
        for name, side, leg, camera in (('left', 1, self.left_leg, self.left_camera),
                                        ('right', -1, self.right_leg, self.right_camera)):
            # both legs turn about the hip's own Y axis in one sense, as
            # the ROS description has it; the left leg's frame is turned
            # 180 degrees, so its own Y is the hip's -Y
            turn = getattr(self, f'{name}_thigh').value * side
            leg.rotate(turn, Y)
            leg.turn = turn
            leg.knee = getattr(self, f'{name}_knee')
            leg.wheel = getattr(self, f'{name}_wheel')
            camera.rotate(getattr(self, f'{name}_pan').value, Z)
            camera.tilt = getattr(self, f'{name}_tilt')


class Side(AssemblyNode):
    """One end of the robot: the turntable and the hip it rolls."""

    dir = Count(1, min=-1, max=1)

    turntable = Link('link-turn-table')
    hip = Hip(dir=dir)

    roll = RotationalPort(unit='deg')
    left_thigh = RotationalPort(unit='deg')
    right_thigh = RotationalPort(unit='deg')
    left_knee = RotationalPort(unit='deg')
    right_knee = RotationalPort(unit='deg')
    left_wheel = RotationalPort(unit='deg')
    right_wheel = RotationalPort(unit='deg')
    left_pan = RotationalPort(unit='deg')
    right_pan = RotationalPort(unit='deg')
    left_tilt = RotationalPort(unit='deg')
    right_tilt = RotationalPort(unit='deg')

    HIP_PORTS = ('left_thigh', 'right_thigh', 'left_knee', 'right_knee',
                 'left_wheel', 'right_wheel', 'left_pan', 'right_pan',
                 'left_tilt', 'right_tilt')

    def check(self):
        _handed(self.dir)

    def render(self):
        self.hip.translate(HIP_OFFSET)

    def simulate(self):
        self.hip.rotate(self.roll.value, X)
        for port in self.HIP_PORTS:
            setattr(self.hip, port, getattr(self, port))
        # the turntable's shaft, coupler, collar and clip are clamped to
        # the hip and roll with it
        table = self.turntable
        spin(table, ['shaft', 'shaft-clip', 'coupler', 'collar'], self.roll.value,
             axis_of(table, 'shaft', *REX_SHAFT_AXIS))


def _joint_drivers():
    drivers = {}
    for end in ENDS:
        drivers[f'{end}_yaw'] = Driver(default=0.0, range=YAW_RANGE, unit='deg')
        drivers[f'{end}_roll'] = Driver(default=0.0, range=ROLL_RANGE, unit='deg')
        for side in SIDES:
            drivers[f'{end}_{side}_thigh'] = Driver(default=0.0, range=THIGH_RANGE, unit='deg')
            drivers[f'{end}_{side}_knee'] = Driver(default=FOOT_REST_BEND, range=KNEE_RANGE, unit='deg')
            drivers[f'{end}_{side}_wheel'] = Driver(default=0.0, range=WHEEL_RANGE, unit='deg')
            drivers[f'{end}_{side}_pan'] = Driver(default=0.0, range=SERVO_RANGE, unit='deg')
            drivers[f'{end}_{side}_tilt'] = Driver(default=0.0, range=SERVO_RANGE, unit='deg')
    return drivers


def pose(**joints):
    """A full target set: every joint at its default unless named.

    ``joints`` may name a driver exactly, or a joint with the end and side
    left out (``knee=-90``) to set it on every leg, or with one of them
    (``front_knee``, ``left_thigh``).
    """
    targets = {}
    for end in ENDS:
        targets[f'{end}_yaw'] = 0.0
        targets[f'{end}_roll'] = 0.0
        for side in SIDES:
            for joint, default in (('thigh', 0.0), ('knee', FOOT_REST_BEND),
                                   ('wheel', 0.0), ('pan', 0.0), ('tilt', 0.0)):
                targets[f'{end}_{side}_{joint}'] = default
    for name, value in joints.items():
        matched = [t for t in targets
                   if t == name or t.endswith('_' + name) or
                   (name.split('_')[0] in ENDS and t.startswith(name.split('_')[0]) and t.endswith('_' + name.split('_', 1)[1]))]
        if not matched:
            raise KeyError(f'{name} names no joint')
        for t in matched:
            targets[t] = float(value)
    return targets


class Don1(AssemblyNode):
    """The robot: the base, and a turntable-hip-legs-cameras side at each end."""

    base = Link('link-base')
    front = Side(dir=1)
    rear = Side(dir=-1)

    locals().update(_joint_drivers())

    #: The poses. Rest is the blueprint's; Stand, Hug and the Walk stance
    #: are the upstream motion scripts' joint targets (stand.py, hug.py,
    #: walk.py); Crouch lowers Stand onto the wheel rims; Duct rolls the
    #: hips a quarter turn so that one wheel per end presses the floor and
    #: the other the ceiling, cambered 45 degrees -- the nearest this
    #: blueprint comes to rolling on its wheels: with the hips level the
    #: wheels sit at the feet's corners above the base, with a leg pointing
    #: down a level wheel sits above the thigh's own knee end, and the foot
    #: fouls the thigh's knee motor beyond 45 degrees of bend.
    instructions = {
        'Rest': Instruction(pose(), duration=2.0),
        'Stand': Instruction(pose(thigh=180.0, knee=-90.0), duration=3.0),
        'Crouch': Instruction(pose(thigh=180.0, knee=-40.0), duration=2.0),
        'Hug': Instruction(pose(thigh=0.0, knee=-180.0), duration=3.0),
        'Walk': Instruction(pose(thigh=180.0, knee=-83.0, front_yaw=-17.0, rear_yaw=17.0,
                                 left_thigh=157.0, right_thigh=-157.0),
                            duration=3.0),
        'Duct': Instruction(pose(front_roll=-90.0, rear_roll=90.0, knee=45.0,
                                 front_thigh=0.0, rear_thigh=180.0),
                            duration=3.0),
        'TurnLeft': Instruction(pose(front_yaw=30.0, rear_yaw=-30.0), duration=2.0),
        'TurnRight': Instruction(pose(front_yaw=-30.0, rear_yaw=30.0), duration=2.0),
        'Roll': Instruction(pose(wheel=360.0), duration=2.0),
        'Look': Instruction(pose(left_pan=-45.0, right_pan=45.0, tilt=-30.0), duration=2.0),
    }

    SIDE_PORTS = ('roll',) + Side.HIP_PORTS

    def render(self):
        self.front.translate([END_OFFSET, 0, END_HEIGHT])
        self.rear.rotate(180, Z).translate([-END_OFFSET, 0, END_HEIGHT])

    def simulate(self):
        for end, side in (('front', self.front), ('rear', self.rear)):
            yaw = getattr(self, f'{end}_yaw')
            side.rotate(yaw, Z)
            for port in self.SIDE_PORTS:
                setattr(side, port, getattr(self, f'{end}_{port}'))

            # the base's drive train for this end: the gear shaft and its
            # worm gear turn with the table, the worm shaft 28 times as
            # fast, the two motor sprockets with the chain
            base = self.base
            prefix = f'motion-{end}'
            # the blueprint's "worm-collar" clamps the gear shaft: its bore
            # is on the yaw axis, not the worm's
            gear_axis = axis_of(base, f'{prefix}-gear-shaft', *REX_SHAFT_AXIS)
            spin(base, [f'{prefix}-gear-shaft', f'{prefix}-gear-shaft-clip',
                        f'{prefix}-wormgear/worm-gear', f'{prefix}-wormgear/hub',
                        f'{prefix}-worm-collar'],
                 yaw, gear_axis)
            worm = yaw * WORM_GEAR_TEETH
            spin(base, [f'{prefix}-wormgear/worm', f'{prefix}-worm-shaft', f'{prefix}-worm-hub',
                        f'{prefix}-worm-sproket'],
                 worm, axis_of(base, f'{prefix}-wormgear/worm', *WORM_AXIS))
            for side_name in SIDES:
                spin(base, [f'{prefix}-{side_name}-sprocket', f'{prefix}-{side_name}-hub'],
                     worm * BASE_WORM_SPROCKET / BASE_MOTOR_SPROCKET,
                     axis_of(base, f'{prefix}-{side_name}-sprocket', *SPROCKET_AXIS))
