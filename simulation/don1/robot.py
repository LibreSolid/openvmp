"""Don1, as a machine.

``robot.assy`` nests the links of the robot and places them; this module
writes that nesting as assemblies with the same numbers and adds what the
blueprint has no notation for: the joints. Each joint turns about the
axis the blueprint's own bearings define (see the design record), and the
24 drivers on the root reach their coordinates by one ``drives`` relation
each, straight down the tree by path. The drive-train parts that visibly
turn -- worms, worm gears, sprockets, the shafts the joints ride on -- are
placed by ``blueprints.load()`` rather than declared, so they stay
hand-turned in ``simulate()`` at their ratios, reading the joints' own
coordinates.
"""

from solid_node.node import AssemblyNode
from solid_node.motion.joints import Revolute
from solid_node.parameters import Count
from solid_node.simulation import Driver, Instruction

from simulation.don1.link import Link, axis_of, spin

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

# link-lower-arm.assy: the knee shaft the foot really turns on. Was stated
# in the side frame (the knee bearings at y -304.9 in a thigh that sits at
# -11.5); carried into the FOOT's own rest frame for
# joint-frame-follows-declarer by inverting the foot's rest placement
# (rotate(FOOT_REST_BEND, X).translate(FOOT_OFFSET)).
KNEE_SHAFT = [0.0, -8.905712537202675, 7.276041795145978]

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


class Wheel(Link):
    """The wheel link, spinning in the foot's bearings."""

    #: The spin, now stated in the wheel's own rest frame
    #: (joint-frame-follows-declarer): Foot.render() turns the wheel 90
    #: degrees about X before placing it, so the bearing line that was -Y
    #: in the foot's frame is +Z here; the anchor restated Foot.render()'s
    #: own translation exactly, so it is the wheel's own placed origin now.
    spin = Revolute(axis=(0, 0, 1), range=WHEEL_RANGE, unit='deg')


class Foot(AssemblyNode):
    """The L-shaped foot: the upper arm with its hook, and the wheel."""

    arm = Link('link-upper-arm')
    wheel = Wheel('link-wheel')

    #: The knee, now stated in the FOOT's own rest frame
    #: (joint-frame-follows-declarer): the knee bearings, the worm gear
    #: and the shaft's own axis all agree on this line, which is not the
    #: foot's own placed origin (the blueprint's 11.5 mm error) --
    #: KNEE_SHAFT above is the old leg-frame value carried through the
    #: inverse of the foot's rest placement.
    knee = Revolute(axis=(1, 0, 0), at=KNEE_SHAFT, unit='deg')

    def render(self):
        self.wheel.rotate(90, X).translate(WHEEL_OFFSET)


class Leg(AssemblyNode):
    """A thigh and the foot on its knee, in the side frame.

    The thigh is turned about its own length by the hip that holds it; the
    knee bends the foot about the knee shaft; the wheel spins. ``side`` is
    not the joint's own axis, which is the same for both legs in the hip's
    frame, but the leg's own frame is turned 180 degrees for the left leg,
    so the thigh's counter-spun clamp shaft needs it.
    """

    side = Count(1, min=-1, max=1)

    thigh = Link('link-lower-arm')
    foot = Foot()

    #: The thigh. The radial-load bearings put the axis along the hip
    #: frame's Y through the side frame's origin, one physical line for
    #: both legs -- but Hip.render() turns each leg's own frame 180
    #: degrees apart (left at Z=180, right at Z=0), so in the leg's own
    #: rest frame that one line reads oppositely: (0, side, 0).
    turn = Revolute(axis=lambda node: (0, node.side, 0), range=THIGH_RANGE, unit='deg')

    def check(self):
        _handed(self.side)

    def render(self):
        self.thigh.translate(THIGH_OFFSET)
        self.foot.rotate(FOOT_REST_BEND, X).translate(FOOT_OFFSET)

    def simulate(self):
        thigh = self.thigh
        # the shaft the hip clamps, its coupler and collar do not turn with
        # the thigh: counter-turn them about the thigh axis, which is the
        # thigh frame's Y axis (the frame sits on it); the turn joint's own
        # coordinate carries no side term, so this hand-written dressing
        # applies it
        spin(thigh, ['rls-shaft', 'coupler', 'motion-collar_clamping_8mmREX'],
             -self.turn.value * self.side, axis_of(thigh, 'rls-shaft', *REX_SHAFT_AXIS))
        # the knee shaft, its worm gear and hub bend with the foot; the
        # knee joint's own coordinate is already the bend past the rest
        # placement
        bend = self.foot.knee.value
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


class CameraArm(Link):
    """The camera link, tilted by the base's servo."""

    #: The tilt, now stated in the camera link's OWN rest frame
    #: (joint-frame-follows-declarer): Camera.render() rotates this body
    #: by +-90 degrees about Z depending on side before placing it, and
    #: that carries the old parent-frame axis (0, -+1, 0) to the SAME
    #: line, (1, 0, 0), at all four corners (front/rear x left/right) --
    #: dir only ever affected the translation, never this rotation -- so
    #: one literal now covers every site and no callable is needed. The
    #: anchor restated Camera.render()'s own translation of this body
    #: exactly, so it is the body's own placed origin now.
    tilt = Revolute(axis=(1, 0, 0), range=SERVO_RANGE, unit='deg')

    def __init__(self, assembly, dir, side, name=None):
        # dir (front/rear) and side (left/right) are separate positions;
        # the blueprint only distinguishes left/right, so its own "dir"
        # parameter (the mirrored STEP variant) is this arm's side, not
        # its end. Neither is read here any more: the joint above no
        # longer needs them (its axis is one literal, correct at every
        # corner).
        super().__init__(assembly, name=name, dir=side)


class Camera(AssemblyNode):
    """A pan-tilt stereo camera: the base on the hip's servo, the camera on
    the base's servo, in the vision frame whose Z is the pan axis."""

    dir = Count(1, min=-1, max=1)
    side = Count(1, min=-1, max=1)

    base = Link('link-camera-servo', dir=side)
    camera = CameraArm('link-camera', dir=dir, side=side)

    #: The pan, vertical through the vision servo's spline. Hip.render()
    #: turns this body about that same Z line before placing it, so the
    #: axis is unaffected; the anchor restated Hip.render()'s own
    #: placement of this body exactly, so it is the body's own placed
    #: origin now.
    pan = Revolute(axis=(0, 0, 1), range=SERVO_RANGE, unit='deg')

    def check(self):
        _handed(self.dir)
        _handed(self.side)

    def render(self):
        x, y, z = CAMERA_OFFSET
        self.base.rotate(self.side * -180, Z)
        self.camera.rotate(self.side * -90, Z).translate([self.dir * self.side * x, y, z])


class Hip(AssemblyNode):
    """The hip with its two legs and two cameras, in the hip frame.

    Each side's frame is turned so that both legs read one file; the left
    is the side at +Y.
    """

    dir = Count(1, min=-1, max=1)

    link = Link('link-hip')
    left_leg = Leg(side=1)
    right_leg = Leg(side=-1)
    left_camera = Camera(dir=dir, side=1)
    right_camera = Camera(dir=dir, side=-1)

    #: The roll: the turntable's bearings put the axis along X through
    #: the hip group's own origin. Side.render() only translates this
    #: body (no rotation), so the axis is unchanged, and the anchor
    #: restated that translation exactly, so it is the body's own
    #: placed origin now.
    roll = Revolute(axis=(1, 0, 0), range=ROLL_RANGE, unit='deg')

    def check(self):
        _handed(self.dir)

    def render(self):
        for side, leg, camera in ((1, self.left_leg, self.left_camera),
                                  (-1, self.right_leg, self.right_camera)):
            leg.rotate(90 + 90 * side, Z).translate(SIDE_OFFSET)
            (camera.translate([-side * VISION_SPREAD, -VISION_REACH, VISION_HEIGHT])
             .rotate(90 + 90 * side, Z).translate(SIDE_OFFSET))


class Side(AssemblyNode):
    """One end of the robot: the turntable and the hip it rolls."""

    dir = Count(1, min=-1, max=1)

    turntable = Link('link-turn-table')
    hip = Hip(dir=dir)

    #: The yaw: the base's two 8 mm REX flanged bearings put the axis
    #: vertical through this end's centre. Don1.render() turns the rear
    #: end 180 degrees about that same Z line before placing it, so the
    #: axis is unaffected at both ends; the anchor restated Don1.render()'s
    #: own placement of this body exactly at both ends, so it is the
    #: body's own placed origin now.
    yaw = Revolute(axis=(0, 0, 1), range=YAW_RANGE, unit='deg')

    def check(self):
        _handed(self.dir)

    def render(self):
        self.hip.translate(HIP_OFFSET)

    def simulate(self):
        # the turntable's shaft, coupler, collar and clip are clamped to
        # the hip and roll with it
        table = self.turntable
        spin(table, ['shaft', 'shaft-clip', 'coupler', 'collar'], self.hip.roll.value,
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

    # The 24 drivers reach their coordinates by path, one relation each;
    # every ratio is 1, the handedness living in the joints' own axes. The
    # knee's offset carries the driver's absolute ROS angle to the joint's
    # bend-past-rest coordinate. Written out for the front end; the rear
    # end is the same, front -> rear.
    front_yaw.drives(front.yaw)
    front_roll.drives(front.hip.roll)
    front_left_thigh.drives(front.hip.left_leg.turn)
    front_right_thigh.drives(front.hip.right_leg.turn)
    front_left_knee.drives(front.hip.left_leg.foot.knee, offset=-FOOT_REST_BEND)
    front_right_knee.drives(front.hip.right_leg.foot.knee, offset=-FOOT_REST_BEND)
    front_left_wheel.drives(front.hip.left_leg.foot.wheel.spin)
    front_right_wheel.drives(front.hip.right_leg.foot.wheel.spin)
    front_left_pan.drives(front.hip.left_camera.pan)
    front_right_pan.drives(front.hip.right_camera.pan)
    front_left_tilt.drives(front.hip.left_camera.camera.tilt)
    front_right_tilt.drives(front.hip.right_camera.camera.tilt)

    rear_yaw.drives(rear.yaw)
    rear_roll.drives(rear.hip.roll)
    rear_left_thigh.drives(rear.hip.left_leg.turn)
    rear_right_thigh.drives(rear.hip.right_leg.turn)
    rear_left_knee.drives(rear.hip.left_leg.foot.knee, offset=-FOOT_REST_BEND)
    rear_right_knee.drives(rear.hip.right_leg.foot.knee, offset=-FOOT_REST_BEND)
    rear_left_wheel.drives(rear.hip.left_leg.foot.wheel.spin)
    rear_right_wheel.drives(rear.hip.right_leg.foot.wheel.spin)
    rear_left_pan.drives(rear.hip.left_camera.pan)
    rear_right_pan.drives(rear.hip.right_camera.pan)
    rear_left_tilt.drives(rear.hip.left_camera.camera.tilt)
    rear_right_tilt.drives(rear.hip.right_camera.camera.tilt)

    def render(self):
        self.front.translate([END_OFFSET, 0, END_HEIGHT])
        self.rear.rotate(180, Z).translate([-END_OFFSET, 0, END_HEIGHT])

    def simulate(self):
        # the base's drive train for each end: the gear shaft and its worm
        # gear turn with the table, the worm shaft 28 times as fast, the
        # two motor sprockets with the chain
        base = self.base
        for end in ENDS:
            prefix = f'motion-{end}'
            yaw = getattr(self, f'{end}_yaw')
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
