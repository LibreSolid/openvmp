"""The contracts of the Don1 simulation.

Placement is judged against an independent composition of the blueprint's
own locations; every joint against the axis its bearings define; every
pose against what it must achieve on the meshes; and motion against the
clearance engine in ``clearance.py``, which compares parts of different
links only: the blueprint's own within-link overlaps are its placements,
not motion, and four of the index's parts are meshes the engine refuses
(see ``clearance.py``).

Every number a contract expects is written here from the blueprint files
and the design, never read off the model.
"""

import contextlib
import math

import numpy as np
import trimesh
from solid_node.simulation import ScenarioTest
from solid_node.test import TestCase

from simulation.don1 import clearance
from simulation.don1.link import Link
from simulation.don1.parts import SideChannel, StepPart
from simulation.don1.robot import (
    Don1, ENDS, FOOT_REST_BEND, REX_SHAFT_AXIS, SIDES, WORM_AXIS, pose)

# robot.assy composed by hand for the front-left side (see the design's
# "Coordinates"): the side frame maps (x, y, z) to (512 - x, -y, -37 + z)
FRONT_LEFT_SIDE_ORIGIN = (512.0, 0.0, -37.0)
FRONT_YAW_AXIS = (272.5, 0.0)                     # vertical through the disc centre
FRONT_ROLL_AXIS_YZ = (0.0, -37.0)                 # along X through the hip origin
FRONT_LEFT_KNEE_AXIS_YZ = (316.4, 71.8)           # along X through the knee shaft
FRONT_LEFT_VISION_ORIGIN = (544.5, 152.0, 49.0)   # the pan axis, vertical
FRONT_LEFT_TILT_AXIS_XZ = (540.375, 73.5)         # along Y through the arm's hub

PART_COUNT = 503

#: The cross-link overlaps the blueprint itself contains at rest (the
#: design's "Interference" decision): shafts in the hubs that clamp them,
#: servo splines in their hubs and arms, the knee shaft in the foot hubs
#: and brackets it misses by 11.5 mm, the gear shaft in the turntable's
#: gearbox and attachment. Templates over ends and sides.
REST_OVERLAP_TEMPLATES_PER_END = (
    ('{E}.turntable.shaft', '{E}.hip.link.hub-rear'),
    ('{E}.turntable.spacer-4', '{E}.hip.link.hub-rear'),
    ('base.motion-{E}-gear-shaft', '{E}.turntable.gearbox-nema17-planetary-100'),
    ('base.motion-{E}-gear-bearing-1', '{E}.turntable.turntable-attachment'),
    ('base.motion-{E}-gear-shaft', '{E}.turntable.turntable-attachment'),
    ('base.motion-{E}-gear-shaft-clip', '{E}.turntable.nema17-mount-wide'),
)
REST_OVERLAP_TEMPLATES_PER_SIDE = (
    ('{E}.hip.link.thign-hub{N}', '{E}.hip.{S}_leg.thigh.rls-shaft'),
    ('{E}.hip.link.thign-hub{N}', '{E}.hip.{S}_leg.thigh.rls-spacer-4'),
    ('{E}.hip.link.servo-vision{N}', '{E}.hip.{S}_camera.base.motion-servo_attach_hub_low_25t'),
    ('{E}.hip.{S}_leg.thigh.knee-shaft', '{E}.hip.{S}_leg.foot.arm.motion-hub_sonic_8mmREX'),
    ('{E}.hip.{S}_leg.thigh.knee-shaft', '{E}.hip.{S}_leg.foot.arm.motion-hub_sonic_8mmREX-2'),
    ('{E}.hip.{S}_leg.thigh.knee-shaft', '{E}.hip.{S}_leg.foot.arm.structure-bracket_flat_1_2'),
    ('{E}.hip.{S}_leg.thigh.knee-shaft', '{E}.hip.{S}_leg.foot.arm.structure-bracket_flat_1_2-2'),
    ('{E}.hip.{S}_camera.base.motion-servo_hs_488hb', '{E}.hip.{S}_camera.camera.motion-servo_attach_arm_25t'),
)


def rest_overlap_pairs():
    pairs = set()
    for end in ENDS:
        for a, b in REST_OVERLAP_TEMPLATES_PER_END:
            pairs.add(frozenset((a.format(E=end), b.format(E=end))))
        for side, suffix in (('left', ''), ('right', '-2')):
            for a, b in REST_OVERLAP_TEMPLATES_PER_SIDE:
                pairs.add(frozenset((a.format(E=end, S=side, N=suffix),
                                     b.format(E=end, S=side, N=suffix))))
    return pairs


def find(node, path):
    for name in path.split('.'):
        node = next(child for child in node.children if child.name == name)
    return node


def leaves(node):
    if not node.children:
        return [node]
    return [leaf for child in node.children for leaf in leaves(child)]


def lowest(node):
    return min(leaf.mesh.bounds[0][2] for leaf in leaves(node))


def highest(node):
    return max(leaf.mesh.bounds[1][2] for leaf in leaves(node))


def origin_of(leaf):
    return clearance.world_matrix(leaf)[:3, 3]


def axis_in_world(leaf, direction, point):
    """A part's own axis (STEP frame) as (direction, point) in the world."""
    matrix = clearance.world_matrix(leaf)
    return (matrix[:3, :3] @ np.asarray(direction, dtype=float),
            matrix[:3, :3] @ np.asarray(point, dtype=float) + matrix[:3, 3])


def distance_to_line(point, direction, on_line):
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    offset = np.asarray(point, dtype=float) - np.asarray(on_line, dtype=float)
    return np.linalg.norm(offset - np.dot(offset, direction) * direction)


def rotate_points(points, angle, direction, on_line):
    matrix = trimesh.transformations.rotation_matrix(
        math.radians(angle), direction, on_line)
    return trimesh.transform_points(points, matrix)


def principal_axis(mesh):
    components = mesh.principal_inertia_components
    return mesh.principal_inertia_vectors[np.argmax(components)]


def tilt_from_horizontal(direction):
    return math.degrees(math.asin(abs(direction[2]) / np.linalg.norm(direction)))


class Don1Test(TestCase):

    node = Don1

    @contextlib.contextmanager
    def posed(self, **joints):
        """Every driver at ``pose(**joints)``, then back to the defaults."""
        self.node.set_state(**pose(**joints))
        try:
            yield
        finally:
            self.node.set_state(**pose())

    # -- blueprint-import ---------------------------------------------------

    def test_every_part_is_placed(self):
        self.assertEqual(len(leaves(self.node)), PART_COUNT)

    def test_parts_nested_three_groups_deep_stand_where_the_blueprint_puts_them(self):
        expected = {
            'front.turntable.turntable': (272.5, 0.0, -4.5),
            'front.hip.link.channel-bottom': (632.5, 0.0, -15.65),
            'front.hip.left_leg.foot.arm.hook': (512.0, 318.1, -313.65),
        }
        for path, point in expected.items():
            with self.subTest(path=path):
                np.testing.assert_allclose(origin_of(find(self.node, path)), point, atol=0.01)

    def test_handed_camera_links_mirror_across_the_hip(self):
        # the parts on the two splines: their origins are the spline axes
        for part in ('base.motion-servo_attach_hub_low_25t', 'camera.motion-servo_attach_arm_25t'):
            with self.subTest(part=part):
                left = origin_of(find(self.node, 'front.hip.left_camera.' + part))
                right = origin_of(find(self.node, 'front.hip.right_camera.' + part))
                np.testing.assert_allclose(left, right * [1, -1, 1], atol=0.01)

    def test_the_hook_is_one_closed_solid(self):
        hook = find(self.node, 'front.hip.left_leg.foot.arm.hook')
        mesh = trimesh.load(hook.stl_file, force='mesh')
        self.assertTrue(mesh.is_volume)
        self.assertAlmostEqual(mesh.volume / 1000.0, 170.6, delta=0.5)

    def test_custom_parts_are_single_solids(self):
        for leaf in leaves(self.node):
            if isinstance(leaf, SideChannel) or (
                    isinstance(leaf, StepPart) and 'openvmp/parts:' in leaf.part):
                with self.subTest(part=leaf.name):
                    self.assertNoDisconnectedSolids(leaf)

    def test_parts_are_coloured_by_material(self):
        colours = {
            find(self.node, 'front.hip.link.channel-bottom').color,
            find(self.node, 'base.motion-front-wormgear.worm-gear').color,
            find(self.node, 'front.hip.left_leg.foot.arm.hook').color,
            find(self.node, 'front.hip.left_leg.foot.wheel.motion-rubber_wheel_136_24').color,
        }
        self.assertEqual(len(colours), 4)
        for leaf in leaves(self.node):
            self.assertRegex(leaf.color, r'^#[0-9a-f]{6}$')

    def test_blueprints_and_parts_are_tracked_sources(self):
        hip = find(self.node, 'front.hip.link')
        self.assertTrue(any(path.endswith('link-hip.assy') for path in hip.files))
        hook = find(self.node, 'front.hip.left_leg.foot.arm.hook')
        self.assertTrue(any(path.endswith('hook.step') for path in hook.files))

    # -- turntable-yaw --------------------------------------------------------

    def test_yaw_turns_the_table_about_the_base_bearings(self):
        disc = find(self.node, 'front.turntable.turntable')
        channel = find(self.node, 'front.hip.link.channel-bottom')
        board = find(self.node, 'base.don1-board-top')
        rest_channel = origin_of(channel)
        rest_board = board.mesh.bounds.copy()
        with self.posed(front_yaw=45.0):
            centre = disc.mesh.bounds.mean(0)
            np.testing.assert_allclose(centre[:2], FRONT_YAW_AXIS, atol=0.2)
            turned = rotate_points([rest_channel], 45.0, [0, 0, 1],
                                   [FRONT_YAW_AXIS[0], FRONT_YAW_AXIS[1], 0.0])[0]
            np.testing.assert_allclose(origin_of(channel), turned, atol=0.05)
            np.testing.assert_allclose(board.mesh.bounds, rest_board, atol=1e-6)

    def test_one_yaw_driver_per_end_and_none_per_base_motor(self):
        names = {name for name in dir(type(self.node)) if name.endswith('_yaw')}
        self.assertEqual(names, {'front_yaw', 'rear_yaw'})
        self.assertFalse([name for name in dir(type(self.node)) if 'motor' in name])

    def test_the_base_worm_spins_28_turns_per_turn_of_the_table(self):
        worm = find(self.node, 'base.motion-front-wormgear.worm')
        sprocket = find(self.node, 'base.motion-front-left-sprocket')
        rest_worm = worm.mesh
        rest_sprocket = sprocket.mesh
        worm_axis = axis_in_world(worm, *WORM_AXIS)
        sprocket_axis = axis_in_world(sprocket, [0, 0, 1], [0, 0, 0])
        with self.posed(front_yaw=10.0):
            for part, rest, axis, angle in ((worm, rest_worm, worm_axis, 280.0),
                                            (sprocket, rest_sprocket, sprocket_axis,
                                             280.0 * 16 / 14)):
                with self.subTest(part=part.name):
                    posed = part.mesh
                    for sense in (angle, -angle):
                        turned = rotate_points(rest.vertices, sense, *axis)
                        if np.abs(turned - posed.vertices).max() < 0.05:
                            break
                    else:
                        self.fail(f'{part.name} did not turn {angle} degrees about its axis')

    # -- hip-roll -------------------------------------------------------------

    def test_roll_turns_the_hip_about_the_turntable_bearings(self):
        shaft = find(self.node, 'front.hip.left_leg.thigh.knee-shaft')
        table = find(self.node, 'front.turntable.channel')
        rest_table = table.mesh.bounds.copy()
        rest_direction, rest_point = axis_in_world(shaft, *REX_SHAFT_AXIS)
        self.assertLess(math.degrees(math.acos(abs(rest_direction[0]))), 0.2)
        self.assertLess(distance_to_line((512.0, 316.4, 71.8), rest_direction, rest_point), 0.1)
        roll_axis = [1, 0, 0], [0.0, FRONT_ROLL_AXIS_YZ[0], FRONT_ROLL_AXIS_YZ[1]]
        with self.posed(front_roll=90.0):
            direction, point = axis_in_world(shaft, *REX_SHAFT_AXIS)
            turned = rotate_points([rest_point], 90.0, *roll_axis)[0]
            self.assertLess(distance_to_line(turned, direction, point), 0.05)
            self.assertLess(distance_to_line((512.0, -108.8, 279.4), direction, point), 0.1)
            np.testing.assert_allclose(table.mesh.bounds, rest_table, atol=1e-6)

    def test_the_hip_hub_stays_on_the_roll_axis(self):
        hub = find(self.node, 'front.hip.link.hub-rear')
        rest = hub.mesh.bounds.mean(0)
        for roll in (60.0, -135.0):
            with self.posed(front_roll=roll):
                centre = hub.mesh.bounds.mean(0)
                turned = rotate_points([rest], roll, [1, 0, 0],
                                       [0.0, FRONT_ROLL_AXIS_YZ[0], FRONT_ROLL_AXIS_YZ[1]])[0]
                np.testing.assert_allclose(centre, turned, atol=0.2)

    # -- thigh-turn -----------------------------------------------------------

    def test_a_half_turn_of_the_thigh_brings_the_foot_under_the_body(self):
        shaft = find(self.node, 'front.hip.left_leg.thigh.knee-shaft')
        foot = find(self.node, 'front.hip.left_leg.foot')
        hook = find(foot, 'arm.hook')
        wheel = find(foot, 'wheel.motion-rubber_wheel_136_24')
        rest_direction, rest_point = axis_in_world(shaft, *REX_SHAFT_AXIS)
        with self.posed(front_left_thigh=180.0):
            direction, point = axis_in_world(shaft, *REX_SHAFT_AXIS)
            turned = rotate_points([rest_point], 180.0, [0, 1, 0], FRONT_LEFT_SIDE_ORIGIN)[0]
            self.assertLess(distance_to_line(turned, direction, point), 0.05)
            self.assertLess(distance_to_line((512.0, 316.4, -145.8), direction, point), 0.1)
            self.assertGreater(hook.mesh.bounds[0][2], wheel.mesh.bounds[0][2])

    def test_the_thigh_bearings_stay_on_the_thigh_axis(self):
        bearing = find(self.node, 'front.hip.left_leg.thigh.rls-bearing-1')
        rest = bearing.mesh.bounds.mean(0)
        for turn in (45.0, 180.0):
            with self.posed(front_left_thigh=turn):
                turned = rotate_points([rest], turn, [0, -1, 0], FRONT_LEFT_SIDE_ORIGIN)[0]
                np.testing.assert_allclose(bearing.mesh.bounds.mean(0), turned, atol=0.2)

    def test_the_four_legs_turn_in_the_same_sense(self):
        hooks = {(end, side): find(self.node, f'{end}.hip.{side}_leg.foot.arm.hook')
                 for end in ENDS for side in SIDES}
        rest = {key: hook.mesh.bounds.mean(0) for key, hook in hooks.items()}
        with self.posed(thigh=60.0):
            moved = {key: hook.mesh.bounds.mean(0) - rest[key] for key, hook in hooks.items()}
            rises = [m[2] for m in moved.values()]
            self.assertLess(max(rises) - min(rises), 0.1)
            # the two hooks of a hip swing the same way along the body,
            # and the rear hip, turned 180 degrees, the opposite way
            for end in ENDS:
                self.assertAlmostEqual(moved[(end, 'left')][0], moved[(end, 'right')][0], delta=0.1)
            self.assertGreater(abs(moved[('front', 'left')][0]), 50.0)
            self.assertAlmostEqual(moved[('front', 'left')][0], -moved[('rear', 'left')][0], delta=0.1)

    # -- knee-bend ------------------------------------------------------------

    def test_the_default_bend_is_the_blueprints_pose(self):
        hook = find(self.node, 'front.hip.left_leg.foot.arm.hook')
        np.testing.assert_allclose(origin_of(hook), (512.0, 318.1, -313.65), atol=0.05)

    def test_the_foot_bends_about_the_knee_shaft(self):
        wheel = find(self.node, 'front.hip.left_leg.foot.wheel.motion-rubber_wheel_136_24')
        rest_origin = origin_of(wheel)
        np.testing.assert_allclose(rest_origin, (512.0, 485.67, -171.4), atol=0.05)
        with self.posed(front_left_knee=0.0):
            axis_point = [0.0, FRONT_LEFT_KNEE_AXIS_YZ[0], FRONT_LEFT_KNEE_AXIS_YZ[1]]
            # unbending by 39 degrees; the left side's frame is turned 180
            # degrees about Z, so its X is the world's -X
            expected = rotate_points([rest_origin], FOOT_REST_BEND, [1, 0, 0], axis_point)[0]
            np.testing.assert_allclose(origin_of(wheel), expected, atol=0.1)

    def test_the_knee_worm_spins_with_the_bend(self):
        worm = find(self.node, 'front.hip.left_leg.thigh.assembly-wormgear.worm')
        rest = worm.mesh
        axis = axis_in_world(worm, *WORM_AXIS)
        with self.posed(front_left_knee=FOOT_REST_BEND + 10.0):
            posed = worm.mesh
            self.assertTrue(any(
                np.abs(rotate_points(rest.vertices, sense, *axis) - posed.vertices).max() < 0.05
                for sense in (280.0, -280.0)))

    # -- wheel-spin -----------------------------------------------------------

    def test_a_spun_wheel_stays_put(self):
        wheel = find(self.node, 'front.hip.left_leg.foot.wheel.motion-rubber_wheel_136_24')
        rest = wheel.mesh
        axis = axis_in_world(wheel, [0, 0, 1], [0, 0, 0])
        with self.posed(front_left_wheel=137.0):
            posed = wheel.mesh
            np.testing.assert_allclose(posed.bounds.mean(0), rest.bounds.mean(0), atol=0.05)
            self.assertTrue(any(
                np.abs(rotate_points(rest.vertices, sense, *axis) - posed.vertices).max() < 0.05
                for sense in (137.0, -137.0)))

    # -- camera-pan-tilt ------------------------------------------------------

    def test_pan_turns_the_base_about_the_vision_servo(self):
        hub = find(self.node, 'front.hip.left_camera.base.motion-servo_attach_hub_low_25t')
        channel = find(self.node, 'front.hip.left_camera.camera.structure-u_channel_low_3')
        rest_channel = channel.mesh
        with self.posed(front_left_pan=60.0):
            np.testing.assert_allclose(hub.mesh.bounds.mean(0)[:2], FRONT_LEFT_VISION_ORIGIN[:2], atol=0.1)
            turned = rotate_points(rest_channel.vertices, 60.0, [0, 0, 1], FRONT_LEFT_VISION_ORIGIN)
            self.assertLess(np.abs(turned - channel.mesh.vertices).max(), 0.05)

    def test_tilt_follows_pan(self):
        arm = find(self.node, 'front.hip.left_camera.camera.motion-servo_attach_arm_25t')
        channel = find(self.node, 'front.hip.left_camera.camera.structure-u_channel_low_3')
        rest_channel = channel.mesh
        hub_rest = [FRONT_LEFT_TILT_AXIS_XZ[0], 136.5, FRONT_LEFT_TILT_AXIS_XZ[1]]
        with self.posed(front_left_pan=90.0, front_left_tilt=45.0):
            direction, point = axis_in_world(arm, [0, 0, 1], [0, 0, 0])
            panned_hub = rotate_points([hub_rest], 90.0, [0, 0, 1], FRONT_LEFT_VISION_ORIGIN)[0]
            self.assertLess(distance_to_line(panned_hub, direction, point), 0.1)
            # the channel is what the pan alone would give, tilted 45
            # degrees about the panned tilt axis
            panned = rotate_points(rest_channel.vertices, 90.0, [0, 0, 1], FRONT_LEFT_VISION_ORIGIN)
            tilt_axis = rotate_points([[0, 1, 0]], 90.0, [0, 0, 1], [0, 0, 0])[0]
            self.assertTrue(any(
                np.abs(rotate_points(panned, sense, tilt_axis, panned_hub) - channel.mesh.vertices).max() < 0.1
                for sense in (45.0, -45.0)))

    # -- robot-poses ----------------------------------------------------------

    def feet(self):
        return [(find(self.node, f'{end}.hip.{side}_leg.foot.arm.hook'),
                 find(self.node, f'{end}.hip.{side}_leg.foot.wheel.motion-rubber_wheel_136_24'))
                for end in ENDS for side in SIDES]

    def test_rest_puts_the_robot_on_its_hooks(self):
        for hook, wheel in self.feet():
            self.assertLess(hook.mesh.bounds[0][2], wheel.mesh.bounds[0][2])

    def test_stand_lifts_the_body_onto_the_feet(self):
        targets = self.node.instructions['Stand'].targets
        self.node.set_state(**targets)
        try:
            base_low = lowest(find(self.node, 'base'))
            feet_low = []
            for hook, wheel in self.feet():
                hook_low, wheel_low = hook.mesh.bounds[0][2], wheel.mesh.bounds[0][2]
                self.assertAlmostEqual(hook_low, wheel_low, delta=2.0)
                feet_low.append(min(hook_low, wheel_low))
            self.assertLess(max(feet_low), base_low - 250.0)
            self.assertLess(max(feet_low) - min(feet_low), 2.0)
        finally:
            self.node.set_state(**pose())

    def test_hug_folds_the_feet_back(self):
        self.node.set_state(**self.node.instructions['Hug'].targets)
        try:
            for end in ENDS:
                for side in SIDES:
                    with self.subTest(end=end, side=side):
                        leg = find(self.node, f'{end}.hip.{side}_leg')
                        knee = find(leg, 'thigh.knee-shaft').mesh.bounds.mean(0)
                        hip = find(self.node, f'{end}.hip.link.hub-rear').mesh.bounds.mean(0)
                        hook = find(leg, 'foot.arm.hook').mesh
                        wheel = find(leg, 'foot.wheel.motion-rubber_wheel_136_24').mesh.bounds.mean(0)
                        self.assertGreater(hook.bounds[0][2], knee[2])
                        self.assertLess(np.linalg.norm(wheel - hip), np.linalg.norm(knee - hip))
        finally:
            self.node.set_state(**pose())

    def test_duct_braces_level_wheels_against_floor_and_ceiling(self):
        self.node.set_state(**self.node.instructions['Duct'].targets)
        try:
            wheels = {(end, side): find(
                self.node, f'{end}.hip.{side}_leg.foot.wheel.motion-rubber_wheel_136_24').mesh
                for end in ENDS for side in SIDES}
            floor = [wheels[('front', 'left')], wheels[('rear', 'right')]]
            ceiling = [wheels[('front', 'right')], wheels[('rear', 'left')]]
            floor_low = [w.bounds[0][2] for w in floor]
            ceiling_high = [w.bounds[1][2] for w in ceiling]
            self.assertLess(max(floor_low) - min(floor_low), 2.0)
            self.assertLess(max(ceiling_high) - min(ceiling_high), 2.0)
            everything = leaves(self.node)
            others = [leaf for leaf in everything
                      if leaf.name != 'motion-rubber_wheel_136_24']
            self.assertLess(max(floor_low), min(leaf.mesh.bounds[0][2] for leaf in others))
            self.assertGreater(min(ceiling_high), max(leaf.mesh.bounds[1][2] for leaf in others))
            # the blueprint's foot cannot put a level wheel below the
            # thigh's knee end, and fouls the knee motor past 45 degrees of
            # bend: the brace is cambered 45 degrees
            for wheel in wheels.values():
                self.assertAlmostEqual(tilt_from_horizontal(principal_axis(wheel)), 45.0, delta=1.0)
        finally:
            self.node.set_state(**pose())

    def test_the_blueprints_rest_overlaps_are_the_recorded_ones(self):
        found = {frozenset((clearance.path_of(a, self.node), clearance.path_of(b, self.node)))
                 for a, b, _ in clearance.shared_volumes(self.node)}
        self.assertEqual(found, rest_overlap_pairs())

    def test_a_pose_that_must_collide_is_caught(self):
        """The clearance contract is evidence only if it can fail: the
        front hip rolled a quarter turn with the left foot folded back
        drives the hook through the hip channel and the other thigh."""
        with self.posed(front_roll=90.0, front_left_thigh=-90.0, front_left_knee=120.0):
            found = {frozenset((clearance.path_of(a, self.node), clearance.path_of(b, self.node)))
                     for a, b, _ in clearance.shared_volumes(self.node)}
        new = found - rest_overlap_pairs()
        self.assertIn(frozenset(('front.hip.left_leg.foot.arm.hook',
                                 'front.hip.link.channel-bottom')), new)

    # -- integrity ------------------------------------------------------------

    def test_solid_integrity(self):
        """The printed and cut parts, each one body; the bought parts are
        assemblies of bodies and are not asked this."""
        self.test_custom_parts_are_single_solids()

    def test_assembly_integrity(self):
        """No two parts of different links share volume at rest beyond
        the blueprint's own recorded overlaps (the motion sweep is the
        scenario below)."""
        self.test_the_blueprints_rest_overlaps_are_the_recorded_ones()


class Don1MotionTest(ScenarioTest):
    """Every instruction in turn, the clearance between links sampled."""

    node = Don1
    dt = 0.1
    meshes = True

    #: Every instruction, the thigh half-turns routed through Rest: a
    #: direct ramp between thighs at 180 and at 0 swings the feet through
    #: the body's centre, where the hooks meet (the design's findings).
    MOVES = ('Stand', 'Crouch', 'Rest', 'Hug', 'Rest', 'Walk', 'Rest', 'Duct', 'Rest',
             'TurnLeft', 'TurnRight', 'Look', 'Roll', 'Rest')
    SAMPLE = 0.5

    def assert_links_clear(self, sim):
        found = {frozenset((clearance.path_of(a, self.node), clearance.path_of(b, self.node)))
                 for a, b, _ in clearance.shared_volumes(self.node)}
        new = found - rest_overlap_pairs()
        self.assertFalse(new, f'at {sim.time:.1f} s new overlaps: {sorted(map(sorted, new))}')

    def test_no_new_overlap_along_the_moves(self):
        sim = self.simulation()
        clock = 0.0
        for move in self.MOVES:
            sim.at(clock).trigger(move)
            clock += self.node.instructions[move].duration
        sim.every(self.SAMPLE, self.assert_links_clear, sim)
        sim.run(clock)
