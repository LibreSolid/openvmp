"""Access to the Don1 blueprints, which live in ``openvmp-models``.

The OpenVMP project describes each robot as PartCAD assemblies: YAML files
(``robots/don1/*.assy``, templated with Jinja) that place catalogue parts
and other assemblies by a PartCAD ``location``.  This module does not
replace those files; it reads them.  A link's parts and where they sit come
from here, and only what the blueprints do not say -- which links move,
about which axes, driven by what -- is written in ``robot.py``.

A location is ``[[x, y, z], [ax, ay, az], angle]``: the part is turned by
``angle`` degrees about the axis through its own origin, then carried to
``[x, y, z]`` -- exactly a solid-node ``rotate`` followed by a
``translate``.  Nested unnamed groups compose the same way, and a nested
``assembly`` is another ``.assy`` file with its own parameters.
"""

import os

import jinja2
import yaml

#: The blueprints repository, cloned beside this package's project.
MODELS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))),
    'openvmp-models')
ROBOT_DIR = os.path.join(MODELS_DIR, 'robots', 'don1')
PARTS_DIR = os.path.join(MODELS_DIR, 'parts')

#: The package the Don1 assemblies live in, as PartCAD names it.
PACKAGE = '//pub/robotics/multimodal/openvmp/robots/don1'


class Placement:
    """One entry of an assembly: a part or a sub-assembly at a location."""

    def __init__(self, name, location, part=None, assembly=None, params=None):
        self.name = name
        self.position, self.axis, self.angle = location
        self.part = part
        self.assembly = assembly
        self.params = params or {}

    def place(self, node):
        """Apply the location to a node as its rest placement."""
        if self.angle:
            node.rotate(self.angle, list(self.axis))
        if any(self.position):
            node.translate(list(self.position))
        return node


def assembly_file(name):
    return os.path.join(ROBOT_DIR, name + '.assy')


def _location(raw):
    position, axis, angle = raw
    return ([float(v) for v in position], [float(v) for v in axis], float(angle))


def _compose(outer, inner):
    """The location of ``inner`` seen from outside ``outer``'s group."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    def matrix(location):
        position, axis, angle = location
        m = np.eye(4)
        if angle:
            m[:3, :3] = Rotation.from_rotvec(
                np.radians(angle) * np.asarray(axis) / np.linalg.norm(axis)).as_matrix()
        m[:3, 3] = position
        return m

    m = matrix(outer) @ matrix(inner)
    rotation = Rotation.from_matrix(m[:3, :3]).as_rotvec()
    angle = float(np.degrees(np.linalg.norm(rotation)))
    axis = (rotation / np.linalg.norm(rotation)).tolist() if angle else [0.0, 0.0, 1.0]
    return (m[:3, 3].tolist(), axis, angle)


def _walk(links, outer, placements):
    for link in links:
        location = _location(link.get('location', [[0, 0, 0], [0, 0, 1], 0]))
        if outer is not None:
            location = _compose(outer, location)
        if 'part' in link:
            placements.append(Placement(link['name'], location, part=link['part']))
        elif 'assembly' in link:
            placements.append(Placement(
                link['name'], location, assembly=link['assembly'].split(':')[1],
                params={k: v for k, v in link.get('params', {}).items()}))
        else:
            # an unnamed group only carries a location
            _walk(link['links'], location, placements)
    return placements


def load(name, **params):
    """The placements of one assembly file, unnamed groups flattened.

    ``params`` are the assembly's PartCAD parameters (``dir`` for the
    handed camera links), reaching the template as ``param_<name>``.
    """
    with open(assembly_file(name)) as f:
        source = f.read()
    rendered = jinja2.Template(source).render(
        **{f'param_{k}': v for k, v in params.items()})
    return _walk(yaml.safe_load(rendered)['links'], None, [])


def blueprint_sources():
    """Every blueprint file a link's geometry depends on."""
    return {os.path.join(ROBOT_DIR, entry) for entry in os.listdir(ROBOT_DIR)
            if entry.endswith('.assy')}
