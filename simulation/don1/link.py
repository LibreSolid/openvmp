"""A blueprint assembly file as one assembly node."""

from solid_node.node import AssemblyNode

from simulation.don1 import blueprints
from simulation.don1.parts import part_node


def unique_names(placements):
    """The file's own names, an index appended where a name repeats."""
    names, seen = [], {}
    for placement in placements:
        base = placement.name.replace('/', '-')
        seen[base] = seen.get(base, 0) + 1
        names.append(base if seen[base] == 1 else f'{base}-{seen[base]}')
    return names


class Link(AssemblyNode):
    """The parts of one ``.assy`` file, each at the location the file gives.

    Identity is the file plus its PartCAD parameters; the children are the
    file's entries in order, nested assemblies as nested links, unnamed
    groups folded into their parts' placements. Every blueprint file is a
    tracked source, so an edited placement rebuilds the link.
    """

    def __init__(self, assembly, name=None, **params):
        self.assembly = assembly
        self.params = params
        super().__init__(assembly, name=name, **params)
        self.placements = blueprints.load(assembly, **params)
        self.parts = []
        for placement, child_name in zip(self.placements, unique_names(self.placements)):
            if placement.part:
                child = part_node(placement.part, name=child_name)
            else:
                child = SubAssembly(placement.assembly, name=child_name, **placement.params)
            self.parts.append(child)
        self.files = self.files | blueprints.blueprint_sources()

    def render(self):
        for placement, child in zip(self.placements, self.parts):
            placement.place(child)
        return self.parts


class SubAssembly(Link):
    """A blueprint assembly placed inside another one (the worm gear set).

    The framework refuses an assembly whose render() returns a node of its
    own class, so a nested link is this subclass; nothing else differs.
    """


def rotate_about(node, angle, axis, point):
    """Turn a node by ``angle`` about the line through ``point`` along
    ``axis``, both in the node's own frame; returns the node."""
    return (node.translate([-c for c in point])
            .rotate(angle, list(axis))
            .translate(list(point)))


def _matrix(placement):
    import numpy as np
    from scipy.spatial.transform import Rotation
    m = np.eye(4)
    if placement.angle:
        axis = np.asarray(placement.axis, dtype=float)
        m[:3, :3] = Rotation.from_rotvec(
            np.radians(placement.angle) * axis / np.linalg.norm(axis)).as_matrix()
    m[:3, 3] = placement.position
    return m


class Line:
    """An axis in a link's frame: a unit direction and a point on it."""

    def __init__(self, direction, point):
        import numpy as np
        direction = np.asarray(direction, dtype=float)
        self.direction = direction / np.linalg.norm(direction)
        self.point = np.asarray(point, dtype=float)


def _nodes(link):
    return dict(zip((child.name for child in link.parts), link.parts))


def node_at(link, path):
    """The child at a ``/``-separated path of names through nested links."""
    node = link
    for name in path.split('/'):
        node = _nodes(node)[name]
    return node


def transform_of(link, path):
    """The rest transform of a child at ``path``, in the link's frame."""
    import numpy as np
    m = np.eye(4)
    node = link
    for name in path.split('/'):
        index = [child.name for child in node.parts].index(name)
        m = m @ _matrix(node.placements[index])
        node = node.parts[index]
    return m


def axis_of(link, path, direction, point):
    """A part's own axis, given in its STEP frame, as a line in the link's frame."""
    m = transform_of(link, path)
    return Line(m[:3, :3] @ direction, m[:3, :3] @ point + m[:3, 3])


def spin(link, paths, angle, line):
    """Turn the parts at ``paths`` by ``angle`` about ``line`` (link frame).

    Each part's motion is applied in its own frame, inside its rest
    placement, so the line is carried back through that placement first.
    """
    import numpy as np
    for path in paths:
        m = transform_of(link, path)
        rotation, position = m[:3, :3], m[:3, 3]
        local_direction = rotation.T @ line.direction
        local_point = rotation.T @ (line.point - position)
        rotate_about(node_at(link, path), angle, local_direction.tolist(),
                     local_point.tolist())
