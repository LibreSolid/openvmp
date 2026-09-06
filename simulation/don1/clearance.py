"""Clearance between the links of the robot, measured on the meshes.

The framework's own interference assertion refuses a part whose STL is
not one closed volume, and twenty-one of the index's parts are not: a
bearing's balls touch its races, a board's components touch the board, a
servo's or a wheel's faces do not quite meet. Envelopes would fill their
bores and fake every shaft fit. So the robot's motion contract is
measured here instead: two parts of *different* links must never share
volume, at any pose. Parts of one link never move relative to each
other; what the blueprint overlaps within a link is its static placement,
reported by ``rest_overlaps`` and not a question about motion.

Each piece's mesh is repaired for the check only: degenerate and
duplicate triangles dropped, vertices merged, then split into bodies
along manifold edges (touching shells come apart), and a body that still
is not closed is replaced by its convex hull, recorded in ``HULLED`` as
the check runs. Bodies are compared as Manifolds, pairwise across links,
after a bounding-box broad phase.
"""

import numpy as np
import manifold3d
import trimesh

from simulation.don1.link import Link

#: Below this shared volume (mm³) two bodies are in contact, not
#: interfering: tangent vendor meshes intersect by rounding.
CONTACT_VOLUME = 1e-3

#: STL files whose bodies had to be hulled, with how many of them.
HULLED = {}

_bodies_cache = {}
_static_matrices = {}


def _manifold_of(mesh):
    manifold = manifold3d.Manifold(manifold3d.Mesh(
        vert_properties=np.ascontiguousarray(mesh.vertices, dtype=np.float32),
        tri_verts=np.ascontiguousarray(mesh.faces, dtype=np.uint32)))
    return manifold if manifold.status() == manifold3d.Error.NoError else None


def repaired_bodies(stl_file):
    """The closed bodies of one artifact: ``[(Manifold, bounds), ...]``."""
    cached = _bodies_cache.get(stl_file)
    if cached is not None:
        return cached
    mesh = trimesh.load(stl_file, force='mesh')
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()
    bodies, hulled = [], 0
    components = trimesh.graph.connected_components(
        mesh.face_adjacency, nodes=np.arange(len(mesh.faces)))
    for body in mesh.submesh(components, only_watertight=False, repair=False):
        if len(body.faces) < 4:
            continue
        manifold = _manifold_of(body) if body.is_volume else None
        if manifold is None:
            hulled += 1
            manifold = manifold3d.Manifold.hull_points(
                np.ascontiguousarray(body.vertices, dtype=np.float32))
            if manifold.status() != manifold3d.Error.NoError:
                continue
        bodies.append((manifold, body.bounds.copy()))
    if hulled:
        HULLED[stl_file] = hulled
    _bodies_cache[stl_file] = bodies
    return bodies


def world_matrix(leaf):
    """The rigid transform placing ``leaf``'s artifact in the world now."""
    base = trimesh.load(leaf.stl_file, force='mesh')
    world = leaf.mesh
    count = min(len(base.vertices), 500)
    picked = np.linspace(0, len(base.vertices) - 1, count).astype(int)
    return trimesh.transformations.affine_matrix_from_points(
        base.vertices[picked].T, world.vertices[picked].T, shear=False, scale=False)


def parts_by_link(root):
    """``[(link, leaf), ...]`` for every leaf, keyed by its outermost link."""
    found = []

    def walk(node, link):
        if isinstance(node, Link) and link is None:
            link = node
        if not node.children:
            found.append((link, node))
            return
        for child in node.children:
            walk(child, link)
    walk(root, None)
    return found


def _placed(matrix, bodies):
    placed = []
    for manifold, bounds in bodies:
        corners = trimesh.bounds.corners(bounds)
        world = trimesh.transform_points(corners, matrix)
        placed.append((manifold.transform(matrix[:3, :]),
                       np.array([world.min(0), world.max(0)])))
    return placed


def _boxes_touch(a, b, margin=0.0):
    return bool(np.all(a[0] <= b[1] + margin) and np.all(b[0] <= a[1] + margin))


def placement_matrix(link, leaf, static_link):
    """``world_matrix``, cached for the parts of the one link that never
    moves -- the root's base, whose heaviest meshes (the NUC boards, the
    battery) would otherwise be transformed at every sample."""
    if link is not None and link is static_link:
        key = id(leaf)
        if key not in _static_matrices:
            _static_matrices[key] = world_matrix(leaf)
        return _static_matrices[key]
    return world_matrix(leaf)


def shared_volumes(root, same_link=False, exclude=()):
    """Every pair of parts sharing volume now: ``[(leaf, leaf, mm³)]``.

    Across links by default; ``same_link=True`` reports pairs within one
    link instead. ``exclude`` holds pairs of part ids to leave out.
    """
    placed = []
    static_link = getattr(root, 'base', None) if isinstance(getattr(root, 'base', None), Link) else None
    for link, leaf in parts_by_link(root):
        bodies = repaired_bodies(leaf.stl_file)
        if not bodies:
            continue
        matrix = placement_matrix(link, leaf, static_link)
        world = _placed(matrix, bodies)
        extent = np.array([[b[0] for _, b in world], [b[1] for _, b in world]])
        placed.append((link, leaf, world,
                       np.array([extent[0].min(0), extent[1].max(0)])))
    excluded = {frozenset(pair) for pair in exclude}
    found = []
    for i in range(len(placed)):
        link_a, leaf_a, bodies_a, box_a = placed[i]
        for j in range(i + 1, len(placed)):
            link_b, leaf_b, bodies_b, box_b = placed[j]
            if (link_a is link_b) != same_link:
                continue
            if not _boxes_touch(box_a, box_b):
                continue
            if frozenset((getattr(leaf_a, 'part', leaf_a.name),
                          getattr(leaf_b, 'part', leaf_b.name))) in excluded:
                continue
            volume = 0.0
            for manifold_a, bounds_a in bodies_a:
                for manifold_b, bounds_b in bodies_b:
                    if _boxes_touch(bounds_a, bounds_b):
                        volume += (manifold_a ^ manifold_b).volume()
            if volume > CONTACT_VOLUME:
                found.append((leaf_a, leaf_b, volume))
    return found


def path_of(leaf, root):
    """The dotted tree path of a leaf below ``root``."""
    def walk(node, trail):
        if node is leaf:
            return trail
        for child in node.children:
            hit = walk(child, trail + [child.name])
            if hit is not None:
                return hit
        return None
    return '.'.join(walk(root, []) or [leaf.name])
