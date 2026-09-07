"""The leaves: one node per catalogue part, its geometry from a STEP file."""

import os

import cadquery as cq
from solid_node.node import CadQueryNode
from solid_node.node.adapters.step import solids_from_faces

from simulation.don1 import catalogue, materials

#: How far apart two edges of a face-only STEP may be and still be sewn.
SEWING_TOLERANCE = 0.05

#: Vendor STEP files are all fillets and threads, and at the framework's
#: 0.1 rad angular deflection the robot's meshes came to 200 MB. Every
#: part declares this instead (ADR-077): the export meshes once, at the
#: declared precision, and the stored-triangulation workaround this module
#: used to carry is gone.
ANGULAR_DEFLECTION = 0.5


def import_step(path, part_id):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f'{part_id}: {path} is missing; run '
            f'`python -m simulation.don1.catalogue fetch` to download the '
            f'vendor parts')
    return cq.importers.importStep(path)


class StepPart(CadQueryNode):
    """One catalogue part, as the index or the blueprints draw it.

    Identity is the part id; two placements of one part share an artifact.
    The STEP file joins the tracked sources, so replacing it rebuilds.
    """

    angular_deflection = ANGULAR_DEFLECTION

    def __init__(self, part, name=None):
        self.part = part
        self.path = catalogue.step_file(part)
        super().__init__(part=part, name=name)
        self.color = materials.colour(part)
        self.files = self.files | {self.path}

    def render(self):
        imported = import_step(self.path, self.part)
        shape = imported.val()
        if not shape.Solids():
            # The hook and the battery come as triangulated surfaces; the
            # framework's helper sews each closed shell into a solid, and
            # this project vouches for the tolerance.
            shape = solids_from_faces(shape, SEWING_TOLERANCE)
        return cq.Workplane(obj=shape)


class SideChannel(CadQueryNode):
    """The hip's side channel: ``don1-side-channel.py`` without PartCAD.

    The blueprint's script takes the 9-hole U-channel from the index and
    cuts a 9 mm-radius hole through the end face and a 35 mm-radius hole
    48 mm in from the other end; this does the same over the channel STEP.
    """

    PART = catalogue.LOCAL_PACKAGE + ':don1-side-channel'
    CHANNEL = '//pub/robotics/parts/gobilda:structure/u_channel_9'
    color = materials.ALUMINIUM
    angular_deflection = ANGULAR_DEFLECTION

    def __init__(self, name=None):
        self.path = catalogue.step_file(self.CHANNEL)
        super().__init__(name=name)
        self.files = self.files | {self.path}

    def render(self):
        channel = import_step(self.path, self.CHANNEL)
        next_face = channel.faces('>X[1]').val()
        punctured = (channel.faces('>X')
                     .workplane(centerOption='CenterOfMass')
                     .circle(9.0)
                     .cutBlind(next_face))
        next_face = punctured.faces('<X[1]').val()
        punctured = (punctured.faces('<X')
                     .workplane(centerOption='CenterOfMass')
                     .move(0.0, -48.0)
                     .circle(35.0)
                     .cutBlind(next_face))
        return cq.Workplane(obj=punctured.val())


def part_node(part_id, name=None):
    """The node for a blueprint part id: a STEP import or a scripted part."""
    if part_id == SideChannel.PART:
        return SideChannel(name=name)
    return StepPart(part=part_id, name=name)
