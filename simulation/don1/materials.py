"""Part colours, one per material a part is made of.

A node's colour is a property of the node, not of the geometry it renders,
so every part carries the colour of the stuff it is made from and nothing
else: the picture reads as structure, drive train, printed parts,
electronics and rubber, whichever link a part is on.  The table is keyed
by the PartCAD part id; ``colour()`` looks the most specific rule up.
"""

# goBILDA structure: anodised aluminium channels, beams, brackets,
# blocks, standoffs, servo frames and hubs
ALUMINIUM = '#c6c9cd'
HUB = '#7d8790'            # the machined hubs, couplers and collars
STAINLESS = '#e2e5e8'      # REX and D shafts, worms
STEEL = '#9a9fa5'          # bearings, spacers, nuts, plates, steel sprockets
BRASS = '#c9a23c'          # the worm gears
ACETAL = '#2b2b2b'         # plastic sprockets and spacers
RUBBER = '#1f1f1f'         # the wheels
MOTOR = '#3a3a3a'          # the steppers
GEARBOX = '#b0b4b8'        # the planetary gearboxes
SERVO = '#262626'          # the HS-488HB servos
DRIVER = '#4a5a6a'         # stepper drivers
BRUSHLESS_DRIVER = '#3b5a8a'
PCB = '#2e7d4f'            # the NUC and Raspberry Pi boards
ARDUINO = '#1e6f9f'
BATTERY = '#7bc043'        # EGO POWER+ lime
SHEET = '#8fa3b3'          # the custom stainless sheet parts
PRINTED = '#f2843b'        # the printed enclosure, mounts and pads
HOOK = '#e05a2b'           # the printed feet

_RULES = (
    ('openvmp/parts:hook', HOOK),
    ('openvmp/parts:enclosure', PRINTED),
    ('openvmp/parts:nema17', PRINTED),
    ('openvmp/parts:turntable-attachment', HUB),
    ('openvmp/parts:', SHEET),
    ('ego:', BATTERY),
    ('intel:', PCB),
    ('raspberrypi:', PCB),
    ('arduino:', ARDUINO),
    ('cloudray:', DRIVER),
    ('brushless_driver/', BRUSHLESS_DRIVER),
    ('stepper_driver/', DRIVER),
    ('stepper/', MOTOR),
    ('gearbox/', GEARBOX),
    ('dfrobot:', RUBBER),
    ('motion/worm_gear', BRASS),
    ('motion/worm', STAINLESS),
    ('motion/shaft', STAINLESS),
    ('motion/bearing', STEEL),
    ('motion/sprocket_steel', STEEL),
    ('motion/sprocket_plastic', ACETAL),
    ('motion/servo_hs', SERVO),
    ('motion/servo_attach', HUB),
    ('motion/hub', HUB),
    ('motion/coupler', HUB),
    ('motion/collar', HUB),
    ('hardware/spacer_plastic', ACETAL),
    ('hardware/', STEEL),
    ('structure/', ALUMINIUM),
)


def colour(part_id):
    for needle, value in _RULES:
        if needle in part_id:
            return value
    raise KeyError(f'no material for {part_id}')
