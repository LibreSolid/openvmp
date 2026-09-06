"""Where each part the blueprints name is found on disk.

A PartCAD part id is ``<package>:<path>``.  The OpenVMP custom parts are
STEP files in ``openvmp-models/parts``; every other part comes from a
vendor package of the public PartCAD index, and those STEP files are not
committed here: ``python -m simulation.don1.catalogue fetch`` downloads
the ones Don1 uses, at the pinned commits below, into ``vendor/`` beside
this module (ignored by Git).
"""

import os
import sys
import urllib.request

from simulation.don1.blueprints import PARTS_DIR

VENDOR_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'vendor')

#: The index packages Don1 draws from, pinned to the commit this layer was
#: built against: package id -> (GitHub repository, commit).
VENDORS = {
    '//pub/robotics/parts/gobilda': (
        'partcad/partcad-robotics-part-vendor-gobilda',
        'daa42b308be6f7f4cadd450c226a38e58f6487d9'),
    '//pub/robotics/parts/dfrobot': (
        'partcad/partcad-robotics-part-vendor-dfrobot',
        'a44bcb2ea1b165f4b83f6426f7860eb49a992345'),
    '//pub/electromechanics/stepperonline': (
        'partcad/partcad-electromechanics-stepperonline',
        'c86aaa1aa4d87556135e7e3ba28d7030bc3ca9b8'),
    '//pub/electromechanics/cloudray': (
        'partcad/partcad-electromechanics-cloudray',
        '9d4451c05ddd6cd33f7d38b15bb09690b88852f8'),
    '//pub/electronics/sbcs/intel': (
        'partcad/partcad-electronics-sbcs-intel',
        '79116020b6353c1e133394b8c2b6b8b71598a655'),
    '//pub/electronics/sbcs/raspberrypi': (
        'partcad/partcad-electronics-sbcs-raspberrypi',
        '14fac981d6fedceabc7c3f2ce07ed0997d77eb38'),
    '//pub/electronics/sbcs/arduino': (
        'partcad/partcad-electronics-sbcs-arduino',
        '75d35e8256bc75f2bee17edf65ff28b51485327b'),
    '//pub/electrical/battery/ego': (
        'partcad/partcad-electrical-ego',
        'af20a6fae0bbaf4b591ec5b7681f2d273150f7d8'),
}

#: The OpenVMP package whose parts are in the blueprints repository.
LOCAL_PACKAGE = '//pub/robotics/multimodal/openvmp/parts'

#: Blueprint parts that are CadQuery scripts rather than STEP files; each
#: has a node of its own in ``parts.py`` and no file to fetch.
SCRIPTED = {LOCAL_PACKAGE + ':don1-side-channel'}


def split(part_id):
    package, path = part_id.split(':')
    return package, path


def vendor_name(package):
    return package.rsplit('/', 1)[1]


def step_file(part_id):
    """The STEP file for a part id, wherever it lives."""
    package, path = split(part_id)
    if part_id in SCRIPTED:
        raise KeyError(f'{part_id} is a script, not a STEP file')
    if package == LOCAL_PACKAGE:
        return os.path.join(PARTS_DIR, path + '.step')
    if package not in VENDORS:
        raise KeyError(f'{part_id}: no known source for package {package}')
    return os.path.join(VENDOR_DIR, vendor_name(package), path + '.step')


def raw_url(part_id):
    package, path = split(part_id)
    repository, commit = VENDORS[package]
    return f'https://raw.githubusercontent.com/{repository}/{commit}/{path}.step'


def missing(part_ids):
    return [pid for pid in part_ids
            if pid not in SCRIPTED and not os.path.exists(step_file(pid))]


def fetch(part_ids, report=print):
    """Download every vendor part of ``part_ids`` that is not on disk."""
    for pid in part_ids:
        package, _ = split(pid)
        if package == LOCAL_PACKAGE or pid in SCRIPTED:
            continue
        target = step_file(pid)
        if os.path.exists(target):
            continue
        os.makedirs(os.path.dirname(target), exist_ok=True)
        report(f'fetching {pid}')
        with urllib.request.urlopen(raw_url(pid)) as response, \
                open(target + '.part', 'wb') as out:
            out.write(response.read())
        os.replace(target + '.part', target)


def don1_part_ids():
    """Every part id the Don1 blueprints reference, through every link."""
    from simulation.don1.blueprints import load
    ids = set()

    def walk(name, params):
        for placement in load(name, **params):
            if placement.part:
                ids.add(placement.part)
            else:
                walk(placement.assembly, placement.params)
    walk('robot', {})
    return sorted(ids)


def main(argv):
    if argv[1:] == ['fetch']:
        fetch(don1_part_ids())
        return 0
    if argv[1:] == ['check']:
        absent = missing(don1_part_ids())
        for pid in absent:
            print('missing', pid, step_file(pid))
        return 1 if absent else 0
    print('usage: python -m simulation.don1.catalogue fetch|check', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
