#!/usr/bin/env python3
"""Check the delivered SDF/map and clearance, without ROS or external Python packages."""

from collections import deque
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo'
STEP = .05


class WarehouseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ET.parse(PACKAGE / 'worlds/turtlebot3_warehouse.world').getroot()
        cls.boxes = []
        for model in cls.world.findall('world/model'):
            if model.get('name') == 'floor':
                continue
            for collision in model.findall('link/collision'):
                pose = list(map(float, model.findtext('pose').split()))
                sx, sy, _ = map(float, collision.findtext('geometry/box/size').split())
                cls.boxes.append((pose[0], pose[1], sx, sy))
        cls.points = json.loads((ROOT / 'docs/warehouse/layout.json').read_text())['points']

    @staticmethod
    def cell(point):
        return round((point[0]+6)/STEP), round((point[1]+5)/STEP)

    def reachable(self, radius, extra=()):
        obstacles = self.boxes + list(extra)
        free = set()
        for i in range(241):
            for j in range(201):
                x, y = -6+i*STEP, -5+j*STEP
                if all(math.hypot(max(abs(x-bx)-sx/2, 0),
                                  max(abs(y-by)-sy/2, 0)) > radius + 1e-8
                       for bx, by, sx, sy in obstacles):
                    free.add((i, j))
        start = self.cell(self.points['parking'])
        self.assertIn(start, free)
        seen, queue = {start}, deque([start])
        while queue:
            i, j = queue.popleft()
            for node in [(i+1, j), (i-1, j), (i, j+1), (i, j-1)]:
                if node in free and node not in seen:
                    seen.add(node)
                    queue.append(node)
        return free, seen

    def test_all_points_and_free_space_connected_with_waffle_margin(self):
        # 0.25 m rotation envelope plus 0.05 m clearance around base_footprint.
        free, seen = self.reachable(.30)
        self.assertEqual(free, seen, 'Unreachable free-space pocket')
        for name, point in self.points.items():
            self.assertIn(self.cell(point), seen, name)

    def test_gates_serve_separate_parking_and_shipping_rooms(self):
        gates = [(-3.9, -3.25, 1.8, .2), (3.2, -3.25, 1.8, .2)]
        _, seen = self.reachable(.30, [gates[0]])
        self.assertNotIn(self.cell(self.points['target1']), seen)
        self.assertNotIn(self.cell(self.points['target4']), seen)
        _, seen = self.reachable(.30, [gates[1]])
        self.assertIn(self.cell(self.points['target1']), seen)
        self.assertNotIn(self.cell(self.points['target4']), seen)

    def test_two_outer_routes_and_no_central_shortcut(self):
        # Cut at the northern cap: a cut halfway along a rack can legitimately
        # be bypassed through two picking aisles and the inner turnaround.
        cuts = [(-4.95, 2.1, 2.1, .2), (4.95, 2.1, 2.1, .2)]
        for cut in cuts:
            _, seen = self.reachable(.30, [cut])
            self.assertIn(self.cell(self.points['target2']), seen)
        _, seen = self.reachable(.30, cuts)
        self.assertFalse(self.cell(self.points['target2']) in seen,
                         'Northern storage cap must prevent a central shortcut')

    def test_map_matches_sdf(self):
        with (PACKAGE / 'maps/warehouse.pgm').open('rb') as source:
            self.assertEqual(source.readline().strip(), b'P5')
            line = source.readline()
            while line.startswith(b'#'):
                line = source.readline()
            width, height = map(int, line.split())
            self.assertEqual(source.readline().strip(), b'255')
            pixels = source.read()
        self.assertEqual((width, height, len(pixels)), (260, 220, 57200))
        for row in range(height):
            y = -5.5+(height-row-.5)*STEP
            for col in range(width):
                x = -6.5+(col+.5)*STEP
                blocked = any(abs(x-bx) <= sx/2+1e-9 and abs(y-by) <= sy/2+1e-9
                              for bx, by, sx, sy in self.boxes)
                expected = 0 if blocked else 254 if -6 < x < 6 and -5 < y < 5 else 205
                self.assertEqual(pixels[row*width+col], expected, (row, col))

    def test_lightweight_geometry(self):
        self.assertEqual(len(self.boxes), 27)
        self.assertFalse(self.world.findall('.//uri'), 'Warehouse must load without external assets')
        racks = [m for m in self.world.findall('world/model') if m.get('name').startswith('rack_')]
        self.assertEqual(len(racks), 6)
        for rack in racks:
            self.assertEqual(len(rack.findall('link/collision')), 1)
            bays = [v for v in rack.findall('link/visual') if v.get('name').startswith('bay_')]
            doubled = rack.get('name') in ('rack_A2', 'rack_B2')
            self.assertEqual(len(bays), 16 if doubled else 8)
            if doubled:
                self.assertEqual({float(v.findtext('pose').split()[1]) for v in bays}, {-.25, .25})
                self.assertEqual(len({v.findtext('material/diffuse') for v in bays}), 2)
            for bay in bays:
                size = list(map(float, bay.findtext('geometry/box/size').split()))
                self.assertEqual(size[:2], [.49, .49])
        for rack in racks:
            x, _, _, _, _, _ = map(float, rack.findtext('pose').split())
            sx, _, _ = map(float, rack.findtext('link/collision/geometry/box/size').split())
            self.assertAlmostEqual(abs(x) - sx/2, .1, places=6)
        self.assertFalse({'turn_a', 'turn_b'} & self.points.keys())
        self.assertEqual([self.points[f'park_{index}'][0] for index in range(1, 8)],
                         [-5.3, -4.3, -3.3, -2.3, -1.3, -.3, .7])
        paint = self.world.find("world/model[@name='floor_markings']")
        self.assertFalse(paint.findall('.//collision'))


class SmallWarehouseTest(unittest.TestCase):
    """The compact layout must retain Waffle-sized access to every teaching point."""

    @classmethod
    def setUpClass(cls):
        cls.world = ET.parse(PACKAGE / 'worlds/turtlebot3_warehouse_small.world').getroot()
        cls.boxes = []
        for model in cls.world.findall('world/model'):
            if model.get('name') == 'floor':
                continue
            for collision in model.findall('link/collision'):
                pose = list(map(float, model.findtext('pose').split()))
                sx, sy, _ = map(float, collision.findtext('geometry/box/size').split())
                cls.boxes.append((pose[0], pose[1], sx, sy))
        cls.points = json.loads((ROOT / 'docs/warehouse/small_layout.json').read_text())['points']

    @staticmethod
    def cell(point):
        return round((point[0]+3)/STEP), round((point[1]+5)/STEP)

    def reachable(self, radius):
        free = set()
        for i in range(121):
            for j in range(201):
                x, y = -3+i*STEP, -5+j*STEP
                if all(math.hypot(max(abs(x-bx)-sx/2, 0),
                                  max(abs(y-by)-sy/2, 0)) > radius + 1e-8
                       for bx, by, sx, sy in self.boxes):
                    free.add((i, j))
        start = self.cell(self.points['parking'])
        self.assertIn(start, free)
        seen, queue = {start}, deque([start])
        while queue:
            i, j = queue.popleft()
            for node in [(i+1, j), (i-1, j), (i, j+1), (i, j-1)]:
                if node in free and node not in seen:
                    seen.add(node)
                    queue.append(node)
        return free, seen

    def test_small_points_are_connected_for_waffle_margin(self):
        free, seen = self.reachable(.30)
        self.assertEqual(free, seen, 'Unreachable free-space pocket')
        for name, point in self.points.items():
            self.assertIn(self.cell(point), seen, name)

    def test_small_map_matches_sdf(self):
        with (PACKAGE / 'maps/warehouse_small.pgm').open('rb') as source:
            self.assertEqual(source.readline().strip(), b'P5')
            line = source.readline()
            while line.startswith(b'#'):
                line = source.readline()
            width, height = map(int, line.split())
            self.assertEqual(source.readline().strip(), b'255')
            pixels = source.read()
        self.assertEqual((width, height, len(pixels)), (140, 220, 30800))
        for row in range(height):
            y = -5.5+(height-row-.5)*STEP
            for col in range(width):
                x = -3.5+(col+.5)*STEP
                blocked = any(abs(x-bx) <= sx/2+1e-9 and abs(y-by) <= sy/2+1e-9
                              for bx, by, sx, sy in self.boxes)
                expected = 0 if blocked else 254 if -3 < x < 3 and -5 < y < 5 else 205
                self.assertEqual(pixels[row*width+col], expected, (row, col))

    def test_small_layout_contains_only_a_storage_and_t1_t2_t4(self):
        names = {model.get('name') for model in self.world.findall('world/model')}
        self.assertEqual({name for name in names if name.startswith('rack_')},
                         {'rack_A1', 'rack_A2', 'rack_A3'})
        self.assertFalse(any(name.startswith('rack_B') for name in names))
        self.assertNotIn('machine', names)
        self.assertNotIn('target3', self.points)
        self.assertEqual(len([key for key in self.points if key.startswith('park_')]), 4)
        self.assertAlmostEqual(self.points['target4'][0], .55)
        self.assertEqual(len(self.boxes), 19)


if __name__ == '__main__':
    unittest.main(verbosity=2)
