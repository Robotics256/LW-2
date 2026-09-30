#!/usr/bin/env python3
"""Generate the teaching warehouse, map and plan from one metric layout (stdlib only)."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo'
DOCS = ROOT / 'docs/warehouse'
RESOLUTION = 0.05
ORIGIN = (-6.5, -5.5)
WIDTH, HEIGHT = 260, 220
SMALL_ORIGIN = (-3.5, -5.5)
SMALL_WIDTH, SMALL_HEIGHT = 140, 220
RACK_SPAN = 4.0
RACK_WIDTH = .44


def add_rack_row(boxes, zone, row, centre_x, y, count, depth, colour,
                 spaced=True):
    """Add spaced, individually collidable rack bays across one storage row.

    The row keeps its 4 m footprint. Gaps are deliberately physical (not only
    visual), so a laser scan can distinguish neighbouring rack ends.
    """
    rack_width = RACK_WIDTH if spaced else RACK_SPAN / count
    gap = (RACK_SPAN - count * rack_width) / (count - 1)
    first_x = centre_x - RACK_SPAN / 2 + rack_width / 2
    for bay in range(1, count + 1):
        x = first_x + (bay - 1) * (rack_width + gap)
        boxes.append((f"rack_{zone}{row}_{bay}", x, y, rack_width, depth,
                      .9, colour))


def layout():
    # (name, centre x/y, length x/y, height, colour)
    boxes = [
        ('wall_north', 0, 5.1, 12.4, .2, 1.2, '#667687'),
        ('wall_south', 0, -5.1, 12.4, .2, 1.2, '#667687'),
        ('wall_west', -6.1, 0, .2, 10, 1.2, '#667687'),
        ('wall_east', 6.1, 0, .2, 10, 1.2, '#667687'),
        ('partition_west', -5.4, -3.25, 1.2, .2, 1.0, '#667687'),
        ('partition_middle', -.35, -3.25, 5.3, .2, 1.0, '#667687'),
        ('partition_east', 5.05, -3.25, 1.9, .2, 1.0, '#667687'),
        # User's I-shaped enclosure: side access to A/B, no central through route.
        ('storage_north', 0, 2.1, 8.2, .2, 1.0, '#667687'),
        ('storage_south', 0, -1.7, 8.2, .2, 1.0, '#667687'),
        ('storage_divider', 0, .2, .2, 3.6, 1.0, '#667687'),
        # Three northern work cells, with broad entrances and a cross aisle below.
        ('work_divider_w', -2.2, 4.25, .2, 1.5, 1.0, '#667687'),
        ('work_divider_e', 2.2, 4.25, .2, 1.5, 1.0, '#667687'),
        ('work_wing_w', -2.2, 3.4, 1.2, .2, 1.0, '#667687'),
        ('work_wing_e', 2.2, 3.4, 1.2, .2, 1.0, '#667687'),
        ('work_stub_w', -5.75, 3.4, .5, .2, 1.0, '#667687'),
        ('work_stub_e', 5.75, 3.4, .5, .2, 1.0, '#667687'),
        ('shipping_divider', 1.8, -4.175, .2, 1.65, 1.0, '#667687'),
        ('receiving', -4.3, 4.5, 1.8, .8, .65, '#e6a23c'),
        ('conveyor', 0, 4.5, 2.0, .8, .65, '#dc8050'),
        ('machine', 4.3, 4.5, 1.8, .8, .85, '#bc647b'),
        ('shipping', 5, -4.3, 1.2, 1.0, .65, '#b28b47'),
    ]
    for zone, x, colour in [('A', -2.1, '#488cc0'), ('B', 2.1, '#419d93')]:
        # Outer rows have lidar-visible gaps; the middle double row remains flush.
        for row, y in enumerate([-1.35, .2, 1.75], 1):
            add_rack_row(boxes, zone, row, x, y, 8, 1.0 if row == 2 else .5, colour,
                         spaced=row != 2)
    points = {
        'parking': [-4.3, -4.5, 0],
        'park_1': [-5.3, -4.5, 0], 'park_2': [-4.3, -4.5, 0],
        'park_3': [-3.3, -4.5, 0], 'park_4': [-2.3, -4.5, 0],
        'park_5': [-1.3, -4.5, 0], 'park_6': [-.3, -4.5, 0],
        'park_7': [.7, -4.5, 0],
        'target1': [-4.3, 3.65, math.pi / 2],
        'target2': [0, 3.65, math.pi / 2],
        'target3': [4.3, 3.65, math.pi / 2],
        'target4': [3.7, -4.3, 0],
        'gate_w': [-3.9, -3.25, math.pi / 2],
        'gate_e': [3.2, -3.25, math.pi / 2],
        'south_w': [-4.9, -2.5, 0], 'south_c': [0, -2.5, 0],
        'south_e': [4.9, -2.5, 0],
        'north_w': [-4.9, 2.75, 0], 'north_c': [0, 2.75, 0],
        'north_e': [4.9, 2.75, 0],
        'way_a': [-4.9, 1.1, math.pi / 2],
        'way_e': [4.9, 1.1, math.pi / 2],
        'pick_a1': [-2.1, -.7, math.pi / 2], 'pick_a2': [-2.1, 1.1, -math.pi / 2],
        'pick_b1': [2.1, -.7, math.pi / 2], 'pick_b2': [2.1, 1.1, -math.pi / 2],
    }
    return boxes, points


def small_layout():
    """A 6 x 10 m warehouse: one storage zone and T1–T3."""
    boxes = [
        ('wall_north', 0, 5.1, 6.4, .2, 1.2, '#667687'),
        ('wall_south', 0, -5.1, 6.4, .2, 1.2, '#667687'),
        ('wall_west', -3.1, 0, .2, 10, 1.2, '#667687'),
        ('wall_east', 3.1, 0, .2, 10, 1.2, '#667687'),
        # Storage is accessed through the two 0.8 m side aisles.
        ('storage_north', 0, 2.1, 4.2, .2, 1.0, '#667687'),
        ('storage_south', 0, -1.7, 4.2, .2, 1.0, '#667687'),
        # Full wall between the T1 and T2 work cells.
        ('wall_t1_t2', 0, 4.15, .2, 1.7, 1.0, '#667687'),
        ('work_stub_w', -2.75, 3.4, .5, .2, 1.0, '#667687'),
        ('work_stub_e', 2.75, 3.4, .5, .2, 1.0, '#667687'),
        # Parking and the T3 shipping cell.
        ('partition_west', -2.65, -3.25, .7, .2, 1.0, '#667687'),  # Closes the 10 cm gap to wall_west.
        ('partition_middle', -.65, -3.25, 1.5, .2, 1.0, '#667687'),
        ('partition_east', 2.25, -3.25, 1.5, .2, 1.0, '#667687'),
        ('shipping_divider', -.25, -4.175, .2, 1.65, 1.0, '#667687'),
        ('receiving', -1.5, 4.5, 1.4, .8, .65, '#e6a23c'),
        ('conveyor', 1.35, 4.5, 1.4, .8, .65, '#dc8050'),
        ('shipping', 1.9, -4.3, 1.1, 1.0, .65, '#b28b47'),
    ]
    # Rows nearest the parking area are A1/A2; the farther rows are B1/B2.
    # Each retains seven racks with lidar-visible gaps and the same map bounds.
    # The two middle rows meet edge-to-edge; clearance stays at the
    # outer aisles next to A1 and B2.
    for zone, row, y in [('A', 1, -1.35), ('A', 2, -.10),
                         ('B', 1, .40), ('B', 2, 1.65)]:
        add_rack_row(boxes, zone, row, 0, y, 7, .5, '#488cc0')
    points = {
        'parking': [-1.75, -4.5, 0],
        'park_1': [-2.35, -4.5, 0], 'park_2': [-1.75, -4.5, 0],
        'park_3': [-1.15, -4.5, 0],
        'target1': [-1.5, 3.65, math.pi / 2],
        'target2': [1.35, 3.65, math.pi / 2],
        'target3': [.55, -4.3, 0],
        'gate_w': [-1.75, -3.25, math.pi / 2],
        'gate_e': [1.1, -3.25, math.pi / 2],
        'south_w': [-2.5, -2.5, 0], 'south_e': [2.5, -2.5, 0],
        'north_w': [-2.5, 2.75, 0], 'north_e': [2.5, 2.75, 0],
        'way_a': [-2.5, 1.1, math.pi / 2],
        'way_e': [2.5, 1.1, math.pi / 2],
        'pick_a1': [-2.5, -.85, math.pi / 2],
        'pick_a2': [-2.5, .15, -math.pi / 2],
    }
    return boxes, points


def rack_sides(name, depth, colour):
    """Square bays back-to-back; distinguish the inventory on the two faces."""
    if depth == 1.0:
        return [('north', .25, colour, '#bad2de'),
                ('south', -.25, '#b97950' if name.startswith('rack_A2_') else '#b69b43', '#e6cf9d')]
    return [('single', 0, colour, '#c6d5df')]


def child(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def geometry(parent, size):
    child(child(child(parent, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))


def visual(link, name, x, y, z, size, colour):
    node = child(link, 'visual', name=name)
    child(node, 'pose', f'{x} {y} {z} 0 0 0')
    geometry(node, size)
    rgb = ' '.join(str(int(colour[i:i+2], 16) / 255) for i in (1, 3, 5)) + ' 1'
    material = child(node, 'material')
    child(material, 'ambient', rgb)
    child(material, 'diffuse', rgb)
    child(node, 'cast_shadows', 'false')


def make_world(boxes, points, filename, world_name, floor_size):
    # Gazebo 11 from Ubuntu 22.04 supports SDF 1.6 and gazebo_ros plugins.
    sdf = ET.Element('sdf', version='1.6')
    world = child(sdf, 'world', name=world_name)
    child(world, 'gravity', '0 0 -9.8')
    physics = child(world, 'physics', name='default_physics', type='ode')
    child(physics, 'max_step_size', .001)
    child(physics, 'real_time_factor', 1)
    child(physics, 'real_time_update_rate', 1000)
    # Provides /gazebo/get_entity_state for saving the exact simulated pose.
    state = child(world, 'plugin', name='gazebo_ros_state',
                  filename='libgazebo_ros_state.so')
    ros = child(state, 'ros')
    child(ros, 'namespace', '/gazebo')
    child(state, 'update_rate', '30.0')
    scene = child(world, 'scene')
    child(scene, 'ambient', '.7 .7 .7 1')
    child(scene, 'background', '.88 .91 .94 1')
    child(scene, 'shadows', 'false')
    light = child(world, 'light', name='sun', type='directional')
    child(light, 'pose', '0 0 10 0 0 0')
    child(light, 'diffuse', '.8 .8 .8 1')
    child(light, 'specular', '.1 .1 .1 1')
    child(light, 'direction', '-.3 .2 -1')
    child(light, 'cast_shadows', 'false')
    floor = child(world, 'model', name='floor')
    child(floor, 'static', 'true')
    link = child(floor, 'link', name='floor_link')
    collision = child(link, 'collision', name='floor_collision')
    child(collision, 'pose', '0 0 -.05 0 0 0')
    geometry(collision, (*floor_size, .1))
    visual(link, 'floor_visual', 0, 0, -.05, (*floor_size, .1), '#e3e6e5')
    for name, x, y, sx, sy, height, colour in boxes:
        model = child(world, 'model', name=name)
        child(model, 'static', 'true')
        child(model, 'pose', f'{x} {y} 0 0 0 0')
        link = child(model, 'link', name='body')
        collision = child(link, 'collision', name='collision')
        child(collision, 'pose', f'0 0 {height/2} 0 0 0')
        geometry(collision, (sx, sy, height))
        if name.startswith('rack_'):
            # Each model is one physically separate rack bay.
            for side, dy, side_colour, cap_colour in rack_sides(name, sy, colour):
                visual(link, f'bay_{side}', 0, dy, height/2,
                       (sx, .49, height), side_colour)
                visual(link, f'cap_{side}', 0, dy, height+.01,
                       (sx-.03, .46, .02), cap_colour)
        else:
            visual(link, 'visual', 0, 0, height/2, (sx, sy, height), colour)
        if name in ('receiving', 'conveyor', 'machine', 'shipping'):
            visual(link, 'work_surface', 0, 0, height+.01,
                   (sx-.12, sy-.12, .02), '#3d4b55')
    markings = child(world, 'model', name='floor_markings')
    child(markings, 'static', 'true')
    link = child(markings, 'link', name='paint')
    for name, (x, y, _) in points.items():
        if name.startswith('park_') or name.startswith('target'):
            colour = '#78b998' if name.startswith('park_') else '#edc96f'
            visual(link, name, x, y, .001, (.7, .7, .002), colour)
    # Floor paint is visual-only: no lidar obstacles or collision lips.
    lane_x = [-4.95, 4.95] if floor_size[0] > 10 else [-2.5, 2.5]
    for index, x in enumerate(lane_x):
        visual(link, f'outer_lane_{index}', x, .2, .001, (.025, 4.2, .002), '#f6cc52')
    ET.indent(sdf)
    ET.ElementTree(sdf).write(PACKAGE / 'worlds' / filename,
                              encoding='utf-8', xml_declaration=True)


def occupied(x, y, boxes):
    return any(abs(x-b[1]) <= b[3]/2 + 1e-9 and abs(y-b[2]) <= b[4]/2 + 1e-9
               for b in boxes)


def make_map(boxes, stem, origin, width, height, free_bounds):
    directory = PACKAGE / 'maps'
    directory.mkdir(exist_ok=True)
    pixels = bytearray()
    xmin, xmax, ymin, ymax = free_bounds
    for row in range(height):
        y = origin[1] + (height-row-.5) * RESOLUTION
        for col in range(width):
            x = origin[0] + (col+.5) * RESOLUTION
            pixels.append(0 if occupied(x, y, boxes) else
                          254 if xmin < x < xmax and ymin < y < ymax else 205)
    (directory / f'{stem}.pgm').write_bytes(
        f'P5\n# Synthetic warehouse ground truth; generated, not SLAM\n{width} {height}\n255\n'.encode() + pixels)
    (directory / f'{stem}.yaml').write_text(
        f'image: {stem}.pgm\nmode: trinary\nresolution: 0.05\n'
        f'origin: [{origin[0]}, {origin[1]}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def make_plan(boxes, points):
    def px(x): return 80 + (x+6.2)*70
    def py(y): return 125 + (5.2-y)*70
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1030" height="1080" viewBox="0 0 1030 1080">',
             '<rect width="1030" height="1080" fill="#f7f9fc"/>',
             '<g font-family="DejaVu Sans, sans-serif" fill="#24384b">',
             '<text x="80" y="48" font-size="28" font-weight="bold">Учебный склад · TurtleBot3</text>',
             '<text x="80" y="80" font-size="16">12 × 10 м  /  2 зоны хранения  /  4 рабочих пункта  /  7 мест стоянки</text>',
             f'<rect x="{px(-6)}" y="{py(5)}" width="840" height="700" fill="#e9eeed"/>']
    for name, x, y, sx, sy, height, colour in boxes:
        parts.append(f'<rect x="{px(x-sx/2):.1f}" y="{py(y+sy/2):.1f}" '
                     f'width="{sx*70:.1f}" height="{sy*70:.1f}" fill="{colour}"/>')
        if name.startswith('rack_'):
            bay_count = round(sx / .5)
            for side, dy, side_colour, _ in rack_sides(name, sy, colour):
                parts.append(f'<rect x="{px(x-sx/2)}" y="{py(y+dy+.25)}" '
                             f'width="{sx*70}" height="35" fill="{side_colour}"/>')
                for bay in range(1, bay_count):
                    xx = px(x-sx/2+bay*.5)
                    parts.append(f'<path d="M {xx} {py(y+dy+.25)} v 35" stroke="#e9f0f6" stroke-width="2"/>')
                label = name[5:] + (' · N' if side == 'north' else ' · S' if side == 'south' else '')
                parts.append(f'<text x="{px(x)}" y="{py(y+dy)+5}" text-anchor="middle" fill="white" font-size="14">{label}</text>')
    for name, (x, y, _) in points.items():
        if name == 'parking':
            continue
        if name.startswith('park_'):
            parts.append(f'<rect x="{px(x-.35)}" y="{py(y+.35)}" width="49" height="49" fill="#a8d2bd" stroke="#4a8d70"/>')
            label = 'P'+name[-1]
        elif name.startswith('target'):
            label = 'T'+name[-1]
        else:
            label = name
        parts.append(f'<circle cx="{px(x)}" cy="{py(y)}" r="4" fill="#24384b"/>')
        parts.append(f'<text x="{px(x)+7}" y="{py(y)-8}" font-size="11">{label}</text>')
    for x, label in [(-4.3, 'T1 · Приёмка'), (0, 'T2 · Конвейер'), (4.3, 'T3 · Станок')]:
        parts.append(f'<text x="{px(x)}" y="{py(4.5)+5}" fill="white" text-anchor="middle" font-size="12">{label}</text>')
    parts.append(f'<text x="{px(5)}" y="{py(-4.3)+5}" fill="white" text-anchor="middle" font-size="11">T4 · Отгрузка</text>')
    for y, label in [(1.25, '0,80 м'), (-.55, '0,80 м')]:
        for x in [-1.4, 3.4]:
            parts.append(f'<text x="{px(x)}" y="{py(y)}" font-size="11" fill="#587080">{label}</text>')
    for x, y, label in [(-2.2, 2.7, '1,10 м'), (-5, 1.8, '2,10 м'),
                        (5, 1.8, '2,10 м'), (0, -2.85, 'Южный проезд · 1,35 м')]:
        parts.append(f'<text x="{px(x)}" y="{py(y)}" text-anchor="middle" font-size="12">{label}</text>')
    parts.extend([
        '<text x="80" y="902" font-size="18" font-weight="bold">План движения</text>',
        '<text x="80" y="931" font-size="15">Внешние ряды разделены физическими промежутками для лидара; центральные A2/B2 остаются вплотную.</text>',
        '<text x="80" y="958" font-size="15">У стеллажей — один робот по резервированию; в основных проездах — встречное движение.</text>',
        '<text x="80" y="985" font-size="15">Обход склада — с запада и востока; центральная стенка разделяет зоны A и B.</text>',
        '<text x="80" y="1021" font-size="13" fill="#587080">Координаты в метрах: начало в центре, +X вправо, +Y вверх. Точки — позиции робота.</text>',
        '<text x="80" y="1045" font-size="13" fill="#587080">Цветные площадки — разметка пола. Стеллажи, оборудование и стены — физические препятствия.</text>',
        '</g></svg>'])
    (DOCS / 'warehouse_plan.svg').write_text('\n'.join(parts), encoding='utf-8')


def make_small_plan(boxes, points):
    """Draw the compact warehouse with storage labels, not a route diagram."""
    def px(x): return 95 + (x + 3.2) * 125
    def py(y): return 100 + (5.2 - y) * 64

    rack_prefixes = {
        'rack_A1_': 'А1_', 'rack_A2_': 'А2_',
        'rack_B1_': 'В1_', 'rack_B2_': 'В2_',
    }
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="980" height="800" viewBox="0 0 980 800">',
        '<rect width="980" height="800" fill="#f7f9fc"/>',
        '<g font-family="DejaVu Sans, sans-serif" fill="#24384b">',
        '<text x="95" y="42" font-size="27" font-weight="bold">Малый учебный склад · TurtleBot3</text>',
        '<text x="95" y="70" font-size="15">6 × 10 м  /  4 ряда по 7 стеллажей  /  T1–T3 и стоянка</text>',
        f'<rect x="{px(-3)}" y="{py(5)}" width="750" height="640" fill="#e9eeed"/>',
    ]
    for name, x, y, sx, sy, height, colour in boxes:
        parts.append(f'<rect x="{px(x-sx/2):.1f}" y="{py(y+sy/2):.1f}" '
                     f'width="{sx*125:.1f}" height="{sy*64:.1f}" fill="{colour}"/>')
        rack_prefix = next((prefix for key, prefix in rack_prefixes.items()
                            if name.startswith(key)), None)
        if rack_prefix is not None:
            bay = name.rsplit('_', 1)[1]
            parts.append(f'<text x="{px(x)}" y="{py(y)+4:.1f}" text-anchor="middle" '
                         f'fill="white" font-size="11" font-weight="bold">{rack_prefix}{bay}</text>')
    for name, (x, y, _) in points.items():
        if name.startswith('park_'):
            square = 42
            parts.append(f'<rect x="{px(x)-square/2:.1f}" y="{py(y)-square/2:.1f}" '
                         f'width="{square}" height="{square}" fill="#a8d2bd" stroke="#4a8d70"/>')
            parts.append(f'<text x="{px(x):.1f}" y="{py(y)+4:.1f}" text-anchor="middle" '
                         f'font-size="12" font-weight="bold">C{name[-1]}</text>')
            continue
        if name not in ('target1', 'target2', 'target3'):
            continue
        label = 'T' + name[-1]
        parts.append(f'<circle cx="{px(x)}" cy="{py(y)}" r="4" fill="#24384b"/>')
        parts.append(f'<text x="{px(x)+7}" y="{py(y)-8}" font-size="11">{label}</text>')
    for x, y, label in [(-1.5, 4.5, 'T1 · Приёмка'),
                        (1.35, 4.5, 'T2 · Конвейер'),
                        (1.9, -4.3, 'T3 · Отгрузка')]:
        parts.append(f'<text x="{px(x)}" y="{py(y)+5}" fill="white" '
                     f'text-anchor="middle" font-size="12">{label}</text>')
    parts.extend([
        '<text x="95" y="762" font-size="13" fill="#587080">А1 и А2 — ряды ближе к стоянке; В1 и В2 — дальние ряды.</text>',
        '</g></svg>',
    ])
    (DOCS / 'small_warehouse_plan.svg').write_text('\n'.join(parts), encoding='utf-8')

def make_navigation_profiles():
    upstream = ROOT / 'ros2_ws/src/turtlebot3/turtlebot3_navigation2/param/humble'
    output = PACKAGE / 'params'
    output.mkdir(exist_ok=True)
    for model, original, radius in [('burger', '0.1', '0.15'),
                                    ('waffle', '0.22', '0.25'),
                                    ('waffle_pi', '0.22', '0.25')]:
        source = (upstream / f'{model}.yaml').read_text()
        amcl_update_thresholds = (
            '    update_min_a: 0.2\n'
            '    update_min_d: 0.25\n')
        if source.count(amcl_update_thresholds) != 1:
            raise ValueError(f'Review upstream AMCL update thresholds for {model}')
        source = source.replace(
            amcl_update_thresholds,
            '    # Refresh map -> odom while the robot waits for its next goal.\n'
            # Without scan updates after a long stop, Nav2 can reject the stale TF
            # before it is able to issue the command that would make AMCL update.
            '    update_min_a: 0.0\n'
            '    update_min_d: 0.0\n')
        old = f'robot_radius: {original}\n'
        if source.count(old) != 2:
            raise ValueError(f'Review upstream local/global costmap radii for {model}')
        result = source.replace(old, f'robot_radius: {radius}\n')
        for old_inflation in ('inflation_radius: 1.0', 'inflation_radius: 0.55'):
            if result.count(old_inflation) != 1:
                raise ValueError(
                    f'Review upstream inflation radius {old_inflation} for {model}')
            result = result.replace(old_inflation, 'inflation_radius: 0.35')

        old_goal_checkers = 'goal_checker_plugins: ["general_goal_checker"]'
        if result.count(old_goal_checkers) != 1:
            raise ValueError(f'Review upstream goal checkers for {model}')
        result = result.replace(
            old_goal_checkers,
            'goal_checker_plugins: ["general_goal_checker", "position_goal_checker"]')
        old_general_checker = (
            '    general_goal_checker:\n'
            '      stateful: True\n'
            '      plugin: "nav2_controller::SimpleGoalChecker"\n'
            '      xy_goal_tolerance: 0.25\n'
            '      yaw_goal_tolerance: 0.25\n')
        if result.count(old_general_checker) != 1:
            raise ValueError(f'Review upstream general goal checker for {model}')
        result = result.replace(
            old_general_checker,
            old_general_checker +
            '\n'
            '    position_goal_checker:\n'
            '      stateful: True\n'
            '      plugin: "nav2_controller::PositionGoalChecker"\n'
            '      xy_goal_tolerance: 0.25\n')
        header = ('# Generated by SCRIPTS/generate_warehouse.py from TurtleBot3 Humble parameters.\n'
                  '# Warehouse footprint, goal-checker, and obstacle-clearance profile.\n')
        (output / f'warehouse_{model}.yaml').write_text(header + result)


def main():
    DOCS.mkdir(parents=True, exist_ok=True)
    boxes, points = layout()
    small_boxes, small_points = small_layout()
    make_world(boxes, points, 'turtlebot3_warehouse.world', 'warehouse', (12.4, 10.4))
    make_map(boxes, 'warehouse', ORIGIN, WIDTH, HEIGHT, (-6, 6, -5, 5))
    make_plan(boxes, points)
    make_world(small_boxes, small_points, 'turtlebot3_warehouse_small.world',
               'warehouse_small', (6.4, 10.4))
    make_map(small_boxes, 'warehouse_small', SMALL_ORIGIN, SMALL_WIDTH, SMALL_HEIGHT,
             (-3, 3, -5, 5))
    make_small_plan(small_boxes, small_points)
    make_navigation_profiles()
    (DOCS / 'layout.json').write_text(json.dumps(
        {'frame': 'world', 'units': 'metres', 'boxes': boxes, 'points': points}, indent=2) + '\n')
    (DOCS / 'points_ground_truth.txt').write_text(''.join(
        f'{name}, {x:.3f}, {y:.3f}, {math.sin(yaw/2):.6f}, '
        f'{math.cos(yaw/2):.6f}, {x:.3f}, {y:.3f}\n'
        for name, (x, y, yaw) in points.items()))
    (DOCS / 'small_layout.json').write_text(json.dumps(
        {'frame': 'world', 'units': 'metres', 'boxes': small_boxes,
         'points': small_points}, indent=2) + '\n')
    (DOCS / 'small_points_ground_truth.txt').write_text(''.join(
        f'{name}, {x:.3f}, {y:.3f}, {math.sin(yaw/2):.6f}, '
        f'{math.cos(yaw/2):.6f}, {x:.3f}, {y:.3f}\n'
        for name, (x, y, yaw) in small_points.items()))
    print(f'Generated large warehouse: {len(boxes)} obstacle boxes; map {WIDTH} x {HEIGHT}.')
    print(f'Generated small warehouse: {len(small_boxes)} obstacle boxes; '
          f'map {SMALL_WIDTH} x {SMALL_HEIGHT}.')


if __name__ == '__main__':
    main()
