"""Evaluation-only footprint/trunk clearance; never an autonomy input."""
import ast
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


class TreeClearance:
    def __init__(self, trees, footprint, robot_radius=None):
        self.trees = trees
        self.footprint = footprint
        self.robot_radius = robot_radius

    @classmethod
    def from_files(cls, world, nav_config):
        world = Path(world)
        tree = ET.parse(world.parent.parent / 'models/ember_tree/model.sdf')
        radius = float(tree.findtext('.//collision/geometry/cylinder/radius'))
        trees = []
        for item in ET.parse(world).getroot().find('world').findall('include'):
            if item.findtext('uri') == 'model://ember_tree':
                x, y, *_ = map(float, item.findtext('pose').split())
                trees.append((item.findtext('name'), x, y, radius))
        config = yaml.safe_load(Path(nav_config).read_text())
        local = config['local_costmap']['local_costmap']['ros__parameters']
        points = local.get('footprint', [])
        points = ast.literal_eval(points) if isinstance(points, str) else points
        return cls(trees, points, None if points else float(local['robot_radius']))

    def closest(self, x, y, yaw):
        closest = (None, None)
        c, s = math.cos(yaw), math.sin(yaw)
        for name, tx, ty, radius in self.trees:
            dx, dy = tx-x, ty-y
            if self.robot_radius is not None:
                clearance = math.hypot(dx, dy)-self.robot_radius-radius
                if closest[0] is None or clearance < closest[0]:
                    closest = (clearance, name)
                continue
            px, py = c*dx+s*dy, -s*dx+c*dy
            distance = math.inf
            inside = False
            for (ax, ay), (bx, by) in zip(self.footprint, self.footprint[1:]+self.footprint[:1]):
                ex, ey = bx-ax, by-ay
                fraction = max(0., min(1., ((px-ax)*ex+(py-ay)*ey)/(ex*ex+ey*ey)))
                distance = min(distance, math.hypot(px-ax-fraction*ex, py-ay-fraction*ey))
                if (ay > py) != (by > py) and px < ax+(py-ay)*ex/ey:
                    inside = not inside
            clearance = (-distance if inside else distance)-radius
            if closest[0] is None or clearance < closest[0]:
                closest = (clearance, name)
        return closest
