"""Construct healthy, burning and burnt versions of the same forest tree geometry."""

from copy import deepcopy
import xml.etree.ElementTree as ET

STATES = {"healthy": 0, "burning": 1, "burnt": 2}


def state_model(
    tree_xml, fire_xml, state, name="fire_source_visual", pose="12 7 0 0 0 0"
):
    if state not in STATES:
        raise ValueError("State must be healthy, burning or burnt")
    tree = deepcopy(ET.fromstring(tree_xml).find("model"))
    if tree is None:
        raise ValueError("Tree model is missing")
    tree.set("name", name)
    old_pose = tree.find("pose")
    if old_pose is not None:
        tree.remove(old_pose)
    ET.SubElement(tree, "pose").text = pose
    if state == "burning":
        flame = ET.fromstring(fire_xml).find("model")
        if flame is None:
            raise ValueError("Fire model is missing")
        for link in flame.findall("link"):
            tree.append(deepcopy(link))
    for visual in tree.findall(".//visual"):
        for plugin in list(visual.findall("plugin")):
            if plugin.get("name", "").endswith("::Thermal"):
                visual.remove(plugin)
        if state == "burnt":
            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material")
            for key in ("ambient", "diffuse"):
                element = material.find(key)
                if element is None:
                    element = ET.SubElement(material, key)
                element.text = "0.06 0.05 0.04 1"
        plugin = ET.SubElement(
            visual,
            "plugin",
            filename="ignition-gazebo-thermal-system",
            name="ignition::gazebo::systems::Thermal",
        )
        ET.SubElement(plugin, "temperature").text = {
            "healthy": "295.15",
            "burning": "573.15",
            "burnt": "318.15",
        }[state]
    root = ET.Element("sdf", version="1.8")
    root.append(tree)
    return ET.tostring(root, encoding="unicode")
