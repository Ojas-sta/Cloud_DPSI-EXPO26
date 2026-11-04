#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET

ref_xml = "/Users/roopalisingh/Desktop/projects/dpsi-exhibition/robotics_world/dpsi_exhibition.xml"
go2_dir = "/Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent/sim_mujoco/unitree_go2"
out_xml = os.path.join(go2_dir, "dpsi_summit_go2_world.xml")

# Parse XML directly with ElementTree
tree = ET.parse(ref_xml)
root = tree.getroot()

# Update model name
root.set("model", "Project Cloud: DPSI Summit 2026 Go2 World")

# Remove AMR robot body from worldbody
worldbody = root.find("worldbody")
for b in list(worldbody):
    if b.tag == "body" and b.get("name") == "robot":
        worldbody.remove(b)

# Remove actuators, sensors, keyframe from root
for child in list(root):
    if child.tag in ["actuator", "sensor", "keyframe"]:
        root.remove(child)

# Convert to string
xml_str = ET.tostring(root, encoding="unicode")

# Insert <include file="go2.xml"/> right after <mujoco ...>
idx = xml_str.find(">") + 1
xml_str = xml_str[:idx] + '\n  <include file="go2.xml"/>' + xml_str[idx:]

with open(out_xml, "w", encoding="utf-8") as f:
    f.write(xml_str)

print("Properly parsed and merged XML without syntax errors!")
