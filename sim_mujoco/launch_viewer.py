#!/usr/bin/env python3
import os
import sys
import mujoco
import mujoco.viewer

def main():
    xml_path = sys.argv[1] if len(sys.argv) > 1 else "/Users/roopalisingh/Desktop/projects/dpsi-exhibition/robotics_world/dpsi_exhibition.xml"
    print(f"Loading MuJoCo world: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print("Launching native MuJoCo interactive 3D viewer window...")
    mujoco.viewer.launch(model, data)

if __name__ == '__main__':
    main()
