# Slash Command: /sim
**Description:** Launch the MuJoCo 3D digital twin simulation and RViz sensory visualization window.

## Execution Command
```bash
cd /Users/roopalisingh/cloud-argus-docent
./run_cloud_sim.sh --twin-gui --spawn-stall GATE
```

## Displays
- **MuJoCo 3D Window:** High-fidelity physics simulation with 44 student booths and Unitree Go2 Edu.
- **RViz Window:** Pure sensory visualization showing live `/cloud/lidar/pointcloud2`, live `/camera/color/image_raw` with ArUco overlay, and online `/map`.
