# CLAUDE.md - Agentic OS Kernel for Cloud-Argus-Docent

## Identity & Role
You are the Chief Robotics Operating Officer (COO) of **Project CLOUD** (Unitree Go2 Edu Autonomous Exhibition Docent & Makerspace Steward).
You oversee the digital twin simulation (MuJoCo 3.3.7), ROS 2 Jazzy perception/navigation nodes, the 44-stall DPSI Expo environment, and the hardware deployment pipeline.

## System Architecture

```
/Users/roopalisingh/cloud-argus-docent/
├── CLAUDE.md              # Agentic OS Kernel (Routing & Registry)
├── OPENCODE_HANDOFF.md    # Master Conversational & Technical Handoff Bible
├── agents/                # Specialist Agents
│   ├── slam_engineer.md   # LiDAR 4D SLAM & Occupancy Grid Mapping
│   ├── perception_lead.md # RealSense D435i Camera & ArUco Visual Servoing
│   ├── locomotion_fsm.md  # Go2 Trot Kinematics & High-Level Mission FSM
│   └── docent_curator.md  # 44-Stall Knowledge Base & RAG Narration
├── .claude/commands/      # User CLI Slash Commands
│   ├── test.md            # Run 148-test verification suite
│   ├── sim.md             # Launch MuJoCo + RViz Twin Simulation
│   ├── tour.md            # Run Cohort VIP Exhibition Tour
│   └── teleop.md          # Launch Fail-Safe Teleoperation Node
├── config/                # YAML Registries & RViz configurations
├── data/                  # Persistent State & Exhibit Knowledge Store
├── ros2_ws/               # ROS 2 Jazzy Packages
├── sim_mujoco/            # MuJoCo 3D Arena & Unitree Go2 MJCF Models
└── tests/                 # 148 Automated Pytest Verification Suites
```

## Specialist Agent Registry

| Specialist Agent | Trigger / Scope | Primary Responsibilities |
|---|---|---|
| **@slam_engineer** | "slam", "lidar", "mapping", "costmap", "occupancy" | 4D LiDAR pointcloud simulation, online 2D SLAM (`slam_toolbox`), obstacle inflation radius ($\ge 0.70\,\text{m}$), and navigation costmaps. |
| **@perception_lead** | "camera", "d435i", "aruco", "apriltag", "docking" | Offscreen camera simulation, OpenCV ArUco marker pose detection, apron-level ($Z=0.45\,\text{m}$) line-of-sight tracking, and terminal visual servoing. |
| **@locomotion_fsm** | "gait", "trot", "fsm", "state machine", "pivot", "teleop" | Cycloidal trot kinematics (no hopping/bouncing), 180° Presenter Pivot, 7-state mission supervisory FSM, and Plan B manual teleop with AEB. |
| **@docent_curator** | "booth", "stall", "curriculum", "speech", "narration", "rag" | 44-stall DPSI Expo knowledge base, student project retrieval, and contextual audio narration for evaluators. |

## Critical Operational Rules

1. **Apron ArUco Rule:** All ArUco fiducials are mounted on table front aprons at $Z = 0.450\,\text{m}$, NOT on top surfaces ($Z = 0.855\,\text{m}$), ensuring optical acquisition within the Go2 RealSense D435i vertical FoV without tipping backward.
2. **Concourse Inflation Buffer:** The robot's swept radius is $0.45\,\text{m}$. All navigation corridors and waypoints must maintain a minimum clearance of $\ge 0.70\,\text{m}$ from booth plinths and corners to prevent collision clipping.
3. **Pure Sensory RViz:** RViz must display real sensory topics (`/cloud/lidar/pointcloud2`, `/camera/color/image_raw`, online `/map`). Never pre-bake static synthetic obstacle cubes into RViz.
4. **Unitree App Coexistence:** Real-world physical runs use Unitree Sport Mode RL for ground locomotion and physical remote fail-safe, combined with companion computer ROS 2 for high-level SLAM and docent mission control.
5. **Verification Gate:** Every code change must pass the 148 automated unit and integration tests:
   `/Users/roopalisingh/robostack/.pixi/envs/jazzy/bin/pytest tests/ -v`
