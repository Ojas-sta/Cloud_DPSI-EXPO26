# Slash Command: /teleop
**Description:** Launch Plan B manual keyboard/gamepad teleoperation node with automatic emergency braking.

## Execution Command
```bash
cd /Users/roopalisingh/cloud-argus-docent
/Users/roopalisingh/robostack/.pixi/envs/jazzy/bin/python scripts/teleop_keyboard.py
```

## Control Mapping
- `W` / `S`: Linear Forward / Backward ($0.4\,\text{m/s}$).
- `A` / `D`: Lateral Left / Right Strafe ($0.3\,\text{m/s}$).
- `Q` / `E`: Angular Yaw Left / Right ($0.6\,\text{rad/s}$).
- `P`: Execute 180° Presenter Pivot Macro.
- `SPACE`: Emergency Brake override.
