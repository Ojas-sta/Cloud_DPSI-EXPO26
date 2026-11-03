# RobotStateReport.msg: Central telemetry heartbeat
# Reference: PROJECT.md Section 7.5 & 7.8

builtin_interfaces/Time stamp
string active_persona           # "ARGUS" or "THE_DOCENT"
string fsm_state                # e.g., "STATE_DOCENT_CRUISE", "STATE_HAZARD_ALERT"
string active_landmark_id       # e.g., "NODE_10"
float32 battery_percentage      # 0.0 to 100.0%
float32 battery_voltage         # Volts (nominal 24.8V)
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_twist
bool emergency_stop_active
bool push_button_ready
