# SpillAlert.msg: Real-time floor hazard detection telemetry
# Reference: PROJECT.md Section 7.8 & Section 4.5

builtin_interfaces/Time stamp
string hazard_type              # "LIQUID_SPILL", "OBSTACLE_CABLE", "ZONE_INCURSION"
float32 confidence              # 0.0 to 1.0 confidence score
float32 estimated_area_sq_m     # Measured surface area of puddle
geometry_msgs/Point location    # Relative or map Cartesian coordinates
string severity                 # "LOW", "MEDIUM", "CRITICAL"
string image_base64_jpeg        # Compressed JPEG snapshot of the hazard for Discord dispatch
