
#!/usr/bin/env bash
# Usage: ./param_setter_scripts.sh [robot_name ...]
# Example: ./param_setter_scripts.sh panda_left panda_right

ROBOTS=("$@")
if [ ${#ROBOTS[@]} -eq 0 ]; then
  ROBOTS=("panda")
fi

# Payload for full collision behaviour (same for all robots)
read -r -d '' FULL_COLLISION_PAYLOAD <<'EOF'
{
  lower_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], 
  upper_torque_thresholds_acceleration:[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],  
  lower_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], 
  upper_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],  
  lower_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], 
  upper_force_thresholds_acceleration: [40.0, 40.0, 40.0, 50.0, 50.0, 50.0],  
  lower_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], 
  upper_force_thresholds_nominal: [40.0, 40.0, 40.0, 50.0, 50.0, 50.0]
}
EOF

for ROBOT in "${ROBOTS[@]}"; do
  SRV_PREFIX="/${ROBOT}_param_service_server"
  echo "Calling ${SRV_PREFIX}/set_full_collision_behavior"
  ros2 service call ${SRV_PREFIX}/set_full_collision_behavior franka_msgs/srv/SetFullCollisionBehavior "${FULL_COLLISION_PAYLOAD}"
done

# Joint stiffness example (uncomment and adapt if needed)
# for ROBOT in "${ROBOTS[@]}"; do
#   SRV_PREFIX="/${ROBOT}_param_service_server"
#   echo "Calling ${SRV_PREFIX}/set_joint_stiffness"
#   ros2 service call ${SRV_PREFIX}/set_joint_stiffness franka_msgs/srv/SetJointStiffness "{ joint_stiffness: [3000,3000,3000,2500,2500,2000,2000] }"
# done


