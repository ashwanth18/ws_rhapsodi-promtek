# Source in every twin terminal (Isaac and ROS): keeps the twin off the Pi /
# real Niryo graph. Same domain + localhost-only discovery on both sides.
#   source src/isaac_twin/scripts/twin_env.sh
export ROS_DOMAIN_ID="${TWIN_ROS_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Interface-pinned Fast DDS profiles (e.g. ~/.ros/so101_fastdds_eth.xml) turn
# off the builtin transports, so Isaac and ROS never discover each other.
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
