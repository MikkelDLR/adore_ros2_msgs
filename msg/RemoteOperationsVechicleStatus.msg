# Information send vehicle to frontend

uint8 vehicle_id
float64 timestamp

uint8 vehicle_mode

uint8 UNKNOWN=0
uint8 STANDSTILL=1 # Could also be called waiting for new goal
uint8 DRIVING=2
uint8 DRIVING_REFERNCE=3
uint8 REQUESTING_REMOTE_ASSISTANCE=4 # This covers both "waiting for waypoints" and waiting for "suggested trajectory acceptance"
uint8 DRIVING_REMOTE_ASSISTANCE_TRAJECTORY=5 # Driving the trajectory generated from waypoints
uint8 DRIVING_REMOTE_ASSISTANCE_COMMANDS=6 # Driving from remote driver

adore_ros2_msgs/VehicleStateDynamic[] vehicle_state # dynamic state
adore_ros2_msgs/VehicleInfo[] vehicle_info # car state (blinkers etc)

adore_ros2_msgs/Trajectory[] suggested_trajectory # One will automatically get generated when entering REQUEST_REMOTE_ASSITANCE, but will be replaced by one that fits waypoints if any are received
