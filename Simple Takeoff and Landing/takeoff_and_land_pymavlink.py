import time
from pymavlink import mavutil

# Establish a connection to the autopilot (change the connection string accordingly)
connection_string = '/dev/ttyUSB0'  # Example connection string for a serial connection -> have to find this from the Raspberry Pi 4 connection with the PixHawk 4
# Alternatively, for UDP connection:
# connection_string = 'udp:127.0.0.1:14550'
# For TCP connection:
# connection_string = 'tcp:127.0.0.1:5760'

"""
We can also use different libraries to write this code, such as:

1. DroneKit-SDK
2. MAVSDK-Python
3. ROS (Robot Operating System)
4. DJI-SDK
5. Parrot SDK

"""

# Create a MAVLink connection
master = mavutil.mavlink_connection(connection_string)

# Wait for the autopilot to establish a connection
master.wait_heartbeat()

# Set mode to guided
mode_id = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
custom_mode = 'GUIDED'
mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    custom_mode.encode('utf-8')
)

# Arm the vehicle
master.arducopter_arm()

# Takeoff to a desired height
target_altitude = 1.0  # Desired altitude in meters
print("Taking off...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude
)

# Wait until reaching the desired height
while True:
    altitude = master.messages['GLOBAL_POSITION_INT'].alt / 1000.0
    print("Ascending - Current altitude: %.2f meters" % altitude)
    if altitude >= target_altitude * 0.95:
        break
    time.sleep(1)

# Reached the desired height
print("Reached the desired height of %d meters!" % target_altitude)

# Land the vehicle
print("Landing...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
)

# Wait until the vehicle has landed
while True:
    if master.messages['GLOBAL_POSITION_INT'].alt <= 0.1:
        break
    print("Waiting for vehicle to land...")
    time.sleep(1)

# Disarm the vehicle
master.arducopter_disarm()

# Close the connection
master.close()

# Shutdown complete
print("Flight completed. Connection closed.")
