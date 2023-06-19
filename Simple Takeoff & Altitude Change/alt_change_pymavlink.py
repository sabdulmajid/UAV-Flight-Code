from pymavlink import mavutil
import time

# Connect to the simulated drone
connection_string = 'tcp:127.0.0.1:5760' # Have to change this to IP of the RP4, which will be connected with ttyACM0
mav = mavutil.mavlink_connection(connection_string)

# Print welcome message
print("Drone TakeOff Testing")
print("Prepare for takeoff!")

# Set the vehicle mode to GUIDED
mode_id = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
custom_mode = mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
mav.mav.set_mode_send(
    mav.target_system,
    mode_id,
    custom_mode
)

# Arm the vehicle
mav.mav.command_long_send(
    mav.target_system, 
    mav.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    0, 1, 0, 0, 0, 0, 0, 0)

# Take off to a desired height
target_altitude = 1  # Desired altitude in meters
print("Taking off...")
mav.mav.command_long_send(
    mav.target_system, 
    mav.target_component, 
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
    0, 0, 0, 0, 0, 0, 0, target_altitude)

# Ascend and provide altitude updates
while True:
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    altitude = msg.alt / 1000.0  # Altitude is in millimeters -----------> Double check this on the pymavlink docs!
    print("Ascending - Current altitude: %.2f meters" % altitude)
    if altitude >= target_altitude * 0.95:
        print("Reached the desired height of %d meters!" % target_altitude)
        break
    time.sleep(1)

# Descend and land
print("Descending and landing...")
mav.mav.command_long_send(
    mav.target_system, 
    mav.target_component, 
    mavutil.mavlink.MAV_CMD_NAV_LAND, 
    0, 0, 0, 0, 0, 0, 0, 0)

# Wait until the vehicle has landed
while True:
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    altitude = msg.alt / 1000.0  # Altitude is in millimeters -----------> Double check this on the pymavlink docs!
    if altitude <= 0.2:  # Assuming landing is complete when altitude is close to 0
        print("Drone has landed successfully!")
        break
    print("Waiting for vehicle to land...")
    time.sleep(1)

# Disarm and close connection
mav.mav.command_long_send(
    mav.target_system, 
    mav.target_component, 
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    0, 0, 0, 0, 0, 0, 0, 0)
mav.close()

# Shutdown complete
print("Turning off the drone playground. Goodbye!")
