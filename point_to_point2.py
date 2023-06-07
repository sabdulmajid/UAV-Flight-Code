from pymavlink import mavutil
import time

# Connect to the SITL simulator
connection_string = 'tcp:127.0.0.1:5760'  # Update the IP address and port if necessary
vehicle = mavutil.mavlink_connection(connection_string)

# Set the start and end coordinates
start_latitude = 28.382731
start_longitude = 36.482608
start_altitude = 0
start_yam = 0 

end_latitude = 28.382503
end_longitude = 36.482018
end_altitude = 0

# Arm the vehicle
vehicle.wait_heartbeat()
vehicle.mav.command_long_send(
    vehicle.target_system, vehicle.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 0, 0, 0, 0, 0, 0)

# Set the vehicle mode to guided
vehicle.mav.set_mode_send(
    vehicle.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4)  # Custom mode number for guided mode (example: 4)

# Send the command to the vehicle to fly to the target waypoint
vehicle.mav.mission_item_send(
    vehicle.target_system, vehicle.target_component, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    2, 0, 0, 0, 0, 0,
    end_latitude, end_longitude, end_altitude)

# Sleep for a few seconds to allow the vehicle to stabilize and reach the target waypoint
time.sleep(10)

# Log the current location every second
while True:
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_latitude = msg.lat * 1e-7
    current_longitude = msg.lon * 1e-7
    current_altitude = msg.relative_alt * 1e-3

    print(f"Current Location: Latitude: {current_latitude}, Longitude: {current_longitude}, Altitude: {current_altitude}")

    time.sleep(1)

# Close the connection to the vehicle
vehicle.close()
