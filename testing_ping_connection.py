from pymavlink import mavutil

# Connect to the SITL simulator
connection_string = 'tcp:127.0.0.1:5760'  # Update the IP address and port if necessary
vehicle = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat message to ensure connection is established
while True:
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        break

# Print some basic information about the vehicle
print(f"Autopilot: {msg.autopilot}")
print(f"System status: {msg.system_status}")

# Close the connection to the vehicle
vehicle.close()
