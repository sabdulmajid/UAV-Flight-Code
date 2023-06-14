from dronekit import connect, VehicleMode
import time

# Connect to the vehicle
connection_string = '/dev/ttyAMA0'  # Replace with the appropriate serial port
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Arm and takeoff
print "Arming motors"
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print "Waiting for arming..."
    time.sleep(1)

print "Taking off"
vehicle.simple_takeoff(2)  # Replace 2 with your desired target altitude (in meters)

# Wait until target altitude is reached
while vehicle.location.global_relative_frame.alt < 2:  # Replace 2 with your desired target altitude (in meters)
    print "Altitude: {} meters".format(vehicle.location.global_relative_frame.alt)
    time.sleep(1)

# Spin the motors -> not sure if the channel values are the same
print "Spinning motors"
vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': 1500, '4': 1500}  # Adjust channel values as needed

# Wait for a few seconds
time.sleep(5)

# Stop the motors
print "Stopping motors"
vehicle.channels.overrides = {'1': 1000, '2': 1000, '3': 1000, '4': 1000}  # Adjust channel values as needed

# Disarm the vehicle
print "Disarming motors"
vehicle.armed = False

# Close the connection
vehicle.close()
