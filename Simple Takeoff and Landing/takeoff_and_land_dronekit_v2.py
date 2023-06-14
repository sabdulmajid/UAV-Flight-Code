import time
from dronekit import connect, VehicleMode

# Connect to the Pixhawk
connection_string = '/dev/ttyAMA0' # /dev/ttyS0
vehicle = connect(connection_string, wait_ready=True)

# Arm the vehicle and wait for arming confirmation
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for vehicle to arm...")
    time.sleep(1)

print("Vehicle armed!")

# Set the mode to guided
vehicle.mode = VehicleMode("STABLIZE")

# Takeoff to a specified altitude (in meters)
target_altitude = 1
vehicle.simple_takeoff(target_altitude)

while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt) # Requires the GPS!

    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Target altitude reached!")
        break

    time.sleep(1)

# Disarm the vehicle
vehicle.armed = False

# Close the connection
vehicle.close()
