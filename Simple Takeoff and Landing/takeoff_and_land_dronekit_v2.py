import time
from dronekit import connect, VehicleMode

# Connect to the Pixhawk
connection_string = '/dev/ttyAMA0'  # Update with your actual connection string
vehicle = connect(connection_string, wait_ready=True)

# Arm the vehicle and wait for arming confirmation
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for vehicle to arm...")
    time.sleep(1)

print("Vehicle armed!")

# Set the mode to guided
vehicle.mode = VehicleMode("GUIDED")

# Takeoff to a specified altitude (in meters)
target_altitude = 10  # Update with your desired altitude
vehicle.simple_takeoff(target_altitude)

while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)

    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Target altitude reached!")
        break

    time.sleep(1)

# Disarm the vehicle
vehicle.armed = False

# Close the connection
vehicle.close()
