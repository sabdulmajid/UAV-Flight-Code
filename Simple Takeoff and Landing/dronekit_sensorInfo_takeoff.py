import time
from dronekit import connect, VehicleMode

# Connect to the simulated drone playground
vehicle = connect('/dev/ttyAMA0', wait_ready=True)  # /dev/ttyS0

# Print welcome message
print("Drone Takeoff Code")
print("Prepare for takeoff!")

# Set the vehicle mode to GUIDED
vehicle.mode = VehicleMode("GUIDED")

# Arm the vehicle
vehicle.armed = True

# Take off to a desired height
target_altitude = 1  # Desired altitude in meters
print("Taking off...")
vehicle.simple_takeoff(target_altitude)

# Ascend and provide altitude updates
while vehicle.location.global_relative_frame.alt < target_altitude * 0.95:
    try:
        altitude = vehicle.location.global_relative_frame.alt
        latitude = vehicle.location.global_relative_frame.lat
        longitude = vehicle.location.global_relative_frame.lon
        # Retrieve other desired sensor data here

        print("Ascending - Current altitude: %.2f meters" % altitude)
        print("Latitude: %.6f, Longitude: %.6f" % (latitude, longitude))
        # Print other sensor data here

        time.sleep(1)
    except Exception as e:
        print("Error retrieving sensor data:", str(e))
