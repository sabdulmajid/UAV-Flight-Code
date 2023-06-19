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
        # Retrieve sensor data
        altitude = vehicle.location.global_relative_frame.alt
        latitude = vehicle.location.global_relative_frame.lat
        longitude = vehicle.location.global_relative_frame.lon
        roll = vehicle.attitude.roll
        pitch = vehicle.attitude.pitch
        yaw = vehicle.attitude.yaw
        voltage = vehicle.battery.voltage
        current = vehicle.battery.current
        remaining_capacity = vehicle.battery.remaining
        ground_speed = vehicle.velocity[0]
        vertical_speed = vehicle.velocity[2]
        heading = vehicle.heading
        armed_status = vehicle.armed
        flight_mode = vehicle.mode.name

        # Calculate battery percentage
        full_capacity_voltage = 12.4  # Adjust this value according to your battery specifications
        empty_capacity_voltage = 10.5  # Adjust this value according to your battery specifications
        battery_percentage = ((voltage - empty_capacity_voltage) / (full_capacity_voltage - empty_capacity_voltage)) * 100
        battery_percentage = max(0, min(100, battery_percentage))  # Ensure the percentage is between 0 and 100

        # Display sensor data
        print("Ascending - Current altitude: %.2f meters" % altitude)
        print("Latitude: %.6f, Longitude: %.6f" % (latitude, longitude))
        print("Roll: %.2f, Pitch: %.2f, Yaw: %.2f" % (roll, pitch, yaw))
        print("Battery Percentage: %.2f%%" % battery_percentage)
        print("Ground Speed: %.2fm/s, Vertical Speed: %.2fm/s" % (ground_speed, vertical_speed))
        print("Heading: %.2f" % heading)
        print("Armed Status: %s, Flight Mode: %s" % (armed_status, flight_mode))

        time.sleep(1)
    except Exception as e:
        print("Error retrieving sensor data:", str(e))

# Reached the desired height
print("Reached the desired height of %d meters!" % target_altitude)
time.sleep(10)

# Descend and land
print("Descending and landing...")
vehicle.mode = VehicleMode("LAND")

# Wait until the vehicle has landed
while not vehicle.is_armable:
    print("Waiting for vehicle to land...")
    time.sleep(1)

# Drone has landed
print("Drone has landed successfully!")

# Disarm and close connection
vehicle.armed = False
vehicle.close()

# Shutdown complete
print("Turning off the drone. Goodbye!")
