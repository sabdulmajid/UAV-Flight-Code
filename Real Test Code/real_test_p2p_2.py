from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
import math

# Set up argument parsing
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

# Function to arm and then takeoff to a user-specified altitude
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    print("Starting ...")

    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        alt = vehicle.location.global_relative_frame.alt
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        roll = vehicle.attitude.roll
        pitch = vehicle.attitude.pitch
        yaw = vehicle.attitude.yaw

        # Print out the location, altitude, roll, pitch, yaw, battery, and velocity for the sake of debugging
        print("Altitude:", alt)
        print("Latitude: %.6f, Longitude: %.6f" % (lat, lon))
        print("Roll: %.2f, Pitch: %.2f, Yaw: %.2f" % (roll, pitch, yaw))

        # Break and return from function just below target altitude
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.99:
            print("Reached target altitude")
            break
        time.sleep(1)

# Define the distances to move
right_distance = 1.5  # meters
forward_distance = 1.5  # meters
left_distance = 1.5  # meters

# Initialize the takeoff sequence to 2m
arm_and_takeoff(2)

print("Takeoff complete")

# Save the home location
home_location = vehicle.location.global_relative_frame

# Move to the right
print("Moving to the right")
right_location = LocationGlobalRelative(home_location.lat, home_location.lon + (right_distance / (111111 * math.cos(home_location.lat))), home_location.alt)
vehicle.simple_goto(right_location)

# Wait until reaching the right location
while True:
    distance = vehicle.location.global_relative_frame.distance_to(right_location)
    if distance < 1:
        print("Reached the right waypoint")
        break
    time.sleep(1)

# Move forward
print("Moving forward")
forward_location = LocationGlobalRelative(home_location.lat + (forward_distance / 111111), home_location.lon + (right_distance / (111111 * math.cos(home_location.lat))), home_location.alt)
vehicle.simple_goto(forward_location)

# Wait until reaching the forward location
while True:
    distance = vehicle.location.global_relative_frame.distance_to(forward_location)
    if distance < 1:
        print("Reached the forward waypoint")
        break
    time.sleep(1)

# Move to the left
print("Moving to the left")
left_location = LocationGlobalRelative(home_location.lat + (forward_distance / 111111), home_location.lon, home_location.alt)
vehicle.simple_goto(left_location)

# Wait until reaching the left location
while True:
    distance = vehicle.location.global_relative_frame.distance_to(left_location)
    if distance < 1:
        print("Reached the left waypoint")
        break
    time.sleep(1)

# Return back to home location
print("Returning home")
vehicle.simple_goto(home_location)

# Wait until reaching home location
while True:
    distance = vehicle.location.global_relative_frame.distance_to(home_location)
    if distance < 1:
        print("Returned to home location")
        break
    time.sleep(1)

print("Mission complete. Returning to land.")

# Land the drone
vehicle.mode = VehicleMode("LAND")

while not vehicle.mode.name == "LAND":
    time.sleep(1)

print("Landing complete")

# Close vehicle object
vehicle.close()
