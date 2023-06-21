from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
from math import sqrt

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

# Define a list of waypoints
waypoints = [
    (28.382081, 36.482996),  # Waypoint 1 - Home location
    (28.382129, 36.482940),  # Waypoint 2
    (28.382039, 36.482997),  # Waypoint 3
]

# Initialize the takeoff sequence to 2m
arm_and_takeoff(2)

print("Takeoff complete")

# Fly to each waypoint
for waypoint in waypoints:
    print("Flying to waypoint: %s" % str(waypoint))
    target_location = LocationGlobalRelative(waypoint[0], waypoint[1], 3)  # 3m altitude for each waypoint
    vehicle.simple_goto(target_location)

    # Wait until reaching the waypoint
    while True:
        distance = sqrt((vehicle.location.global_relative_frame.lat - target_location.lat)**2 +
                        (vehicle.location.global_relative_frame.lon - target_location.lon)**2)
        if distance < 1:
            print("Reached waypoint: %s" % str(waypoint))
            break
        time.sleep(1)

    # Add a delay between waypoints
    time.sleep(20)  # 20 seconds delay

print("Mission complete. Returning to land.")

# Land the drone
vehicle.mode = VehicleMode("LAND")

while not vehicle.mode.name == "LAND":
    time.sleep(1)

print("Landing complete")

# Close vehicle object
vehicle.close()
