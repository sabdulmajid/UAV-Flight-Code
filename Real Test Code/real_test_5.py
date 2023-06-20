from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

# Add sensor data variables
alt = 0
lat = 0
lon = 0
roll = 0
pitch = 0
yaw = 0

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)
# 921600 is the baudrate that you have set in the mission planner or qgc

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
    global alt, lat, lon, roll, pitch, yaw

    print("Basic pre-arm checks")
    print("Starting .....")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
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
        # Update sensor data variables
        alt = vehicle.location.global_relative_frame.alt
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        roll = vehicle.attitude.roll
        pitch = vehicle.attitude.pitch
        yaw = vehicle.attitude.yaw

        print("Altitude:", alt)
        print("Latitude: %.6f, Longitude: %.6f" % (lat, lon))
        print("Roll: %.2f, Pitch: %.2f, Yaw: %.2f" % (roll, pitch, yaw))

        # Break and return from function just below target altitude.
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Initialize the takeoff sequence to 3m
arm_and_takeoff(3)

print("Take off complete")

# Hover for 10 seconds
time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
