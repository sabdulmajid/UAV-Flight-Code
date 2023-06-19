from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyAMA0')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s') % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  print("Starting .....")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print ("Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print ("Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
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

        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

# Initialize the takeoff sequence to 3m
arm_and_takeoff(3)

print("Take off complete")

# Hover for 30 seconds
time.sleep(30)

print("Now let's land!")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
