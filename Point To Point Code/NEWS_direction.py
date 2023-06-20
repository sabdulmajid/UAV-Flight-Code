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
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print ("Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

#Function to send velocity command to my vehicle
def set_velocity_body(Vx,Vy,Vz):
  msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,
    0,0,
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
    0b0000111111000111,
    0,0,0,
    Vx,Vy,Vz,
    0,0,0,
    0,0)
  vehicle.send_mavlink(msg)
  vehicle.flush()

arm_and_takeoff(2) # Take off to target altitude of 2 meters

counter=0
while counter==2:
  set_velocity_body(1,0,0)
  print("Direction: North relative to heading of drone")
  time.sleep(1)
  counter=counter+1

counter=0
while counter==2:
  set_velocity_body(-1,0,0)
  print("Direction: South relative to heading of drone")
  time.sleep(1)
  counter=counter+1
  
counter=0
while counter==2:
  set_velocity_body(0,1,0)
  print("Direction: east relative to heading of drone")
  time.sleep(1)
  counter=counter+1

counter=0
while counter==2:
  set_velocity_body(0,-1,0)
  print("Direction: west relative to heading of drone")
  time.sleep(1)
  counter=counter+1

  
print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
