from dronekit import connect, VehicleMode
import time

# Connect to the vehicle
connection_string = '/dev/ttyAMA0'
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Arm and takeoff
print "Arming motors"
vehicle.mode = VehicleMode("STABILIZE") # Might have to set it to "STABILIZE" for indoor testing without GPS
vehicle.armed = True
while not vehicle.armed:
    print "Waiting for arming..."
    time.sleep(1)

print "Taking off"
vehicle.simple_takeoff(1)  # Target altitute (in meters)

# Wait until target altitude is reached
while vehicle.location.global_relative_frame.alt < 2:  # Replace 2 with your desired target altitude (in meters)
    print "Altitude: {} meters".format(vehicle.location.global_relative_frame.alt) # Python 2.7 code...this is not Python 3 compatible
    time.sleep(1)

# Reached the desired height
print "Reached the desired height of %d meters!" % target_altitude

# Descend and land
print "Descending and landing..."
vehicle.mode = VehicleMode("LAND")

# Wait until the vehicle has landed
while not vehicle.is_armable:
    print "Waiting for vehicle to land..."
    time.sleep(1)

# Drone has landed
print "Drone has landed successfully!"

# Disarm and close connection
vehicle.armed = False
vehicle.close()

# Shutdown complete
print "Turning off the drone. Goodbye!"
