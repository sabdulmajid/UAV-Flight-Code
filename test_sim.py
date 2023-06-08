# test_sim.py
from dronekit import connect, VehicleMode
import time

# Connect to the simulated drone
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Arm the simulated drone
vehicle.armed = True

# Take off to a specified altitude
vehicle.simple_takeoff(10)

# Monitor the drone's altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= 10 * 0.95:
        print("Target altitude reached")
        break
    time.sleep(1)

# Land the simulated drone
vehicle.mode = VehicleMode("LAND")

# Close the simulated drone connection
vehicle.close()
