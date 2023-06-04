# test_sim.py
from dronekit import connect

# Connect to the simulated drone
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Arm the simulated drone
vehicle.armed = True

# Take off to a specified altitude
vehicle.simple_takeoff(10)

# Monitor the drone's altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)

# Simulation to ask drone to go from one point to another