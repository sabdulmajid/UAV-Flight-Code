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
