import time
from dronekit import connect, VehicleMode

# Connect to the simulated drone playground
vehicle = connect('/dev/ttyAMA0', wait_ready=True)  # /dev/ttyS0

