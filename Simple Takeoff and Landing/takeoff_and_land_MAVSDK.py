import mavsdk

# Create a connection to the vehicle
vehicle = mavsdk.connect('/dev/ttyAMA0', baud=57600)

# Wait for the vehicle to connect
while not vehicle.is_connected():
    pass

# Arm the vehicle
vehicle.arm()

# Set the motor PWM to 100%
for i in range(4):
    vehicle.action.set_motor_speed(i, 100)

# Wait for 1 second
time.sleep(1)

# Turn off the motors
for i in range(4):
    vehicle.action.set_motor_speed(i, 0)

# Disarm the vehicle
vehicle. disarm()

# Close the connection to the vehicle
vehicle.disconnect()
