import time
import pigpio

# Connect to the Pixhawk
pi = pigpio.pi()

# Set the motor PWM to 100%
for i in range(4):
    pi.set_servo_pulsewidth(i, 1000)

# Wait for 1 second
time.sleep(1)

# Turn off the motors
for i in range(4):
    pi.set_servo_pulsewidth(i, 0)

# Disconnect from the Pixhawk
pi.stop()
