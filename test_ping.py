import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("tcp:127.0.0.1:5760")

master.mav.ping_send(
    int(time.time() * 1e6), # Unix time in microseconds
    0, # Ping number
    0, # Request ping of all systems
    0 # Request ping of all components
)

master.wait_heartbeat()

while True:
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)