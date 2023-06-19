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


"""
Example of a parse tree for the sentence: "Add the values of variables 'a' and 'b' and store the result in variable 'c'"

                         Sentence
                            |
                +-----------+------------+
                |                        |
             Action                  Assignment
                |                        |
          +-----+-----+          +-------+-------+
          |           |          |               |
        Add         Value      Identifier      Identifier
          |           |          |               |
      +---+---+       |          |               |
      |       |     Variables    a               b
    Values               |
      |                  |
  +---+---+              c
  |       |
  a       b
"""