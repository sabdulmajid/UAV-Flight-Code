import asyncio
from mavsdk import System


async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Check if the drone is ready for flight
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("Drone is not armed. Please arm the drone before running this script.")
            return

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Drone is not in the air. Please take off before running this script.")
            return

    # Define the start and end waypoints
    start_latitude = 28.382731
    start_longitude = 36.482608
    start_altitude = 0

    end_latitude = 28.382503
    end_longitude = 36.482018
    end_altitude = 0

    # Go to the start waypoint
    print("Going to start waypoint...")
    await drone.action.goto_location(start_latitude, start_longitude, start_altitude, 0)

    # Wait until the drone reaches the start waypoint
    async for position in drone.telemetry.position():
        if (
            abs(position.latitude_deg - start_latitude) < 0.000001 and
            abs(position.longitude_deg - start_longitude) < 0.000001 and
            abs(position.absolute_altitude_m - start_altitude) < 0.1
        ):
            break

    print("Reached start waypoint.")

    # Go to the end waypoint
    print("Going to end waypoint...")
    await drone.action.goto_location(end_latitude, end_longitude, end_altitude, 0)
