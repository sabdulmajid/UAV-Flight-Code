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

