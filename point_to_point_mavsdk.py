import asyncio
from mavsdk import System


async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

