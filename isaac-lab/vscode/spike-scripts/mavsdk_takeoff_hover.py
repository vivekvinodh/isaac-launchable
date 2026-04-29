#!/usr/bin/env python3
"""
Test takeoff + hover + land using MAVSDK.
Use this to visually verify PX4-Pegasus integration is producing real flight.
"""

import asyncio
from mavsdk import System


async def run():
    drone = System()
    print("Connecting to drone on udpin://:14540...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()
    print("-- Armed.")

    print("-- Taking off (default 2.5m altitude)")
    await drone.action.takeoff()

    print("-- Hovering 15 seconds — watch the /viewer")
    await asyncio.sleep(15)

    print("-- Landing")
    await drone.action.land()

    # Wait for landing
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    print("-- Landed.")

    print("-- Disarming")
    await drone.action.disarm()
    print("-- Done.")


if __name__ == "__main__":
    asyncio.run(run())
