#!/usr/bin/env python3
"""
Test arming the drone using MAVSDK.
Connects to PX4 SITL and attempts to arm the vehicle.
"""

import asyncio
from mavsdk import System


async def run():
    """Main function to connect and arm the drone."""

    # Create a drone system instance
    drone = System()

    print("Connecting to drone on udp://:14540...")
    await drone.connect(system_address="udp://:14540")

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
    try:
        await drone.action.arm()
        print("-- Armed successfully!")

        # Wait a moment to see the armed state
        await asyncio.sleep(2)

        print("-- Disarming")
        await drone.action.disarm()
        print("-- Disarmed successfully!")

    except Exception as e:
        print(f"Error during arming: {e}")
        return


if __name__ == "__main__":
    # Run the async function
    asyncio.run(run())
