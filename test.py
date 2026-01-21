# #!/usr/bin/env python3

# import asyncio
# from mavsdk import System


# async def run():
#     """
#     This is the "main" function.
#     It first creates the drone object and initializes it.

#     Then it registers tasks to be run in parallel (one can think of them as
#     threads):
#         - print_altitude: print the altitude
#         - print_flight_mode: print the flight mode
#         - observe_is_in_air: this monitors the flight mode and returns when the
#                              drone has been in air and is back on the ground.

#     Finally, it goes to the actual works: arm the drone, initiate a takeoff
#     and finally land.

#     Note that "observe_is_in_air" is not necessary, but it ensures that the
#     script waits until the drone is actually landed, so that we receive
#     feedback during the landing as well.
#     """

#     # Init the drone
#     drone = System()
#     await drone.connect(system_address="udpin://0.0.0.0:14540")

#     print("Waiting for drone to connect...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print("-- Connected to drone!")
#             break

#     # Start parallel tasks
#     print_altitude_task = asyncio.ensure_future(print_altitude(drone))
#     print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))

#     running_tasks = [print_altitude_task, print_flight_mode_task]
#     termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

#     async for health in drone.telemetry.health():
#         if health.is_global_position_ok and health.is_home_position_ok:
#             print("-- Global position state is good enough for flying.")
#             break

#     # Execute the maneuvers
#     print("-- Arming")
#     await drone.action.arm()

#     print("-- Taking off")
#     await drone.action.set_takeoff_altitude(10.0)
#     await drone.action.takeoff()

#     await asyncio.sleep(10)

#     print("-- Landing")
#     await drone.action.land()

#     # Wait until the drone is landed (instead of exiting after 'land' is sent)
#     await termination_task


# async def print_altitude(drone):
#     """Prints the altitude when it changes"""

#     previous_altitude = None

#     async for position in drone.telemetry.position():
#         altitude = round(position.relative_altitude_m)
#         if altitude != previous_altitude:
#             previous_altitude = altitude
#             print(f"Altitude: {altitude}")


# async def print_flight_mode(drone):
#     """Prints the flight mode when it changes"""

#     previous_flight_mode = None

#     async for flight_mode in drone.telemetry.flight_mode():
#         if flight_mode != previous_flight_mode:
#             previous_flight_mode = flight_mode
#             print(f"Flight mode: {flight_mode}")


# async def observe_is_in_air(drone, running_tasks):
#     """Monitors whether the drone is flying or not and
#     returns after landing"""

#     was_in_air = False

#     async for is_in_air in drone.telemetry.in_air():
#         if is_in_air:
#             was_in_air = is_in_air

#         if was_in_air and not is_in_air:
#             for task in running_tasks:
#                 task.cancel()
#                 try:
#                     await task
#                 except asyncio.CancelledError:
#                     pass
#             await asyncio.get_event_loop().shutdown_asyncgens()
#             return


# if __name__ == "__main__":
#     # Start the main function
#     asyncio.run(run())











#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk import mission_raw


async def px4_connect_drone():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            return drone


async def run():
    drone = await px4_connect_drone()
    await run_drone(drone)


async def run_drone(drone):
    mission_items = []

    mission_items.append(
        mission_raw.MissionItem(
            # start seq at 0
            0,
            # MAV_FRAME command. 3 is WGS84 + relative altitude
            3,
            # command. 16 is a basic waypoint
            16,
            # first one is current
            1,
            # auto-continue. 1: True, 0: False
            1,
            # param1
            0,
            # param2 - Acceptance radius
            10,
            # param3 - 0 (pass through the waypoint normally)
            0,
            # param4 - Desired yaw angle at waypoint
            float("nan"),
            # param5 - latitude
            int(47.40271757 * 10**7),
            # param6 - longitude
            int(8.54285027 * 10**7),
            # param7 - altitude
            30.0,
            # mission_type.
            0,
        )
    )

    mission_items.append(
        mission_raw.MissionItem(
            1,
            3,
            16,
            0,
            1,
            0,
            10,
            0,
            float("nan"),
            int(47.40271757 * 10**7),
            int(8.54361892 * 10**7),
            30.0,
            0,
        )
    )

    print("-- Uploading mission")
    await drone.mission_raw.upload_mission(mission_items)
    print("-- Done")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())