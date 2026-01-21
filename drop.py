#!/usr/bin/env python3
"""
simple_drop_mission.py

Usage:
    python3 simple_drop_mission.py <target_lat> <target_lon> [--alt ALT] [--tol METERS]

Example:
    python3 simple_drop_mission.py 47.397742 8.545594 --alt 30 --tol 3
"""

import asyncio
import math
import sys
import argparse
from mavsdk import System
from mavsdk import (telemetry, action)

# --------- Utility functions ---------
def haversine_distance_m(lat1, lon1, lat2, lon2):
    """Return great-circle distance between two points (in meters)."""
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# --------- Mission class ---------
class DropMission:
    def __init__(self, connection_url: str = "udp://:14540"):
        """
        connection_url: MAVSDK connection string (udp://, serial:///dev/..., etc.)
        """
        self.connection_url = connection_url
        self.drone = System()
        self._home_lat = None
        self._home_lon = None
        self._home_rel_alt = None

    async def connect(self, timeout_s: float = 10.0):
        print(f"[+] Connecting to drone on {self.connection_url} ...")
        await self.drone.connect(system_address=self.connection_url)

        # wait for connection
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[+] MAVSDK: connected to vehicle")
                break

        # fetch and store home position (first valid global position / home)
        # this uses telemetry.home() which returns once when a home is available
        try:
            async for home in self.drone.telemetry.home():
                # home is an object with latitude_deg, longitude_deg, absolute_altitude_m, relative_altitude_m
                self._home_lat = home.latitude_deg
                self._home_lon = home.longitude_deg
                self._home_rel_alt = home.relative_altitude_m
                print(f"[+] Home position captured: lat={self._home_lat}, lon={self._home_lon}, rel_alt={self._home_rel_alt}")
                break
        except Exception as e:
            print(f"[!] Could not read home position: {e}")

    async def arm_and_takeoff(self, target_alt_m: float = 30.0):
        print("[*] Arming...")
        await self.drone.action.arm()

        print(f"[*] Taking off to {target_alt_m} m...")
        await self.drone.action.set_takeoff_altitude(target_alt_m)
        await self.drone.action.takeoff()

        # wait until altitude reached (within tolerance)
        reached = False
        async for pos in self.drone.telemetry.position():
            # position has latitude_deg, longitude_deg, absolute_altitude_m
            # telemetry.position yields many updates; break once above ~90% of target
            alt = pos.relative_altitude_m if hasattr(pos, "relative_altitude_m") else None
            if alt is None:
                # fallback: try absolute altitude - home altitude
                try:
                    alt = pos.absolute_altitude_m - self._home_rel_alt
                except Exception:
                    alt = None
            if alt is not None:
                print(f"    current rel alt = {alt:.1f} m")
                if alt >= target_alt_m * 0.9:
                    reached = True
                    break
            await asyncio.sleep(0.5)
        if reached:
            print("[+] Takeoff altitude reached (approx).")
        else:
            print("[!] Takeoff altitude detection timed out (continuing).")

    async def goto_location(self, lat: float, lon: float, alt_m: float, yaw_deg: float = 0.0):
        """
        Use action.goto_location to fly to the target.
        alt_m should be absolute altitude (AGL/relative depending on autopilot). 
        Many autopilots expect absolute altitude in meters (WGS84), but MAVSDK action.goto_location expects
        absolute altitude in meters above mean sea level. To avoid complexity we assume target altitude is relative
        to home/takeoff altitude and set takeoff altitude earlier. Many boards handle this transparently.
        """
        print(f"[*] Navigating to target: lat={lat}, lon={lon}, alt={alt_m}, yaw={yaw_deg}")
        # Note: goto_location is: goto_location(latitude_deg, longitude_deg, altitude_m, yaw_deg)
        await self.drone.action.goto_location(lat, lon, alt_m, yaw_deg)

    async def wait_until_reached(self, target_lat: float, target_lon: float, tol_m: float = 3.0, timeout_s: float = 120.0):
        """Monitor position until the drone is within tol_m of target or timeout."""
        print(f"[*] Waiting until within {tol_m} m of target (timeout {timeout_s}s)...")
        start = asyncio.get_event_loop().time()
        async for pos in self.drone.telemetry.position():
            cur_lat = pos.latitude_deg
            cur_lon = pos.longitude_deg
            dist = haversine_distance_m(cur_lat, cur_lon, target_lat, target_lon)
            print(f"    Current position: lat={cur_lat:.6f}, lon={cur_lon:.6f}, distance={dist:.1f} m")
            if dist <= tol_m:
                print(f"[+] Arrived within {tol_m} m of target (distance={dist:.1f} m).")
                return True
            if asyncio.get_event_loop().time() - start > timeout_s:
                print("[!] wait_until_reached: timeout reached")
                return False
            await asyncio.sleep(1.0)

    async def drop_payload(self):
        """
        Drop payload implementation.

        IMPORTANT:
        Replace the body of this method with your hardware-specific actuation command.
        Examples:
         - If you have a servo connected to a Pixhawk and exposed via MAVLink as DO_SET_SERVO,
           you can use the MAVLink plugin to send the appropriate command.
         - If you have a companion microcontroller listening over MAVLink/serial, trigger it here.

        For safety, I only simulate the drop here and wait 2 seconds to mimic actuation.
        """
        print("[*] drop_payload: called -> performing payload release sequence...")

        # --- PLACEHOLDER: real actuation goes here ---
        # Example: send mavlink command to set servo PWM (pseudo-code / depends on your setup)
        # await self.drone.mavlink.send_command_int(
        #     command=176,  # MAV_CMD_DO_SET_SERVO
        #     param1=9,     # servo number
        #     param2=1100   # PWM value
        # )
        #
        # Or use self.drone.param.set_param_int(...) if you've configured a channel to toggle
        #
        # WARNING: The exact API / parameter names depend on MAVSDK and autopilot version.
        # Test on bench (props removed) before any real drop / flight.

        # Simulated drop:
        await asyncio.sleep(2.0)
        print("[+] Payload drop sequence completed (simulated).")

    async def return_to_launch(self):
        print("[*] Commanding RTL (return to launch)...")
        await self.drone.action.return_to_launch()

    async def mission_to_coordinate_and_drop(self, target_lat: float = 47.397940, target_lon: float =8.545592, altitude_m: float = 30.0, tolerance_m: float = 3.0):
        """
        High-level mission:
         1. Arm & takeoff to altitude_m
         2. Fly to (target_lat, target_lon)
         3. Wait until arrival, call drop_payload
         4. RTL
        """
        # Ensure connected
        # Note: connect() should have been called already, but check basic connection
        await self.connect()  # you may skip if already connected externally

        # Arm & takeoff
        await self.arm_and_takeoff(altitude_m)

        # Navigate to target
        # Many autopilots expect absolute alt; we reuse altitude_m
        await self.goto_location(target_lat, target_lon, altitude_m, yaw_deg=0.0)

        # Wait until reached
        ok = await self.wait_until_reached(target_lat, target_lon, tol_m=tolerance_m, timeout_s=180)
        if not ok:
            print("[!] Did not reach target within timeout; attempting drop anyway (or abort).")

        # Drop payload via class method
        try:
            await self.drop_payload()
        except Exception as e:
            print(f"[!] drop_payload() raised exception: {e}")

        # Command RTL
        await self.return_to_launch()

        # Optionally, wait for landed state or disarm
        print("[*] Mission complete: RTL issued. Monitor vehicle for landed/disarm as needed.")

# --------- CLI wrapper ---------
async def main():
    parser = argparse.ArgumentParser(description="Simple drop mission using MAVSDK")
    parser.add_argument("--lat", type=float, default = 47.397940, help="Target latitude")
    parser.add_argument("--lon", type=float, default = 8.545592, help="Target longitude")
    parser.add_argument("--alt", type=float, default=30.0, help="Target altitude (meters)")
    parser.add_argument("--tol", type=float, default=3.0, help="Acceptable arrival tolerance (meters)")
    parser.add_argument("--connection", type=str, default="udp://:14540", help="MAVSDK connection URL (default udp://:14540)")
    args = parser.parse_args()

    mission = DropMission(connection_url=args.connection)
    # You can call mission.connect() here if you want to separate connect from mission invocation
    await mission.mission_to_coordinate_and_drop(args.lat, args.lon, altitude_m=args.alt, tolerance_m=args.tol)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Interrupted by user; exiting.")
