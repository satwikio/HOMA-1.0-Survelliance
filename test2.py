#!/usr/bin/env python3
"""
Precise survey planner for convex polygons (GPS lat/lon) with lines starting from the
first vertex at a specified angle. Uses local Azimuthal Equidistant projection
(centered on polygon centroid) for minimal distance/bearing distortion.

Outputs MAVSDK MissionItem list (full constructor fields filled).
"""

from typing import List, Tuple, Optional
import math
import numpy as np
from shapely.geometry import Polygon, LineString, Point, MultiLineString
from shapely import affinity
from pyproj import CRS, Transformer
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import math


class PreciseSurveyPlanner:
    """
    Precise survey planner for convex polygons.

    polygon_coords: List of (lat, lon) in decimal degrees (WGS84).
    spacing_m: spacing between successive parallel lines (meters).
    angle_deg: orientation (degrees clockwise from North) of the transect lines.
               (i.e., the lines will be parallel to this heading).
    """

    def __init__(
        self,
        polygon_coords: List[Tuple[float, float]],
        altitude_m: float,
        spacing_m: float,
        angle_deg: float,
        speed_m_s: float = 5.0,
        camera_trigger_distance_m: float = 0.0,
        turn_distance_m: float = 0.0,
        min_segment_length_m: float = 0.01,
        min_waypoint_sep_m: float = 0.01,
    ):
        if len(polygon_coords) < 3:
            raise ValueError("polygon_coords must have at least 3 vertices (convex polygon)")

        self.polygon_coords = polygon_coords  # list of (lat, lon)
        self.altitude_m = float(altitude_m)
        self.spacing_m = float(spacing_m)
        self.angle_deg = float(angle_deg)
        self.speed_m_s = float(speed_m_s)
        self.camera_trigger_distance_m = float(camera_trigger_distance_m)
        self.turn_distance_m = float(turn_distance_m)
        self.min_segment_length_m = float(min_segment_length_m)
        self.min_waypoint_sep_m = float(min_waypoint_sep_m)

        # centroid for local projection
        lats = [p[0] for p in polygon_coords]
        lons = [p[1] for p in polygon_coords]
        self.centroid_lat = float(sum(lats) / len(lats))
        self.centroid_lon = float(sum(lons) / len(lons))

        # build AEQD projection centered on centroid (preserves distances from center)
        proj_str = (
            f"+proj=aeqd +lat_0={self.centroid_lat} +lon_0={self.centroid_lon} "
            "+datum=WGS84 +units=m +no_defs"
        )
        self.wgs84 = CRS.from_epsg(4326)
        self.local_crs = CRS.from_proj4(proj_str)
        self.to_local = Transformer.from_crs(self.wgs84, self.local_crs, always_xy=True)
        self.from_local = Transformer.from_crs(self.local_crs, self.wgs84, always_xy=True)

        # prepare local polygon (x,y in meters)
        # NOTE: Transformer.transform expects (lon, lat) when always_xy=True
        local_pts = [self.to_local.transform(lon, lat) for lat, lon in self.polygon_coords]
        self.poly_local = Polygon(local_pts)
        if not self.poly_local.is_valid:
            self.poly_local = self.poly_local.buffer(0.0)

        # precompute centroid in local coords for rotation origin
        self.cx = float(self.poly_local.centroid.x)
        self.cy = float(self.poly_local.centroid.y)


    def _rotated_local_polygon(self) -> Polygon:
        """Rotate polygon by -angle so transects become vertical (x = const)."""
        return affinity.rotate(self.poly_local, -self.angle_deg, origin=(self.cx, self.cy), use_radians=False)

    def _first_vertex_rotated_x(self) -> float:
        """Compute the x coordinate (in rotated-local frame) of the first vertex (polygon_coords[0])."""
        lat0, lon0 = self.polygon_coords[0]
        x0, y0 = self.to_local.transform(lon0, lat0)
        p_local = Point(x0, y0)
        p_rot = affinity.rotate(p_local, -self.angle_deg, origin=(self.cx, self.cy), use_radians=False)
        return float(p_rot.x)

    def generate_path_local_rotated(self) -> List[Tuple[float, float]]:
        """
        Returns list of (x, y) points in the rotated-local frame (meters),
        ordered as a zig-zag (boustrophedon) covering polygon transects.
        """

        rotated = self._rotated_local_polygon()
        minx, miny, maxx, maxy = rotated.bounds

        # compute x positions for vertical lines that start at x_first and go both directions
        x_first = self._first_vertex_rotated_x()

        # Determine integer range of i such that x_first + i * spacing_m covers bounding box
        i_min = math.floor((minx - x_first) / self.spacing_m) - 1
        i_max = math.ceil((maxx - x_first) / self.spacing_m) + 1

        xs = [x_first + i * self.spacing_m for i in range(i_min, i_max + 1)]

        # extend vertical line sufficiently in Y so intersections are clean
        extend_y = max(maxy - miny, self.turn_distance_m * 4.0) + 100.0  # a buffer

        segments = []
        for x in xs:
            line = LineString([(x, miny - extend_y), (x, maxy + extend_y)])
            inter = rotated.intersection(line)
            if inter.is_empty:
                continue
            # keep only LineString or MultiLineString segments
            segs = []
            if isinstance(inter, LineString):
                segs = [inter]
            elif isinstance(inter, MultiLineString):
                segs = [g for g in inter.geoms if isinstance(g, LineString)]
            else:
                # skip Points or other geometry (touches at a vertex)
                continue

            # For convex polygon, typically one seg. Keep segments with length >= min_segment_length_m
            for seg in segs:
                if float(seg.length) < self.min_segment_length_m:
                    continue
                # extend endpoints in Y direction for turn_distance_m
                x0, y0 = seg.coords[0]
                x1, y1 = seg.coords[-1]
                y_min, y_max = min(y0, y1), max(y0, y1)
                y_min_ext = y_min - self.turn_distance_m
                y_max_ext = y_max + self.turn_distance_m
                seg_ext = LineString([(x, y_min_ext), (x, y_max_ext)])
                segments.append( (x, seg_ext) )

        if not segments:
            return []

        # sort by x ascending and build zig-zag path
        segments.sort(key=lambda it: it[0])
        ordered_rotated_points: List[Tuple[float, float]] = []
        reverse = False
        for x, seg in segments:
            pts = list(seg.coords)
            if reverse:
                pts = list(reversed(pts))
            # append start & end of each segment (you can densify if you want interval waypoints)
            ordered_rotated_points.append( (float(pts[0][0]), float(pts[0][1])) )
            ordered_rotated_points.append( (float(pts[-1][0]), float(pts[-1][1])) )
            reverse = not reverse

        # deduplicate near-duplicates (in rotated local meter-space)
        deduped = []
        last_x, last_y = None, None
        for (x,y) in ordered_rotated_points:
            if last_x is None:
                deduped.append((x,y))
                last_x, last_y = x, y
            else:
                dx = x - last_x
                dy = y - last_y
                if (dx*dx + dy*dy) >= (self.min_waypoint_sep_m ** 2):
                    deduped.append((x,y))
                    last_x, last_y = x, y
                else:
                    # skip almost-duplicate point
                    continue

        return deduped

    def _rotate_back_local(self, x_rot: float, y_rot: float) -> Tuple[float, float]:
        """Rotate a point from rotated-local frame back to original local frame (meters)."""
        p_rot = Point(x_rot, y_rot)
        p_back = affinity.rotate(p_rot, self.angle_deg, origin=(self.cx, self.cy), use_radians=False)
        return float(p_back.x), float(p_back.y)

    def generate_mission_items(self) -> List[MissionItem]:
        """
        Generate MAVSDK MissionItem list (full constructor) from the planned transect path.
        Returned mission items use the exact lat/lon computed from the precise local projection.
        """

        rotated_points = self.generate_path_local_rotated()
        if not rotated_points:
            return []

        mission_items: List[MissionItem] = []

        for (x_r, y_r) in rotated_points:
            # rotate back to original local coords (meters) then project to lon/lat
            x_local, y_local = self._rotate_back_local(x_r, y_r)
            lon, lat = self.from_local.transform(x_local, y_local)   # always_xy=True => (lon, lat)
            # NOTE: keep full double precision; do NOT round here unless you have reason
            mi = MissionItem(
                latitude_deg=float(lat),
                longitude_deg=float(lon),
                relative_altitude_m=self.altitude_m,
                speed_m_s=self.speed_m_s,
                is_fly_through=True,
                gimbal_pitch_deg=0.0,
                gimbal_yaw_deg=0.0,
                camera_action=MissionItem.CameraAction.START_PHOTO_INTERVAL
                              if self.camera_trigger_distance_m > 0.0
                              else MissionItem.CameraAction.NONE,
                loiter_time_s=0.0,
                camera_photo_interval_s=0.0,
                acceptance_radius_m=2.0,
                yaw_deg=float("nan"),
                camera_photo_distance_m=self.camera_trigger_distance_m,
                vehicle_action=MissionItem.VehicleAction.NONE,
            )
            mission_items.append(mi)

        return mission_items




def compute_spacing_from_camera(
    altitude_m: float,
    *,
    # either provide sensor+focal OR provide FOV (degrees). If both, sensor+focal takes precedence.
    sensor_width_mm: Optional[float] = None,
    sensor_height_mm: Optional[float] = None,
    focal_length_mm: Optional[float] = None,
    horizontal_fov_deg: Optional[float] = None,
    vertical_fov_deg: Optional[float] = None,
    # desired overlaps (fractions 0..1)
    sidelap_fraction: float = 0.4,   # typical 60% sidelap
    frontlap_fraction: float = 0.7,  # typical 70% frontlap
    # whether camera is oriented so that sensor_width maps to across-track (sidelap)
    sensor_width_is_across_track: bool = True,
) -> Tuple[float, float, float, float]:
    """
    Compute transect spacing and photo interval from camera parameters.

    Returns (transect_spacing_m, photo_interval_m, footprint_across_m, footprint_along_m)

    - footprint_across_m is the ground footprint dimension across-track (used for transect spacing).
    - footprint_along_m is the ground footprint dimension along-track (used for photo interval).

    Notes:
    - If sensor+focal are provided, use similar-triangles formula (recommended).
    - Otherwise, if FOV values provided (degrees), use them.
    - Assumes nadir pointing camera (no tilt). If camera tilt exists, use more advanced geometry.
    """

    # Validate overlaps
    if not (0.0 <= sidelap_fraction < 1.0):
        raise ValueError("sidelap_fraction should be in [0.0, 1.0)")
    if not (0.0 <= frontlap_fraction < 1.0):
        raise ValueError("frontlap_fraction should be in [0.0, 1.0)")

    # Compute footprint dims (meters)
    if sensor_width_mm is not None and sensor_height_mm is not None and focal_length_mm is not None:
        # similar triangles (recommended)
        # footprint = altitude * (sensor_dim / focal_length)
        fw = altitude_m * (sensor_width_mm / focal_length_mm)
        fh = altitude_m * (sensor_height_mm / focal_length_mm)
    elif horizontal_fov_deg is not None and vertical_fov_deg is not None:
        # from FOV
        hfov = math.radians(horizontal_fov_deg)
        vfov = math.radians(vertical_fov_deg)
        fw = 2.0 * altitude_m * math.tan(hfov / 2.0)
        fh = 2.0 * altitude_m * math.tan(vfov / 2.0)
    else:
        raise ValueError("Provide either sensor_width_mm/sensor_height_mm/focal_length_mm or horizontal_fov_deg/vertical_fov_deg")

    # Decide which footprint dimension is across-track (for transect spacing)
    if sensor_width_is_across_track:
        footprint_across = fw
        footprint_along = fh
    else:
        footprint_across = fh
        footprint_along = fw

    # spacing between transects to achieve desired sidelap
    transect_spacing_m = footprint_across * (1.0 - sidelap_fraction)
    # photo interval along track to achieve desired frontlap
    photo_interval_m = footprint_along * (1.0 - frontlap_fraction)

    # Safety: clamp to small positive values
    transect_spacing_m = max(transect_spacing_m, 0.01)
    photo_interval_m = max(photo_interval_m, 0.01)

    return transect_spacing_m, photo_interval_m, footprint_across, footprint_along




async def run():
    drone = System()
    # await drone.connect(system_address="serial:///dev/ttyUSB0:57600")   # Real Drone with Telemetry
    await drone.connect(system_address="udp://:14540")  #Simulated Drone

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone connected")
            break

    print("Waiting for GPS & home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ GPS OK")
            break

    # ------------------------------------------------------
    # Define survey region (quadrilateral corners)
    # ------------------------------------------------------
    # region = [
    #     (47.397940, 8.545592),  # Corner 1
    #     (47.397618, 8.545540),  # Corner 2
    #     (47.397749, 8.546070),  # Corner 3
    #     (47.397885, 8.546089),  # Corner 4
    # ]


#For the Lab

    region = [
        (31.782797, 77.003261),  # Corner 1
        (31.783100, 77.003356),  # Corner 2
        (31.782872, 77.004067),  # Corner 3
        (31.782576, 77.003901),  # Corner 4
    ]


    # # camera specs (example: typical small drone camera)
    # sensor_width_mm = 6.17    # e.g. 1/2.3" sensor width ~6.17 mm
    # sensor_height_mm = 4.55   # height in mm
    # focal_length_mm = 4.0     # focal length in mm
    altitude_m = 25.0         # flight altitude (meters AGL you intend)
    horizontal_fov_deg = 100.0
    vertical_fov_deg = 20.0

    sidelap = 0.6   # 60% side overlap
    frontlap = 0.7  # 70% front overlap

    transect_spacing_m, photo_interval_m, fw, fh = compute_spacing_from_camera(
        altitude_m,
        # sensor_width_mm=sensor_width_mmPHOTO,
        # sensor_height_mm=sensor_height_mm,
        # focal_length_mm=focal_length_mm,
        horizontal_fov_deg=horizontal_fov_deg,
        vertical_fov_deg=vertical_fov_deg,
        sidelap_fraction=sidelap,
        frontlap_fraction=frontlap,
        sensor_width_is_across_track=True
    )

    print(f"Footprint across-track: {fw:.2f} m, along-track: {fh:.2f} m")
    print(f"Transect spacing = {transect_spacing_m:.2f} m, photo interval = {photo_interval_m:.2f} m")

    # Now construct planner (using earlier PreciseSurveyPlanner)
    planner = PreciseSurveyPlanner(
        polygon_coords=region,        # your polygon list [(lat,lon), ...]
        altitude_m=altitude_m,
        spacing_m=transect_spacing_m,
        angle_deg=0.0, # user-supplied angle
        speed_m_s=3.0,
        camera_trigger_distance_m=photo_interval_m,
        turn_distance_m=0.1,
    )

    mission_items = planner.generate_mission_items()
    # Optionally densify each transect into extra waypoints at photo_interval_m if you want exact shot points.

    mission_plan = MissionPlan(mission_items)

    print("Uploading mission...")
    await drone.mission.upload_mission(mission_plan)

    print("Arming...")
    await drone.action.arm()

    print("Starting mission...")
    await drone.mission.start_mission()

    # Monitor until mission is complete
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/"
              f"{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("✅ Mission complete")
            break

    print("Returning to launch...")
    await drone.action.return_to_launch()







if __name__ == "__main__":
    asyncio.run(run())
