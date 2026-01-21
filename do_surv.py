#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")   # adjust if needed

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
    region = [
        (47.39803986, 8.54557254),  # Corner 1
        (47.39803622, 8.54501464),  # Corner 2
        (47.39782562, 8.54500928),  # Corner 3
        (47.39783288, 8.54590000),  # Corner 4
    ]


    altitude = 5.0   # meters
    speed = 10.0       # m/s
    spacing = 0.00005  # approx 5m in lat/lon, adjust as needed

    mission_items = []

    # ------------------------------------------------------
    # Generate lawnmower pattern between north/south edges
    # For simplicity: we sweep between Corner1->Corner2 and Corner4->Corner3
    # ------------------------------------------------------
    lat_start = region[0][0]
    lon_left = min(region[0][1], region[3][1])
    lon_right = max(region[1][1], region[2][1])
    lat_end = region[2][0]

    direction = True  # left to right
    lat = lat_start
    

    while lat > lat_end:
        if direction:
            mission_items.append(
                MissionItem(lat, lon_left, altitude, speed,
                            is_fly_through=True,
                            gimbal_pitch_deg=float('nan'),
                            gimbal_yaw_deg=float('nan'),
                            camera_action=MissionItem.CameraAction.NONE,
                            loiter_time_s=0,
                            camera_photo_interval_s=0,
                            acceptance_radius_m=float('nan'),
                            yaw_deg=float('nan'),
                            camera_photo_distance_m=0,
                            vehicle_action=MissionItem.VehicleAction.NONE)
            )
            mission_items.append(
                MissionItem(lat, lon_right, altitude, speed,
                            is_fly_through=True,
                            gimbal_pitch_deg=float('nan'),
                            gimbal_yaw_deg=float('nan'),
                            camera_action=MissionItem.CameraAction.NONE,
                            loiter_time_s=0,
                            camera_photo_interval_s=0,
                            acceptance_radius_m=float('nan'),
                            yaw_deg=float('nan'),
                            camera_photo_distance_m=0,
                            vehicle_action=MissionItem.VehicleAction.NONE)
            )
        else:
            mission_items.append(
                MissionItem(lat, lon_right, altitude, speed,
                            is_fly_through=True,
                            gimbal_pitch_deg=float('nan'),
                            gimbal_yaw_deg=float('nan'),
                            camera_action=MissionItem.CameraAction.NONE,
                            loiter_time_s=0,
                            camera_photo_interval_s=0,
                            acceptance_radius_m=float('nan'),
                            yaw_deg=float('nan'),
                            camera_photo_distance_m=0,
                            vehicle_action=MissionItem.VehicleAction.NONE)
            )
            mission_items.append(
                MissionItem(lat, lon_left, altitude, speed,
                            is_fly_through=True,
                            gimbal_pitch_deg=float('nan'),
                            gimbal_yaw_deg=float('nan'),
                            camera_action=MissionItem.CameraAction.NONE,
                            loiter_time_s=0,
                            camera_photo_interval_s=0,
                            acceptance_radius_m=float('nan'),
                            yaw_deg=float('nan'),
                            camera_photo_distance_m=0,
                            vehicle_action=MissionItem.VehicleAction.NONE)
            )

        direction = not direction
        lat -= spacing   # move “down” for next strip

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












# #!/usr/bin/env python3
# import asyncio
# from mavsdk import System
# from mavsdk.mission import MissionItem, MissionPlan


# async def run():
#     drone = System()
#     await drone.connect(system_address="udp://:14540")   # adjust if needed

#     print("Waiting for connection...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print("✅ Drone connected")
#             break

#     print("Waiting for GPS & home position...")
#     async for health in drone.telemetry.health():
#         if health.is_global_position_ok and health.is_home_position_ok:
#             print("✅ GPS OK")
#             break

#     """
#     QGroundControl-style survey / transect generator (Python + MAVSDK)

#     Function:
#         generate_survey_mission_items(polygon_coords, altitude_m, grid_spacing_m, ...)
#     returns:
#         list of mavsdk.mission.MissionItem objects ready for upload.

#     Notes:
#     - polygon_coords: list of (lat, lon) in cyclic order (clockwise OR ccw).
#     - angle_deg: rotation of transect lines in degrees clockwise from north (QGC style).
#     - This implementation uses UTM projection for metric accuracy.
#     - Inspired by QGC's TransectStyleComplexItem algorithm (rotate -> grid -> clip -> rotate back).
#     See QGroundControl source / plan format for details. :contentReference[oaicite:1]{index=1}
#     """

#     from typing import List, Tuple, Optional
#     import math
#     import numpy as np
#     from shapely.geometry import Polygon, LineString, MultiLineString
#     from shapely import affinity
#     from pyproj import Transformer, CRS
#     from mavsdk.mission import MissionItem

#     # ---------- helpers: projection ----------
#     def _utm_crs_for_latlon(lat: float, lon: float) -> CRS:
#         """Return EPSG CRS for UTM zone covering (lat, lon)."""
#         zone = int((lon + 180.0) / 6) + 1
#         if lat >= 0:
#             epsg = 32600 + zone
#         else:
#             epsg = 32700 + zone
#         return CRS.from_epsg(epsg)

#     def _build_transformers(ref_lat: float, ref_lon: float):
#         """Return (to_utm, from_utm) transformers using a UTM zone chosen by polygon centroid."""
#         utm_crs = _utm_crs_for_latlon(ref_lat, ref_lon)
#         wgs84 = CRS.from_epsg(4326)
#         to_utm = Transformer.from_crs(wgs84, utm_crs, always_xy=True)   # lon,lat -> x,y
#         from_utm = Transformer.from_crs(utm_crs, wgs84, always_xy=True) # x,y -> lon,lat
#         return to_utm, from_utm

#     # ---------- main generator ----------
#     def generate_survey_mission_items(
#         polygon_coords: List[Tuple[float, float]],   # list of (lat, lon)
#         altitude_m: float,
#         grid_spacing_m: float,
#         angle_deg: float = 0.0,                      # degrees clockwise from north
#         turn_distance_m: float = 10.0,
#         camera_trigger_distance_m: float = 0.0,      # >0 to enable distance-based camera triggering
#         speed_m_s: float = 5.0,
#         min_segment_length_m: float = 1.0,
#         fly_through: bool = True,
#     ) -> List[MissionItem]:
#         """
#         Produce MAVSDK MissionItem list covering the polygon with a lawnmower (survey) pattern.
#         """

#         if len(polygon_coords) < 3:
#             raise ValueError("polygon_coords must contain at least 3 points")

#         # 1) projection: choose UTM zone around polygon centroid for metric coords
#         lats = [p[0] for p in polygon_coords]
#         lons = [p[1] for p in polygon_coords]
#         centroid_lat = sum(lats) / len(lats)
#         centroid_lon = sum(lons) / len(lons)
#         to_utm, from_utm = _build_transformers(centroid_lat, centroid_lon)

#         # shapely polygon in UTM
#         poly_xy = Polygon([to_utm.transform(lon, lat) for lat, lon in polygon_coords])
#         if not poly_xy.is_valid:
#             poly_xy = poly_xy.buffer(0.0)  # try fix

#         # 2) rotate polygon so transects align with Y axis:
#         #    shapely.affinity.rotate uses degrees counter-clockwise positive.
#         #    we receive angle_deg as clockwise-from-north; to align transects we rotate by -angle_deg.
#         #    rotate around polygon centroid (Cartesian coordinates)
#         cx, cy = poly_xy.centroid.x, poly_xy.centroid.y
#         rotated = affinity.rotate(poly_xy, -angle_deg, origin=(cx, cy), use_radians=False)

#         # 3) create vertical lines (x = const) spaced by grid_spacing_m, clipped to rotated polygon
#         minx, miny, maxx, maxy = rotated.bounds
#         # extend beyond polygon height for stable intersections
#         extend_y = (maxy - miny) * 2.0 + turn_distance_m * 4.0

#         # compute range of x positions to cover entire bounding box
#         start_x = minx - grid_spacing_m * 2
#         end_x = maxx + grid_spacing_m * 2
#         num_lines = max(1, int(math.ceil((end_x - start_x) / grid_spacing_m)) + 2)
#         xs = [start_x + i * grid_spacing_m for i in range(num_lines + 1)]

#         clipped_segments = []
#         for x in xs:
#             line = LineString([(x, miny - extend_y), (x, maxy + extend_y)])
#             inter = rotated.intersection(line)
#             if inter.is_empty:
#                 continue
#             # intersection might be LineString, MultiLineString, or a Point (touch). We only want line segments.
#             if isinstance(inter, LineString):
#                 segs = [inter]
#             elif isinstance(inter, MultiLineString):
#                 segs = [g for g in inter.geoms if isinstance(g, LineString)]
#             else:
#                 # skip points or other geometries
#                 continue
#             for seg in segs:
#                 if seg.length < min_segment_length_m:
#                     continue
#                 # extend segment endpoints along Y (transect direction) by turn_distance_m
#                 x0, y0 = seg.coords[0]
#                 x1, y1 = seg.coords[-1]
#                 # ensure order low->high on y for consistent extension
#                 y_min, y_max = min(y0, y1), max(y0, y1)
#                 y_min_ext = y_min - turn_distance_m
#                 y_max_ext = y_max + turn_distance_m
#                 seg_ext = LineString([(x, y_min_ext), (x, y_max_ext)])
#                 clipped_segments.append(seg_ext)

#         if not clipped_segments:
#             # no coverage (polygon too small or spacing too large)
#             return []

#         # 4) sort segments left->right (by centroid x) and build zig-zag (boustrophedon) path
#         clipped_segments.sort(key=lambda s: s.centroid.x)
#         ordered_points = []
#         reverse = False
#         for seg in clipped_segments:
#             pts = list(seg.coords)
#             if reverse:
#                 pts = list(reversed(pts))
#             # append segment endpoints (2 points). For long segments you could densify, but not necessary here.
#             ordered_points.append(pts[0])
#             ordered_points.append(pts[-1])
#             reverse = not reverse

#         # 5) rotate path back to original orientation (inverse rotation by +angle_deg)
#         #    rotate about same centroid (cx, cy)
#         path_points_utm = []
#         for (x, y) in ordered_points:
#             p_rot_back = affinity.rotate(LineString([(x, y), (x, y)]), angle_deg, origin=(cx, cy), use_radians=False)
#             # rotate returns a Linestring; grab first coordinate
#             xr, yr = p_rot_back.coords[0]
#             path_points_utm.append((xr, yr))

#         # 6) transform UTM back to lat/lon and build MissionItem list
#         mission_items: List[MissionItem] = []
#         for (x, y) in path_points_utm:
#             lon, lat = from_utm.transform(x, y)   # returns (lon, lat)
#             # Build a full MAVSDK MissionItem (fill required fields)
#             mi = MissionItem(
#                 latitude_deg=lat,
#                 longitude_deg=lon,
#                 relative_altitude_m=altitude_m,
#                 speed_m_s=speed_m_s,
#                 is_fly_through=fly_through,
#                 gimbal_pitch_deg=0.0,
#                 gimbal_yaw_deg=0.0,
#                 camera_action=MissionItem.CameraAction.START_PHOTO_INTERVAL
#                             if camera_trigger_distance_m > 0.0
#                             else MissionItem.CameraAction.NONE,
#                 loiter_time_s=0.0,
#                 camera_photo_interval_s=0.0,
#                 acceptance_radius_m=2.0,
#                 yaw_deg=float("nan"),
#                 camera_photo_distance_m=camera_trigger_distance_m,
#                 vehicle_action=MissionItem.VehicleAction.NONE,
#             )
#             mission_items.append(mi)

#         return mission_items



#     poly = [
#             (47.39803986, 8.54557254),
#             (47.39803622, 8.54501464),
#             (47.39782562, 8.54500928),
#             (47.39783288, 8.54590000),
#         ]
#     items = generate_survey_mission_items(
#             polygon_coords=poly,
#             altitude_m=5.0,
#             grid_spacing_m=5.0,
#             angle_deg=90.0,
#             turn_distance_m=0.0,
#             camera_trigger_distance_m=0.1,
#             speed_m_s=10.0
#         )


#         # print(f"Generated {len(items)} mission items")
#         # for i, it in enumerate(items[:8]):
#         #     print(i, it.latitude_deg, it.longitude_deg)


#     mission_plan = MissionPlan(items)

#     print("Uploading mission...")
#     await drone.mission.upload_mission(mission_plan)

#     print("Arming...")
#     await drone.action.arm()

#     print("Starting mission...")
#     await drone.mission.start_mission()

#     # Monitor until mission is complete
#     async for mission_progress in drone.mission.mission_progress():
#         print(f"Mission progress: {mission_progress.current}/"
#               f"{mission_progress.total}")
#         if mission_progress.current == mission_progress.total:
#             print("✅ Mission complete")
#             break

#     print("Returning to launch...")
#     await drone.action.return_to_launch()


# if __name__ == "__main__":
#     asyncio.run(run())














# #!/usr/bin/env python3
# import asyncio
# from mavsdk import System
# from mavsdk.mission import MissionItem, MissionPlan
# import math


# async def run():
#     drone = System()
#     await drone.connect(system_address="udp://:14540")   # adjust if needed

#     print("Waiting for connection...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print("✅ Drone connected")
#             break

#     print("Waiting for GPS & home position...")
#     async for health in drone.telemetry.health():
#         if health.is_global_position_ok and health.is_home_position_ok:
#             print("✅ GPS OK")
#             break

#     # ------------------------------------------------------
#     # Define survey region (quadrilateral corners)
#     # ------------------------------------------------------
#     region = [
#         (47.39803986, 8.54557254),  # Corner 1
#         (47.39803622, 8.54501464),  # Corner 2
#         (47.39782562, 8.54500928),  # Corner 3
#         (47.39783288, 8.54590000),  # Corner 4
#     ]

#     altitude = 5.0   # meters
#     speed = 10.0       # m/s
#     spacing = 5.0  # approx 5m in lat/lon, adjust as needed




#     # You might also want a geometry library like shapely to help with polygon clipping
#     from shapely.geometry import Polygon, LineString

#     def generate_survey_mission_items(
#         polygon_coords,    # list of (lat, lon) in cyclic order (e.g. clockwise)
#         altitude_m,         # relative or absolute (you may decide)
#         grid_spacing_m,     # distance between adjacent transect lines, in meters (ground distance)
#         angle_deg=0.0,       # grid orientation angle (from north, or from East axis — you must decide)
#         turn_distance_m=10.0,  # extra “turnaround” extension beyond polygon for safe turning
#         fly_through=True,     # whether to fly through or stop at waypoints
#         camera_trigger_dist=None,  # if you want camera trigger every X meters
#     ) -> list[MissionItem]:
#         """
#         Returns a list of MAVSDK MissionItem objects forming a survey pattern over the polygon.
#         """

#         # 1. Build shapely polygon in a projected coordinate system (e.g. local east-north)  
#         #    For simplicity, approximate small region: convert lat/lon to local planar coords    
#         #    (You might use a UTM or simple equirectangular approx if area is small.)

#         # For brevity, here's a naive lat-lon to local easting/northing (approx):
#         def latlon_to_xy(ref_lat, ref_lon, lat, lon):
#             # approximate: 1 deg lat ~ 111e3 m, 1 deg lon ~ 111e3 * cos(lat)
#             dy = (lat - ref_lat) * 111000.0
#             dx = (lon - ref_lon) * 111000.0 * math.cos(math.radians(ref_lat))
#             return dx, dy

#         ref_lat, ref_lon = polygon_coords[0]
#         pts_xy = [latlon_to_xy(ref_lat, ref_lon, lat, lon) for lat, lon in polygon_coords]
#         poly = Polygon(pts_xy)

#         # 2. Generate parallel “grid lines” covering the bounding box, rotated by angle
#         #    Choose a bounding “strip” region that fully encloses polygon.

#         # Compute bounding box in XY
#         minx, miny, maxx, maxy = poly.bounds

#         # Compute a direction vector for the grid lines
#         theta = math.radians(angle_deg)
#         dx = math.cos(theta)
#         dy = math.sin(theta)
#         # The “normal” direction (perpendicular) for spacing:
#         nx = -dy
#         ny = dx

#         # Determine number of lines needed to cover bounding box in the normal direction
#         # Project all polygon vertices onto the normal direction to get their min/max
#         projections = [x * nx + y * ny for x, y in pts_xy]
#         p_min = min(projections)
#         p_max = max(projections)

#         # We will draw lines at positions from p_min to p_max with step = grid_spacing
#         lines = []
#         i0 = math.floor(p_min / grid_spacing_m) - 1
#         i1 = math.ceil(p_max / grid_spacing_m) + 1
#         for i in range(i0, i1 + 1):
#             print('d')
#             # offset along normal
#             offset = i * grid_spacing_m
#             # define a long line in “grid direction” passing through offset in normal direction
#             # param t (along dx, dy)
#             # line: (x, y) = (nx * offset, ny * offset) + t*(dx, dy)
#             # choose t range big enough
#             t0 = -1e5
#             t1 = +1e5
#             x0 = nx * offset + t0 * dx
#             y0 = ny * offset + t0 * dy
#             x1 = nx * offset + t1 * dx
#             y1 = ny * offset + t1 * dy
#             lines.append(LineString([(x0, y0), (x1, y1)]))

#         # 3. Clip each line with polygon to get segments inside polygon interior
#         from shapely.geometry import  MultiLineString

#         clipped_segments = []
#         for ln in lines:
#             inter = poly.intersection(ln)
#             if inter.is_empty:
#                 continue
#             if isinstance(inter, LineString):
#                 clipped_segments.append(inter)
#             elif isinstance(inter, MultiLineString):
#                 for part in inter.geoms:
#                     if isinstance(part, LineString):
#                         clipped_segments.append(part)
#             # ✅ Ignore points or other geometry types


#         # 4. Order these segments in an alternating (zigzag) order
#         #    For example: sort by centroid along normal direction
#         def seg_centroid_proj(seg):
#             cx, cy = seg.centroid.x, seg.centroid.y
#             return cx * nx + cy * ny

#         clipped_segments.sort(key=seg_centroid_proj)
#         # Now zigzag: reverse every alternate segment's direction
#         ordered_points = []
#         reverse = False
#         for seg in clipped_segments:
#             pts = list(seg.coords)
#             if reverse:
#                 pts = list(reversed(pts))
#             ordered_points.extend(pts)
#             reverse = not reverse

#         # 5. Optionally extend each segment by “turn_distance” beyond endpoints, 
#         #    along the grid direction vector (dx, dy)
#         extended_points = []
#         for (x, y) in ordered_points:
#             print('ab')
#             # compute a small offset along (dx, dy) to extend
#             x_ext = x + (turn_distance_m * dx)
#             y_ext = y + (turn_distance_m * dy)
#             extended_points.append((x_ext, y_ext))
#         # (Actually you should do extension per segment start / end, not per point; this is naive.)

#         # 6. Convert back XY to lat/lon and build MissionItems
#         mission_items = []
#         for (x, y) in extended_points:
#             print('a')
#             # invert latlon_to_xy
#             # x = (lon - ref_lon)*111k*cos, y = (lat - ref_lat)*111k
#             lat = ref_lat + (y / 111000.0)
#             lon = ref_lon + (x / (111000.0 * math.cos(math.radians(ref_lat))))
#             mi = MissionItem(
#                 latitude_deg=lat,
#                 longitude_deg=lon,
#                 relative_altitude_m=altitude_m,
#                 speed_m_s=5.0,
#                 is_fly_through=fly_through,
#                 gimbal_pitch_deg=0.0,
#                 gimbal_yaw_deg=0.0,
#                 camera_action=MissionItem.CameraAction.START_PHOTO_INTERVAL
#                             if (camera_trigger_dist and camera_trigger_dist > 0)
#                             else MissionItem.CameraAction.NONE,
#                 loiter_time_s=0.0,
#                 camera_photo_interval_s=0.0,          # use this if time-based triggering
#                 acceptance_radius_m=0.0,
#                 yaw_deg=float('nan'),
#                 camera_photo_distance_m=camera_trigger_dist if camera_trigger_dist else 0.0,
#                 vehicle_action=MissionItem.VehicleAction.NONE,
#             )

#             mission_items.append(mi)

#         return mission_items





#     mission_items = generate_survey_mission_items(
#     polygon_coords=region,
#     altitude_m=altitude,
#     grid_spacing_m=spacing,
#     angle_deg=0.0,
# )

#     mission_plan = MissionPlan(mission_items)

#     print("Uploading mission...")
#     await drone.mission.upload_mission(mission_plan)

#     print("Arming...")
#     await drone.action.arm()

#     print("Starting mission...")
#     await drone.mission.start_mission()

#     # Monitor until mission is complete
#     async for mission_progress in drone.mission.mission_progress():
#         print(f"Mission progress: {mission_progress.current}/"
#               f"{mission_progress.total}")
#         if mission_progress.current == mission_progress.total:
#             print("✅ Mission complete")
#             break

#     print("Returning to launch...")
#     await drone.action.return_to_launch()


# if __name__ == "__main__":
#     asyncio.run(run())


