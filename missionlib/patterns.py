"""Search pattern generators. Each returns a list of (lat, lon) waypoints."""

from .geo import offset_latlon


def expanding_square(center_lat, center_lon, leg_unit_m, num_legs):
    """
    Classic SAR expanding square: heading cycles N, E, S, W, and leg length
    grows by one unit every two legs (1,1,2,2,3,3,...). Starts at the center
    and spirals outward, so it covers the area around a last-known-position
    with priority given to the area closest to that point.
    """
    waypoints = []
    lat, lon = center_lat, center_lon
    headings = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # (north_mult, east_mult)
    leg_count = 1
    for i in range(num_legs):
        north_mult, east_mult = headings[i % 4]
        length = leg_unit_m * leg_count
        lat, lon = offset_latlon(lat, lon, north_mult * length, east_mult * length)
        waypoints.append((lat, lon))
        if i % 2 == 1:
            leg_count += 1
    return waypoints


def lawnmower(center_lat, center_lon, width_m, height_m, lane_spacing_m):
    """
    Boustrophedon ("lawnmower") sweep: parallel lanes running east-west,
    alternating direction, stepped north by lane_spacing_m each pass.
    Better than expanding square for covering a known rectangular area
    evenly rather than spiraling out from a point.
    """
    waypoints = []
    half_w, half_h = width_m / 2.0, height_m / 2.0
    south_lat, south_lon = offset_latlon(center_lat, center_lon, -half_h, -half_w)
    lanes = max(1, int(height_m // lane_spacing_m) + 1)
    going_east = True
    for i in range(lanes):
        north_offset = i * lane_spacing_m
        if going_east:
            start = offset_latlon(south_lat, south_lon, north_offset, 0)
            end = offset_latlon(south_lat, south_lon, north_offset, width_m)
        else:
            start = offset_latlon(south_lat, south_lon, north_offset, width_m)
            end = offset_latlon(south_lat, south_lon, north_offset, 0)
        waypoints.append(start)
        waypoints.append(end)
        going_east = not going_east
    return waypoints


PATTERNS = {
    "expanding_square": expanding_square,
    "lawnmower": lawnmower,
}
