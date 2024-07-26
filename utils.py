import math
from typing import List, Optional, Union

from constants import CAR_OVERHANG, OFFSET
from models import Point, ProcessedRacepoint, ProcessedWaypoint


def calculate_distance(point1: Point, point2: Point) -> float:
    """Calculates the Euclidean distance between two points."""
    (x1, y1) = point1
    (x2, y2) = point2
    return math.hypot(x2 - x1, y2 - y1)


def calculate_bearing(start_point: Point, end_point: Point) -> float:
    """Calculates the bearing (angle in degrees) from start_point to end_point"""
    (start_x, start_y) = start_point
    (finish_x, finish_y) = end_point
    direction_in_radians = math.atan2(finish_y - start_y, finish_x - start_x)
    return math.degrees(direction_in_radians)


def normalize_angle(angle: float) -> float:
    """Normalizes an angle to be within the range [-180, 180)"""
    return (angle + 180) % 360 - 180


def calculate_turn_angle(current_bearing: float, target_bearing: float) -> float:
    """Calculates the turn angle (in degrees) from current_bearing to target_bearing"""
    difference = target_bearing - current_bearing
    return normalize_angle(difference)


def get_point_at_bearing(start_point: Point, bearing: float, distance: float) -> Point:
    """Calculates the point at a given bearing and distance from a starting point"""
    (x, y) = start_point
    radians_to_target = math.radians(bearing)
    x2 = x + math.cos(radians_to_target) * distance
    y2 = y + math.sin(radians_to_target) * distance
    return x2, y2


def get_intersection_of_two_lines(
    line_a_point_1: Point,
    line_a_point_2: Point,
    line_b_point_1: Point,
    line_b_point_2: Point,
) -> Optional[Point]:
    """Calculates the intersection point of two lines"""
    (x1, y1) = line_a_point_1
    (x2, y2) = line_a_point_2
    (x3, y3) = line_b_point_1
    (x4, y4) = line_b_point_2
    denominator = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4))
    if denominator == 0.0:
        return None  # Lines are parallel
    z1 = (x1 * y2) - (y1 * x2)
    z2 = (x3 * y4) - (y3 * x4)
    x = ((z1 * (x3 - x4)) - ((x1 - x2) * z2)) / denominator
    y = ((z1 * (y3 - y4)) - ((y1 - y2) * z2)) / denominator
    return x, y


def get_projected_point_on_line(
    point: Point,
    line_point1: Point,
    line_point2: Point,
) -> Point:
    """Projects a point onto a line defined by two points"""
    (x1, y1) = line_point1
    (x2, y2) = line_point2
    (x3, y3) = point
    if x1 == x2:
        return x1, y3  # Handle vertical line case
    m = (y2 - y1) / (x2 - x1)  # Calculate slope
    b = y1 - m * x1  # Calculate y-intercept
    x = (x3 + m * y3 - m * b) / (1 + m**2)  # Projected point's x-coordinate
    y = m * x + b  # Projected point's y-coordinate (using line equation)
    return x, y


def get_edge_point(
    previous_point: Point,
    mid_point: Point,
    next_point: Point,
    direction_offset: OFFSET,
    distance: float,
) -> Point:
    """Calculates a point offset from the midpoint of a track segment"""
    if mid_point in [previous_point, next_point]:
        track_heading = calculate_bearing(previous_point, next_point)
    else:
        bearing_to_midpoint = calculate_bearing(previous_point, mid_point)
        bearing_from_midpoint = calculate_bearing(mid_point, next_point)
        turn_angle = calculate_turn_angle(bearing_to_midpoint, bearing_from_midpoint)
        track_heading = bearing_to_midpoint + turn_angle / 2
    bearing_to_edge = track_heading + direction_offset.value
    return get_point_at_bearing(mid_point, bearing_to_edge, distance)


def get_adj_linepoints(line: list, i: int) -> tuple:
    """Returns the previous and next waypoints in a circular list."""
    n = len(line)
    prev_index = (i - 1) % n
    next_index = (i + 1) % n
    return line[prev_index], line[next_index]


def get_section_boundaries(raceline: list, idx: int) -> tuple:
    """Returns the section start ID and section switch flag for a racepoint."""
    prev_rp = raceline[idx - 1]
    curr_rp = raceline[idx]

    if idx == 0:
        return 0, True  # Default section change flag for the first racepoint

    if (curr_rp.is_in_straight_section != prev_rp.is_in_straight_section) or (
        curr_rp.is_in_curved_section != prev_rp.is_in_curved_section
    ):  # Check if the current racepoint is the start or end of a section
        if curr_rp.is_in_straight_section or curr_rp.is_in_curved_section:
            return idx, True
    else:
        return raceline[idx - 1].section_start_id, False


def get_processed_waypoints(
    waypoints: List[List[float]], track_width: float
) -> List[ProcessedWaypoint]:
    """Processes a list of waypoints to include additional information."""
    processed_waypoints = []

    for i, waypoint in enumerate(waypoints):
        # Get previous and next waypoints for safe edge calculations
        prev_wp, next_wp = get_adj_linepoints(waypoints, i)

        # Calculate left_safe and right_safe using the get_edge_point function
        left_safe = get_edge_point(
            prev_wp, waypoint, next_wp, OFFSET.LEFT, track_width / 2 + CAR_OVERHANG
        )
        right_safe = get_edge_point(
            prev_wp, waypoint, next_wp, OFFSET.RIGHT, track_width / 2 + CAR_OVERHANG
        )

        # Create ProcessedWaypoint with the new attributes
        processed_waypoint = ProcessedWaypoint(
            idx=i,
            point=Point(*waypoint),
            left_safe=left_safe,
            right_safe=right_safe,
        )
        processed_waypoints.append(processed_waypoint)

    return processed_waypoints


def get_processed_raceline(
    racepoints: List[List[Union[float, bool]]], track_width: float
) -> List[ProcessedRacepoint]:
    """Processes raceline to include additional information."""
    raceline = []

    for idx, racepoint in enumerate(racepoints):
        section_start_id, is_new_section = get_section_boundaries(racepoints, idx)
        raceline = ProcessedRacepoint(
            idx=idx,
            point=Point(racepoint[0], racepoints[1]),
            opt_speed=racepoint[2],
            is_in_straight_section=racepoint[3],
            is_in_curved_section=racepoint[4],
            section_start_id=section_start_id,
            is_new_section=is_new_section,
        )
        raceline.append(raceline)

    return raceline


def find_point_projection_on_the_line_between(
    point: Point,
    line_start: Point,
    line_end: Point,
) -> Point:
    """Finds the point where a perpendicular line drawn from a given point intersects a line defined by two endpoints"""
    x1, y1 = line_start
    x2, y2 = line_end
    x, y = point

    # Calculate the slope and y-intercept of the line
    if x2 == x1:
        # Handle the case when the line is vertical
        return (x1, y)

    slope = (y2 - y1) / (x2 - x1)
    y_intercept = y1 - slope * x1

    # Calculate the slope of the perpendicular line (negative reciprocal of the original line's slope)
    perp_slope = -1 / slope

    # Calculate the y-intercept of the perpendicular line using the point and perpendicular slope
    perp_y_intercept = y - perp_slope * x

    # Calculate the x-coordinate of the intersection point
    intersection_x = (perp_y_intercept - y_intercept) / (slope - perp_slope)

    # Calculate the y-coordinate of the intersection point
    intersection_y = slope * intersection_x + y_intercept

    return (intersection_x, intersection_y)


def is_collinear_with(point: Point, point_a: Point, point_b: Point) -> bool:
    """Determines if three points are collinear."""
    x0, y0 = point
    x1, y1 = point_a
    x2, y2 = point_b

    # Calculate the cross product of the vectors formed by the points
    cross_product = (x2 - x1) * (y0 - y1) - (y2 - y1) * (x0 - x1)

    # If the cross product is zero, the points are collinear
    return cross_product == 0


def is_boxed_between(point, point_a, point_b):
    """Check if a point lies between two points on the line segment joining them"""
    x, y = point
    x1, y1 = point_a
    x2, y2 = point_b

    # Check if the point is within the bounding box of point_a and point_b
    if (min(x1, x2) <= x <= max(x1, x2)) and (min(y1, y2) <= y <= max(y1, y2)):
        return True  # The point lies between point_a and point_b

    return False  # The point is outside the segment


def is_point_in_the_region_between(point: Point, start: Point, finish: Point) -> bool:
    """Checks if a point lies in the infinite bridge region between two other points"""
    perpendicular_intersection = find_point_projection_on_the_line_between(
        point, start, finish
    )

    return is_collinear_with(point, start, finish) and is_boxed_between(
        perpendicular_intersection, start, finish
    )


def get_closest_racepoints(
    position: Point,
    search_start_ids: list,
    raceline: List[ProcessedRacepoint],
) -> List[int]:
    """Returns indices of the closest racepoints to a given position."""

    prev_rp_id, next_rp_id = search_start_ids
    prev_rp, next_rp = raceline[prev_rp_id], raceline[next_rp_id]

    # Check if the position is between the two racepoints
    if is_point_in_the_region_between(position, prev_rp.point, next_rp.point):
        return search_start_ids

    # Calculate distances
    distances = [
        calculate_distance(position, prev_rp),
        calculate_distance(position, next_rp),
    ]

    # Determine the closer racepoint and update search_start_ids accordingly
    new_search_start_ids = (
        [prev_rp_id - 1, prev_rp_id]
        if distances[0] < distances[1]
        else [next_rp_id, next_rp_id + 1]
    )

    return get_closest_racepoints(position, new_search_start_ids, raceline)
