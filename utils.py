import math
from typing import List, Optional, Tuple

from models import Point, ProcessedRacepoint


def calculate_distance(
    point1: Tuple[float, float], point2: Tuple[float, float]
) -> float:
    """Calculates the Euclidean distance between two points."""
    (x1, y1) = point1
    (x2, y2) = point2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def calculate_bearing(
    start_point: Tuple[float, float], end_point: Tuple[float, float]
) -> float:
    """Calculates the bearing (angle in degrees) from start_point to end_point.
    Args:
        start_point: Coordinates of the starting point.
        end_point: Coordinates of the ending point.
    Returns:
        The bearing in degrees.
    """
    (start_x, start_y) = start_point
    (finish_x, finish_y) = end_point
    direction_in_radians = math.atan2(finish_y - start_y, finish_x - start_x)
    return math.degrees(direction_in_radians)


def normalize_angle(angle: float) -> float:
    """Normalizes an angle to be within the range [-180, 180)."""
    return (angle + 180) % 360 - 180


def calculate_turn_angle(current_bearing: float, target_bearing: float) -> float:
    """Calculates the turn angle (in degrees) from current_bearing to target_bearing."""
    difference = target_bearing - current_bearing
    return normalize_angle(difference)


def is_point_approximately_between(
    point: Tuple[float, float],
    start: Tuple[float, float],
    finish: Tuple[float, float],
    tolerance: float = 90.0,
) -> bool:
    """Checks if a point lies approximately between two other points.
    Args:
        point: Coordinates of the point to check.
        start: Coordinates of the starting point.
        finish: Coordinates of the ending point.
        tolerance: Maximum allowed angle deviation (in degrees) for the point to be considered between start and finish.
    Returns:
        True if the point is approximately between start and finish, False otherwise.
    """
    bearing_from_start = calculate_bearing(start, point)
    bearing_to_finish = calculate_bearing(point, finish)
    turn_angle = abs(calculate_turn_angle(bearing_from_start, bearing_to_finish))
    return turn_angle < tolerance


def get_point_at_bearing(
    start_point: Tuple[float, float], bearing: float, distance: float
) -> Tuple[float, float]:
    """Calculates the point at a given bearing and distance from a starting point.
    Args:
        start_point: Coordinates of the starting point (x, y).
        bearing: Bearing (angle in degrees) from the starting point.
        distance: Distance from the starting point.
    Returns:
        Coordinates of the calculated point (x, y).
    """
    (x, y) = start_point
    radians_to_target = math.radians(bearing)
    x2 = x + math.cos(radians_to_target) * distance
    y2 = y + math.sin(radians_to_target) * distance
    return x2, y2


def get_intersection_of_two_lines(
    line_a_point_1: Tuple[float, float],
    line_a_point_2: Tuple[float, float],
    line_b_point_1: Tuple[float, float],
    line_b_point_2: Tuple[float, float],
) -> Optional[Tuple[float, float]]:
    """Calculates the intersection point of two lines.
    Args:
        line_a_point_1: First point on line A.
        line_a_point_2: Second point on line A.
        line_b_point_1: First point on line B.
        line_b_point_2: Second point on line B.
    Returns:
        The coordinates of the intersection point as a tuple (x, y),
        or None if the lines are parallel.
    """
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
    point: Tuple[float, float],
    line_point1: Tuple[float, float],
    line_point2: Tuple[float, float],
) -> Tuple[float, float]:
    """Projects a point onto a line defined by two points.
    Args:
        point: The point to project.
        line_point1: The first point defining the line.
        line_point2: The second point defining the line.
    Returns:
        The coordinates of the projected point on the line.
    """
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
    previous_point: Tuple[float, float],
    mid_point: Tuple[float, float],
    future_point: Tuple[float, float],
    direction_offset: int,
    distance: float,
) -> Tuple[float, float]:
    """Calculates a point offset from the midpoint of a track segment.
    Args:
        previous_point: Coordinates of the point before the midpoint.
        mid_point: Coordinates of the midpoint.
        future_point: Coordinates of the point after the midpoint.
        direction_offset: Angle offset (in degrees) from the track direction, must be either 90 or -90.
        distance: Distance from the midpoint to the edge point.
    Returns:
        Coordinates of the edge point.
    """
    if direction_offset not in [90, -90]:
        raise ValueError("direction_offset must be either 90 or -90")
    if previous_point == mid_point:
        raise ValueError("previous_point cannot be the same as mid_point")
    bearing_to_midpoint = calculate_bearing(previous_point, mid_point)
    if mid_point == future_point:
        track_heading = bearing_to_midpoint
    else:
        bearing_from_midpoint = calculate_bearing(mid_point, future_point)
        turn_angle = calculate_turn_angle(bearing_to_midpoint, bearing_from_midpoint)
        track_heading = bearing_to_midpoint + turn_angle / 2
    bearing_to_edge = track_heading + direction_offset
    return get_point_at_bearing(mid_point, bearing_to_edge, distance)


def find_perpendicular_intersection(
    point: Tuple[float, float],
    line_start: Tuple[float, float],
    line_end: Tuple[float, float],
) -> Tuple[float, float]:
    """
    Finds the point where a perpendicular line drawn from a given point intersects a line defined by two endpoints.

    Args:
        point (

        ): The point from which the perpendicular line is drawn (x, y).
        line_start (Point): The starting point of the line (x1, y1).
        line_end (Point): The ending point of the line (x2, y2).

    Returns:
        Point: The intersection point of the perpendicular line and the original line.
    """
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


def get_adj_racepoints(raceline: list, i: int) -> tuple:
    """Returns the previous and next waypoints in a circular list."""
    n = len(raceline)
    prev_index = (i - 1) % n
    next_index = (i + 1) % n
    return raceline[prev_index], raceline[next_index]


def get_section_boundaries(raceline: list, idx: int) -> tuple:
    """Returns the section start ID and section switch flag for a racepoint."""
    if idx == 0:
        return 0, False  # No section change for the first racepoint

    prev_rp = raceline[idx - 1]
    curr_rp = raceline[idx]

    if (curr_rp.is_in_straight_section != prev_rp.is_in_straight_section) or (
        curr_rp.is_in_curved_section != prev_rp.is_in_curved_section
    ):
        # Check if the current racepoint is the start or end of a section
        if curr_rp.is_in_straight_section or curr_rp.is_in_curved_section:
            return idx, True
    else:
        return raceline[idx - 1].section_start_id, False


def get_processed_raceline(
    racepoints: list, track_width: float
) -> List[ProcessedRacepoint]:
    """Processes a list of racepoints to include additional information."""
    car_overhang = 0.5  # TODO Replace with the actual value from constants
    raceline = []

    for i, rp in enumerate(racepoints):
        # Get previous and next racepoints for safe edge calculations
        prev_rp, next_rp = get_adj_racepoints(raceline, i)

        # Calculate left_safe and right_safe using the get_edge_point function
        left_safe = get_edge_point(
            prev_rp, rp, next_rp, 90, track_width / 2 + car_overhang
        )
        right_safe = get_edge_point(
            prev_rp, rp, next_rp, -90, track_width / 2 + car_overhang
        )

        # Create ProcessedRacepoint with the new attributes
        racepoint = ProcessedRacepoint(
            idx=i,
            point=Point(x=rp[0], y=rp[1]),
            opt_speed=rp[2],
            is_in_straight_section=rp[3],
            is_in_curved_section=rp[4],
            section_start_id=0,  # Placeholder, will be updated later
            is_new_section=True,  # Placeholder, will be updated later
            left_safe=left_safe,
            right_safe=right_safe,
        )
        raceline.append(racepoint)

    # Update section_start_id and is_new_section
    for id_ in range(1, len(raceline)):
        section_start_id, is_new_section = get_section_boundaries(raceline, id_)
        raceline[id_] = raceline[id_]._replace(
            section_start_id=section_start_id, is_new_section=is_new_section
        )

    return raceline
