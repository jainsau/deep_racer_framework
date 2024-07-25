from enum import Enum
from typing import Dict, List, Union


class EpisodeStatus(Enum):
    CRASHED = "crashed"
    OFF_TRACK = "off_track"
    COMPLETED_LAP = "lap_complete"
    IMMOBILIZED = "immobilized"
    ALL_WHEELS_OFF_TRACK = "all_wheels_off_track"


class Vehicle:
    LENGTH: float = 0.365
    WIDTH: float = 0.225
    SAFE_OVERHANG: float = min(LENGTH, WIDTH) / 2


REAL_WORLD: Dict[str, Union[int, float, List[Union[None, float]]]] = {
    "steps_per_second": 15,
    "box_obstacle_width": 0.38,
    "box_obstacle_length": 0.24,
    "max_speeds": [
        None,
        0.01,
        0.02,
        0.04,
        0.1,
        0.15,
        0.25,
        0.4,
        0.6,
        0.9,
        1.1,
        1.3,
        1.5,
        1.7,
        2.0,
        2.2,
        2.3,
        2.6,
        2.7,
        3.1,
        3.3,
        3.4,
        3.6,
        3.8,
        4.0,
    ],
}


class ParamNames(Enum):
    X = "x"
    Y = "y"
    ALL_WHEELS_ON_TRACK = "all_wheels_on_track"
    CLOSEST_WAYPOINTS = "closest_waypoints"
    DISTANCE_FROM_CENTER = "distance_from_center"
    IS_CRASHED = "is_crashed"
    IS_LEFT_OF_CENTER = "is_left_of_center"
    IS_OFFTRACK = "is_offtrack"
    IS_REVERSED = "is_reversed"
    HEADING = "heading"
    PROGRESS = "progress"
    PROJECTION_DISTANCE = "projection_distance"
    SPEED = "speed"
    STEERING_ANGLE = "steering_angle"
    STEPS = "steps"
    TRACK_LENGTH = "track_length"
    TRACK_WIDTH = "track_width"
    WAYPOINTS = "waypoints"
    CLOSEST_OBJECTS = "closest_objects"
    OBJECTS_DISTANCE = "objects_distance"
    OBJECTS_DISTANCE_FROM_CENTER = "objects_distance_from_center"
    OBJECTS_HEADING = "objects_heading"


EDGE_ERROR_TOLERANCE: float = 0.01
