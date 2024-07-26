from enum import Enum

VEHICLE_LENGTH: float = 0.30
VEHICLE_WIDTH: float = 0.15
CAR_OVERHANG = 0.5  # Max 50% of the car width can hang outside the track
DISTANCE_SIGMA = 0.1
SPEED_SIGMA = 0.4
ANGLE_SIGMA = 10
STEPS_PER_SECOND: int = 15
TINY_REWARD: float = 1e-4
EDGE_ERROR_TOLERANCE: float = 0.01


class OFFSET(Enum):
    LEFT = 90
    RIGHT = -90


PARAM_NAMES = {
    "X": "x",
    "Y": "y",
    "ALL_WHEELS_ON_TRACK": "all_wheels_on_track",
    "CLOSEST_WAYPOINTS": "closest_waypoints",
    "DISTANCE_FROM_CENTER": "distance_from_center",
    "IS_CRASHED": "is_crashed",
    "IS_LEFT_OF_CENTER": "is_left_of_center",
    "IS_OFFTRACK": "is_offtrack",
    "IS_REVERSED": "is_reversed",
    "HEADING": "heading",
    "PROGRESS": "progress",
    "PROJECTION_DISTANCE": "projection_distance",
    "SPEED": "speed",
    "STEERING_ANGLE": "steering_angle",
    "STEPS": "steps",
    "TRACK_LENGTH": "track_length",
    "TRACK_WIDTH": "track_width",
    "WAYPOINTS": "waypoints",
    "CLOSEST_OBJECTS": "closest_objects",
    "OBJECTS_DISTANCE": "objects_distance",
    "OBJECTS_DISTANCE_FROM_CENTER": "objects_distance_from_center",
    "OBJECTS_HEADING": "objects_heading",
}
