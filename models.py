from collections import namedtuple

Point = namedtuple("Point", ["x", "y"])

ProcessedWaypoint = namedtuple(
    "ProcessedWaypoint",
    [
        "idx",
        "point",
        "left_safe",
        "right_safe",
    ],
)

ProcessedRacepoint = namedtuple(
    "ProcessedRacepoint",
    [
        "idx",
        "point",
        "opt_speed",
        "is_in_straight_section",
        "is_in_curved_section",
        "section_start_id",
        "is_new_section",
    ],
)
