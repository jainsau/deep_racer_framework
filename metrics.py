import math
from functools import cached_property
from typing import List, Optional

from constants import (
    ANGLE_SIGMA,
    DISTANCE_SIGMA,
    SPEED_SIGMA,
    STEPS_PER_SECOND,
    TINY_REWARD,
    VEHICLE_WIDTH,
    ParamNames,
)
from models import Point, ProcessedRacepoint, ProcessedWaypoint
from utils import (
    calculate_bearing,
    calculate_distance,
    calculate_turn_angle,
    find_point_projection_on_the_line_between,
    get_closest_racepoints,
    get_processed_waypoints,
)


class StepMetrics:
    def __init__(
        self,
        params: dict,
        raceline: List[ProcessedRacepoint],
        waypoints: List[List[ProcessedWaypoint]],
        previous_step: Optional["StepMetrics"],
    ):
        self.all_wheels_on_track: bool = bool(
            params.get(ParamNames["ALL_WHEELS_ON_TRACK"])
        )
        self.x: float = float(params.get(ParamNames["X"], 0.0))
        self.y: float = float(params.get(ParamNames["Y"], 0.0))
        self.closest_objects: list = params.get(ParamNames["CLOSEST_OBJECTS"])
        self.closest_waypoints: list = params.get(ParamNames["CLOSEST_WAYPOINTS"])
        self.distance_from_center: float = float(
            params.get(ParamNames["DISTANCE_FROM_CENTER"])
        )
        self.heading: float = float(params.get(ParamNames["HEADING"]))
        self.is_crashed: bool = bool(params.get(ParamNames["IS_CRASHED"]))
        self.is_left_of_center: bool = bool(params.get(ParamNames["IS_LEFT_OF_CENTER"]))
        self.is_offtrack: bool = bool(params.get(ParamNames["IS_OFFTRACK"]))
        self.is_reversed: bool = bool(params.get(ParamNames["IS_REVERSED"]))
        self.objects_distance: list = params.get(ParamNames["OBJECTS_DISTANCE"])
        self.objects_heading: list = params.get(ParamNames["OBJECTS_HEADING"])
        self.objects_left_of_center: list = params.get(
            ParamNames["OBJECTS_LEFT_OF_CENTER"]
        )
        self.objects_location: list = params.get(ParamNames["OBJECTS_LOCATION"])
        self.objects_speed: list = params.get(ParamNames["OBJECTS_SPEED"])
        self.progress: float = float(params.get(ParamNames["PROGRESS"]))
        self.speed: float = float(params.get(ParamNames["SPEED"]))
        self.steering_angle: float = float(params.get(ParamNames["STEERING_ANGLE"]))
        self.steps: int = int(params.get(ParamNames["STEPS"]))
        self.track_length: float = float(params.get(ParamNames["TRACK_LENGTH"]))
        self.track_width: float = float(params.get(ParamNames["TRACK_WIDTH"]))
        self.waypoints: List[ProcessedWaypoint] = waypoints or get_processed_waypoints(
            params.get(ParamNames["WAYPOINTS"])
        )
        self.raceline: List[ProcessedRacepoint] = raceline
        self.previous_step: Optional["StepMetrics"] = (
            previous_step  # is None for new laps
        )

    @cached_property
    def position(self) -> tuple:
        return Point(x=self.x, y=self.y)

    @cached_property
    def is_lap_new(self) -> bool:
        return self.progress == 0.0

    @cached_property
    def is_lap_complete(self):
        return self.progress == 100.0

    @cached_property
    def is_final_step(self) -> bool:
        return (
            self.is_lap_complete
            or self.is_crashed
            or self.is_offtrack
            or self.is_reversed
        )

    @cached_property
    def action_sequence_length(self) -> int:
        if (
            self.previous_step
            and self.previous_step.speed == self.speed
            and self.previous_step.steering_angle == self.steering_angle
        ):
            return self.previous_step.action_sequence_length + 1
        return 1

    @cached_property
    def is_right_of_center(self) -> bool:
        return not self.is_left_of_center

    @cached_property
    def distance_from_edge(self) -> float:
        return max(
            0.0, (self.track_width + VEHICLE_WIDTH) / 2 - self.distance_from_center
        )

    @cached_property
    def step_distance(self) -> float:
        if self.previous_step:
            return calculate_distance(self.previous_step.position, self.position)
        return 0.0

    @cached_property
    def total_distance(self) -> float:
        if self.progress > 0:
            return self.progress * self.track_length
        return 0.0

    @cached_property
    def time(self) -> float:
        return self.steps / STEPS_PER_SECOND

    @cached_property
    def predicted_lap_time(self) -> float:
        if self.progress > 0:
            return round((100 / self.progress) * self.time, 2)
        return 0.0

    @cached_property
    def previous_waypoint(self) -> ProcessedWaypoint:
        return self.waypoints[int(self.closest_waypoints[0])]

    @cached_property
    def next_waypoint(self) -> ProcessedWaypoint:
        return self.waypoints[int(self.closest_waypoints[1])]

    @cached_property
    def distance_to_prev_waypoint(self) -> float:
        return calculate_distance(self.position, self.previous_waypoint.point)

    @cached_property
    def distance_to_next_waypoint(self) -> float:
        return calculate_distance(self.position, self.next_waypoint.point)

    @cached_property
    def distance_from_closest_waypoint(self) -> float:
        return min(self.distance_to_prev_waypoint, self.distance_to_next_waypoint)

    @cached_property
    def closest_waypoint(self) -> List[ProcessedWaypoint]:
        return (
            self.previous_waypoint
            if self.distance_to_prev_waypoint < self.distance_to_next_waypoint
            else self.next_waypoint
        )

    @cached_property
    def closest_racepoints(self) -> List[int]:
        return get_closest_racepoints(
            self.position, self.closest_waypoints, self.raceline
        )

    @cached_property
    def previous_racepoint(self) -> ProcessedRacepoint:
        return self.raceline[self.closest_racepoints[0]]

    @cached_property
    def next_racepoint(self) -> ProcessedRacepoint:
        return self.raceline[self.closest_racepoints[1]]

    @cached_property
    def distance_to_prev_racepoint(self) -> float:
        return calculate_distance(self.position, self.previous_racepoint.point)

    @cached_property
    def distance_to_next_racepoint(self) -> float:
        return calculate_distance(self.position, self.next_racepoint.point)

    @cached_property
    def distance_from_closest_racepoint(self) -> float:
        return min(self.distance_to_prev_racepoint, self.distance_to_next_racepoint)

    @cached_property
    def closest_racepoint(self) -> ProcessedRacepoint:
        return (
            self.previous_racepoint
            if self.distance_to_prev_racepoint < self.distance_to_next_racepoint
            else self.next_racepoint
        )

    @cached_property
    def leftsafe_bearing(self):
        leftsafe_bearing = calculate_bearing(
            self.position, self.next_waypoint.left_safe
        )
        return leftsafe_bearing

    @cached_property
    def rightsafe_bearing(self):
        rightsafe_bearing = calculate_bearing(
            self.position, self.next_waypoint.right_safe
        )
        return rightsafe_bearing

    @cached_property
    def left_right_skew(self) -> float:
        return calculate_turn_angle(self.rightsafe_bearing, self.leftsafe_bearing)

    @cached_property
    def is_steering_straight(self) -> bool:
        return abs(self.steering_angle) < 0.01

    @cached_property
    def is_steering_left(self) -> bool:
        return self.steering_angle > 0 and not self.is_steering_straight

    @cached_property
    def is_steering_right(self) -> bool:
        return self.steering_angle < 0 and not self.is_steering_straight

    @cached_property
    def desired_bearing(self) -> float:
        return calculate_bearing(self.position, self.next_racepoint)

    @cached_property
    def true_bearing(self) -> float:
        if self.previous_step:
            return calculate_bearing(self.previous_step.position, self.position)
        return self.heading

    @cached_property
    def track_bearing(self) -> float:
        return calculate_bearing(self.previous_waypoint, self.next_waypoint)

    @cached_property
    def track_skew(self) -> float:
        return calculate_turn_angle(self.true_bearing, self.track_bearing)

    @cached_property
    def raceline_bearing(self) -> float:
        return calculate_bearing(self.previous_racepoint, self.next_racepoint)

    @cached_property
    def raceline_skew(self) -> float:
        return calculate_turn_angle(self.true_bearing, self.raceline_bearing)

    @cached_property
    def has_crashed_since_the_beginning_of_the_lap(self) -> bool:
        if self.previous_step:  # if not a new lap
            return (
                self.previous_step.has_crashed_since_the_beginning_of_the_lap
                or self.is_crashed
            )
        return self.is_crashed

    @cached_property
    def has_crashed_since_the_beginning_of_the_section(self) -> Optional[bool]:
        if not self.previous_racepoint.is_new_section:  # if not a new section
            return (
                self.previous_step.has_crashed_since_the_beginning_of_the_section
                or self.is_crashed
            )
        return self.is_crashed

    @cached_property
    def is_in_straight_section(self) -> bool:
        return (
            self.previous_racepoint.is_in_straight_section
            and self.next_racepoint.is_in_straight_section
        )

    @cached_property
    def is_in_curved_section(self) -> bool:
        return (
            self.previous_racepoint.is_in_curved_section
            and self.next_racepoint.is_in_curved_section
        )

    @cached_property
    def is_headed_out_of_lookahead_cone(self) -> bool:
        right_to_curr_skew = calculate_turn_angle(self.rightsafe_bearing, self.heading)
        left_to_curr_skew = calculate_turn_angle(self.heading, self.leftsafe_bearing)

        return (
            abs((right_to_curr_skew + left_to_curr_skew) - self.left_right_skew) <= 2
        )  # margin of error

    @cached_property
    def is_steering_out_of_lookahead_cone(self) -> bool:
        right_to_curr_skew = calculate_turn_angle(
            self.rightsafe_bearing, self.steering_angle
        )
        left_to_curr_skew = calculate_turn_angle(
            self.steering_angle, self.leftsafe_bearing
        )

        return (
            abs((right_to_curr_skew + left_to_curr_skew) - self.left_right_skew) <= 2
        )  # margin of error

    @cached_property
    def opt_speed(self) -> float:
        return self.closest_racepoint.opt_speed

    @cached_property
    def opt_position(self) -> tuple:
        return find_point_projection_on_the_line_between(
            self.position,
            self.previous_racepoint.point,
            self.next_racepoint.point,
        )

    @cached_property
    def heading_z_score(self) -> float:
        reward = pow(math.e, -((self.raceline_skew**2) / (2 * (ANGLE_SIGMA**2))))
        return reward

    @cached_property
    def distance_z_score(self) -> float:
        reward = pow(
            math.e,
            -(
                (calculate_distance(self.position, self.opt_position) ** 2)
                / (2 * (DISTANCE_SIGMA**2))
            ),
        )
        return reward

    @cached_property
    def speed_z_score(self) -> float:
        score = pow(
            math.e, -((self.speed - self.opt_speed) ** 2) / (2 * (SPEED_SIGMA**2))
        )
        return score

    @cached_property
    def straight_section_score(self) -> float:
        if self.previous_racepoint.is_in_straight_section:
            return (
                (self.speed_z_score**2)
                * (self.distance_z_score)
                * (self.heading_z_score**2)
            )
        return 0.0

    @cached_property
    def curved_section_score(self) -> float:
        if self.previous_racepoint.is_in_curved_section:
            return (
                (self.speed_z_score**2)
                * (self.distance_z_score)
                * (self.heading_z_score**2)
            )
        return 0.0

    def _calculate_straight_section_bonus(self) -> float:
        bonus = self.straight_section_score
        prev = self.previous_step
        while prev and prev.is_in_straight_section:
            bonus += prev.straight_section_score
            prev = prev.previous_step
        return bonus

    @cached_property
    def straight_section_bonus(self) -> float:
        if (
            self.previous_racepoint.is_in_straight_section
            and not self.next_racepoint.is_in_straight_section
            and not self.has_crashed_since_the_beginning_of_the_section
        ):
            bonus = self._calculate_straight_section_bonus()
            return bonus
        return 0.0

    def _calculate_curved_section_bonus(self) -> float:
        bonus = self.curved_section_score
        prev = self.previous_step
        while prev and prev.is_in_curved_section:
            bonus += prev.curved_section_score
            prev = prev.previous_step
        return bonus

    @cached_property
    def curved_section_bonus(self) -> float:
        if (
            self.previous_racepoint.is_in_curved_section
            and not self.next_racepoint.is_in_curved_section
            and not self.has_crashed_since_the_beginning_of_the_section
        ):
            return self._calculate_curved_section_bonus()
        return 0.0

    @cached_property
    def track_completion_bonus(self) -> float:
        if self.is_lap_complete and not self.has_crashed_since_the_beginning_of_the_lap:
            return 100.0
        return 0.0

    @cached_property
    def took_unpardonable_action(self) -> bool:
        if (
            self.is_offtrack
            or self.raceline_skew > 30
            or self.is_headed_out_of_lookahead_cone
            or self.is_steering_out_of_lookahead_cone
            or (self.is_steering_left and self.desired_bearing < 0)
            or (self.is_steering_right and self.desired_bearing > 0)
            or (self.opt_speed - self.speed > 1)
        ):
            return True
        return False

    @cached_property
    def immediate_reward(self) -> float:
        if self.took_unpardonable_action:
            return TINY_REWARD
        return (self.speed_z_score + self.distance_z_score + self.heading_z_score) ** 2

    @cached_property
    def long_term_reward(self) -> float:
        return (
            self.straight_section_bonus
            + self.curved_section_bonus
            + self.track_completion_bonus
        )

    @cached_property
    def total_step_reward(self) -> float:
        return self.immediate_reward + self.long_term_reward
