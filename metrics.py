import math
from typing import List, Optional, Tuple

from constants import REAL_WORLD, TINY_REWARD, EpisodeStatus, ParamNames, Vehicle
from racelines.ace_speedway import raceline
from utils import (
    ProcessedRacepoint,
    calculate_bearing,
    calculate_distance,
    calculate_turn_angle,
    find_perpendicular_intersection,
    get_edge_point,
    get_intersection_of_two_lines,
    get_point_at_bearing,
    get_processed_raceline,
    get_projected_point_on_line,
    is_point_approximately_between,
)

# global state
previous_step = None
raceline = get_processed_raceline(raceline)


class StepMetrics:
    def __init__(self, params: dict, previous_step: Optional["StepMetrics"]):
        self.all_wheels_on_track: bool = bool(
            params.get(ParamNames.ALL_WHEELS_ON_TRACK)
        )
        self.x: float = float(params.get(ParamNames.X, 0.0))
        self.y: float = float(params.get(ParamNames.Y, 0.0))
        self.closest_objects: list = params.get(ParamNames.CLOSEST_OBJECTS)
        self.closest_waypoints: list = params.get(ParamNames.CLOSEST_WAYPOINTS)
        self.distance_from_center: float = float(
            params.get(ParamNames.DISTANCE_FROM_CENTER)
        )
        self.heading: float = float(params.get(ParamNames.HEADING))
        self.is_crashed: bool = bool(params.get(ParamNames.IS_CRASHED))
        self.is_left_of_center: bool = bool(params.get(ParamNames.IS_LEFT_OF_CENTER))
        self.is_offtrack: bool = bool(params.get(ParamNames.IS_OFFTRACK))
        self.is_reversed: bool = bool(params.get(ParamNames.IS_REVERSED))
        self.objects_distance: list = params.get(ParamNames.OBJECTS_DISTANCE)
        self.objects_heading: list = params.get(ParamNames.OBJECTS_HEADING)
        self.objects_left_of_center: list = params.get(
            ParamNames.OBJECTS_LEFT_OF_CENTER
        )
        self.objects_location: list = params.get(ParamNames.OBJECTS_LOCATION)
        self.objects_speed: list = params.get(ParamNames.OBJECTS_SPEED)
        self.progress: float = float(params.get(ParamNames.PROGRESS))
        self.speed: float = float(params.get(ParamNames.SPEED))
        self.steering_angle: float = float(params.get(ParamNames.STEERING_ANGLE))
        self.steps: int = int(params.get(ParamNames.STEPS))
        self.track_length: float = float(params.get(ParamNames.TRACK_LENGTH))
        self.track_width: float = float(params.get(ParamNames.TRACK_WIDTH))
        self.waypoints: List[List[float]] = params.get(ParamNames.WAYPOINTS)
        self.previous_step: Optional["StepMetrics"] = previous_step

    @property
    def is_right_of_center(self) -> bool:
        return not self.is_left_of_center

    @property
    def previous_waypoint(self) -> List[float]:
        return self.waypoints[int(self.closest_waypoints[0])]

    @property
    def next_waypoint(self) -> List[float]:
        return self.waypoints[int(self.closest_waypoints[1])]

    @property
    def distance_to_prev_waypoint(self) -> float:
        return calculate_distance((self.x, self.y), self.previous_waypoint)

    @property
    def distance_to_next_waypoint(self) -> float:
        return calculate_distance((self.x, self.y), self.next_waypoint)

    @property
    def distance_from_closest_waypoint(self) -> float:
        return min(self.distance_to_prev_waypoint, self.distance_to_next_waypoint)

    @property
    def closest_waypoint(self) -> int:
        return (
            self.previous_waypoint
            if self.distance_to_prev_waypoint < self.distance_to_next_waypoint
            else self.next_waypoint
        )

    @property
    def raceline(self) -> List[ProcessedRacepoint]:
        return raceline  # ref to global var

    @property
    def closest_racepoints(self) -> List[int]:
        # TODO replace dummy code
        return [0, 1]

    @property
    def previous_racepoint(self) -> ProcessedRacepoint:
        return self.raceline[int(self.closest_racepoints[0])]

    @property
    def next_racepoint(self) -> ProcessedRacepoint:
        return self.raceline[int(self.closest_racepoints[1])]

    @property
    def distance_to_prev_racepoint(self) -> float:
        return calculate_distance((self.x, self.y), self.previous_racepoint.point)

    @property
    def distance_to_next_racepoint(self) -> float:
        return calculate_distance((self.x, self.y), self.next_racepoint.point)

    @property
    def distance_from_closest_racepoint(self) -> float:
        return min(self.distance_to_prev_racepoint, self.distance_to_next_racepoint)

    @property
    def closest_racepoint(self) -> ProcessedRacepoint:
        return (
            self.previous_racepoint
            if self.distance_to_prev_racepoint < self.distance_to_next_racepoint
            else self.next_racepoint
        )

    @property
    def opt_speed(self) -> float:
        return self.closest_racepoint.opt_speed

    @property
    def opt_position(self) -> tuple:
        return find_perpendicular_intersection(
            (self.x, self.y),
            self.previous_racepoint.point,
            self.next_racepoint.point,
        )

    @property
    def leftsafe_bearing(self):
        leftsafe_bearing = calculate_bearing(
            self.current_position, self.next_racepoint.left_safe
        )
        return leftsafe_bearing

    @property
    def rightsafe_bearing(self):
        rightsafe_bearing = calculate_bearing(
            self.current_position, self.next_racepoint.right_safe
        )
        return rightsafe_bearing

    @property
    def left_right_skew(self) -> float:
        return abs(calculate_turn_angle(self.leftsafe_bearing, self.rightsafe_bearing))

    # @property
    # def distance_from_edge(self) -> float:
    #     return max(
    #         0.0, (self.track_width + Vehicle.WIDTH) / 2 - self.distance_from_center
    #     )
    #
    # @property
    # def time(self) -> float:
    #     return self.steps / REAL_WORLD["steps_per_second"]
    #
    # @property
    # def is_complete_lap(self) -> bool:
    #     return self.progress == 100.0
    #
    # @property
    # def is_final_step(self) -> bool:
    #     return (
    #         self.is_complete_lap
    #         or self.is_crashed
    #         or self.is_offtrack
    #         or self.is_reversed
    #     )
    #
    # @property
    # def predicted_lap_time(self) -> float:
    #     if self.progress > 0:
    #         return round(
    #             100 / self.progress * self.steps / REAL_WORLD["steps_per_second"], 2
    #         )
    #     return 0.0
    #
    @property
    def is_steering_straight(self) -> bool:
        return abs(self.steering_angle) < 0.01

    @property
    def is_steering_left(self) -> bool:
        return self.steering_angle > 0 and not self.is_steering_straight

    @property
    def is_steering_right(self) -> bool:
        return self.steering_angle < 0 and not self.is_steering_straight

    @property
    def track_bearing(self) -> float:
        return calculate_bearing(self.previous_waypoint, self.next_waypoint)

    @property
    def desired_bearing(self) -> float:
        return calculate_bearing((self.x, self.y), self.next_racepoint)

    @property
    def true_bearing(self) -> float:
        if self.previous_step:
            return calculate_bearing(
                (self.previous_step.x, self.previous_step.y), (self.x, self.y)
            )
        return self.heading

    #
    # @property
    # def action_sequence_length(self) -> int:
    #     if (
    #         self.previous_step
    #         and self.previous_step.speed == self.speed
    #         and self.previous_step.steering_angle == self.steering_angle
    #     ):
    #         return self.previous_step.action_sequence_length + 1
    #     return 1
    #
    # @property
    # def distance(self) -> float:
    #     if self.previous_step:
    #         return calculate_distance(
    #             (self, previous_step.x, self.previous_step.y), (self.x, self.y)
    #         )
    #     return 0.0
    #
    # @property
    # def track_speed(self) -> float:
    #     return self.distance / REAL_WORLD["steps_per_second"]
    #
    # @property
    # def progress_speed(self) -> float:
    #     if self.previous_step:
    #         return max(
    #             0.0,
    #             (
    #                 self.track_length
    #                 * (self.progress - self.previous_step.progress)
    #                 / 100
    #             )
    #             / REAL_WORLD["steps_per_second"],
    #         )
    #     return 0.0
    #
    # @property
    # def skew(self) -> float:
    #     return calculate_turn_angle(self.true_bearing, self.track_bearing)
    #
    # @property
    # def total_distance(self) -> float:
    #     if self.previous_step:
    #         return self.previous_step.distance + self.distance
    #     return self.distance
    #
    @property
    def is_lap_complete(self):
        return self.progress == 100.0

    @property
    def has_crashed_since_the_beginning_of_the_lap(self) -> bool:
        if self.previous_step:
            return (
                self.previous_step.has_crashed_since_the_beginning_of_the_lap
                or self.is_crashed
            )
        return self.is_crashed

    @property
    def is_in_straight_section(self) -> bool:
        return self.previous_racepoint.is_in_straight_section

    @property
    def straight_section_start_id(self) -> int:
        return self.previous_racepoint.section_start_id

    @property
    def has_crashed_since_the_beginning_of_the_straight_section(self) -> bool:
        if self.previous_racepoint.is_new_section:
            return self.is_crashed
        return (
            self.previous_step.has_crashed_since_the_beginning_of_the_straight_section
            or self.is_crashed
        )

    @property
    def is_in_curved_section(self) -> bool:
        return self.previous_racepoint.is_in_curved_section

    @property
    def curved_section_start_id(self) -> int:
        return self.previous_racepoint.section_start_id

    @property
    def has_crashed_since_the_beginning_of_the_curved_section(self) -> bool:
        if self.previous_racepoint.is_new_section:
            return self.is_crashed
        return (
            self.previous_step.has_crashed_since_the_beginning_of_the_curved_section
            or self.is_crashed
        )

    @property
    def racetrack_bearing(self) -> float:
        return calculate_bearing(self.previous_racepoint, self.next_racepoint)

    @property
    def racetrack_skew(self) -> float:
        return calculate_turn_angle(self.true_bearing, self.racetrack_bearing)

    @property
    def is_headed_out_of_lookahead_cone(self) -> bool:
        left_to_curr_skew = abs(
            calculate_turn_angle(self.leftsafe_bearing, self.heading)
        )
        right_to_curr_skew = abs(
            calculate_turn_angle(self.heading, self.rightsafe_bearing)
        )

        return (
            left_to_curr_skew > self.left_right_skew
            or right_to_curr_skew > self.left_right_skew
        )

    @property
    def is_steering_out_of_lookahead_cone(self) -> bool:
        left_to_curr_skew = abs(
            calculate_turn_angle(self.leftsafe_bearing, self.steering_angle)
        )
        right_to_curr_skew = abs(
            calculate_turn_angle(self.rightsafe_bearing, self.steering_angle)
        )

        return (
            left_to_curr_skew > self.left_right_skew
            or right_to_curr_skew > self.left_right_skew
        )

    @property
    def heading_z_score(self) -> float:
        sigma = 10
        reward = pow(math.e, -((self.racetrack_skew**2) / (2 * (sigma**2))))
        return reward

    @property
    def distance_z_score(self) -> float:
        # TODO: clean-up
        left_edge = self.closest_racepoint.left_safe
        right_edge = self.closest_racepoint.right_safe
        if is_point_approximately_between(
            self.current_position, left_edge, self.opt_position
        ):
            sigma = calculate_distance(left_edge, self.opt_position) / 3
        else:
            sigma = calculate_distance(self.opt_position, right_edge) / 3

        reward = pow(
            math.e,
            -(
                (calculate_distance(self.current_position, self.opt_position) ** 2)
                / (2 * (sigma**2))
            ),
        )

        return reward

    @property
    def speed_z_score(self) -> float:
        sigma_speed = 0.33
        score = pow(
            math.e, -((self.speed - self.opt_speed) ** 2) / (2 * (sigma_speed**2))
        )
        return score

    @property
    def straight_section_score(self) -> float:
        if self.previous_racepoint.is_in_straight_section:
            return (
                (self.speed_z_score**2)
                * (self.distance_z_score)
                * (self.heading_z_score**2)
            )
        return 0.0

    @property
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
        while prev:
            if prev.is_in_straight_section:
                bonus += prev.straight_section_score
                prev = prev.previous_step
            else:
                break
        return bonus

    @property
    def straight_section_bonus(self) -> float:
        if (
            self.previous_racepoint.is_in_straight_section
            and not self.next_racepoint.is_in_straight_section
            and not self.has_crashed_since_the_beginning_of_the_straight_section
        ):
            bonus = self._calculate_straight_section_bonus()
            return bonus
        return 0.0

    def _calculate_curved_section_bonus(self) -> float:
        bonus = self.curved_section_score
        prev = self.previous_step
        while prev:
            if prev.is_in_curved_section:
                bonus += prev.curved_section_score
                prev = prev.previous_step
            else:
                break
        return bonus

    @property
    def curved_section_bonus(self) -> float:
        if (
            self.previous_racepoint.is_in_curved_section
            and not self.next_racepoint.is_in_curved_section
            and not self.has_crashed_since_the_beginning_of_the_curved_section
        ):
            bonus = self._calculate_curved_section_bonus()
            return bonus
        return 0.0

    @property
    def track_completion_bonus(self) -> float:
        if self.is_lap_complete and self.has_crashed_since_the_beginning_of_the_lap:
            return 100.0
        return 0.0

    @property
    def took_unpardonable_action(self) -> bool:
        if (
            self.is_offtrack
            or self.racetrack_skew > 30
            or self.is_headed_out_of_lookahead_cone
            or self.is_steering_out_of_lookahead_cone
            or (self.is_steering_left and self.desired_bearing < 0)
            or (self.is_steering_right and self.desired_bearing > 0)
            or (self.opt_speed - self.speed > 1)
        ):
            return True
        return False

    @property
    def immediate_reward(self) -> float:
        if self.took_unpardonable_action:
            return TINY_REWARD
        return (self.speed_z_score + self.distance_z_score + self.heading_z_score) ** 2

    @property
    def long_term_reward(self) -> float:
        return (
            self.straight_section_bonus
            + self.curved_section_bonus
            + self.track_completion_bonus
        )

    @property
    def total_step_reward(self) -> float:
        return self.immediate_reward + self.long_term_reward
