#
# DeepRacer Framework
#
# Version 1.2.0
#
# Copyright (c) 2021 dmh23
#

import math


# -------------------------------------------------------------------------------
#
# CONSTANTS
#
# -------------------------------------------------------------------------------


class ParamNames:
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
    X = "x"
    Y = "y"
    CLOSEST_OBJECTS = "closest_objects"
    OBJECTS_DISTANCE = "objects_distance"
    OBJECTS_DISTANCE_FROM_CENTER = "objects_distance_from_center"
    OBJECTS_HEADING = "objects_heading"
    OBJECTS_LEFT_OF_CENTER = "objects_left_of_center"
    OBJECTS_LOCATION = "objects_location"
    OBJECTS_SPEED = "objects_speed"
    OBJECT_IN_CAMERA = "object_in_camera"


class RealWorld:
    STEPS_PER_SECOND = 15

    VEHICLE_LENGTH = 0.365
    VEHICLE_WIDTH = 0.225

    BOX_OBSTACLE_WIDTH = 0.38
    BOX_OBSTACLE_LENGTH = 0.24

    MAX_SPEEDS = [
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
    ]

    SAFE_CAR_OVERHANG = min(VEHICLE_LENGTH, VEHICLE_WIDTH) / 2


# -------------------------------------------------------------------------------
#
# GEOMETRY
#
# -------------------------------------------------------------------------------


def get_distance_between_points(first, second):
    (x1, y1) = first
    (x2, y2) = second

    x_diff = x2 - x1
    y_diff = y2 - y1

    return math.sqrt(x_diff * x_diff + y_diff * y_diff)


def get_bearing_between_points(start, finish):
    (start_x, start_y) = start
    (finish_x, finish_y) = finish

    direction_in_radians = math.atan2(finish_y - start_y, finish_x - start_x)
    return math.degrees(direction_in_radians)


def get_angle_in_proper_range(angle):
    if angle >= 180:
        return angle - 360
    elif angle <= -180:
        return 360 + angle
    else:
        return angle


def get_turn_between_directions(current, required):
    difference = required - current
    return get_angle_in_proper_range(difference)


def is_point_between(point, start, finish):
    bearing_from_start = get_bearing_between_points(start, point)
    bearing_to_finish = get_bearing_between_points(point, finish)
    return abs(get_turn_between_directions(bearing_from_start, bearing_to_finish)) < 1


def get_point_at_bearing(start_point, bearing: float, distance: float):
    (x, y) = start_point

    radians_to_target = math.radians(bearing)

    x2 = x + math.cos(radians_to_target) * distance
    y2 = y + math.sin(radians_to_target) * distance

    return x2, y2


# Intersection of two lines comes from Wikipedia
# https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line


def get_intersection_of_two_lines(
    line_a_point_1, line_a_point_2, line_b_point_1, line_b_point_2
):
    (x1, y1) = line_a_point_1
    (x2, y2) = line_a_point_2
    (x3, y3) = line_b_point_1
    (x4, y4) = line_b_point_2

    denominator = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4))

    if denominator == 0.0:
        return None

    z1 = (x1 * y2) - (y1 * x2)
    z2 = (x3 * y4) - (y3 * x4)

    x = ((z1 * (x3 - x4)) - ((x1 - x2) * z2)) / denominator
    y = ((z1 * (y3 - y4)) - ((y1 - y2) * z2)) / denominator

    return x, y


# -------------------------------------------------------------------------------
#
# WAYPOINT INFO CACHE
#
# -------------------------------------------------------------------------------


def get_edge_point(previous, mid, future, direction_offset: int, distance: float):
    assert direction_offset in [90, -90]
    assert previous != mid

    (previous_x, previous_y) = previous
    (mid_x, mid_y) = mid
    (next_x, next_y) = future

    degrees_to_mid_point = math.degrees(
        math.atan2(mid_y - previous_y, mid_x - previous_x)
    )
    if mid == future:
        track_heading_degrees = degrees_to_mid_point
    else:
        degrees_from_mid_point = math.degrees(
            math.atan2(next_y - mid_y, next_x - mid_x)
        )
        degrees_difference = get_turn_between_directions(
            degrees_to_mid_point, degrees_from_mid_point
        )
        track_heading_degrees = degrees_to_mid_point + degrees_difference / 2

    radians_to_edge_point = math.radians(track_heading_degrees + direction_offset)

    x = mid_x + math.cos(radians_to_edge_point) * distance
    y = mid_y + math.sin(radians_to_edge_point) * distance

    return x, y


class ProcessedWaypoint:
    def __init__(self, point, left_safe, right_safe):
        (self.x, self.y) = point
        self.left_safe = left_safe
        self.right_safe = right_safe


def get_processed_waypoints(waypoints, track_width):
    if waypoints[0] == waypoints[-1]:
        previous = waypoints[-2]
    else:
        previous = waypoints[-1]

    left_safe = previous
    right_safe = previous

    edge_error_tolerance = 0.01

    processed_waypoints = []

    for i, w in enumerate(waypoints):
        # Tracks often contain a repeated waypoint, suspect this is deliberate to mess up waypoint algorithms!
        if previous != w:
            if i < len(waypoints) - 1:
                future = waypoints[i + 1]
            else:
                future = waypoints[0]

            previous_left = left_safe
            previous_right = right_safe

            left_safe = get_edge_point(
                previous, w, future, 90, track_width / 2 + RealWorld.SAFE_CAR_OVERHANG
            )
            if (
                get_distance_between_points(previous_left, left_safe)
                < edge_error_tolerance
            ):
                left_safe = previous_left

            right_safe = get_edge_point(
                previous, w, future, -90, track_width / 2 + RealWorld.SAFE_CAR_OVERHANG
            )
            if (
                get_distance_between_points(previous_right, right_safe)
                < edge_error_tolerance
            ):
                right_safe = previous_right

            previous = w

        processed_waypoints.append(ProcessedWaypoint(w, left_safe, right_safe))

    return processed_waypoints


# -------------------------------------------------------------------------------
#
# REMEMBER A PREVIOUS STEP IN THIS EPISODE
#
# -------------------------------------------------------------------------------


class _HistoricStep:  # CHANGE: HistoricStep -> _HistoricStep
    def __init__(self, framework, previous_step):
        self.x = framework.x
        self.y = framework.y
        self.progress = framework.progress
        self.action_speed = framework.action_speed
        self.action_steering_angle = framework.action_steering_angle
        self.closest_waypoint_id = framework.closest_waypoint_id
        self.next_waypoint_id = framework.next_waypoint_id
        self.slide = framework.slide

        if previous_step:
            self.distance = get_distance_between_points(
                (previous_step.x, previous_step.y), (self.x, self.y)
            )
        else:
            self.distance = 0.0  # Causes issues if we use: framework.progress / 100 * framework.track_length


# -------------------------------------------------------------------------------
#
# FRAMEWORK
#
# -------------------------------------------------------------------------------


class _Framework:  # CHANGE: Framework -> _Framework
    def __init__(self, params):
        # Real PRIVATE variables set here
        self.processed_waypoints = get_processed_waypoints(
            track_original, params[ParamNames.TRACK_WIDTH]
        )
        self._history = []
        self._previous_front_object = -1

        # Definitions only of variables to use in your reward method, real values are set during process_params()
        self.x = 0.0
        self.y = 0.0
        self.start_waypoint_id = 0
        self.all_wheels_on_track = True
        self.previous_waypoint_id = 0
        self.previous_waypoint_x = 0.0
        self.previous_waypoint_y = 0.0
        self.next_waypoint_id = 0
        self.next_waypoint_x = 0.0
        self.next_waypoint_y = 0.0
        self.closest_waypoint_id = 0
        self.closest_waypoint_x = 0.0
        self.closest_waypoint_y = 0.0
        self.distance_from_closest_waypoint = 0.0
        self.distance_from_center = 0.0
        self.distance_from_edge = 0.0
        self.distance_from_extreme_edge = 0.0
        self.is_left_of_center = False
        self.is_right_of_center = False
        self.is_crashed = False
        self.is_off_track = False
        self.is_reversed = False
        self.is_complete_lap = False
        self.steps = 0
        self.time = 0.0
        self.is_final_step = False
        self.progress = 0.0
        self.predicted_lap_time = 0.0
        self.waypoints = []
        self.track_length = 0.0
        self.track_width = 0.0
        self.track_speed = 0.0
        self.progress_speed = 0.0
        # self.progress_speeds = []
        self.action_speed = 0.0
        self.action_steering_angle = 0.0
        self.action_sequence_length = 0
        self.is_steering_left = False
        self.is_steering_right = False
        self.is_steering_straight = False
        self.heading = 0.0
        self.track_bearing = 0.0
        self.true_bearing = 0.0
        self.slide = 0.0
        self.skew = 0.0
        self.max_slide = 0.0
        self.recent_max_slide = 0.0
        self.max_skew = 0.0
        self.total_distance = 0.0
        self.objects_location = []
        self.just_passed_waypoint_ids = []
        self.time_at_waypoint = []
        self.projected_distance = 0.0
        self.projected_progress_distance = 0.0
        self.projected_finish_left = False
        self.max_possible_track_speed = 0.0
        self.corner_cutting = 0.0

        # New stuff for OA ################################
        self.has_objects = False
        self.step_when_passed_object = [-1] * 20
        self.front_object_id = None
        self.rear_object_id = None
        self.distance_to_front_object = None
        self.distance_to_rear_object = None
        self.front_object_is_left_of_centre = False
        self.rear_object_is_left_of_centre = False
        self.projected_hit_object = False

    def process_params(self, params):
        self.x = float(params[ParamNames.X])
        self.y = float(params[ParamNames.Y])

        self.all_wheels_on_track = bool(params[ParamNames.ALL_WHEELS_ON_TRACK])

        self.previous_waypoint_id = int(params[ParamNames.CLOSEST_WAYPOINTS][0])
        self.previous_waypoint_x, self.previous_waypoint_y = params[
            ParamNames.WAYPOINTS
        ][self.previous_waypoint_id]
        self.next_waypoint_id = int(params[ParamNames.CLOSEST_WAYPOINTS][1])
        self.next_waypoint_x, self.next_waypoint_y = params[ParamNames.WAYPOINTS][
            self.next_waypoint_id
        ]

        distance_to_previous_waypoint = get_distance_between_points(
            (self.x, self.y), params[ParamNames.WAYPOINTS][self.previous_waypoint_id]
        )
        distance_to_next_waypoint = get_distance_between_points(
            (self.x, self.y), params[ParamNames.WAYPOINTS][self.next_waypoint_id]
        )
        if distance_to_previous_waypoint < distance_to_next_waypoint:
            self.closest_waypoint_id = self.previous_waypoint_id
            self.closest_waypoint_x = self.previous_waypoint_x
            self.closest_waypoint_y = self.previous_waypoint_y
            self.distance_from_closest_waypoint = distance_to_previous_waypoint
        else:
            self.closest_waypoint_id = self.next_waypoint_id
            self.closest_waypoint_x = self.next_waypoint_x
            self.closest_waypoint_y = self.next_waypoint_y
            self.distance_from_closest_waypoint = distance_to_next_waypoint

        self.distance_from_center = float(params[ParamNames.DISTANCE_FROM_CENTER])
        self.distance_from_edge = float(
            max(0.0, params[ParamNames.TRACK_WIDTH] / 2 - self.distance_from_center)
        )
        self.distance_from_extreme_edge = float(
            max(
                0.0,
                (params[ParamNames.TRACK_WIDTH] + RealWorld.VEHICLE_WIDTH) / 2
                - self.distance_from_center,
            )
        )

        self.is_left_of_center = bool(params[ParamNames.IS_LEFT_OF_CENTER])
        self.is_right_of_center = not self.is_left_of_center

        self.is_crashed = bool(params[ParamNames.IS_CRASHED])
        self.is_off_track = bool(params[ParamNames.IS_OFFTRACK])
        self.is_reversed = bool(params[ParamNames.IS_REVERSED])

        self.steps = int(round(params[ParamNames.STEPS]))
        self.time = self.steps / RealWorld.STEPS_PER_SECOND
        self.progress = float(params[ParamNames.PROGRESS])
        self.is_complete_lap = self.progress == 100.0
        self.is_final_step = (
            self.is_complete_lap
            or self.is_crashed
            or self.is_off_track
            or self.is_reversed
        )
        if self.progress > 0:
            self.predicted_lap_time = round(
                100 / self.progress * self.steps / RealWorld.STEPS_PER_SECOND, 2
            )
        else:
            self.predicted_lap_time = 0.0

        self.waypoints = params[ParamNames.WAYPOINTS]
        self.track_length = params[ParamNames.TRACK_LENGTH]
        self.track_width = params[ParamNames.TRACK_WIDTH]

        self.action_speed = params[ParamNames.SPEED]
        self.action_steering_angle = params[ParamNames.STEERING_ANGLE]

        self.is_steering_straight = abs(self.action_steering_angle) < 0.01
        self.is_steering_left = (
            self.action_steering_angle > 0 and not self.is_steering_straight
        )
        self.is_steering_right = (
            self.action_steering_angle < 0 and not self.is_steering_straight
        )

        self.heading = params[ParamNames.HEADING]
        self.track_bearing = get_bearing_between_points(
            (self.previous_waypoint_x, self.previous_waypoint_y),
            (self.next_waypoint_x, self.next_waypoint_y),
        )

        self.max_possible_track_speed = RealWorld.MAX_SPEEDS[
            min(self.steps, len(RealWorld.MAX_SPEEDS) - 1)
        ]

        self.objects_location = params[ParamNames.OBJECTS_LOCATION]

        #
        # Print object info for use by DRG
        #

        # Not step 1 because there's still a bug (?!) that means the reward function is not called until step 2!!!
        if self.steps == 2 and len(self.objects_location) > 0:
            print("DRG-OBJECTS:", self.objects_location)

        #
        # Record history
        #

        if self.steps <= 2:
            self._history = []
            self.time_at_waypoint = [None] * len(self.waypoints)
            self.step_when_passed_object = [-1] * 20
            self._previous_front_object = -1

        if self._history:
            previous_step = self._history[-1]
        else:
            previous_step = None

        this_step = _HistoricStep(self, previous_step)
        self._history.append(this_step)

        #
        # Calculations that use the history
        #

        if previous_step:
            if (
                previous_step.x != self.x or previous_step.y != self.y
            ):  # Otherwise, keep existing true_bearing
                if self.progress - previous_step.progress >= 0.05:
                    self.true_bearing = get_bearing_between_points(
                        (previous_step.x, previous_step.y), (self.x, self.y)
                    )
            if (
                previous_step.action_speed == self.action_speed
                and previous_step.action_steering_angle == self.action_steering_angle
            ):
                self.action_sequence_length += 1
            else:
                self.action_sequence_length = 1

            speed_calculate_steps = self._history[-6:]
            speed_calculate_distance = sum(s.distance for s in speed_calculate_steps)
            speed_calculate_time = (
                len(speed_calculate_steps) / RealWorld.STEPS_PER_SECOND
            )
            self.track_speed = speed_calculate_distance / speed_calculate_time

            progress_speed_distance = (
                (self.progress - speed_calculate_steps[0].progress)
                / 100
                * self.track_length
            )
            progress_speed_calculate_time = (
                len(speed_calculate_steps) - 1
            ) / RealWorld.STEPS_PER_SECOND
            self.progress_speed = max(
                0.0, progress_speed_distance / progress_speed_calculate_time
            )

            self.just_passed_waypoint_ids = self._get_just_passed_waypoint_ids(
                previous_step.next_waypoint_id, self.next_waypoint_id
            )

            progress_gain = self.progress - previous_step.progress
            if progress_gain < 0:
                self.corner_cutting = 0
            elif this_step.distance == 0:
                self.corner_cutting = 1
            else:
                progress_distance = progress_gain / 100 * self.track_length
                self.corner_cutting = progress_distance / this_step.distance

            self.recent_max_slide = 0.0
            for h in self._history[-4:]:
                if abs(h.slide) > abs(self.recent_max_slide):
                    self.recent_max_slide = h.slide

        else:
            self.action_sequence_length = 1
            self.true_bearing = self.heading
            self.progress_speed = 0.0
            self.track_speed = 0.0
            self.total_distance = 0.0
            self.max_skew = 0.0
            self.max_slide = 0.0
            self.recent_max_slide = 0.0
            self.just_passed_waypoint_ids = []
            self.start_waypoint_id = self.closest_waypoint_id
            if (
                len(self.time_at_waypoint) > self.start_waypoint_id
            ):  # FUDGE TO PASS VALIDATION IN CONSOLE
                self.time_at_waypoint[self.start_waypoint_id] = self.time
            self.corner_cutting = 1

        self.slide = get_turn_between_directions(self.heading, self.true_bearing)
        self.skew = get_turn_between_directions(self.track_bearing, self.true_bearing)
        self.total_distance += this_step.distance

        if abs(self.slide) > abs(self.max_slide):
            self.max_slide = self.slide
        if abs(self.skew) > abs(self.max_skew):
            self.max_skew = self.skew

        #
        # Object Avoidance Calculations
        #

        object_locations = params[ParamNames.OBJECTS_LOCATION]
        objects_left_of_center = params[ParamNames.OBJECTS_LEFT_OF_CENTER]
        closest_objects = params[ParamNames.CLOSEST_OBJECTS]

        self.has_objects = len(object_locations) > 0
        if self.has_objects:
            self.front_object_id = int(closest_objects[1])
            self.rear_object_id = int(closest_objects[0])

            if self.rear_object_id == self._previous_front_object:
                self.step_when_passed_object[self.rear_object_id] = self.steps

            self._previous_front_object = self.front_object_id

            self.distance_to_front_object = get_distance_between_points(
                (self.x, self.y), object_locations[self.front_object_id]
            )
            self.distance_to_rear_object = get_distance_between_points(
                (self.x, self.y), object_locations[self.rear_object_id]
            )

            self.front_object_is_left_of_centre = objects_left_of_center[
                self.front_object_id
            ]
            self.rear_object_is_left_of_centre = objects_left_of_center[
                self.rear_object_id
            ]

        else:
            self.front_object_id = None
            self.rear_object_id = None
            self.distance_to_front_object = None
            self.distance_to_rear_object = None
            self.front_object_is_left_of_centre = False
            self.rear_object_is_left_of_centre = None

        #
        # Projected distance calculation
        #

        self.projected_hit_object = False
        (
            self.projected_distance,
            self.projected_progress_distance,
            self.projected_finish_left,
        ) = self._calculate_projected_distance_on_track()
        if self.has_objects:
            object_hit_distance = self._calculate_object_hit_distance(
                object_locations[self.front_object_id]
            )
            if (
                object_hit_distance is not None
                and object_hit_distance < self.projected_distance
            ):
                self.projected_distance = object_hit_distance
                self.projected_hit_object = True
            elif len(object_locations) > 1:
                second_object_id = self.front_object_id + 1
                if second_object_id == len(object_locations):
                    second_object_id = 0
                second_object_hit_distance = self._calculate_object_hit_distance(
                    object_locations[second_object_id]
                )
                if (
                    second_object_hit_distance is not None
                    and second_object_hit_distance < self.projected_distance
                ):
                    self.projected_distance = second_object_hit_distance

    def _calculate_projected_distance_on_track(self):
        heading = get_angle_in_proper_range(self.true_bearing)
        point = (self.x, self.y)

        previous_left = self.processed_waypoints[self.previous_waypoint_id].left_safe
        previous_right = self.processed_waypoints[self.previous_waypoint_id].right_safe

        (
            previous_progress_distance,
            next_progress_distance,
        ) = self._calculate_progress_distances(
            point,
            self.waypoints[self.previous_waypoint_id],
            self.waypoints[self.next_waypoint_id],
            self.is_left_of_center,
            self.distance_from_center,
        )
        progress_distance = 0.0
        is_first_step = True

        previous_waypoint = self.waypoints[self.previous_waypoint_id]
        for w in (
            self.processed_waypoints[self.next_waypoint_id :]
            + self.processed_waypoints[: self.next_waypoint_id]
        ):
            (
                off_track_distance,
                off_track_point,
                off_left,
            ) = self._get_off_track_distance_and_point(
                point, heading, previous_left, previous_right, w
            )

            if off_track_distance is None:
                previous_left = w.left_safe
                previous_right = w.right_safe
                if is_first_step:
                    is_first_step = False
                    progress_distance = next_progress_distance
                else:
                    progress_distance += get_distance_between_points(
                        (w.x, w.y), previous_waypoint
                    )
                previous_waypoint = (w.x, w.y)
            elif off_track_distance == 0.0:
                return 0.0, 0.0, False
            else:
                (
                    final_previous_progress_distance,
                    final_next_progress_distance,
                ) = self._calculate_progress_distances(
                    off_track_point,
                    previous_waypoint,
                    (w.x, w.y),
                    off_left,
                    self.track_width / 2 + RealWorld.SAFE_CAR_OVERHANG,
                )

                if is_first_step:
                    progress_distance = (
                        next_progress_distance - final_next_progress_distance
                    )
                else:
                    progress_distance += final_previous_progress_distance
                return off_track_distance, progress_distance, off_left

    @staticmethod
    def _get_off_track_distance_and_point(
        point, heading: float, previous_left, previous_right, processed_waypoint
    ):
        left_safe = processed_waypoint.left_safe
        right_safe = processed_waypoint.right_safe

        direction_to_left_target = get_bearing_between_points(point, left_safe)
        direction_to_right_target = get_bearing_between_points(point, right_safe)

        relative_direction_to_left_target = get_turn_between_directions(
            heading, direction_to_left_target
        )
        relative_direction_to_right_target = get_turn_between_directions(
            heading, direction_to_right_target
        )

        if relative_direction_to_left_target >= 0 >= relative_direction_to_right_target:
            return None, None, None
        else:
            point2 = get_point_at_bearing(
                point, heading, 1
            )  # Just some random distance (1m)
            if left_safe == previous_left:
                off_track_left = previous_left
            else:
                off_track_left = get_intersection_of_two_lines(
                    point, point2, left_safe, previous_left
                )
            if right_safe == previous_right:
                off_track_right = previous_right
            else:
                off_track_right = get_intersection_of_two_lines(
                    point, point2, right_safe, previous_right
                )

            left_bearing = get_bearing_between_points(point, off_track_left)
            right_bearing = get_bearing_between_points(point, off_track_right)

            distances = []
            end_points = []
            off_left = []
            if abs(get_turn_between_directions(left_bearing, heading)) < 1:
                if is_point_between(off_track_left, left_safe, previous_left):
                    distances += [get_distance_between_points(point, off_track_left)]
                    end_points += [off_track_left]
                    off_left += [True]
            if abs(get_turn_between_directions(right_bearing, heading)) < 1:
                if is_point_between(off_track_right, right_safe, previous_right):
                    distances += [get_distance_between_points(point, off_track_right)]
                    end_points += [off_track_right]
                    off_left += [False]

            if len(distances) == 2 and distances[1] > distances[0]:
                return distances[1], end_points[1], off_left[1]
            elif len(distances) > 0:
                return distances[0], end_points[0], off_left[0]
            else:
                return 0.0, None, None

    @staticmethod
    def _calculate_progress_distances(
        point, previous_waypoint, next_waypoint, is_left, distance_from_centre
    ):
        track_bearing = get_bearing_between_points(previous_waypoint, next_waypoint)

        if is_left:
            offset = -90
        else:
            offset = 90

        radians_to_centre_point = math.radians(track_bearing + offset)

        (x, y) = point
        centre_point = (
            x + math.cos(radians_to_centre_point) * distance_from_centre,
            y + math.sin(radians_to_centre_point) * distance_from_centre,
        )

        return get_distance_between_points(
            centre_point, previous_waypoint
        ), get_distance_between_points(centre_point, next_waypoint)

    def _calculate_object_hit_distance(self, obj_middle):
        heading = get_angle_in_proper_range(self.true_bearing)
        point = (self.x, self.y)

        point2 = get_point_at_bearing(
            point, heading, 1
        )  # Just some random distance (1m) to define line
        track_bearing = self._get_track_bearing_at_point(obj_middle)
        safe_border = (
            min(RealWorld.VEHICLE_WIDTH, RealWorld.VEHICLE_LENGTH) / 3
        )  # Effectively enlarge the box

        front_middle = get_point_at_bearing(
            obj_middle, track_bearing, RealWorld.BOX_OBSTACLE_LENGTH / 2 + safe_border
        )
        front_left = get_point_at_bearing(
            front_middle,
            track_bearing + 90,
            RealWorld.BOX_OBSTACLE_WIDTH / 2 + safe_border,
        )
        front_right = get_point_at_bearing(
            front_middle,
            track_bearing - 90,
            RealWorld.BOX_OBSTACLE_WIDTH / 2 + safe_border,
        )

        rear_middle = get_point_at_bearing(
            obj_middle, track_bearing, -RealWorld.BOX_OBSTACLE_LENGTH / 2 - safe_border
        )
        rear_left = get_point_at_bearing(
            rear_middle,
            track_bearing + 90,
            RealWorld.BOX_OBSTACLE_WIDTH / 2 + safe_border,
        )
        rear_right = get_point_at_bearing(
            rear_middle,
            track_bearing - 90,
            RealWorld.BOX_OBSTACLE_WIDTH / 2 + safe_border,
        )

        distances = []
        for box_side in [
            (front_left, front_right),
            (rear_left, rear_right),
            (front_left, rear_left),
            (front_right, rear_right),
        ]:
            (box_point1, box_point2) = box_side
            hit_point = get_intersection_of_two_lines(
                point, point2, box_point1, box_point2
            )
            if hit_point is not None and is_point_between(
                hit_point, box_point1, box_point2
            ):
                # Make sure it's in front of us!
                bearing_to_hit_point = get_bearing_between_points(point, hit_point)
                if abs(get_turn_between_directions(bearing_to_hit_point, heading)) < 1:
                    distances.append(get_distance_between_points(point, hit_point))

        if not distances:
            return None
        else:
            return min(distances)

    def _get_track_bearing_at_point(self, point):
        closest_waypoint = self._get_closest_waypoint_id(point)
        (before_waypoint, after_waypoint) = self.get_waypoint_ids_before_and_after(
            point, closest_waypoint
        )
        return get_bearing_between_points(
            self.waypoints[before_waypoint], self.waypoints[after_waypoint]
        )

    def _get_closest_waypoint_id(self, point):
        distance = get_distance_between_points(self.waypoints[0], point)
        closest_id = 0
        for i, w in enumerate(self.waypoints[1:]):
            new_distance = get_distance_between_points(w, point)
            if new_distance < distance:
                distance = new_distance
                closest_id = i + 1
        return closest_id

    def get_waypoint_ids_before_and_after(
        self, point, closest_waypoint_id: int, prefer_forwards=False
    ):
        assert 0 <= closest_waypoint_id < len(self.waypoints)

        previous_id = self._get_previous_waypoint_id(closest_waypoint_id)
        next_id = self._get_next_waypoint_id(closest_waypoint_id)

        previous_waypoint = self.waypoints[previous_id]
        next_waypoint = self.waypoints[next_id]
        closest_waypoint = self.waypoints[closest_waypoint_id]

        target_dist = get_distance_between_points(closest_waypoint, previous_waypoint)
        if target_dist == 0.0:
            previous_ratio = 99999.0
        else:
            previous_ratio = (
                get_distance_between_points(point, previous_waypoint) / target_dist
            )

        target_dist = get_distance_between_points(closest_waypoint, next_waypoint)
        if target_dist == 0.0:
            next_ratio = 99999.0
        else:
            next_ratio = get_distance_between_points(point, next_waypoint) / target_dist

        if prefer_forwards:  # Make the behind waypoint appear 5% further away
            previous_ratio *= 1.05

        if previous_ratio > next_ratio:
            return closest_waypoint_id, next_id
        else:
            return previous_id, closest_waypoint_id

    def _get_next_waypoint_id(self, waypoint_id):
        if waypoint_id >= len(self.waypoints) - 1:
            return 0
        else:
            return waypoint_id + 1

    def _get_previous_waypoint_id(self, waypoint_id):
        if waypoint_id < 1:
            return len(self.waypoints) - 1
        else:
            return waypoint_id - 1

    def _get_just_passed_waypoint_ids(
        self, previous_next_waypoint_id, current_next_waypoint_id
    ):
        if previous_next_waypoint_id == current_next_waypoint_id:
            return []

        difference = current_next_waypoint_id - previous_next_waypoint_id

        if difference < -10 or 1 <= difference <= 10:
            result = []
            w = previous_next_waypoint_id
            while w != current_next_waypoint_id:
                if self.time_at_waypoint[w] is None:
                    result.append(w)
                    self.time_at_waypoint[w] = self.time
                w += 1
                if w >= len(self.waypoints):
                    w = 0

            return result
        else:
            return []

    def get_track_distance_between_waypoints(self, start: int, finish: int):
        distance = 0
        assert 0 <= start < len(self.waypoints)
        assert 0 <= finish < len(self.waypoints)

        while start != finish:
            next_wp = self._get_next_waypoint_id(start)
            distance += get_distance_between_points(
                self.waypoints[start], self.waypoints[next_wp]
            )
            start = next_wp

        return distance

    def get_progress_speed(self, steps: int):
        assert steps >= 1
        if steps >= len(self._history):
            return None

        progress_speed_distance = (
            (self.progress - self._history[-steps - 1].progress)
            / 100
            * self.track_length
        )
        progress_speed_calculate_time = steps / RealWorld.STEPS_PER_SECOND
        return max(0.0, progress_speed_distance / progress_speed_calculate_time)

    def print_debug(self):
        print("x, y                      ", round(self.x, 3), round(self.y, 3))
        print("all_wheels_on_track       ", self.all_wheels_on_track)
        print("previous_waypoint_id      ", self.previous_waypoint_id)
        print(
            "previous_waypoint_x, y    ",
            round(self.previous_waypoint_x, 3),
            round(self.previous_waypoint_y, 3),
        )
        print("next_waypoint_id          ", self.next_waypoint_id)
        print(
            "next_waypoint_x, y        ",
            round(self.next_waypoint_x, 3),
            round(self.next_waypoint_y, 3),
        )
        print("closest_waypoint_id       ", self.closest_waypoint_id)
        print(
            "closest_waypoint_x, y     ",
            round(self.closest_waypoint_x, 3),
            round(self.closest_waypoint_y, 3),
        )
        print(
            "distance_from_closest_waypoint ",
            round(self.distance_from_closest_waypoint, 2),
        )
        print("distance_from_center      ", round(self.distance_from_center, 2))
        print("distance_from_edge        ", round(self.distance_from_edge, 2))
        print(
            "distance_from_extreme_edge     ", round(self.distance_from_extreme_edge, 2)
        )
        print(
            "is_left/right_of_center   ",
            self.is_left_of_center,
            self.is_right_of_center,
        )
        print("is_crashed / reversed     ", self.is_crashed, self.is_reversed)
        print("is_off_track              ", self.is_off_track)
        print("is_complete_lap           ", self.is_complete_lap)
        print("steps, is_final_step      ", self.steps, self.is_final_step)
        print("time                      ", round(self.time, 2))
        print("predicted_lap_time        ", round(self.predicted_lap_time, 2))
        print("progress                  ", round(self.progress, 2))
        print("waypoints  (SIZE)         ", len(self.waypoints))
        print(
            "track_length, width       ",
            round(self.track_length, 2),
            round(self.track_width, 2),
        )
        print("action_speed              ", round(self.action_speed, 2))
        print("action_steering_angle     ", round(self.action_steering_angle, 1))
        print("action_sequence_length    ", self.action_sequence_length)
        print(
            "is_steering_left/right    ", self.is_steering_left, self.is_steering_right
        )
        print("is_steering_straight      ", self.is_steering_straight)
        print("heading                   ", round(self.heading, 2))
        print("track_bearing             ", round(self.track_bearing, 2))
        print("true_bearing              ", round(self.true_bearing, 2))
        print(
            "slide  / max / recent     ",
            round(self.slide, 2),
            round(self.max_slide, 2),
            round(self.recent_max_slide, 2),
        )
        print(
            "skew / max_skew           ", round(self.skew, 2), round(self.max_skew, 2)
        )
        print("total_distance            ", round(self.total_distance, 2))
        print("track_speed               ", round(self.track_speed, 2))
        print("progress_speed            ", round(self.progress_speed, 2))
        print("just_passed_waypoint_ids  ", self.just_passed_waypoint_ids)
        print("time_at_waypoint          ", self.time_at_waypoint)
        print("projected_distance        ", self.projected_distance)


# -------------------------------------------------------------------------------
#
# Generate optimal raceline and calculate corresponding speeds
#
# -------------------------------------------------------------------------------


racepoints = [[-2.03954, -5.95967, 4.0, 1.0, 0.0],
 [-1.74085, -6.00654, 4.0, 1.0, 0.0],
 [-1.44222, -6.05378, 4.0, 1.0, 0.0],
 [-1.14367, -6.10147, 4.0, 1.0, 0.0],
 [-0.8452, -6.14972, 4.0, 1.0, 0.0],
 [-0.54684, -6.19862, 4.0, 1.0, 0.0],
 [-0.24862, -6.2483, 4.0, 1.0, 0.0],
 [0.04771, -6.29512, 4.0, 1.0, 0.0],
 [0.34325, -6.33652, 4.0, 1.0, 0.0],
 [0.63767, -6.36976, 3.64089, 1.0, 0.0],
 [0.93057, -6.3923, 3.25251, 0.0, 0.0],
 [1.22148, -6.40177, 2.96657, 0.0, 1.0],
 [1.50978, -6.39595, 2.76198, 0.0, 1.0],
 [1.79468, -6.37297, 2.59506, 0.0, 1.0],
 [2.07524, -6.33109, 2.47074, 0.0, 1.0],
 [2.35027, -6.26889, 2.37363, 0.0, 1.0],
 [2.61834, -6.18525, 2.29829, 0.0, 1.0],
 [2.87773, -6.07939, 2.23736, 0.0, 1.0],
 [3.12635, -5.95087, 2.2035, 0.0, 1.0],
 [3.36201, -5.80001, 2.18018, 0.0, 1.0],
 [3.58242, -5.62763, 2.1786, 0.0, 1.0],
 [3.78554, -5.43528, 2.19257, 0.0, 1.0],
 [3.96979, -5.22496, 2.22086, 0.0, 1.0],
 [4.13414, -4.99889, 2.25966, 0.0, 1.0],
 [4.27805, -4.75932, 2.31776, 0.0, 1.0],
 [4.4016, -4.50843, 2.39671, 0.0, 1.0],
 [4.50539, -4.24821, 2.48671, 0.0, 1.0],
 [4.59025, -3.98036, 2.60247, 0.0, 1.0],
 [4.65739, -3.70634, 2.73299, 0.0, 1.0],
 [4.70804, -3.42735, 2.90583, 0.0, 1.0],
 [4.74378, -3.14441, 3.10125, 0.0, 1.0],
 [4.76615, -2.85835, 3.33697, 0.0, 0.0],
 [4.77672, -2.56987, 3.63232, 1.0, 0.0],
 [4.77716, -2.27956, 3.98568, 1.0, 0.0],
 [4.78548, -2.00292, 3.5566, 0.0, 0.0],
 [4.80318, -1.73053, 3.23334, 0.0, 0.0],
 [4.83172, -1.46326, 2.98331, 0.0, 1.0],
 [4.87238, -1.20195, 2.78299, 0.0, 1.0],
 [4.92625, -0.94742, 2.61732, 0.0, 1.0],
 [4.99432, -0.70047, 2.47446, 0.0, 1.0],
 [5.0775, -0.46194, 2.33357, 0.0, 1.0],
 [5.17689, -0.23288, 2.32145, 0.0, 1.0],
 [5.29207, -0.01343, 2.30628, 0.0, 1.0],
 [5.42284, 0.19616, 2.29624, 0.0, 1.0],
 [5.56918, 0.39563, 2.28477, 0.0, 1.0],
 [5.7313, 0.58459, 2.27949, 0.0, 1.0],
 [5.90973, 0.76269, 2.27847, 0.0, 1.0],
 [6.10555, 0.92959, 2.27739, 0.0, 1.0],
 [6.32137, 1.08531, 2.27384, 0.0, 1.0],
 [6.56538, 1.23149, 2.26136, 0.0, 1.0],
 [6.79496, 1.39844, 2.20268, 0.0, 1.0],
 [7.00401, 1.58238, 2.13998, 0.0, 1.0],
 [7.19118, 1.78225, 2.08921, 0.0, 1.0],
 [7.35523, 1.99682, 2.05263, 0.0, 1.0],
 [7.49512, 2.22468, 2.02362, 0.0, 1.0],
 [7.60981, 2.46436, 2.00748, 0.0, 1.0],
 [7.69841, 2.71418, 2.0, 0.0, 1.0],
 [7.7601, 2.97241, 2.00082, 0.0, 1.0],
 [7.79417, 3.23713, 2.00955, 0.0, 1.0],
 [7.8, 3.50632, 2.01848, 0.0, 1.0],
 [7.77691, 3.77777, 2.03921, 0.0, 1.0],
 [7.72458, 4.04902, 2.06796, 0.0, 1.0],
 [7.64304, 4.31746, 2.1056, 0.0, 1.0],
 [7.53282, 4.58031, 2.14425, 0.0, 1.0],
 [7.39476, 4.83474, 2.18961, 0.0, 1.0],
 [7.23036, 5.07787, 2.24363, 0.0, 1.0],
 [7.04181, 5.30702, 2.30075, 0.0, 1.0],
 [6.83197, 5.51974, 2.36299, 0.0, 1.0],
 [6.6042, 5.71409, 2.42325, 0.0, 1.0],
 [6.36167, 5.88898, 2.48615, 0.0, 1.0],
 [6.1069, 6.04425, 2.55023, 0.0, 1.0],
 [5.84181, 6.18022, 2.61004, 0.0, 1.0],
 [5.56792, 6.29727, 2.62161, 0.0, 1.0],
 [5.28634, 6.39511, 2.62627, 0.0, 1.0],
 [4.99821, 6.47335, 2.6255, 0.0, 1.0],
 [4.70494, 6.53149, 2.62179, 0.0, 1.0],
 [4.40843, 6.56907, 2.62506, 0.0, 1.0],
 [4.11093, 6.58596, 2.62773, 0.0, 1.0],
 [3.81472, 6.58234, 2.64592, 0.0, 1.0],
 [3.52166, 6.55892, 2.66489, 0.0, 1.0],
 [3.23308, 6.51655, 2.70009, 0.0, 1.0],
 [2.9498, 6.45627, 2.74185, 0.0, 1.0],
 [2.67233, 6.37913, 2.81637, 0.0, 1.0],
 [2.40079, 6.28635, 2.9014, 0.0, 1.0],
 [2.13516, 6.17909, 3.01908, 0.0, 1.0],
 [1.87523, 6.05859, 3.1761, 0.0, 1.0],
 [1.62065, 5.92616, 3.38918, 0.0, 0.0],
 [1.3709, 5.78326, 3.66652, 1.0, 0.0],
 [1.12541, 5.63138, 4.0, 1.0, 0.0],
 [0.88349, 5.47212, 4.0, 1.0, 0.0],
 [0.64439, 5.30709, 4.0, 1.0, 0.0],
 [0.40732, 5.13795, 4.0, 1.0, 0.0],
 [0.1661, 4.96239, 4.0, 1.0, 0.0],
 [-0.07737, 4.79087, 4.0, 1.0, 0.0],
 [-0.32376, 4.62469, 4.0, 1.0, 0.0],
 [-0.57365, 4.46508, 4.0, 1.0, 0.0],
 [-0.82747, 4.31309, 3.82767, 1.0, 0.0],
 [-1.08555, 4.16966, 3.6689, 1.0, 0.0],
 [-1.34811, 4.03558, 3.56447, 0.0, 0.0],
 [-1.61522, 3.91146, 3.48316, 0.0, 0.0],
 [-1.88688, 3.79784, 3.42655, 0.0, 0.0],
 [-2.16302, 3.69518, 3.38554, 0.0, 0.0],
 [-2.4435, 3.60386, 3.36692, 0.0, 0.0],
 [-2.7281, 3.52416, 3.36405, 0.0, 0.0],
 [-3.01653, 3.45627, 3.35458, 0.0, 0.0],
 [-3.30849, 3.40045, 3.35684, 0.0, 0.0],
 [-3.60363, 3.35687, 3.34037, 0.0, 0.0],
 [-3.90152, 3.32587, 3.32082, 0.0, 0.0],
 [-4.20116, 3.30783, 3.13572, 0.0, 1.0],
 [-4.49467, 3.27581, 2.97409, 0.0, 1.0],
 [-4.78313, 3.22872, 2.84248, 0.0, 1.0],
 [-5.06569, 3.16568, 2.73682, 0.0, 1.0],
 [-5.34133, 3.08609, 2.64347, 0.0, 1.0],
 [-5.60901, 2.9894, 2.56472, 0.0, 1.0],
 [-5.86766, 2.87522, 2.51725, 0.0, 1.0],
 [-6.11631, 2.74355, 2.47434, 0.0, 1.0],
 [-6.35395, 2.5944, 2.44782, 0.0, 1.0],
 [-6.5796, 2.42796, 2.43128, 0.0, 1.0],
 [-6.79228, 2.24459, 2.4356, 0.0, 1.0],
 [-6.99115, 2.04488, 2.44331, 0.0, 1.0],
 [-7.17532, 1.82951, 2.46233, 0.0, 1.0],
 [-7.34399, 1.59931, 2.49026, 0.0, 1.0],
 [-7.49641, 1.35529, 2.52589, 0.0, 1.0],
 [-7.63193, 1.0986, 2.57916, 0.0, 1.0],
 [-7.75016, 0.83067, 2.63521, 0.0, 1.0],
 [-7.85081, 0.55298, 2.69593, 0.0, 1.0],
 [-7.93374, 0.26715, 2.76438, 0.0, 1.0],
 [-7.99904, -0.02516, 2.83953, 0.0, 1.0],
 [-8.04703, -0.32222, 2.91711, 0.0, 1.0],
 [-8.0782, -0.6224, 2.99655, 0.0, 1.0],
 [-8.09317, -0.92424, 3.06828, 0.0, 1.0],
 [-8.09261, -1.22658, 3.08583, 0.0, 1.0],
 [-8.07669, -1.52841, 2.9889, 0.0, 1.0],
 [-8.0445, -1.8286, 2.88866, 0.0, 1.0],
 [-7.99507, -2.12587, 2.7921, 0.0, 1.0],
 [-7.92756, -2.41871, 2.71816, 0.0, 1.0],
 [-7.84151, -2.70541, 2.64451, 0.0, 1.0],
 [-7.7366, -2.98412, 2.59632, 0.0, 1.0],
 [-7.61305, -3.25314, 2.5542, 0.0, 1.0],
 [-7.47125, -3.51087, 2.52433, 0.0, 1.0],
 [-7.31186, -3.75591, 2.51021, 0.0, 1.0],
 [-7.13577, -3.98716, 2.5009, 0.0, 1.0],
 [-6.94391, -4.20368, 2.51583, 0.0, 1.0],
 [-6.73744, -4.40495, 2.54018, 0.0, 1.0],
 [-6.51752, -4.59061, 2.57489, 0.0, 1.0],
 [-6.28527, -4.76051, 2.63141, 0.0, 1.0],
 [-6.04188, -4.9148, 2.70919, 0.0, 1.0],
 [-5.78855, -5.05385, 2.80576, 0.0, 1.0],
 [-5.52641, -5.17826, 2.92836, 0.0, 1.0],
 [-5.25657, -5.28878, 3.08693, 0.0, 1.0],
 [-4.98013, -5.38645, 3.26653, 0.0, 0.0],
 [-4.69806, -5.47231, 3.50902, 0.0, 0.0],
 [-4.41131, -5.54768, 3.8213, 1.0, 0.0],
 [-4.12078, -5.614, 4.0, 1.0, 0.0],
 [-3.82729, -5.67279, 4.0, 1.0, 0.0],
 [-3.53159, -5.72571, 4.0, 1.0, 0.0],
 [-3.2343, -5.7745, 4.0, 1.0, 0.0],
 [-2.93593, -5.82097, 4.0, 1.0, 0.0],
 [-2.63709, -5.8669, 4.0, 1.0, 0.0],
 [-2.33829, -5.91314, 4.0, 1.0, 0.0]]  # replace: raceline


# -------------------------------------------------------------------------------
#
# Framework overrides
#
# -------------------------------------------------------------------------------


class ProcessedRacepoint:
    def __init__(
        self,
        point,
        opt_speed,
        is_in_straight_section,
        is_in_curved_section,
    ) -> None:
        (self.x, self.y) = point
        self.opt_speed = opt_speed
        self.is_in_straight_section = is_in_straight_section
        self.is_in_curved_section = is_in_curved_section


def get_processed_racepoints(racepoints):
    previous = racepoints[-2] if racepoints[0] == racepoints[-1] else racepoints[-1]

    processed_racepoints = []
    for w in racepoints:
        if previous != w:
            previous = w

        processed_racepoints.append(ProcessedRacepoint(w[:2], w[2], w[3], w[4]))

    return processed_racepoints


class HistoricStep(_HistoricStep):
    def __init__(self, framework, previous_step):
        super().__init__(framework, previous_step)
        self.straight_section_score = framework.straight_section_score
        self.curved_section_score = framework.curved_section_score


class Framework(_Framework):
    def __init__(self, params):
        super().__init__(params)
        self.racepoints = get_processed_racepoints(racepoints)
        self.straight_section_score = 0
        self.curved_section_score = 0
        self.has_crashed_since_the_beginning_of_the_lap = False
        self.has_crashed_since_the_beginning_of_the_straight_section = False
        self.has_crashed_since_the_beginning_of_the_curved_section = False

    def process_params(self, params):
        super().process_params(params)
        self.current_position = (self.x, self.y)
        self.optimal_position = (self.closest_racepoint.x, self.closest_racepoint.y)
        self.closest_waypoint = self.processed_waypoints[self.closest_waypoint_id]
        self.closest_racepoint = self.racepoints[self.closest_waypoint_id]
        self.prev_racepoint = self.racepoints[self.previous_waypoint_id]
        self.next_waypoint = self.processed_waypoints[self.next_waypoint_id]
        self.next_racepoint = self.racepoints[self.next_waypoint_id]
        self.next_optimal_position = (self.next_racepoint.x, self.next_racepoint.y)
        self.opt_speed = self.closest_racepoint.opt_speed
        if self.steps <= 2:
            self.has_crashed_since_the_beginning_of_the_lap = False
        if not self.prev_racepoint.is_in_straight_section and self.next_racepoint.is_in_straight_section:
            self.has_crashed_since_the_beginning_of_the_straight_section = False
            self.straight_section_start_id = self.next_waypoint_id
        if not self.prev_racepoint.is_in_curved_section and self.next_racepoint.is_in_curved_section:
            self.has_crashed_since_the_beginning_of_the_curved_section = False
            self.curved_section_start_id = self.next_waypoint_id
        if self.is_crashed:
            self.has_crashed_since_the_beginning_of_the_lap = True
            self.has_crashed_since_the_beginning_of_the_straight_section = True
            self.has_crashed_since_the_beginning_of_the_curved_section = True
        if self.next_racepoint.is_in_straight_section:
            self.straight_section_score = self.speed_z_score * self.distance_z_score * self.heading_z_score
        if self.next_racepoint.is_in_curved_section:
            self.curved_section_score = self.speed_z_score * self.distance_z_score * self.heading_z_score
        self._history[-1].straight_section_score = self.straight_section_score
        self._history[-1].curved_section_score = self.curved_section_score
        if len(self._history) >= 1000: # avoid memory leaks
            self._history.pop(0)

    @property
    def racetrack_bearing(self):
        racetrack_bearing = 0.0
        if self.prev_racepoint != self.next_racepoint:
            racetrack_bearing = get_bearing_between_points(
                [self.prev_racepoint.x, self.prev_racepoint.y],
                [self.next_racepoint.x, self.next_racepoint.y],
            )
        return racetrack_bearing

    @property
    def leftsafe_bearing(self):
        leftsafe_bearing = get_bearing_between_points(
            self.current_position, self.next_waypoint.left_safe
        )
        return leftsafe_bearing

    @property
    def rightsafe_bearing(self):
        rightsafe_bearing = get_bearing_between_points(
            self.current_position, self.next_waypoint.right_safe
        )
        return rightsafe_bearing

    @property
    def racetrack_skew(self):
        racetrack_skew = get_turn_between_directions(
            self.racetrack_bearing, self.true_bearing
        )
        return racetrack_skew

    @property
    def distance_z_score(self):
        left_edge = self.closest_waypoint.left_safe
        right_edge = self.closest_waypoint.right_safe
        if is_point_between(self.current_position, left_edge, self.optimal_position):
            sigma = get_distance_between_points(left_edge, self.optimal_position) / 3
        else:
            sigma = get_distance_between_points(self.optimal_position, right_edge) / 3

        reward = pow(
            math.e,
            -(
                (
                    get_distance_between_points(
                        self.current_position, self.optimal_position
                    )
                    ** 2
                )
                / (2 * (sigma**2))
            ),
        )
        return reward

    @property
    def heading_z_score(self):
        sigma = 10
        reward = pow(
            math.e,
            -((self.racetrack_skew**2) / (2 * (sigma**2))),
        )
        return reward

    @property
    def speed_z_score(self):
        # opt speed should not vary too far from what's computed for the nearest points on the racepath
        behind = (self.closest_waypoint_id - 3)
        ahead = (self.closest_waypoint_id + 3) % len(self.processed_waypoints)
        speeds = self.racepoints[behind : ahead]
        mean_speed = sum(speeds) / len(speeds) 
        sigma_speed = sum([(speed - mean_speed)**2 for speed in speeds]) / (len(speeds) - 1)
        score = pow(
            math.e, ((self.action_speed - self.opt_speed) ** 2) / (2 * (sigma_speed**2))
        )
        return score

    @property
    def is_headed_out_of_lookahead_cone(self):
        left_right_skew = abs(
            get_turn_between_directions(self.leftsafe_bearing, self.rightsafe_bearing)
        )
        left_to_curr_skew = abs(
            get_turn_between_directions(self.leftsafe_bearing, self.heading)
        )
        right_to_curr_skew = abs(
            get_turn_between_directions(self.rightsafe_bearing, self.heading)
        )

        return (
            left_to_curr_skew > left_right_skew or right_to_curr_skew > left_right_skew
        )

    @property
    def is_steering_out_of_lookahead_cone(self):
        left_right_skew = abs(
            get_turn_between_directions(self.leftsafe_bearing, self.rightsafe_bearing)
        )
        left_to_curr_skew = abs(
            get_turn_between_directions(
                self.leftsafe_bearing, self.action_steering_angle
            )
        )
        right_to_curr_skew = abs(
            get_turn_between_directions(
                self.rightsafe_bearing, self.action_steering_angle
            )
        )

        return (
            left_to_curr_skew > left_right_skew or right_to_curr_skew > left_right_skew
        )


# -------------------------------------------------------------------------------
#
# REWARD FUNCTION MASTER WRAPPER
#
# -------------------------------------------------------------------------------


def reward_function(params):
    global framework_global
    if not framework_global:
        framework_global = Framework(params)
    framework_global.process_params(params)
    raw_reward = float(get_reward(framework_global))
    if raw_reward > 0:
        return raw_reward
    else:
        tiny_reward = 0.0001
        print(
            "WARNING - Invalid reward "
            + str(raw_reward)
            + " replaced with "
            + str(tiny_reward)
        )
        return tiny_reward


framework_global = None

# -------------------------------------------------------------------------------
#
# REWARD FUNCTION ... ... ... ...
# #REF: https://blog.gofynd.com/how-we-broke-into-the-top-1-of-the-aws-deepracer-virtual-circuit-573ba46c275
#
# -------------------------------------------------------------------------------


class Reward:
    def __init__(self, f: Framework) -> None:
        self.f = f
        self.speed_score = self.f.speed_z_score
        self.distance_score = self.f.distance_z_score
        self.heading_score = self.f.heading_z_score

    @property
    def track_completion_bonus(self):
        reward = 0
        if self.f.is_complete_lap and not self.f.has_crashed_since_the_beginning_of_the_lap:
            reward = 100
        return reward

    @property
    def straight_section_bonus(self):
        reward = 0.0
        if self.f.prev_racepoint.is_in_straight_section and not self.f.next_racepoint.is_in_straight_section \
                and not self.f.has_crashed_since_the_beginning_of_the_straight_section:
            start = self.f.straight_section_start_id
            end = self.f.next_waypoint_id
            steps = self.f._history[start:end]
            reward = sum(s.straight_section_score for s in steps)
        return reward

    @property
    def curved_section_bonus(self):
        reward = 0.0
        if self.f.prev_racepoint.is_in_curved_section and not self.f.next_racepoint.is_in_curved_section \
                and not self.f.has_crashed_since_the_beginning_of_the_curved_section:
            start = self.f.curved_section_start_id
            end = self.f.next_waypoint_id
            steps = self.f._history[start:end]
            reward = sum(s.curved_section_score for s in steps)
        return reward


def get_reward(f: Framework):
    r = Reward(f)

    # immediate component of the reward
    ic = (r.speed_score + r.distance_score + r.heading_score) ** 2

    # if an unpardonable action is taken, then the immediate reward is 0
    if (
        r.f.is_off_track
        or r.f.racetrack_skew > 30
        or r.f.is_headed_out_of_lookahead_cone
        or r.f.is_steering_out_of_lookahead_cone
        or (
            r.f.is_steering_left
            and get_bearing_between_points(r.f.current_position, r.f.next_racepoint) < 0
        )
        or (
            r.f.is_steering_right
            and get_bearing_between_points(r.f.current_position, r.f.next_racepoint) > 0
        )
        or (
            r.f.opt_speed - r.f.action_speed > 1
            and r.f.closest_racepoint.is_in_straight_section
        )
        or (
            r.f.action_speed - r.f.opt_speed < -1.5
            and r.f.closest_racepoint.is_in_curved_section
        )
    ):
        ic = 1e-3

    # long term component of the reward
    lc = r.track_completion_bonus + r.straight_section_bonus + r.curved_section_bonus

    return max(ic + lc, 1e-3)
