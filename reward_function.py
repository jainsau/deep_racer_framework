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

track_original = [
    (-2.0369359850883484, -5.943336009979248),
    (-1.738064467906952, -5.989463567733765),
    (-1.4393694996833801, -6.0367231369018555),
    (-1.1408880352973938, -6.085307598114014),
    (-0.8426246047019958, -6.135216951370239),
    (-0.5444198995828629, -6.185476541519165),
    (-0.2462899014353752, -6.236176490783691),
    (0.051524948328733444, -6.288690090179443),
    (0.3483338952064514, -6.346576452255249),
    (0.6436918079853058, -6.411493539810181),
    (0.9376374185085297, -6.482528448104858),
    (1.2304140329360962, -6.5582499504089355),
    (1.5257545113563538, -6.62269401550293),
    (1.8273234963417053, -6.6392905712127686),
    (2.1288928985595703, -6.618226051330566),
    (2.4260480403900146, -6.563058614730835),
    (2.7137099504470825, -6.470383882522583),
    (2.9867550134658813, -6.340842008590698),
    (3.2398234605789185, -6.175666332244873),
    (3.4722654819488525, -5.982369661331177),
    (3.693007469177246, -5.7756829261779785),
    (3.904446840286255, -5.559537410736084),
    (4.102475523948669, -5.331040382385254),
    (4.286980986595154, -5.091476917266846),
    (4.4685304164886475, -4.849627494812012),
    (4.6437458992004395, -4.603174448013306),
    (4.799426555633545, -4.344087600708008),
    (4.9133594036102295, -4.064393043518066),
    (4.9756760597229, -3.768771529197693),
    (4.990587472915649, -3.466975450515747),
    (4.966310501098633, -3.165669083595276),
    (4.918224096298218, -2.8671375513076782),
    (4.859284400939941, -2.570557475090027),
    (4.777156829833984, -2.279561996459961),
    (4.701304912567139, -1.9868600368499756),
    (4.636876106262207, -1.6913945078849792),
    (4.5772013664245605, -1.3949394822120667),
    (4.535637140274048, -1.09552800655365),
    (4.543611526489258, -0.7937583029270172),
    (4.607089042663574, -0.49833714962005615),
    (4.716185092926025, -0.21654874039813876),
    (4.864802122116089, 0.04658450186252594),
    (5.0469725131988525, 0.2877318002283573),
    (5.259093523025513, 0.5029039569199085),
    (5.501067399978638, 0.68390753865242),
    (5.7621095180511475, 0.8364308476448059),
    (6.030410528182983, 0.9759470522403717),
    (6.3061418533325195, 1.0999865233898163),
    (6.591607570648193, 1.1995515525341034),
    (6.881825923919678, 1.2845369279384613),
    (7.158332347869873, 1.4054960906505585),
    (7.401469945907593, 1.584732472896576),
    (7.61121392250061, 1.8022159934043884),
    (7.785490274429321, 2.0490845441818237),
    (7.923514366149902, 2.3179259300231934),
    (8.023163080215454, 2.6032029390335083),
    (8.080552339553833, 2.899960517883301),
    (8.094262838363647, 3.2018975019454956),
    (8.066569089889526, 3.5028750896453857),
    (8.001795530319214, 3.7981364727020264),
    (7.9055304527282715, 4.084710478782654),
    (7.7861008644104, 4.362487554550171),
    (7.640097379684448, 4.627154588699341),
    (7.465439081192017, 4.873879671096802),
    (7.268836498260499, 5.103580951690673),
    (7.060459852218628, 5.3227269649505615),
    (6.847865343093871, 5.5377869606018075),
    (6.625417470932007, 5.742587089538574),
    (6.38864803314209, 5.930606126785278),
    (6.136788606643677, 6.097854375839233),
    (5.870970010757446, 6.241891384124756),
    (5.5932300090789795, 6.361319065093994),
    (5.305620431900024, 6.454447984695435),
    (5.010941505432129, 6.522075414657593),
    (4.7127039432525635, 6.572042465209961),
    (4.412917375564575, 6.61158561706543),
    (4.111389875411987, 6.634212017059326),
    (3.8090630769729614, 6.639834403991699),
    (3.506811022758484, 6.630667686462402),
    (3.204898953437805, 6.6133339405059814),
    (2.9031169414520264, 6.593855381011963),
    (2.601729989051819, 6.569056510925293),
    (2.301701545715332, 6.531422138214111),
    (2.0059205293655378, 6.468984127044677),
    (1.721157550811766, 6.367913961410522),
    (1.4560169577598558, 6.222998619079589),
    (1.2288108468055725, 6.025204420089722),
    (1.0323731005191792, 5.795286893844604),
    (0.8291173726320267, 5.571383476257324),
    (0.6197642162442207, 5.3531599044799805),
    (0.40789822209626614, 5.137371540069582),
    (0.19429956376552582, 4.923299312591553),
    (-0.023909807205200195, 4.713939905166626),
    (-0.2506375387310982, 4.513866901397705),
    (-0.48888780921697617, 4.327690601348877),
    (-0.7386728525161743, 4.157300591468811),
    (-0.9981980323791504, 4.00212287902832),
    (-1.265850007534027, 3.861425995826721),
    (-1.540815532207489, 3.73561954498291),
    (-1.8222485184669495, 3.6250386238098145),
    (-2.1093080639839172, 3.530017375946045),
    (-2.401389002799988, 3.45179545879364),
    (-2.697199583053589, 3.3890894651412964),
    (-2.995718002319336, 3.3408654928207397),
    (-3.2961915731430054, 3.306867003440857),
    (-3.5979559421539307, 3.2873164415359497),
    (-3.900317072868347, 3.2859150171279907),
    (-4.201757431030273, 3.30913245677948),
    (-4.501722574234009, 3.3475000858306885),
    (-4.803642988204956, 3.360534429550171),
    (-5.104934930801392, 3.335668444633484),
    (-5.4039366245269775, 3.2906545400619507),
    (-5.698654651641846, 3.2232314348220825),
    (-5.985719680786133, 3.1284295320510864),
    (-6.2612550258636475, 3.004058003425598),
    (-6.521104574203491, 2.849557399749756),
    (-6.761330842971802, 2.6660499572753906),
    (-6.978301525115967, 2.4555970430374146),
    (-7.168851852416992, 2.2209489941596985),
    (-7.333338975906372, 1.9673035144805908),
    (-7.476794481277466, 1.701131522655487),
    (-7.608717441558838, 1.4290199875831604),
    (-7.726512908935547, 1.1505467295646667),
    (-7.82514500617981, 0.8647176027297974),
    (-7.905004262924194, 0.5730820894241333),
    (-7.967764854431152, 0.27728550136089325),
    (-8.017833709716797, -0.020941782742738724),
    (-8.057097673416138, -0.32076960802078247),
    (-8.082846403121948, -0.6220587491989136),
    (-8.09486985206604, -0.9242066740989657),
    (-8.093004941940308, -1.2265869975090027),
    (-8.080519199371338, -1.5287280082702637),
    (-8.055050611495972, -1.8300365209579468),
    (-8.013290405273438, -2.129516959190372),
    (-7.954392671585083, -2.426103949546814),
    (-7.878217458724976, -2.718727946281433),
    (-7.788170099258423, -3.007396936416626),
    (-7.681349277496336, -3.2902565002441437),
    (-7.555041313171385, -3.5649654865264924),
    (-7.4097075462341335, -3.8301074504852255),
    (-7.250068664550781, -4.086897611618042),
    (-7.071657419204714, -4.330975055694577),
    (-6.87251162528992, -4.558440446853634),
    (-6.653365612030029, -4.76670503616333),
    (-6.415413618087766, -4.95312547683716),
    (-6.161219596862789, -5.116795778274538),
    (-5.894645929336548, -5.259450912475586),
    (-5.617578506469727, -5.380439519882202),
    (-5.328897953033451, -5.469967603683471),
    (-5.03255558013916, -5.529980421066284),
    (-4.733617067337036, -5.5756354331970215),
    (-4.433925628662109, -5.616093158721924),
    (-4.134026527404785, -5.65498685836792),
    (-3.8341100215911865, -5.693745136260986),
    (-3.5342825651168823, -5.7331860065460205),
    (-3.2345045804977417, -5.772996425628662),
    (-2.9348185062408447, -5.813499450683594),
    (-2.635298490524292, -5.8552086353302),
    (-2.3359915018081665, -5.898420095443726),
    (-2.0369359850883484, -5.943336009979248),
]

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


racepoints = [[-2.03946, -5.95965, 4.0, 1.0, 0.0],
 [-1.74092, -6.00694, 4.0, 1.0, 0.0],
 [-1.44231, -6.0543, 4.0, 1.0, 0.0],
 [-1.14374, -6.10194, 4.0, 1.0, 0.0],
 [-0.84521, -6.14976, 4.0, 1.0, 0.0],
 [-0.54735, -6.19659, 3.82444, 1.0, 0.0],
 [-0.25071, -6.24014, 3.50493, 0.0, 0.0],
 [0.04467, -6.27828, 3.24924, 0.0, 0.0],
 [0.33857, -6.30896, 3.0364, 0.0, 1.0],
 [0.63061, -6.33016, 2.85578, 0.0, 1.0],
 [0.92031, -6.34005, 2.69708, 0.0, 1.0],
 [1.20706, -6.33685, 2.55788, 0.0, 1.0],
 [1.49009, -6.31894, 2.4324, 0.0, 1.0],
 [1.7685, -6.28475, 2.4324, 0.0, 1.0],
 [2.04114, -6.23284, 2.4324, 0.0, 1.0],
 [2.30663, -6.16183, 2.4324, 0.0, 1.0],
 [2.56324, -6.07049, 2.4324, 0.0, 1.0],
 [2.80883, -5.95775, 2.49235, 0.0, 1.0],
 [3.04277, -5.82565, 2.51568, 0.0, 1.0],
 [3.26429, -5.6755, 2.53557, 0.0, 1.0],
 [3.47263, -5.50843, 2.55508, 0.0, 1.0],
 [3.66711, -5.32553, 2.57144, 0.0, 1.0],
 [3.84701, -5.12778, 2.58667, 0.0, 1.0],
 [4.01164, -4.9161, 2.60049, 0.0, 1.0],
 [4.16026, -4.69142, 2.60755, 0.0, 1.0],
 [4.29207, -4.45463, 2.60755, 0.0, 1.0],
 [4.4062, -4.20664, 2.60755, 0.0, 1.0],
 [4.50169, -3.94842, 2.60755, 0.0, 1.0],
 [4.57737, -3.681, 3.17153, 0.0, 1.0],
 [4.63909, -3.408, 3.40128, 0.0, 0.0],
 [4.68831, -3.13048, 3.68356, 1.0, 0.0],
 [4.7266, -2.84937, 4.0, 1.0, 0.0],
 [4.75563, -2.5655, 3.45164, 0.0, 0.0],
 [4.77716, -2.27956, 2.99899, 0.0, 1.0],
 [4.79271, -1.998, 2.6824, 0.0, 1.0],
 [4.81271, -1.7178, 2.41507, 0.0, 1.0],
 [4.84108, -1.44024, 2.11914, 0.0, 1.0],
 [4.88153, -1.16664, 2.11914, 0.0, 1.0],
 [4.93751, -0.89847, 2.11914, 0.0, 1.0],
 [5.01226, -0.63743, 2.11914, 0.0, 1.0],
 [5.10935, -0.38587, 2.11914, 0.0, 1.0],
 [5.23433, -0.14808, 2.28641, 0.0, 1.0],
 [5.38245, 0.07649, 2.49968, 0.0, 1.0],
 [5.54961, 0.28908, 2.78125, 0.0, 1.0],
 [5.73211, 0.4914, 3.18275, 0.0, 1.0],
 [5.92641, 0.68546, 3.74135, 1.0, 0.0],
 [6.12894, 0.8736, 3.12015, 0.0, 1.0],
 [6.3361, 1.05836, 2.7311, 0.0, 1.0],
 [6.54074, 1.24266, 2.45446, 0.0, 1.0],
 [6.74033, 1.43025, 2.24631, 0.0, 1.0],
 [6.93107, 1.6235, 2.0, 0.0, 1.0],
 [7.10926, 1.82449, 2.0, 0.0, 1.0],
 [7.27134, 2.03492, 2.0, 0.0, 1.0],
 [7.41364, 2.25609, 2.0, 0.0, 1.0],
 [7.53227, 2.48872, 2.0, 0.0, 1.0],
 [7.62048, 2.73348, 2.04597, 0.0, 1.0],
 [7.67881, 2.98719, 2.09183, 0.0, 1.0],
 [7.70777, 3.24702, 2.1393, 0.0, 1.0],
 [7.70789, 3.51034, 2.18457, 0.0, 1.0],
 [7.67963, 3.77468, 2.22892, 0.0, 1.0],
 [7.62353, 4.0376, 2.27312, 0.0, 1.0],
 [7.5402, 4.29666, 2.31754, 0.0, 1.0],
 [7.43051, 4.54939, 2.36213, 0.0, 1.0],
 [7.29558, 4.79333, 2.40656, 0.0, 1.0],
 [7.13684, 5.02606, 2.45052, 0.0, 1.0],
 [6.95608, 5.24533, 2.49388, 0.0, 1.0],
 [6.75534, 5.44914, 2.53718, 0.0, 1.0],
 [6.53684, 5.63585, 2.58118, 0.0, 1.0],
 [6.30288, 5.80423, 2.62609, 0.0, 1.0],
 [6.05573, 5.95344, 2.6718, 0.0, 1.0],
 [5.79755, 6.08302, 2.71812, 0.0, 1.0],
 [5.53037, 6.19277, 2.7654, 0.0, 1.0],
 [5.2561, 6.28275, 2.81448, 0.0, 1.0],
 [4.97648, 6.35318, 2.86288, 0.0, 1.0],
 [4.69312, 6.40445, 2.91352, 0.0, 1.0],
 [4.40748, 6.43705, 2.96236, 0.0, 1.0],
 [4.12087, 6.45155, 3.00671, 0.0, 1.0],
 [3.83448, 6.44852, 3.04961, 0.0, 1.0],
 [3.54933, 6.42855, 3.08933, 0.0, 1.0],
 [3.26634, 6.39225, 3.12535, 0.0, 1.0],
 [2.98627, 6.34017, 3.15502, 0.0, 1.0],
 [2.70981, 6.27281, 3.17797, 0.0, 1.0],
 [2.43756, 6.19059, 3.19349, 0.0, 1.0],
 [2.17002, 6.09382, 3.20417, 0.0, 0.0],
 [1.90767, 5.98274, 3.2079, 0.0, 0.0],
 [1.65096, 5.85748, 3.8907, 1.0, 0.0],
 [1.39831, 5.7226, 4.0, 1.0, 0.0],
 [1.14843, 5.5813, 4.0, 1.0, 0.0],
 [0.89997, 5.43675, 4.0, 1.0, 0.0],
 [0.647, 5.29241, 4.0, 1.0, 0.0],
 [0.39245, 5.15029, 4.0, 1.0, 0.0],
 [0.1363, 5.0107, 4.0, 1.0, 0.0],
 [-0.12147, 4.87394, 4.0, 1.0, 0.0],
 [-0.38091, 4.7403, 4.0, 1.0, 0.0],
 [-0.64207, 4.61007, 4.0, 1.0, 0.0],
 [-0.905, 4.48357, 4.0, 1.0, 0.0],
 [-1.16976, 4.3611, 4.0, 1.0, 0.0],
 [-1.43642, 4.24298, 4.0, 1.0, 0.0],
 [-1.70503, 4.12957, 4.0, 1.0, 0.0],
 [-1.97566, 4.02123, 4.0, 1.0, 0.0],
 [-2.24803, 3.91746, 4.0, 1.0, 0.0],
 [-2.52192, 3.81791, 4.0, 1.0, 0.0],
 [-2.79716, 3.72226, 4.0, 1.0, 0.0],
 [-3.07354, 3.63015, 4.0, 1.0, 0.0],
 [-3.35091, 3.54126, 4.0, 1.0, 0.0],
 [-3.62908, 3.45525, 4.0, 1.0, 0.0],
 [-3.90792, 3.37178, 4.0, 1.0, 0.0],
 [-4.18727, 3.29052, 3.70843, 1.0, 0.0],
 [-4.467, 3.21113, 3.22085, 0.0, 0.0],
 [-4.74077, 3.135, 2.88165, 0.0, 1.0],
 [-5.01269, 3.05528, 2.62579, 0.0, 1.0],
 [-5.28107, 2.96872, 2.42557, 0.0, 1.0],
 [-5.5442, 2.87226, 2.42557, 0.0, 1.0],
 [-5.80031, 2.76299, 2.42557, 0.0, 1.0],
 [-6.04744, 2.63816, 2.42557, 0.0, 1.0],
 [-6.28327, 2.49518, 2.42557, 0.0, 1.0],
 [-6.50487, 2.33179, 2.61729, 0.0, 1.0],
 [-6.71331, 2.15161, 2.65945, 0.0, 1.0],
 [-6.90813, 1.95575, 2.69914, 0.0, 1.0],
 [-7.08888, 1.74524, 2.73611, 0.0, 1.0],
 [-7.25505, 1.52104, 2.77063, 0.0, 1.0],
 [-7.40609, 1.2841, 2.80403, 0.0, 1.0],
 [-7.54141, 1.03541, 2.83151, 0.0, 1.0],
 [-7.66036, 0.77605, 2.85865, 0.0, 1.0],
 [-7.76229, 0.50722, 2.88273, 0.0, 1.0],
 [-7.84658, 0.23032, 2.9039, 0.0, 1.0],
 [-7.91269, -0.0531, 2.92311, 0.0, 1.0],
 [-7.9602, -0.34136, 2.94059, 0.0, 1.0],
 [-7.98889, -0.63268, 2.95593, 0.0, 1.0],
 [-7.99871, -0.92529, 2.96914, 0.0, 1.0],
 [-7.98982, -1.21749, 2.98048, 0.0, 1.0],
 [-7.96253, -1.50767, 2.99094, 0.0, 1.0],
 [-7.91731, -1.7944, 2.99058, 0.0, 1.0],
 [-7.85467, -2.07641, 2.97994, 0.0, 1.0],
 [-7.77519, -2.35255, 2.96585, 0.0, 1.0],
 [-7.67945, -2.62184, 2.94812, 0.0, 1.0],
 [-7.56802, -2.88338, 2.92775, 0.0, 1.0],
 [-7.44146, -3.13636, 2.89485, 0.0, 1.0],
 [-7.30026, -3.38004, 2.85545, 0.0, 1.0],
 [-7.14491, -3.6137, 2.81315, 0.0, 1.0],
 [-6.97581, -3.83664, 2.76664, 0.0, 1.0],
 [-6.79336, -4.04814, 2.76664, 0.0, 1.0],
 [-6.5978, -4.24739, 2.76664, 0.0, 1.0],
 [-6.38935, -4.43346, 2.76664, 0.0, 1.0],
 [-6.1682, -4.6053, 2.76664, 0.0, 1.0],
 [-5.93454, -4.76171, 3.15183, 0.0, 1.0],
 [-5.69164, -4.90574, 3.28113, 0.0, 0.0],
 [-5.44047, -5.03795, 3.42617, 0.0, 0.0],
 [-5.18189, -5.15898, 3.58063, 0.0, 0.0],
 [-4.9167, -5.26945, 3.75148, 1.0, 0.0],
 [-4.64559, -5.37002, 3.9437, 1.0, 0.0],
 [-4.36924, -5.46138, 4.0, 1.0, 0.0],
 [-4.08828, -5.54427, 4.0, 1.0, 0.0],
 [-3.80331, -5.61947, 4.0, 1.0, 0.0],
 [-3.51491, -5.68783, 4.0, 1.0, 0.0],
 [-3.22361, -5.75025, 4.0, 1.0, 0.0],
 [-2.92994, -5.80766, 4.0, 1.0, 0.0],
 [-2.63439, -5.86103, 4.0, 1.0, 0.0],
 [-2.33742, -5.91135, 4.0, 1.0, 0.0]]  # replace: raceline


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
        self.is_in_curved_section = is_in_curved_section
        self.is_in_straight_section = is_in_straight_section


def get_processed_racepoints(racepoints):
    previous = racepoints[-2] if racepoints[0] == racepoints[-1] else racepoints[-1]

    processed_racepoints = []
    for i, w in enumerate(racepoints):
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
        self.has_crashed_since_beginning_of_lap = False
        self.has_crashed_since_beginning_of_straight_section = False
        self.has_crashed_since_beginning_of_curved_section = False

    def process_params(self, params):
        super().process_params(params)
        self.next_waypoint = self.processed_waypoints[self.next_waypoint_id]
        self.current_position = (self.x, self.y)
        self.closest_waypoint = self.processed_waypoints[self.closest_waypoint_id]
        self.closest_racepoint = self.racepoints[self.closest_waypoint_id]
        self.opt_speed = self.closest_racepoint.opt_speed
        self.next_racepoint = self.racepoints[self.next_waypoint_id]
        self.prev_racepoint = self.racepoints[self.previous_waypoint_id]
        self.racetrack_bearing = get_bearing_between_points(
            [self.prev_racepoint.x, self.prev_racepoint.y],
            [self.next_racepoint.x, self.next_racepoint.y],
        )
        self.leftsafe_bearing = get_bearing_between_points(
            self.current_position, self.next_waypoint.left_safe
        )
        self.rightsafe_bearing = get_bearing_between_points(
            self.current_position, self.next_waypoint.right_safe
        )
        self.racetrack_skew = get_turn_between_directions(
            self.racetrack_bearing, self.true_bearing
        )
        self.optimal_position = (self.closest_racepoint.x, self.closest_racepoint.y)
        self.next_optimal_position = (self.next_racepoint.x, self.next_racepoint.y)
        if len(self._history) >= 1000:
            self._history.pop(0)
        if self.steps <= 2:
            self.has_crashed_since_beginning_of_lap = False
        if not self.prev_racepoint.is_in_straight_section and self.next_racepoint.is_in_straight_section:
            self.has_crashed_since_beginning_of_straight_section = False
            self.straight_section_start_id = self.next_waypoint_id
        if not self.prev_racepoint.is_in_curved_section and self.next_racepoint.is_in_curved_section:
            self.has_crashed_since_beginning_of_curved_section = False
            self.curved_section_start_id = self.next_waypoint_id
        if self.is_crashed:
            self.has_crashed_since_beginning_of_lap = True
            self.has_crashed_since_beginning_of_straight_section = True
            self.has_crashed_since_beginning_of_curved_section = True
        if self.next_racepoint.is_in_straight_section:
            self.straight_section_score = self.speed_z_factor * self.distance_z_factor * self.heading_z_factor
        if self.next_racepoint.is_in_curved_section:
            self.curved_section_score = self.speed_z_factor * self.distance_z_factor * self.heading_z_factor
        self._history[-1].straight_section_score = self.straight_section_score
        self._history[-1].curved_section_score = self.curved_section_score

    @property
    def distance_z_factor(self):
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
    def heading_z_factor(self):
        sigma = 10
        reward = pow(
            math.e,
            -((self.racetrack_skew**2) / (2 * (sigma**2))),
        )
        return reward

    @property
    def speed_z_factor(self):
        speeds = self.racepoints[
            (self.closest_waypoint_id - 3) : (
                (self.closest_waypoint_id + 6) % len(self.processed_waypoints)
            )
        ]
        sigma = 1.3333
        reward = pow(
            math.e, ((self.action_speed - self.opt_speed) ** 2) / (2 * (sigma**2))
        )
        return reward

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

MIN_SPEED = 2.0  # replace: MIN_SPEED
MAX_SPEED = 4.0  # replace: MAX_SPEED


class Reward:
    def __init__(self, f: Framework) -> None:
        self.f = f
        self.speed_reward = self.f.speed_z_factor
        self.distance_reward = self.f.distance_z_factor
        self.heading_reward = self.f.heading_z_factor

    @property
    def track_completion_reward(self):
        reward = 0
        if self.f.is_complete_lap and not self.f.has_crashed_since_beginning_of_lap:
            reward = 100
        return reward

    @property
    def straight_section_bonus(self):
        reward = 0.0
        if self.f.prev_racepoint.is_in_straight_section and not self.f.next_racepoint.is_in_straight_section \
                and not self.f.has_crashed_since_beginning_of_straight_section:
            start = self.f.straight_section_start_id
            end = self.f.next_waypoint_id
            steps = self.f._history[start:end]
            reward = sum(s.straight_section_score for s in steps)
        return reward

    @property
    def curved_section_bonus(self):
        reward = 0.0
        if self.f.prev_racepoint.is_in_curved_section and not self.f.next_racepoint.is_in_curved_section \
                and not self.f.has_crashed_since_beginning_of_curved_section:
            start = self.f.curved_section_start_id
            end = self.f.next_waypoint_id
            steps = self.f._history[start:end]
            reward = sum(s.curved_section_score for s in steps)
        return reward


def get_reward(f: Framework):
    r = Reward(f)

    # speed component of the reward
    # TODO: * speed_maintain_bonus
    sc = 5 * r.speed_reward

    # distance component of the reward
    # TODO: * distance_reduction_bonus
    dc = 10 * r.distance_reward

    # heading component of the reward
    # TODO: * steering_angle_maintain_bonus
    hc = 10 * r.heading_reward

    # immediate component of the reward
    ic = (hc + dc + sc) ** 2

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
    # lc = curve_bonus + intermediate_progress_bonus + straight_section_bonus
    lc = r.track_completion_reward + r.straight_section_bonus + r.curved_section_bonus

    return max(ic + lc, 1e-3)