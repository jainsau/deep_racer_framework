import math

import numpy as np
import pandas as pd
import seaborn as sns
from matplotlib import pyplot as plt
from scipy import stats

from lib.track import plot_single


# Uses previous and next coordinates to calculate the radius of the curve,
# so you need to pass a list with form [[x1,y1],[x2,y2],[x3,y3]]
# Input 3 coordinates [[x1,y1],[x2,y2],[x3,y3]]
def circle_radius(coordinates):
    # Flatten the list and assign to variables (makes code easier to read later)

    x1, y1, x2, y2, x3, y3 = [i for sub in coordinates for i in sub]

    a = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2
    b = (
        (x1**2 + y1**2) * (y3 - y2)
        + (x2**2 + y2**2) * (y1 - y3)
        + (x3**2 + y3**2) * (y2 - y1)
    )
    c = (
        (x1**2 + y1**2) * (x2 - x3)
        + (x2**2 + y2**2) * (x3 - x1)
        + (x3**2 + y3**2) * (x1 - x2)
    )
    d = (
        (x1**2 + y1**2) * (x3 * y2 - x2 * y3)
        + (x2**2 + y2**2) * (x1 * y3 - x3 * y1)
        + (x3**2 + y3**2) * (x2 * y1 - x1 * y2)
    )

    # In case 'a' is zero (so radius is infinity)
    try:
        r = abs((b**2 + c**2 - 4 * a * d) / abs(4 * a**2)) ** 0.5
    except Exception as e:
        r = 999

    return r


# Returns indices of next index and index + lookahead
# We need this to calculate the radius for next track section.
def circle_indexes(mylist, index_car, add_index_1=0, add_index_2=0):
    list_len = len(mylist)

    # if index >= list_len:
    #     raise ValueError("Index out of range in circle_indexes()")

    # Use modulo to consider that track is cyclical
    index_1 = (index_car + add_index_1) % list_len
    index_2 = (index_car + add_index_2) % list_len

    return [index_car, index_1, index_2]


# For each point in racing track, check if left curve (returns boolean)
def is_left_curve(coordinates):
    # Flatten the list and assign to variables (makes code easier to read later)

    x1, y1, x2, y2, x3, y3 = [i for sub in coordinates for i in sub]
    return ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)) > 0


# Calculate the distance between 2 points
def dist_2_points(x1, x2, y1, y2):
    return abs((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def speed(track, min_speed, max_speed, look_ahead_points):
    # Calculate the radius for every point of the track
    radius = []
    for i in range(len(track)):
        indexes = circle_indexes(track, i, add_index_1=-1, add_index_2=1)
        coordinates = [track[indexes[0]], track[indexes[1]], track[indexes[2]]]
        radius.append(circle_radius(coordinates))

    # Get the max_velocity for the smallest radius
    # That value should multiply by a constant multiple
    v_min_r = min(radius) ** 0.5
    constant_multiple = min_speed / v_min_r

    if look_ahead_points == 0:
        # Get the maximal velocity from radius
        max_velocity = [(constant_multiple * i**0.5) for i in radius]
        # Get velocity from max_velocity (cap at MAX_SPEED)
        velocity = [min(v, max_speed) for v in max_velocity]
        return velocity

    else:
        # Looks at the next n radii of points and takes the minimum
        # goal: reduce lookahead until car crashes bc no time to break
        radius_lookahead = []
        for i in range(len(radius)):
            next_n_radius = []
            for j in range(look_ahead_points + 1):
                index = circle_indexes(mylist=radius, index_car=i, add_index_1=j)[1]
                next_n_radius.append(radius[index])
            radius_lookahead.append(min(next_n_radius))
        max_velocity_lookahead = [
            (constant_multiple * i**0.5) for i in radius_lookahead
        ]
        velocity_lookahead = [min(v, max_speed) for v in max_velocity_lookahead]
        return velocity_lookahead


def plot_speeds(track, lookahead, speed_with_lookahead, speed_without_lookahead):
    x = [i[0] for i in track]
    y = [i[1] for i in track]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax = sns.scatterplot(
        x=x, y=y, hue=speed_without_lookahead, palette="vlag"
    ).set_title("Without lookahead")

    fig, ax = plt.subplots(figsize=(8, 5))
    ax = sns.scatterplot(x=x, y=y, hue=speed_with_lookahead, palette="vlag").set_title(
        f"With lookahead: {lookahead}"
    )


def time(track, speed_):
    distance_to_prev = []

    for i in range(len(track)):
        indexes = circle_indexes(track, i, add_index_1=-1, add_index_2=0)[0:2]
        coordinates = [track[indexes[0]], track[indexes[1]]]
        dist_to_prev = dist_2_points(
            x1=coordinates[0][0],
            x2=coordinates[1][0],
            y1=coordinates[0][1],
            y2=coordinates[1][1],
        )
        distance_to_prev.append(dist_to_prev)

    time_to_prev = [(distance_to_prev[i] / speed_[i]) for i in range(len(track))]

    total_time = sum(time_to_prev)
    print(
        f"Total time for track, if racing line and speeds are followed perfectly: {total_time} s"
    )
    return time_to_prev


def everything(track, speed_, high_speed_factor, low_speed_factor):
    # also add track type
    max_speed, min_speed = max(speed_), min(speed_)

    race_track_everything = []
    for i in range(len(track)):
        curr_step_speed = speed_[i]
        is_track_straight = curr_step_speed >= high_speed_factor * max_speed
        is_track_curved = curr_step_speed <= low_speed_factor * min_speed
        race_track_everything.append(
            [
                track[i][0],
                track[i][1],
                curr_step_speed,
                is_track_straight,
                is_track_curved,
            ]
        )

    # Round to 5 decimals
    race_track_everything = np.around(race_track_everything, 5).tolist()
    return race_track_everything


def action_space(track, velocity, min_speed, max_speed):
    # Calculate the radius for every point of the racing_track
    radius = []
    for i in range(len(track)):
        indexes = circle_indexes(
            track, i, add_index_1=-1, add_index_2=1
        )  # CHANGE BACK? 1;2
        coordinates = [
            track[indexes[0]],
            track[indexes[1]],
            track[indexes[2]],
        ]
        radius.append(circle_radius(coordinates))

    # Calculate curve direction
    left_curve = []
    for i in range(len(track)):
        indexes = circle_indexes(track, i, add_index_1=-1, add_index_2=1)
        coordinates = [
            track[indexes[1]],
            track[indexes[0]],
            track[indexes[2]],
        ]
        left_curve.append(is_left_curve(coordinates))

    # Calculate radius with + and - for direction (+ is left, - is right)
    radius_direction = []
    for i in range(len(track)):
        radius_with_direction = radius[i]
        if not left_curve[i]:
            radius_with_direction *= -1
        radius_direction.append(radius_with_direction)

    # Calculate steering with + and -
    dist_wheels_front_back = 0.165  # meters
    steering = []
    for i in range(len(track)):
        steer = math.degrees(math.asin(dist_wheels_front_back / radius_direction[i]))
        steering.append(steer)

    # Merge relevant lists into dataframe
    all_actions = pd.DataFrame({"velocity": velocity, "steering": steering})

    # Steering: Find standard deviation so that probability of >10 degrees steering is 5%
    steering_sd = -15 / stats.norm.ppf(0.05)

    # Velocity: Find standard deviation so that probability of >0.25m/s deviation is 0%
    # Note: Here, probability is set to 0%, so no noise regarding velocity
    velocity_sd = -0.25 / stats.norm.ppf(0.00)

    all_actions_norm = all_actions.copy()

    all_actions_norm_len = len(all_actions_norm)
    resample_size = 1000

    # Add gaussian noise to action space
    for i in range(all_actions_norm_len):
        v_true = all_actions_norm.iloc[i]["velocity"]
        s_true = all_actions_norm.iloc[i]["steering"]
        v_norm = np.random.normal(loc=v_true, scale=velocity_sd, size=resample_size)
        s_norm = np.random.normal(loc=s_true, scale=steering_sd, size=resample_size)
        vs_norm = pd.DataFrame(
            np.column_stack([v_norm, s_norm]), columns=["velocity", "steering"]
        )
        all_actions_norm = pd.concat(
            [all_actions_norm, vs_norm], axis=0, ignore_index=True
        )

    # Take out actions with max speed, so that they are not affected by gaussian noise
    # We do this because there are disproportionally many points with max speed, so
    # K-Means will focus too much on these
    all_actions_norm = all_actions_norm[all_actions_norm["velocity"] < max_speed]

    # Add initial actions to action space (to make clustering more focused on initial actions)
    add_n_initial_actions = int(resample_size / 8)
    add_initial_actions = pd.DataFrame()
    for i in range(add_n_initial_actions):
        add_initial_actions = pd.concat(
            [add_initial_actions, all_actions], axis=0, ignore_index=True
        )
    all_actions_norm = pd.concat(
        [all_actions_norm, add_initial_actions], axis=0, ignore_index=True
    )

    # sample bc less compute time
    all_actions_norm_less = all_actions_norm.sample(frac=0.01).reset_index(drop=True)

    X = all_actions_norm

    # Calculate action space with KMeans

    from sklearn.preprocessing import MinMaxScaler
    from sklearn.cluster import MiniBatchKMeans

    # Rescale data with minmax
    minmax_scaler = MinMaxScaler()
    x_minmax = pd.DataFrame(
        minmax_scaler.fit_transform(X), columns=["velocity", "steering"]
    )

    # KMeans
    # remove 2 actions from KMeans so that low speed & high steering actions can be manually included
    n_clusters = 21 - 2
    model = MiniBatchKMeans(n_clusters=n_clusters).fit(x_minmax)

    # Centroids (interpretable)
    from sklearn.preprocessing import MinMaxScaler

    minmax_scaler = MinMaxScaler()
    x_minmax_fit = minmax_scaler.fit(X)
    x_centroids = pd.DataFrame(
        x_minmax_fit.inverse_transform(model.cluster_centers_),
        columns=["velocity", "steering"],
    )

    # Add 2 manual actions
    # Reason: When car starts new episode, it does not start on or direction of racing line, so
    # it cannot steer enough to get on racing line
    manual_actions = pd.DataFrame(
        {"velocity": [min_speed, min_speed], "steering": [30, -30]}
    )
    x_centroids = pd.concat([x_centroids, manual_actions], ignore_index=True)

    action_space_e = x_centroids.copy()

    # Output JSON format
    action_space_for_json = action_space_e[["steering", "velocity"]].copy()

    action_space_for_json = action_space_for_json.round(1)
    action_space_for_json.columns = ["steering_angle", "speed"]
    action_space_for_json["index"] = action_space_for_json.index
    return action_space_for_json


def plot_track_type(all_things, straight=True):
    track = [[p[0], p[1]] for p in all_things if p[3 if straight else 4] == 1]
    plot_single(track)
