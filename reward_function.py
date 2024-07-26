from constants import PARAM_NAMES 
from metrics import StepMetrics
from racelines.ace_speedway import raceline  # to be updated for each track
from utils import get_processed_raceline
from pprint import pprint
# maintain global state
raceline = get_processed_raceline(raceline)
waypoints = None
previous_step = None


def reward_function(params: dict) -> float:
    global previous_step, waypoints

    # Check if this is a new lap
    if bool(params.get(PARAM_NAMES["PROGRESS"], 0.0) == 0.0):
        # Reset previous step if new lap
        previous_step = None

    # Calculate step metrics
    step_metrics = StepMetrics(params, raceline, waypoints, previous_step)

    # Cache processed waypoints
    if not waypoints:
        waypoints = step_metrics.waypoints

    # Update previous step
    previous_step = step_metrics

    # Return the total reward for the step
    return step_metrics.total_step_reward

#
# import random
#
#
#
# for i in range(1000):
#     sample_params = {
#         "all_wheels_on_track": random.choice([True]),  # Randomly True or False
#         "x": random.uniform(0, 10),  # Random x-coordinate between 0 and 10 meters
#         "y": random.uniform(0, 10),  # Random y-coordinate between 0 and 10 meters
#         "closest_objects": [random.randint(0, 10), random.randint(0, 10)],  # Random indices between 0 and 10
#         "closest_waypoints": random.sample(range(10), 2),  # Random indices between 0 and 10
#         "distance_from_center": random.uniform(0, 5),  # Random distance between 0 and 5 meters
#         "is_crashed": random.choice([False]),  # Randomly True or False
#         "is_left_of_center": random.choice([True, False]),  # Randomly True or False
#         "is_offtrack": random.choice([False]),  # Randomly True or False
#         "is_reversed": random.choice([False]),  # Randomly True or False
#         "heading": random.uniform(-180, 180),  # Random heading between -180 and 180 degrees
#         "objects_speed": [random.uniform(0, 10) for _ in range(5)],  # List of 5 random speeds between 0 and 10 m/s
#         "progress": random.uniform(0, 100),  # Random progress percentage between 0 and 100
#         "speed": random.uniform(0, 10),  # Random speed between 0 and 10 m/s
#         "steering_angle": random.uniform(-30, 30),  # Random steering angle between -30 and 30 degrees
#         "steps": random.randint(0, 1000),  # Random number of steps between 0 and 1000
#         "track_length": random.uniform(100, 1000),  # Random track length between 100 and 1000 meters
#         "track_width": random.uniform(5, 15),  # Random track width between 5 and 15 meters
#         "waypoints": [(random.uniform(0, 10), random.uniform(0, 10)) for _ in range(10)]  # List of 10 random waypoints
#     }
#
#
#     r = reward_function(sample_params)
#

# sample_params = {'all_wheels_on_track': True, 'x': 1.2292023107648375, 'y': 2.3433056336318625, 'closest_objects': [0, 6], 'closest_waypoints': [7, 10], 'distance_from_center': 1.542960305979969, 'is_crashed': False, 'is_left_of_center': False, 'is_offtrack': False, 'is_reversed': False, 'heading': 52.62596098812, 'objects_speed': [0.41939796485879977, 2.0606284362986624, 4.130272257729204, 1.0160598686096123, 9.484024589238905], 'progress': 1.7560035759679127, 'speed': 2.6251340473310627, 'steering_angle': -15.602265537282609, 'steps': 122, 'track_length': 184.8017957444523, 'track_width': 7.652449128644845, 'waypoints': [(5.1429440528109645, 6.105165961420659), (9.819255888996901, 8.069706448203162), (8.500337601996051, 0.11634305412228763), (5.4699865538564705, 3.6271559610561264), (1.6459324028367106, 6.454945048339424), (4.7252678790853935, 7.093959295695508), (6.3066104299075185, 2.3390177170469473), (4.806086000501231, 0.06705972543179395), (1.332663002688791, 0.1441907535564435), (5.732363429829158, 1.333809853580632)]}
# pprint(sample_params)
# print(reward_function(sample_params))
