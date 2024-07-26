from constants import PARAM_NAMES 
from metrics import StepMetrics
from racelines.ace_speedway import raceline  # to be updated for each track
from utils import get_processed_raceline

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


import random



for i in range(1000):
    sample_params = {
        "all_wheels_on_track": random.choice([True]),  # Randomly True or False
        "x": random.uniform(0, 10),  # Random x-coordinate between 0 and 10 meters
        "y": random.uniform(0, 10),  # Random y-coordinate between 0 and 10 meters
        "closest_objects": [random.randint(0, 10), random.randint(0, 10)],  # Random indices between 0 and 10
        "closest_waypoints": [random.randint(0, 10), random.randint(0, 10)],  # Random indices between 0 and 10
        "distance_from_center": random.uniform(0, 5),  # Random distance between 0 and 5 meters
        "is_crashed": random.choice([False]),  # Randomly True or False
        "is_left_of_center": random.choice([True, False]),  # Randomly True or False
        "is_offtrack": random.choice([False]),  # Randomly True or False
        "is_reversed": random.choice([False]),  # Randomly True or False
        "heading": random.uniform(-180, 180),  # Random heading between -180 and 180 degrees
        "objects_speed": [random.uniform(0, 10) for _ in range(5)],  # List of 5 random speeds between 0 and 10 m/s
        "progress": random.uniform(0, 100),  # Random progress percentage between 0 and 100
        "speed": random.uniform(0, 10),  # Random speed between 0 and 10 m/s
        "steering_angle": random.uniform(-30, 30),  # Random steering angle between -30 and 30 degrees
        "steps": random.randint(0, 1000),  # Random number of steps between 0 and 1000
        "track_length": random.uniform(100, 1000),  # Random track length between 100 and 1000 meters
        "track_width": random.uniform(5, 15),  # Random track width between 5 and 15 meters
        "waypoints": [(random.uniform(0, 10), random.uniform(0, 10)) for _ in range(10)]  # List of 10 random waypoints
    }

    print(sample_params)

    r = reward_function(sample_params)
    print(r)

