from constants import ParamNames
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
    if bool(params.get(ParamNames["PROGRESS"], 0.0) == 0.0):
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
