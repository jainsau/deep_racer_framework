from raceline.ace_speedway import raceline

from constants import ParamNames
from metrics import StepMetrics
from utils import get_processed_raceline

# maintain global state
raceline = get_processed_raceline(raceline)
previous_step = None


def get_reward(params: dict) -> float:
    global previous_step

    # Check if this is a new lap
    if bool(params.get(ParamNames["PROGRESS"], 0.0) == 0.0):
        # Reset previous step if new lap
        previous_step = None

    # Calculate step metrics
    step_metrics = StepMetrics(params, raceline, previous_step)

    # Update previous step
    previous_step = step_metrics

    # Return the total reward for the step
    return step_metrics.total_step_reward
