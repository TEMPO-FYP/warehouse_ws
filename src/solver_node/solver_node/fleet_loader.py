"""Fleet discovery helper for the TEMPO Task Allocation Solver."""

from typing import List, Optional

DEFAULT_FLEET = [
    'mobile_robot_1',
    'mobile_robot_2',
]


def get_default_fleet_robots(count: Optional[int] = None) -> List[str]:
    """
    Return the list of available robot instance IDs.

    :param count: Optional cap on the number of robots to return.
    :return: List of robot identifiers, e.g. ['mobile_robot_1', 'mobile_robot_2'].
    """
    if count is not None and count > 0:
        return [f'mobile_robot_{i + 1}' for i in range(count)]
    return list(DEFAULT_FLEET)
