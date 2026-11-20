"""Configuration assembly shared by launch and offline validation."""
import copy
import yaml

# Teleop's stand-in for /local_costmap/published_footprint (collision monitor approach shape).
TELEOP_FOOTPRINT_TOPIC = '/teleop/footprint'


def merge(base, updates):
    result = copy.deepcopy(base)
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def set_sim_time(tree, enabled):
    if isinstance(tree, dict):
        for key, value in tree.items():
            if key == 'ros__parameters':
                value['use_sim_time'] = enabled
            else:
                set_sim_time(value, enabled)


def padded_footprint(nav2_overrides, costmap='local_costmap'):
    """Flat [x0, y0, ...] of the costmap footprint padded like Nav2's padFootprint."""
    params = nav2_overrides[costmap][costmap]['ros__parameters']
    padding = params['footprint_padding']

    def pad(value):
        return value + padding if value > 0 else value - padding if value < 0 else value
    return [pad(float(value)) for point in yaml.safe_load(params['footprint']) for value in point]


DIRECTION_DEADBAND = 1e-3  # m/s


def directional_footprint(points, margin, vx):
    """Flat [x0, y0, ...] polygon with its leading edge (along vx) pushed out by margin."""
    out = list(points)
    sign = 1 if vx > DIRECTION_DEADBAND else -1 if vx < -DIRECTION_DEADBAND else 0
    if sign and margin > 0:
        for i in range(0, len(out), 2):
            if out[i] * sign > 0:
                out[i] += sign * margin
    return out
