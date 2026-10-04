"""Configuration assembly shared by launch and offline validation."""
import copy


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
