from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from g1_navigation.configuration import merge, set_sim_time

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/g1_navigation/config'


def test_configs_parse_and_motion_is_disabled_by_default():
    for path in CONFIG.glob('*.yaml'):
        assert isinstance(yaml.safe_load(path.read_text()), dict)
    robot = yaml.safe_load((CONFIG / 'robot.yaml').read_text())['/**']['ros__parameters']
    assert not robot['hardware_validation_complete']
    assert not robot['calibration_confirmed']
    assert robot['allowed_fsm_ids'] == [-1]
    assert 0 < robot['command_duration'] < robot['command_timeout']


def test_parameter_merge_preserves_plugin_defaults_and_sets_nested_clock():
    defaults = {'a': {'ros__parameters': {'plugin': 'x', 'speed': 1}},
                'b': {'b': {'ros__parameters': {}}}}
    result = merge(defaults, {'a': {'ros__parameters': {'speed': 0.2}}})
    set_sim_time(result, True)
    assert result['a']['ros__parameters'] == {'plugin': 'x', 'speed': 0.2, 'use_sim_time': True}
    assert result['b']['b']['ros__parameters']['use_sim_time']
    assert defaults['a']['ros__parameters']['speed'] == 1


def test_behavior_trees_have_no_autonomous_recovery_motion():
    for path in (ROOT / 'src/g1_navigation/behavior_trees').glob('*.xml'):
        tree = ET.parse(path)
        tags = {node.tag for node in tree.iter()}
        assert 'FollowPath' in tags
        assert not tags.intersection({'Spin', 'BackUp', 'DriveOnHeading'})


def test_dependencies_are_commit_pinned():
    for name in ('dependencies.repos', 'hardware.repos'):
        entries = yaml.safe_load((ROOT / name).read_text())['repositories']
        for entry in entries.values():
            assert len(entry['version']) == 40
            int(entry['version'], 16)
