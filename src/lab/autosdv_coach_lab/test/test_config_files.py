"""The package's launch and param files parse, and the param files carry the
keys the nodes read."""
import glob
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.mark.parametrize('path', sorted(glob.glob(os.path.join(PKG, 'launch', '*.xml'))))
def test_launch_xml_well_formed(path):
    ET.parse(path)


def _params(path):
    doc = yaml.safe_load(open(path))
    return doc['/**']['ros__parameters']


@pytest.mark.parametrize('path', sorted(glob.glob(os.path.join(PKG, 'config', 'virtual_coach', '*.yaml'))))
def test_virtual_coach_scenarios(path):
    from autosdv_coach_lab.coach_script import CoachScript, DetectorModel
    p = _params(path)
    CoachScript(initial_distance=p['initial_distance'], lateral_offset=p['lateral_offset'],
                durations=p['segment_durations'], speeds=p['segment_speeds'], accel=p['accel'])
    DetectorModel(noise_std=p['noise_std'], latency=p['latency'],
                  dropout_prob=p['dropout_prob'], dropout_windows=p['dropout_windows'])


def test_planner_params_load_reference_controller():
    from autosdv_coach_lab import pursuit
    p = _params(os.path.join(PKG, 'config', 'board_pursuit_planner.param.yaml'))
    pursuit.PlannerParams(**{k: p[k] for k in (
        'v_max', 'accel_max', 'decel_max', 'brake_decel', 'envelope_decel', 'reaction_time',
        'standoff_min', 'path_length', 'path_step')})
    assert 'controller' not in p, 'controller.* would merge under a student param file'
    pursuit.load_controller(p['controller_class'], {})
