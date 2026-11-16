"""The topic files students pass to ros2 bag record match what the tools read."""

from pathlib import Path

import pytest

from autosdv_lab3 import record

RECORD = Path(__file__).resolve().parents[1] / 'config' / 'record'


def topics(name):
    # plain topic names, one per line, nothing else: the file is passed with $(cat ...)
    lines = (RECORD / name).read_text().splitlines()
    assert all(ln and ln == ln.strip() and ln.startswith('/') for ln in lines)
    return lines


@pytest.mark.parametrize('name, kind', [('map_topics.txt', 'map'), ('run_topics.txt', 'run')])
def test_topic_file_matches_the_recorder(name, kind):
    t = topics(name)
    assert len(t) == len(set(t))
    assert t == record.topics_for(kind, 'robin_w')


def test_replay_plays_only_sensor_inputs_the_run_bag_holds():
    # lab3_replay.launch.xml runs the sensing chain itself (LiDAR preprocessing, the
    # IMU corrector, the velocity converter, the sensor kit's TF): replaying their
    # outputs, /tf, or anything localization and planning publish would give each
    # topic two publishers
    t = topics('replay_topics.txt')
    assert set(t) <= set(topics('run_topics.txt'))
    assert t == ['/sensing/lidar/iv_points', '/sensing/camera/zedxm/imu/data',
                 '/vehicle/status/velocity_status']
