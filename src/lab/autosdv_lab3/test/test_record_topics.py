"""The topic files students pass to ros2 bag record match what the tools read."""

from pathlib import Path

import pytest

from autosdv_lab3 import record

RECORD = Path(__file__).resolve().parents[1] / 'config' / 'record'


def topics(name):
    lines = (RECORD / name).read_text().splitlines()
    return [ln.strip() for ln in lines if ln.strip() and not ln.lstrip().startswith('#')]


@pytest.mark.parametrize('name, kind', [('map_topics.txt', 'map'), ('run_topics.txt', 'run')])
def test_topic_file_matches_the_recorder(name, kind):
    t = topics(name)
    assert len(t) == len(set(t))
    assert t == record.topics_for(kind, 'robin_w')
