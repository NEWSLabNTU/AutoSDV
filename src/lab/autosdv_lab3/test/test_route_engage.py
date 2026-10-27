"""lab3_route's engage: a refusal right after a route is set is retried, not fatal."""

from types import SimpleNamespace

from autosdv_lab3 import route


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def api_with(monkeypatch, answers, available=True):
    """An AdApi with no ROS: spin advances a fake clock, engage answers from a list."""
    clock = Clock()
    monkeypatch.setattr(route.time, 'monotonic', clock)
    api = route.AdApi.__new__(route.AdApi)
    api.op = SimpleNamespace(is_autonomous_mode_available=available)
    api.calls = 0

    def spin(seconds=0.1):
        clock.t += seconds

    def engage():
        api.calls += 1
        return answers[min(api.calls, len(answers)) - 1]

    api.spin, api.engage = spin, engage
    return api, clock


def test_a_refusal_is_retried_until_it_succeeds(monkeypatch):
    refused = (False, 'The target mode is not available. Please check the diagnostics.')
    api, _ = api_with(monkeypatch, [refused, refused, (True, '')])
    assert api.engage_when_available(120.0) == (True, '')
    assert api.calls == 3


def test_refusals_until_the_deadline_are_an_error(monkeypatch):
    api, clock = api_with(monkeypatch, [(False, 'The target mode is not available.')])
    ok, msg = api.engage_when_available(10.0)
    assert not ok and msg.startswith('engage refused: The target mode')
    assert 9.0 <= clock.t <= 11.5


def test_never_available_says_so(monkeypatch):
    api, _ = api_with(monkeypatch, [(True, '')], available=False)
    ok, msg = api.engage_when_available(5.0)
    assert not ok and msg == 'autonomous mode never became available'
    assert api.calls == 0
