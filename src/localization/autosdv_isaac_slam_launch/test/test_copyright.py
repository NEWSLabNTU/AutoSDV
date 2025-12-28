#!/usr/bin/env python3
"""Test copyright headers."""

from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Check copyright headers."""
    rc = main(argv=['--exclude', 'build', 'install', 'log', '.'])
    assert rc == 0, 'Found copyright errors'
