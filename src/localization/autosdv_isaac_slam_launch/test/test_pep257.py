#!/usr/bin/env python3
"""Test pep257 compliance."""

from ament_pep257.main import main
import pytest


@pytest.mark.pep257
@pytest.mark.linter
def test_pep257():
    """Check pep257 compliance."""
    rc = main(argv=['--exclude', 'build', 'install', 'log', '.'])
    assert rc == 0, 'Found pep257 errors'
