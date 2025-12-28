#!/usr/bin/env python3
"""Test flake8 compliance."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check flake8 compliance."""
    rc, errors = main_with_errors(argv=['--exclude', 'build', 'install', 'log', '.'])
    assert rc == 0, f'Found {errors} flake8 errors'
