import pytest

from opensourceleg.utilities import get_active_ports


def test_get_active_ports():
    active_ports = get_active_ports()
