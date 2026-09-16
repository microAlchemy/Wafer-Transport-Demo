import math
import pytest
from wafer_transport_sim.core import (DetectionFilter, Sequence, destination_error,
                                      docking_speed, select_command, placed)


def test_feedback_sequence_does_not_skip_or_finish_on_timeout():
    sequence = Sequence(('HOME', 'PICK', 'DONE'))
    assert not sequence.advance(False, 10)
    assert sequence.state == 'HOME'
    assert sequence.expired(31, 30)
    assert sequence.advance(True, 32)
    assert sequence.state == 'PICK'
    assert not sequence.expired(33, 30)
    assert not sequence.advance(False, 34)
    assert sequence.advance(True, 35)
    assert sequence.state == 'DONE'
    assert not sequence.advance(True, 36)


def test_qr_confirmation_requires_consecutive_known_observations():
    f = DetectionFilter(3)
    assert f.update('STATION_C') == ''
    assert f.update('STATION_C') == ''
    assert f.update('STATION_C') == 'STATION_C'
    assert f.update('UNKNOWN') == ''
    assert f.update('STATION_C') == ''
    assert f.update('STATION_B') == ''


@pytest.mark.parametrize('state', ['WAIT_FOR_CARRIER', 'VERIFY_CARRIER', 'STOP',
                                  'DROP_CARRIER', 'VERIFY_DROP', 'DELIVERY_COMPLETE', 'FAULT'])
def test_stationary_states_override_navigation(state):
    assert select_command(state, (0.035, 0.1), (0.018, 0.0), True) == (0, 0)


def test_navigation_arbitration_and_stale_inputs():
    assert select_command('FOLLOW_PATH', (0.035, 0.1), (0.018, 0), True) == (0.035, 0.1)
    assert select_command('PRECISION_DOCK', (0.035, 0.1), (0.018, 0), True) == (0.018, 0)
    assert select_command('FOLLOW_PATH', (0.035, 0.1), (0.018, 0), False) == (0, 0)


def test_destination_validation_and_latching():
    assert destination_error('STATION_B', False) == ''
    assert destination_error('STATION_D', False)
    assert destination_error('STATION_B', True)


def test_docking_requires_distance_not_qr():
    assert docking_speed(0.2, 0.01, 0.018, 0.8) == 0.018
    assert docking_speed(0.012, 0.01, 0.018, 0.8) < 0.018
    for distance in (0.005, -0.1, math.nan):
        assert docking_speed(distance, 0.01, 0.018, 0.8) == 0


def test_delivery_checks_horizontal_and_vertical_placement():
    target = (1.145, 1.04, 0.134)
    assert placed(target, target)
    assert not placed((1.025, 1.04, 0.134), target)
    assert not placed((1.145, 1.04, 0.16), target)
    assert not placed((math.nan, 1.04, 0.134), target)
