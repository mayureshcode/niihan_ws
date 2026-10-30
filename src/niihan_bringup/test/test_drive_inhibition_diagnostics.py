"""Regression checks: diagnostics describe, never override, the stop gate."""
import math
from niihan_bringup.contracts import DriveState


def test_drive_inhibition_reasons_follow_existing_output():
    s=DriveState()
    s.receive(0.2,0.1,10.0)
    assert s.inhibition_reason(10.0) == 'drive_disabled'
    assert s.output(10.0) == (0,0)
    s.enabled=True
    assert s.inhibition_reason(10.0) == 'permission_denied'
    s.permitted=True
    assert s.inhibition_reason(10.0) is None
    assert s.output(10.0) == (0.2,0.1)
    assert s.inhibition_reason(10.26) == 'command_stale'
    assert s.output(10.26) == (0,0)
    s.estop=True
    assert s.inhibition_reason(10.0) == 'e_stop'
    assert s.output(10.0) == (0,0)


def test_invalid_and_future_command_remain_inhibited():
    s=DriveState(enabled=True,permitted=True)
    s.receive(math.nan,0.0,10)
    assert s.inhibition_reason(10) == 'command_stale'
    assert s.output(10) == (0,0)
    s.receive(0.2,0.0,10)
    assert s.inhibition_reason(9.99) == 'command_stale'
    assert s.output(9.99) == (0,0)
