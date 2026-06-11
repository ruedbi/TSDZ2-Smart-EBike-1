"""Unit tests for battery-change hysteresis detection logic (mirrors ebike_app.c)."""

import pytest

# 10S pack defaults from config.h / main.h
BATTERY_CELLS_NUMBER = 10
RISE_HYSTERESIS_X10 = BATTERY_CELLS_NUMBER * 1
DROP_HYSTERESIS_X10 = BATTERY_CELLS_NUMBER * 25 // 10
FULL_VOLTAGE_X10 = 410


def is_battery_change_detected(voltage_now_x10, reference_voltage_x10, shutdown_voltage_x10):
    """Python port of is_battery_change_detected() in src/ebike_app.c."""
    if reference_voltage_x10 == 0:
        return True
    if shutdown_voltage_x10 != 0:
        if voltage_now_x10 >= shutdown_voltage_x10 + RISE_HYSTERESIS_X10:
            return True
        if (
            shutdown_voltage_x10 <= reference_voltage_x10 - DROP_HYSTERESIS_X10
            and voltage_now_x10 >= FULL_VOLTAGE_X10
        ):
            return True
    return False


@pytest.mark.parametrize(
    "v_ref, v_shutdown, v_now, expected",
    [
        (0, 0, 410, True),       # first boot / migration
        (410, 0, 410, False),    # abrupt power loss, V_shutdown not saved
        (410, 410, 410, False),  # power cycle, no ride
        (410, 405, 410, False),  # short ride
        (410, 360, 390, True),   # deep ride, partial recharge
        (410, 360, 420, True),   # deep ride, full recharge
        (410, 360, 410, True),   # deep ride, fresh 41V pack
        (410, 417, 420, False),  # top-up after short ride (intentionally ignored)
    ],
)
def test_battery_change_scenarios(v_ref, v_shutdown, v_now, expected):
    assert is_battery_change_detected(v_now, v_ref, v_shutdown) == expected


def test_rise_threshold_boundary():
    # rise needs strictly >= shutdown + RISE (10 on 10S)
    assert is_battery_change_detected(369, 360, 360) is False
    assert is_battery_change_detected(370, 360, 360) is True


def test_drop_threshold_boundary():
    v_ref = 410
    # shutdown must be <= ref - DROP (385); use v_now below rise path from shutdown
    assert is_battery_change_detected(395, v_ref, 386) is False
    assert is_battery_change_detected(410, v_ref, 385) is True


if __name__ == "__main__":
    pytest.main()
