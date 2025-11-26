# Display Data Mapping Analysis

## Overview
The display interprets the 16-bit value sent in bytes 6-7 as a speed in km/h. The maximum displayable value is **99.9 km/h**. The calculation pattern is:

```
display_value = ui16_display_data_factor / ui16_display_data
```

Therefore:
```
ui16_display_data = ui16_display_data_factor / display_value
```

For a value `v` to be displayed:
```
ui16_display_data = ui16_display_data_factor / v
```

## Key Constants

- **OEM_WHEEL_FACTOR**: 
  - 1435 (km/h mode)
  - 900 (mph mode)
- **ui16_display_data_factor** = `OEM_WHEEL_FACTOR * ui8_oem_wheel_diameter`
  - Typical wheel diameter: 16-28 inches
  - Range: 1435*16 = 22,960 to 1435*28 = 40,180 (km/h mode)
  - Range: 900*16 = 14,400 to 900*28 = 25,200 (mph mode)

## Display Format
- Maximum displayable: **99.9** (sent as km/h, display converts to mph if needed)
- When mph is set, max becomes 62.4 (99.9/1.6)
- Values exceeding 99.9 will overflow/display incorrectly

## Analysis by Case

### Torque Sensor Calibration Cases

#### Case 1: `ui16_pedal_weight_x100 / 10U` (line 3485)
- **Formula**: `ui16_display_data = ui16_display_data_factor / (ui16_pedal_weight_x100 / 10U)`
- **Display value**: `(ui16_pedal_weight_x100 / 10U)` (in kg*10, so actual weight in kg)
- **Range**: Assuming weight 0-200 kg → 0-2000 (after /10: 0-200)
- **Issue**: If weight < 1.44 kg (km/h) or < 0.9 kg (mph), display > 99.9
- **Status**: ⚠️ **POTENTIAL ISSUE** - Very light weights could exceed limit

#### Case 2: `ui8_torque_sensor_step_to_display` (line 3488)
- **Formula**: `ui16_display_data = (ui16_display_data_factor / ui8_torque_sensor_step_to_display) * 10U`
- **Display value**: `ui8_torque_sensor_step_to_display / 10` (0-25.5 if uint8_t)
- **Range**: 0-255 (uint8_t), after /10: 0-25.5
- **Status**: ✅ **SAFE** - Max 25.5 < 99.9

#### Case 3: `ui16_torque_sensor_value_to_display` (line 3491)
- **Formula**: `ui16_display_data = ui16_display_data_factor / ui16_torque_sensor_value_to_display`
- **Display value**: `ui16_torque_sensor_value_to_display`
- **Range**: Unknown, but if < 144 (km/h) or < 90 (mph), exceeds limit
- **Status**: ⚠️ **POTENTIAL ISSUE** - Small values could exceed limit

### Startup Display Cases

#### Case 4: Battery SOC (line 3498-3501)
- **Formula KM**: `ui16_display_data = ui16_display_data_factor / ui16_battery_SOC_percentage_x10`
- **Formula MILES**: `ui16_display_data = (ui16_display_data_factor / ui16_battery_SOC_percentage_x10) * 10U`
- **Display value KM**: `ui16_battery_SOC_percentage_x10 / 10` (0-100%)
- **Display value MILES**: `ui16_battery_SOC_percentage_x10 / 100` (0-10.0%)
- **Range**: 0-1000 (0-100.0%)
- **Status KM**: ⚠️ **POTENTIAL ISSUE** - SOC < 1.44% (km/h) or < 0.9% (mph) exceeds limit
- **Status MILES**: ✅ **SAFE** - Max 10.0 < 99.9

#### Case 5: Battery Voltage (line 3506)
- **Formula**: `ui16_display_data = ui16_display_data_factor / ui16_battery_voltage_calibrated_x10`
- **Display value**: `ui16_battery_voltage_calibrated_x10 / 10` (volts)
- **Range**: Typically 30-50V (300-500 in x10 format)
- **Status**: ✅ **SAFE** - Typical range 30-50V < 99.9

### Function Status Cases

#### Case 6: FUNCTION_STATUS_OFF (line 3517)
- **Formula**: `ui16_display_data = ui16_display_data_factor / FUNCTION_STATUS_OFF`
- **FUNCTION_STATUS_OFF** = 1
- **Display value**: 1
- **Status**: ⚠️ **ISSUE** - Will display 22,960-40,180 (km/h) or 14,400-25,200 (mph) - **FAR EXCEEDS 99.9!**

#### Case 7: FUNCTION_STATUS_ON (line 3520)
- **Formula**: `ui16_display_data = ui16_display_data_factor / FUNCTION_STATUS_ON`
- **FUNCTION_STATUS_ON** = 100 + 5 = 105
- **Display value**: 105
- **Status**: ⚠️ **ISSUE** - Will display 105, which exceeds 99.9!

#### Case 8: Lights Configuration (line 3527)
- **Formula**: `ui16_display_data = ui16_display_data_factor / (ui8_display_lights_configuration * 100 + 5)`
- **Display value**: `ui8_display_lights_configuration * 100 + 5`
- **Range**: 5, 105, 205, 305... (if uint8_t: 0-255 → 5-25505)
- **Status**: ⚠️ **ISSUE** - Any non-zero value exceeds 99.9!

#### Case 9: Riding Mode (line 3530)
- **Formula**: `ui16_display_data = ui16_display_data_factor / (ui8_display_riding_mode * 100 + 5)`
- **Display value**: `ui8_display_riding_mode * 100 + 5`
- **Range**: 5, 105, 205, 305... (if uint8_t: 0-255 → 5-25505)
- **Status**: ⚠️ **ISSUE** - Any non-zero value exceeds 99.9!

### Main Data Display Cases (switch statement)

#### Case 10: Motor Temperature (line 3536-3539)
- **Formula KM**: `ui16_display_data = (ui16_display_data_factor / ui16_motor_temperature_filtered_x10) * 10U`
- **Formula MILES**: `ui16_display_data = ui16_display_data_factor / ui16_motor_temperature_filtered_x10`
- **Display value KM**: `ui16_motor_temperature_filtered_x10 / 100` (°C)
- **Display value MILES**: `ui16_motor_temperature_filtered_x10 / 10` (°C)
- **Range**: Typically 0-150°C (0-1500 in x10 format)
- **Status KM**: ✅ **SAFE** - Max 15.0°C < 99.9 (but seems wrong - should show actual temp)
- **Status MILES**: ⚠️ **POTENTIAL ISSUE** - Temp < 1.44°C (km/h) or < 0.9°C (mph) exceeds limit

#### Case 11: Battery SOC (line 3543-3546)
- **Same as Case 4** - Same issues apply

#### Case 12: Battery Voltage (line 3551)
- **Same as Case 5** - Safe

#### Case 13: Battery Current (line 3554)
- **Formula**: `ui16_display_data = ui16_display_data_factor / ui8_battery_current_filtered_x10`
- **Display value**: `ui8_battery_current_filtered_x10 / 10` (amps)
- **Range**: 0-255 (uint8_t), after /10: 0-25.5A
- **Status**: ✅ **SAFE** - Max 25.5A < 99.9

#### Case 14: Battery Power (line 3560-3563)
- **Formula KM**: `ui16_display_data = ui16_display_data_factor / ui16_battery_power_filtered_x10`
- **Formula MILES**: `ui16_display_data = ui16_display_data_factor / (ui16_battery_power_filtered_x10 / 10U)`
- **Display value KM**: `ui16_battery_power_filtered_x10 / 10` (watts)
- **Display value MILES**: `ui16_battery_power_filtered_x10 / 100` (watts)
- **Range**: Could be 0-2000W+ (0-20000+ in x10 format)
- **Status KM**: ⚠️ **POTENTIAL ISSUE** - Power < 14.4W (km/h) or < 9W (mph) exceeds limit
- **Status MILES**: ✅ **SAFE** - Max 20.0W < 99.9 (but seems wrong - should show actual power)

#### Case 15: Throttle ADC (line 3566)
- **Formula**: `ui16_display_data = ui16_display_data_factor / (ui16_adc_throttle >> 2)`
- **Display value**: `ui16_adc_throttle >> 2` (ADC value / 4)
- **Range**: 0-1023 (10-bit ADC), after >>2: 0-255
- **Status**: ✅ **SAFE** - Max 255 < 99.9 (but wait, 255 > 99.9!)
- **Status**: ⚠️ **ISSUE** - ADC values > 399 (>>2 = 99.75) will exceed 99.9!

#### Case 16: Pedal Torque (line 3569)
- **Formula**: `ui16_display_data = ui16_display_data_factor / ui16_adc_pedal_torque`
- **Display value**: `ui16_adc_pedal_torque` (raw ADC value)
- **Range**: 0-1023 (10-bit ADC)
- **Status**: ⚠️ **POTENTIAL ISSUE** - ADC < 144 (km/h) or < 90 (mph) exceeds limit

#### Case 17: Pedal Cadence (line 3573-3581)
- **Formula MILES**: `ui16_display_data = (ui16_display_data_factor / ui8_pedal_cadence_RPM) * 10U`
- **Formula KM**: Conditional - if > 99: `/ ui8_pedal_cadence_RPM`, else: `(ui16_display_data_factor / ui8_pedal_cadence_RPM) * 10U`
- **Display value MILES**: `ui8_pedal_cadence_RPM / 10` (RPM)
- **Display value KM**: If > 99: `ui8_pedal_cadence_RPM`, else: `ui8_pedal_cadence_RPM / 10`
- **Range**: 0-255 (uint8_t)
- **Status MILES**: ✅ **SAFE** - Max 25.5 RPM < 99.9 (but seems wrong - should show actual RPM)
- **Status KM**: ⚠️ **ISSUE** - If cadence > 99, displays directly, which can exceed 99.9!

#### Case 18: Wheel Speed (line 3585)
- **Formula**: `ui16_display_data = ui16_display_data_factor / ui16_oem_wheel_speed_time`
- **Display value**: `ui16_oem_wheel_speed_time` (time in ms/2)
- **Range**: Varies with speed - higher speed = lower time
- **Status**: ⚠️ **POTENTIAL ISSUE** - Very high speeds (low time values) could exceed limit
- **Note**: This is the same calculation used for actual wheel speed display, so it should be correct

#### Case 19: Battery Current Target (line 3589)
- **Formula**: `ui16_display_data = (ui16_display_data_factor / ui8_adc_battery_current_target) * 10U`
- **Display value**: `ui8_adc_battery_current_target / 10` (amps)
- **Range**: 0-255 (uint8_t), after /10: 0-25.5A
- **Status**: ✅ **SAFE** - Max 25.5A < 99.9
- **Comment says**: "value <= 99" - assumes ui8_adc_battery_current_target <= 99

#### Case 20: Watt Hours (line 3593-3596)
- **Formula KM**: `ui16_display_data = ui16_display_data_factor / (uint16_t) ui32_wh_x10`
- **Formula MILES**: `ui16_display_data = ui16_display_data_factor / (uint16_t) (ui32_wh_x10 / 10U)`
- **Display value KM**: `ui32_wh_x10 / 10` (Wh)
- **Display value MILES**: `ui32_wh_x10 / 100` (Wh)
- **Range**: Could be 0-10000+ Wh (0-100000+ in x10 format)
- **Status KM**: ⚠️ **POTENTIAL ISSUE** - Wh < 14.4 (km/h) or < 9 (mph) exceeds limit
- **Status MILES**: ✅ **SAFE** - Max 100.0Wh < 99.9 (but seems wrong - should show actual Wh)

#### Case 21: Motor Speed ERPS (line 3599)
- **Formula**: `ui16_display_data = ui16_display_data_factor / ui16_motor_speed_erps`
- **Display value**: `ui16_motor_speed_erps` (electrical revolutions per second)
- **Range**: Could be 0-650+ (see MOTOR_OVER_SPEED_ERPS)
- **Status**: ⚠️ **POTENTIAL ISSUE** - ERPS < 144 (km/h) or < 90 (mph) exceeds limit

#### Case 22: Duty Cycle Percent (line 3605)
- **Formula**: `ui16_display_data = (ui16_display_data_factor / ui16_duty_cycle_percent) * 10U`
- **ui16_duty_cycle_percent** = `(ui8_duty_cycle_target * 100 / 255) - 1` (range: -1 to 99)
- **Display value**: `ui16_duty_cycle_percent / 10` (0-9.9%)
- **Range**: -1 to 99 (but negative clamped to 0 in display)
- **Status**: ⚠️ **ISSUE** - If ui16_duty_cycle_percent = 0, division by zero or very large result!
- **Status**: ⚠️ **ISSUE** - If ui16_duty_cycle_percent < 14.4 (km/h) or < 9 (mph), exceeds 99.9!

## Summary of Issues

### Critical Issues (Will Always Exceed 99.9):
1. **FUNCTION_STATUS_OFF** (Case 6) - Displays 22,960-40,180
2. **FUNCTION_STATUS_ON** (Case 7) - Displays 105
3. **Lights Configuration** (Case 8) - Displays 105, 205, 305...
4. **Riding Mode** (Case 9) - Displays 105, 205, 305...
5. **Duty Cycle Percent** (Case 22) - Division by zero risk, small values exceed limit

### Potential Issues (May Exceed 99.9):
1. **Throttle ADC** (Case 15) - Values > 399 exceed limit
2. **Pedal Cadence KM** (Case 17) - Values > 99 exceed limit
3. **Pedal Torque** (Case 16) - Small ADC values exceed limit
4. **Motor Speed ERPS** (Case 21) - Small values exceed limit
5. **Battery Power KM** (Case 14) - Very low power exceeds limit
6. **Watt Hours KM** (Case 20) - Very low Wh exceeds limit
7. **Motor Temperature MILES** (Case 10) - Very low temps exceed limit
8. **Battery SOC KM** (Case 4, 11) - Very low SOC exceeds limit

### Safe Cases:
1. Torque sensor step (Case 2)
2. Battery voltage (Case 5, 12)
3. Battery current (Case 13)
4. Battery current target (Case 19) - if value <= 99 as comment states
5. Battery SOC MILES (Case 4, 11)
6. Battery power MILES (Case 14)
7. Motor temperature KM (Case 10) - but seems to show wrong scale
8. Pedal cadence MILES (Case 17) - but seems to show wrong scale

## Recommendations

1. **Add clamping**: Ensure `ui16_display_data` never results in display value > 99.9
2. **Fix function status cases**: These should not use the same calculation pattern
3. **Fix duty cycle**: Handle division by zero and small values
4. **Review scaling**: Some MILES cases seem to divide by 10 unnecessarily, making values too small
5. **Add validation**: Check input ranges before calculation to prevent overflow

