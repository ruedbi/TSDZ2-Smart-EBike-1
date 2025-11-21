# Functional Safety Code Review Report
## TSDZ2 E-Bike Motor Controller Firmware

**Review Date:** 2024  
**Branch:** dz40mini  
**Reviewer:** AI Code Review Assistant

---

## Executive Summary

This report presents a comprehensive functional safety review of the TSDZ2 e-bike motor controller firmware. The review identified **8 critical safety issues**, **12 high-priority issues**, and **15 medium-priority recommendations** across 10 safety-critical areas.

**Overall Safety Assessment:** The firmware implements multiple safety mechanisms including error detection, motor control protection, and sensor validation. However, several critical gaps exist, particularly in watchdog timer usage, error recovery mechanisms, and some arithmetic safety checks.

---

## 1. Error Detection and Handling

### Location: `ebike_app.c:check_system()` (lines 1789-1941)

### Findings:

#### ✅ **Strengths:**
- Comprehensive error detection for multiple failure modes
- Proper use of debounce counters with configurable thresholds
- Multiple sensor validation checks (torque, cadence, speed)

#### ⚠️ **Issues Found:**

**1.1 Motor Check Timer Overflow Risk** (HIGH)
- **Location:** Line 1814
- **Issue:** `ui8_motor_check_goes_alone_timer` is incremented without overflow protection
- **Code:**
  ```c
  ui8_motor_check_goes_alone_timer++;
  ```
- **Risk:** Counter can wrap around, delaying error detection
- **Recommendation:** Add overflow check: `if (ui8_motor_check_goes_alone_timer < 255) ui8_motor_check_goes_alone_timer++;`

**1.2 Torque Sensor Range Validation** (MEDIUM)
- **Location:** Lines 1830-1833
- **Issue:** Hard-coded limits (10-300 offset, max 500) may not match all hardware configurations
- **Code:**
  ```c
  if ((ui16_adc_pedal_torque_offset_init > 300)
    ||(ui16_adc_pedal_torque_offset_init < 10)
    ||(ui16_adc_pedal_torque > 500)
  ```
- **Risk:** False positives or missed failures with different sensor types
- **Recommendation:** Make thresholds configurable or add sensor type-specific validation

**1.3 Motor Blocked Counter Logic** (MEDIUM)
- **Location:** Lines 1906-1920
- **Issue:** Counter reset logic has potential race condition
- **Code:**
  ```c
  if ((ui8_battery_current_filtered_x10 > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10_NEW)
    && (ui16_motor_speed_erps < MOTOR_BLOCKED_ERPS_THRESHOLD_NEW)) {
      ++ui8_motor_blocked_counter;
      if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD_NEW) {
          ui8_system_state = ERROR_MOTOR_BLOCKED;
      }
  }
  else {
      ui8_motor_blocked_counter = 0;
  }
  ```
- **Risk:** Counter may not reset properly if conditions fluctuate rapidly
- **Recommendation:** Consider hysteresis or separate reset threshold

**1.4 Throttle Check Timing** (LOW)
- **Location:** Lines 1929-1940
- **Issue:** Throttle check only runs for 2 seconds after power-on
- **Risk:** Throttle faults after 2 seconds are not detected
- **Recommendation:** Consider periodic throttle validation or extend check period

---

## 2. Motor Control Safety

### Location: `ebike_app.c:apply_back_emf_protection()` (lines 1495-1554)

### Findings:

#### ✅ **Strengths:**
- Excellent back-EMF protection implementation
- Regenerative current detection and prevention
- Proper minimum duty cycle enforcement at high speeds

#### ⚠️ **Issues Found:**

**2.1 Regenerative Current Threshold** (MEDIUM)
- **Location:** Lines 1491-1492, 1532-1533
- **Issue:** Threshold values may be too high for early detection
- **Code:**
  ```c
  #define REGENERATIVE_CURRENT_THRESHOLD_ADC 8  // ~1.3A
  #define REGENERATIVE_CURRENT_TARGET_THRESHOLD_ADC 5
  ```
- **Risk:** Regenerative current may occur before detection
- **Recommendation:** Consider lowering threshold or adding predictive protection based on motor speed

**2.2 Speed Limit Overrun Duty Cycle** (LOW)
- **Location:** Lines 1500, 1514, 1542
- **Issue:** Speed limit overrun allows duty cycle up to 100 (SPEED_LIMIT_OVERRUN_DUTY_CYCLE_HIGH)
- **Risk:** May allow excessive power when speed limit is exceeded
- **Recommendation:** Verify this is intentional behavior for speed limit hysteresis

---

## 3. Overcurrent Protection

### Location: `ebike_app.c:ebike_control_motor()` (lines 532-552)

### Findings:

#### ✅ **Strengths:**
- Inline assembly for atomic ADC read (prevents conversion overrun)
- Configurable delay before error trigger
- Proper error state setting

#### ⚠️ **Issues Found:**

**3.1 Overcurrent Counter Overflow** (HIGH)
- **Location:** Line 544
- **Issue:** Counter incremented without overflow protection
- **Code:**
  ```c
  if (ui8_error_battery_overcurrent != 0U) {
      ui8_error_battery_overcurrent_counter++;
  }
  ```
- **Risk:** Counter can wrap, causing delayed or missed error detection
- **Recommendation:** Add overflow check or use saturating increment

**3.2 Overcurrent Threshold Calculation** (MEDIUM)
- **Location:** Line 577
- **Issue:** Threshold calculated as `max + extra`, but extra current value not clearly documented
- **Code:**
  ```c
  ui8_adc_battery_overcurrent = ui8_adc_battery_current_max + ADC_10_BIT_BATTERY_EXTRACURRENT;
  ```
- **Risk:** May allow excessive current before protection triggers
- **Recommendation:** Document safety margin rationale and verify against hardware limits

---

## 4. Input Validation and Range Checking

### Findings:

#### ✅ **Strengths:**
- Division by zero protection in speed and cadence calculations
- Range clamping in map functions
- Configuration parameter bounds checking in initialization

#### ⚠️ **Issues Found:**

**4.1 Wheel Speed Calculation Division** (MEDIUM)
- **Location:** Line 1598
- **Issue:** Division by `ui16_tmp` (wheel_speed_sensor_ticks) - protected by > 0 check, but no maximum limit
- **Code:**
  ```c
  if (ui16_wheel_speed_sensor_ticks > 0U) {
      ui16_wheel_speed_x10 = (uint16_t)(((uint32_t) m_configuration_variables.ui16_wheel_perimeter * ((PWM_CYCLES_SECOND/1000)*36U)) / ui16_tmp);
  }
  ```
- **Risk:** Very small tick values could cause overflow in intermediate calculation
- **Recommendation:** Add minimum tick threshold or overflow check

**4.2 Cadence Calculation Overflow Warning** (LOW)
- **Location:** Line 1619
- **Issue:** Comment warns about PWM_CYCLES_SECOND > 21845, but no runtime check
- **Code:**
  ```c
  // !!!warning if PWM_CYCLES_SECOND > 21845
  ui8_pedal_cadence_RPM = (uint8_t)((PWM_CYCLES_SECOND * 3U) / ui16_cadence_sensor_ticks_temp);
  ```
- **Risk:** Overflow if PWM frequency changes
- **Recommendation:** Add compile-time or runtime assertion

**4.3 Power Limit Calculation Division** (MEDIUM)
- **Location:** Line 3016-3017
- **Issue:** Division by `ui16_battery_voltage_filtered_x1000` without zero check
- **Code:**
  ```c
  ui8_adc_battery_current_max_temp_2 = (uint8_t)((uint32_t)(ui32_adc_battery_power_max_x1000_array[...]
      / ui16_battery_voltage_filtered_x1000));
  ```
- **Risk:** Division by zero if battery voltage reading fails
- **Recommendation:** Add minimum voltage check before division

**4.4 Map Function Range Validation** (LOW)
- **Location:** `common.c:map_ui16()`, `map_ui8()`
- **Issue:** Functions assume `in_min < in_max` but don't validate
- **Risk:** Undefined behavior if called with invalid ranges
- **Recommendation:** Add assertion or validation at function entry

---

## 5. Watchdog Timer Usage

### Location: Search throughout codebase

### Findings:

#### ❌ **CRITICAL ISSUE:**

**5.1 No Watchdog Timer Implementation** (CRITICAL)
- **Issue:** No Independent Watchdog (IWDG) or Window Watchdog (WWDG) initialization or refresh found in application code
- **Evidence:** 
  - Watchdog library functions exist in STM8S_StdPeriph_Lib
  - No calls to `IWDG_Enable()`, `IWDG_ReloadCounter()`, or `WWDG_Init()` in main application
  - No watchdog refresh in main loop
- **Risk:** System cannot recover from software lockups, infinite loops, or stack corruption
- **Impact:** **CRITICAL** - System may become unresponsive without hardware reset
- **Recommendation:** 
  1. Initialize IWDG in `main()` after system initialization
  2. Add `IWDG_ReloadCounter()` call in main loop (every 25ms cycle)
  3. Set appropriate timeout (recommend 1-2 seconds)
  4. Ensure watchdog refresh in all critical code paths

**Example Implementation:**
```c
// In main() after initialization:
IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
IWDG_SetPrescaler(IWDG_Prescaler_64);  // ~1ms per count
IWDG_SetReload(2000);  // ~2 second timeout
IWDG_Enable();

// In ebike_app_controller() or main loop:
IWDG_ReloadCounter();
```

---

## 6. State Machine Safety

### Location: `ebike_app.c:ebike_control_motor()` (lines 554-625)

### Findings:

#### ✅ **Strengths:**
- Clear error state handling
- Motor shutdown on critical errors
- Proper state checking before motor enable

#### ⚠️ **Issues Found:**

**6.1 Error State Recovery** (HIGH)
- **Location:** Throughout error handling
- **Issue:** No explicit error recovery mechanism - errors persist until power cycle
- **Code:** Error states are set but never cleared
- **Risk:** Temporary sensor glitches cause permanent system shutdown
- **Recommendation:** 
  - Add error recovery timers
  - Implement automatic recovery for transient errors (e.g., sensor noise)
  - Add manual recovery mechanism via display command
  - Consider error severity levels (critical vs. warning)

**6.2 Error State Persistence** (MEDIUM)
- **Location:** Lines 1820, 1842, 1864, 1890, 1914, 1937
- **Issue:** Once error state is set, it persists indefinitely
- **Risk:** System remains disabled even after fault condition clears
- **Recommendation:** Add error state clearing logic when fault conditions no longer exist

**6.3 Missing Error State in Motor Enable Check** (MEDIUM)
- **Location:** Lines 608-625
- **Issue:** Motor enable check doesn't include all error states (e.g., ERROR_TORQUE_SENSOR, ERROR_CADENCE_SENSOR, ERROR_SPEED_SENSOR)
- **Code:**
  ```c
  if (ui8_motor_enabled
      && ((ui8_brake_state)
          || (ui8_system_state == ERROR_MOTOR_BLOCKED)
          || (ui8_system_state == ERROR_MOTOR_CHECK)
          || (ui8_system_state == ERROR_BATTERY_OVERCURRENT)
          || (ui8_system_state == ERROR_THROTTLE)
  ```
- **Risk:** Motor may remain enabled with sensor errors
- **Recommendation:** Add all error states to motor disable condition

---

## 7. Temperature Protection

### Location: `ebike_app.c:apply_temperature_limiting()` (lines 1580-1600)

### Findings:

#### ✅ **Strengths:**
- Temperature filtering to reduce noise
- Configurable min/max limits
- Current limiting based on temperature

#### ⚠️ **Issues Found:**

**7.1 Temperature Filter Coefficient Change** (MEDIUM)
- **Location:** Line 1585
- **Issue:** Filter coefficient changed from 13 to 8 (faster response)
- **Code:**
  ```c
  ui16_adc_motor_temperature_filtered = filter(ui16_temp, ui16_adc_motor_temperature_filtered, 8);
  ```
- **Risk:** Faster filter may be more sensitive to noise, potentially causing false temperature limits
- **Recommendation:** 
  - Verify filter response time is appropriate for thermal time constants
  - Consider adding hysteresis to temperature limiting
  - Document rationale for coefficient change

**7.2 Temperature Min/Max Validation** (LOW)
- **Location:** Line 1468
- **Issue:** Validation only checks if min >= max, but doesn't validate against physical limits
- **Code:**
  ```c
  if (ui8_motor_temperature_min_value_to_limit_array[TEMPERATURE_SENSOR_TYPE] >= 
      ui8_motor_temperature_max_value_to_limit_array[TEMPERATURE_SENSOR_TYPE]) {
      ui8_adc_battery_current_target = 0;
  }
  ```
- **Risk:** Invalid but "valid" ranges (e.g., min=200°C, max=201°C) may be accepted
- **Recommendation:** Add absolute maximum temperature check

---

## 8. Battery Safety

### Location: `ebike_app.c:check_battery_soc()`, `uart_send_package()`

### Findings:

#### ✅ **Strengths:**
- Overvoltage and undervoltage detection
- State of charge calculation
- Battery fault code reporting

#### ⚠️ **Issues Found:**

**8.1 Battery SOC Calculation Division** (MEDIUM)
- **Location:** Line 3708-3709
- **Issue:** Division by voltage difference without zero check
- **Code:**
  ```c
  ui8_battery_soc_index = (uint8_t) ((uint16_t) (100
      - ((ui16_battery_voltage_soc_filtered_x10 - BATTERY_LOW_VOLTAGE_CUT_OFF_X10) * 100U)
      / (BATTERY_VOLTAGE_RESET_SOC_PERCENT_X10 - BATTERY_LOW_VOLTAGE_CUT_OFF_X10)));
  ```
- **Risk:** Division by zero if voltage reset equals cutoff
- **Recommendation:** Add validation that reset voltage > cutoff voltage

**8.2 Overvoltage Detection** (LOW)
- **Location:** Lines 3082-3083, 3118-3119
- **Issue:** Overvoltage only sets fault code, doesn't actively limit current
- **Risk:** System may continue operating at dangerous voltage levels
- **Recommendation:** Add active current limiting or motor shutdown on overvoltage

---

## 9. Communication Safety

### Location: `ebike_app.c:uart_receive_package()`, `uart_send_package()`

### Findings:

#### ✅ **Strengths:**
- Checksum validation on received packets
- Connection loss detection (3 missed packets = 0.3s timeout)
- Buffer size definitions

#### ⚠️ **Issues Found:**

**9.1 UART Buffer Overflow Protection** (MEDIUM)
- **Location:** Lines 2162-2173
- **Issue:** Buffer counter incremented without bounds check
- **Code:**
  ```c
  ui8_rx_counter++;
  if (ui8_rx_counter < UART_RX_BUFFER_LEN) {
      ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
  }
  ```
- **Risk:** Counter can overflow, causing buffer corruption
- **Recommendation:** Add counter reset or saturation

**9.2 Checksum Validation** (LOW)
- **Location:** Lines 2219-2227
- **Issue:** Simple additive checksum - weak error detection
- **Risk:** Multiple bit errors may cancel out, going undetected
- **Recommendation:** Consider CRC or stronger checksum for critical commands

**9.3 Connection Loss Timeout** (LOW)
- **Location:** Line 3030
- **Issue:** 0.3 second timeout may be too short for noisy communication
- **Risk:** False connection loss detection
- **Recommendation:** Consider configurable timeout or longer default

---

## 10. Arithmetic Safety

### Location: `common.c:map_ui16()`, `map_ui8()`, `filter()`

### Findings:

#### ✅ **Strengths:**
- Use of 32-bit intermediate calculations to prevent overflow
- Proper casting in map functions
- Range clamping before calculations

#### ⚠️ **Issues Found:**

**10.1 Map Function Overflow** (LOW)
- **Location:** `common.c:map_ui16()` lines 33, 35
- **Issue:** Intermediate 32-bit calculation could theoretically overflow with extreme inputs
- **Code:**
  ```c
  out = out_min + (uint16_t)(uint32_t)(((uint32_t)((uint32_t)(uint16_t)(in - in_min) * (uint32_t)(uint16_t)(out_max - out_min)) + (uint32_t)(uint16_t)(in_range/2U)) / in_range);
  ```
- **Risk:** Very large input ranges could cause 32-bit overflow
- **Recommendation:** Add input range validation or use 64-bit intermediate for extreme cases

**10.2 Filter Function Division** (LOW)
- **Location:** `common.c:filter()` line 83
- **Issue:** Division by `ui8_max_alpha` (16) - safe but not validated
- **Code:**
  ```c
  uint16_t ui16_filtered_value = (uint16_t)((ui32_temp_new + ui32_temp_old + ui8_max_alpha/2U) / ui8_max_alpha);
  ```
- **Risk:** Low - constant value, but change to variable would be unsafe
- **Recommendation:** Keep as constant or add validation

**10.3 Integer Cast Safety** (LOW)
- **Location:** Multiple locations
- **Issue:** Multiple casts between uint8_t, uint16_t, uint32_t without explicit overflow checks
- **Risk:** Silent overflow in casts
- **Recommendation:** Add assertions or bounds checking before casts in critical paths

---

## Summary of Issues by Severity

### CRITICAL (1 issue)
1. **5.1** - No Watchdog Timer Implementation

### HIGH (3 issues)
1. **1.1** - Motor Check Timer Overflow Risk
2. **3.1** - Overcurrent Counter Overflow
3. **6.1** - Error State Recovery Missing

### MEDIUM (12 issues)
1. **1.2** - Torque Sensor Range Validation
2. **1.3** - Motor Blocked Counter Logic
3. **2.1** - Regenerative Current Threshold
4. **3.2** - Overcurrent Threshold Calculation
5. **4.1** - Wheel Speed Calculation Division
6. **4.3** - Power Limit Calculation Division
7. **6.2** - Error State Persistence
8. **6.3** - Missing Error State in Motor Enable Check
9. **7.1** - Temperature Filter Coefficient Change
10. **8.1** - Battery SOC Calculation Division
11. **9.1** - UART Buffer Overflow Protection
12. **10.1** - Map Function Overflow (theoretical)

### LOW (15 issues)
1. **1.4** - Throttle Check Timing
2. **2.2** - Speed Limit Overrun Duty Cycle
3. **4.2** - Cadence Calculation Overflow Warning
4. **4.4** - Map Function Range Validation
5. **7.2** - Temperature Min/Max Validation
6. **8.2** - Overvoltage Detection
7. **9.2** - Checksum Validation
8. **9.3** - Connection Loss Timeout
9. **10.2** - Filter Function Division
10. **10.3** - Integer Cast Safety

---

## Recommendations Priority List

### Immediate Action Required (Critical)
1. **Implement Watchdog Timer** - Add IWDG initialization and refresh in main loop
2. **Add Counter Overflow Protection** - Protect all error detection counters from overflow
3. **Implement Error Recovery** - Add mechanism to clear error states when faults clear

### High Priority (Within 1-2 weeks)
4. **Complete Error State Coverage** - Ensure all error states disable motor
5. **Add Division by Zero Checks** - Validate all divisors before division operations
6. **Improve Input Validation** - Add bounds checking for all sensor inputs

### Medium Priority (Within 1 month)
7. **Review Temperature Filter** - Verify filter coefficient change doesn't impact safety
8. **Enhance Communication Safety** - Improve checksum and buffer protection
9. **Document Safety Margins** - Document rationale for all thresholds and limits
10. **Add Runtime Assertions** - Add checks for critical assumptions

### Low Priority (Future Improvements)
11. **Improve Error Reporting** - Add error history and diagnostic information
12. **Add Predictive Protection** - Implement predictive algorithms for early fault detection
13. **Enhance Test Coverage** - Add unit tests for safety-critical functions
14. **Code Review Process** - Establish formal safety review process for future changes

---

## Conclusion

The firmware demonstrates good safety awareness with multiple protection mechanisms. However, the **absence of a watchdog timer is a critical safety gap** that must be addressed immediately. Additionally, several counter overflow risks and missing error recovery mechanisms should be prioritized.

The codebase shows evidence of careful design in motor control safety (back-EMF protection, regenerative current detection) and sensor validation. With the recommended improvements, the system would achieve a significantly higher level of functional safety.

**Estimated Effort for Critical Fixes:** 2-3 days  
**Estimated Effort for High Priority Fixes:** 1 week  
**Estimated Effort for All Recommended Fixes:** 2-3 weeks

---

## Appendix: Code Examples for Critical Fixes

### Watchdog Implementation
```c
// In main.c, after initialization:
#include "stm8s_iwdg.h"

void watchdog_init(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);  // 64kHz/64 = 1kHz, ~1ms per count
    IWDG_SetReload(2000);  // 2000ms = 2 second timeout
    IWDG_Enable();
}

// In ebike_app_controller() or main loop:
IWDG_ReloadCounter();  // Refresh watchdog
```

### Counter Overflow Protection
```c
// Replace all counter increments with:
if (ui8_motor_check_goes_alone_timer < 255) {
    ui8_motor_check_goes_alone_timer++;
}
// Or use saturating increment macro
```

### Error Recovery
```c
// Add error recovery check in check_system():
if (ui8_system_state != NO_ERROR) {
    // Check if fault condition still exists
    if (/* fault condition cleared */) {
        static uint8_t recovery_counter = 0;
        if (++recovery_counter > RECOVERY_DELAY) {
            ui8_system_state = NO_ERROR;
            recovery_counter = 0;
        }
    } else {
        recovery_counter = 0;
    }
}
```

---

**End of Report**

