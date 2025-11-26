# Motor Operation Diagrams

## Overview
This document contains diagrams showing how the TSDZ2 motor controller operates, including control flow, assist modes, and power management.

---

## Diagram 1: Overall Motor Control Flow

```
┌─────────────────────────────────────────────────────────────┐
│              ebike_app_controller()                         │
│              (called every 25ms)                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Sensor Reading & Calculations                        │
│  • Battery voltage                                           │
│  • Pedal torque (ADC)                                       │
│  • Pedal cadence (RPM)                                      │
│  • Wheel speed                                              │
│  • Motor speed (ERPS)                                       │
│  • Brake state                                              │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              ebike_control_motor()                          │
│              (called every 25ms)                            │
└────────────────────────┬────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────┐
│ Reset Control    │          │ Field Weakening  │
│ Variables        │          │ Check            │
│ (Safety)         │          │                  │
└──────────────────┘          └──────────────────┘
         │                               │
         └───────────────┬───────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              Select Riding Mode                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ POWER_ASSIST_MODE    → apply_power_assist()        │   │
│  │ TORQUE_ASSIST_MODE   → apply_torque_assist()       │   │
│  │ CADENCE_ASSIST_MODE  → apply_cadence_assist()      │   │
│  │ eMTB_ASSIST_MODE     → apply_emtb_assist()         │   │
│  │ HYBRID_ASSIST_MODE   → apply_hybrid_assist()       │   │
│  │ CRUISE_MODE          → apply_cruise()              │   │
│  │ WALK_ASSIST_MODE     → apply_walk_assist()         │   │
│  └─────────────────────────────────────────────────────┘   │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Optional Functions                                   │
│  • Throttle control (if enabled)                            │
│  • Temperature limiting (if enabled)                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Safety & Limits                                      │
│  • Speed limit                                              │
│  • Back-EMF protection                                      │
│  • Battery overcurrent check                                │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Controller Parameters                            │
│  • Duty cycle ramp up/down                                  │
│  • Battery current target                                   │
│  • Duty cycle target                                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Motor Enable/Disable Logic                          │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 2: Motor Enable/Disable State Machine

```
                    ┌──────────────┐
                    │   START      │
                    │ Motor = OFF  │
                    └──────┬───────┘
                           │
                           ▼
         ┌─────────────────────────────────┐
         │  Check Enable Conditions        │
         │  • Brake = OFF                  │
         │  • Motor speed < re-enable ERPS │
         │  • Current target > 0            │
         └──────┬──────────────────┬───────┘
                │ YES               │ NO
                ▼                   ▼
    ┌──────────────────┐    ┌──────────────────┐
    │ Enable Motor     │    │ Motor Stays OFF  │
    │ • Set enabled=1  │    │                  │
    │ • Reset duty=0   │    │                  │
    │ • Reset FW=0     │    │                  │
    │ • Enable PWM     │    │                  │
    └──────┬───────────┘    └────────┬─────────┘
           │                         │
           │                         │
           ▼                         │
    ┌──────────────────┐             │
    │  Motor = ON      │             │
    │  (Running)       │             │
    └──────┬───────────┘             │
           │                         │
           │                         │
           ▼                         │
    ┌─────────────────────────────────┐
    │  Check Disable Conditions      │
    │  • Brake = ON?                  │
    │  • Error state?                 │
    │  • SOC saved flag?              │
    │  • Motor stopped + no current?  │
    └──────┬──────────────────┬───────┘
           │ YES               │ NO
           ▼                   │
    ┌──────────────────┐       │
    │ Disable Motor    │       │
    │ • Set enabled=0  │       │
    │ • Disable PWM    │       │
    └──────────────────┘       │
           │                   │
           └───────────────────┘
```

---

## Diagram 3: Power Assist Mode Flow

```
┌─────────────────────────────────────────────────────────────┐
│              apply_power_assist()                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Check Assist Without Pedal Rotation                  │
│  (if enabled and torque > threshold)                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Apply Startup Boost                                  │
│  (if enabled)                                               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate Human Power                                │
│  Pedal Torque (ADC) × Cadence (RPM)                         │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate Assist Power                               │
│  Human Power × Power Assist Multiplier                       │
│  Formula: (torque × cadence × multiplier) / 480            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate Battery Current Target                    │
│  Current = (Assist Power × 1000) / Battery Voltage          │
│  Convert to ADC steps                                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Motor Ramp                                      │
│  (acceleration/deceleration based on speed)                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Limit Current to Max                                │
│  If target > max → set to max                               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Duty Cycle Target                                │
│  If current > 0 → duty = MAX                                │
│  Else → duty = 0                                            │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 4: Torque Assist Mode Flow

```
┌─────────────────────────────────────────────────────────────┐
│              apply_torque_assist()                           │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Apply Smooth Start                                  │
│  (if enabled)                                               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Check Assist Without Pedal Rotation                  │
│  (if enabled and torque > threshold)                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate Torque Assist Current                     │
│  Current = (Torque Delta × Torque Factor) / 110             │
│  Directly proportional to pedal torque                      │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Motor Ramp                                      │
│  (acceleration/deceleration based on speed)                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Limit Current to Max                                │
│  If target > max → set to max                               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Duty Cycle Target                                │
│  If current > 0 → duty = MAX                                │
│  Else → duty = 0                                            │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 5: Cadence Assist Mode Flow

```
┌─────────────────────────────────────────────────────────────┐
│              apply_cadence_assist()                          │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Check Pedal Cadence > 0                             │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Simulate Pedal Torque                                │
│  Simulated torque = assist_level + cadence                  │
│  (used for calculation, not actual torque)                   │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Apply Smooth Start                                  │
│  (gradual ramp-up)                                          │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate Cadence Assist Current                    │
│  Current = (simulated_torque × 200000) / Battery Voltage    │
│  Convert to ADC steps                                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Restore Actual Torque Delta                         │
│  (restore original value after calculation)                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Motor Ramp                                      │
│  (acceleration/deceleration based on speed)                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Limit Current to Max                                │
│  If target > max → set to max                               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Duty Cycle Target                                │
│  If current > 0 → duty = MAX                                │
│  Else → duty = 0                                            │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 6: Field Weakening Operation

```
┌─────────────────────────────────────────────────────────────┐
│         Field Weakening Check                                │
│         (in ebike_control_motor)                            │
└────────────────────────┬────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────┐
│ Motor Speed >    │    AND   │ Current < Target │
│ FW Threshold?    │          │                  │
│ (490 ERPS)       │          │                  │
└──────────────────┘          └──────────────────┘
         │                               │
         └───────────────┬───────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate Field Weakening Offset                     │
│  Delta = Motor Speed - FW Threshold                         │
│  Offset Max = Delta / 32                                    │
│  Limited to FW_HALL_COUNTER_OFFSET_MAX                       │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Enable Field Weakening                               │
│  Set ui8_field_weakening_enabled = 1                        │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Motor Controller (motor.c)                          │
│         PWM Interrupt Handler                                │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Field Weakening Logic                                │
│  When:                                                       │
│  • Field weakening enabled                                  │
│  • Duty cycle = MAX                                         │
│  • Current < target                                         │
│  Then:                                                      │
│  • Increment hall counter offset                            │
│  • Advance commutation angle                                │
│  • Allows higher motor speed                                │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 7: Duty Cycle and Current Control Loop

```
┌─────────────────────────────────────────────────────────────┐
│         Motor Controller (motor.c)                          │
│         PWM Interrupt Handler                                │
│         (called at high frequency)                           │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Read Current State                                   │
│  • Actual duty cycle (ui8_g_duty_cycle)                     │
│  • Actual battery current (filtered)                        │
│  • Target duty cycle                                        │
│  • Target battery current                                   │
└────────────────────────┬────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────┐
│ Decrease Duty?   │    OR    │ Increase Duty?   │
│ Conditions:      │          │ Conditions:      │
│ • Target < actual│          │ • Target > actual│
│ • Current > target│         │ • Current < target│
│ • Phase current > max│      │                  │
│ • Motor overspeed│          │                  │
│ • Low voltage    │          │                  │
│ • Brake active   │          │                  │
└──────────────────┘          └──────────────────┘
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────┐
│ Ramp Down        │          │ Ramp Up          │
│ • Decrement duty │          │ • Increment duty │
│ • Or decrement FW│          │ • Or increment FW│
│   offset         │          │   (if at max)    │
└──────────────────┘          └──────────────────┘
         │                               │
         └───────────────┬───────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Apply Duty Cycle to PWM                              │
│  • Update PWM registers                                      │
│  • Control motor phase currents                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 8: Safety and Protection Systems

```
┌─────────────────────────────────────────────────────────────┐
│         Safety Checks (in ebike_control_motor)               │
└────────────────────────┬────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────┐
│ Safety Conditions│          │ Reset Control    │
│ Check            │          │ Variables        │
└──────────────────┘          └──────────────────┘
         │                               │
         ▼                               ▼
┌─────────────────────────────────────────────────────────────┐
│         Safety Conditions (Disable Motor)                    │
│  • Brake active                                              │
│  • Motor not enabled                                        │
│  • ERROR_MOTOR_BLOCKED                                      │
│  • ERROR_MOTOR_CHECK                                        │
│  • ERROR_BATTERY_OVERCURRENT                                │
│  • ERROR_THROTTLE                                           │
│  • Assist level = OFF                                       │
│  • Riding mode parameter = 0                                │
│  • Battery SOC saved flag                                   │
│  • System error (if error assist disabled)                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         If Safety Condition Met                              │
│  • Reset ramp up/down to defaults                           │
│  • Set current target = 0                                    │
│  • Set duty cycle target = 0                                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         If Safe to Operate                                  │
│  • Limit current to hardware max                            │
│  • Set overcurrent threshold                                │
│  • Limit target current to max                              │
│  • Limit ramp steps to min values                           │
│  • Apply control parameters to controller                   │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 9: Speed Limiting and Back-EMF Protection

```
┌─────────────────────────────────────────────────────────────┐
│         Speed Limit Check                                    │
│         apply_speed_limit()                                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Check if Speed Limit Enabled                        │
│  (ui8_wheel_speed_max > 0)                                   │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Compare Wheel Speed to Limit                        │
│  If speed > limit:                                          │
│  • Reduce duty cycle target                                 │
│  • Gradually ramp down power                                │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Back-EMF Protection                                  │
│         apply_back_emf_protection()                          │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Check Motor Speed                                    │
│  If motor speed > field weakening threshold:                │
│  • Calculate minimum duty cycle for motor speed             │
│  • Ensure duty cycle doesn't go too low                     │
│  • Prevents regenerative current                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Detect Regenerative Current                          │
│  If:                                                         │
│  • Current target is very low                               │
│  • Actual current is negative (regenerative)                 │
│  • Motor speed is high                                      │
│  Then:                                                       │
│  • Increase duty cycle to prevent regeneration             │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 10: Hybrid Assist Mode (Power + Torque)

```
┌─────────────────────────────────────────────────────────────┐
│              apply_hybrid_assist()                           │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Check Pedal Cadence > 0                              │
│  (or startup assist active)                                  │
└────────────────────────┬────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────┐
│ Calculate        │          │ Calculate        │
│ Torque Assist    │          │ Power Assist     │
│ Current          │          │ Current          │
└──────────────────┘          └──────────────────┘
         │                               │
         │                               │
         │  Torque Current =             │
         │  (torque_delta × factor) / 110│
         │                               │
         │  Power Current =              │
         │  (power × 1000) / voltage     │
         │                               │
         └───────────────┬───────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Select Maximum Current                              │
│  Target = MAX(torque_current, power_current)               │
│  (whichever is higher)                                       │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Motor Ramp                                      │
│  (acceleration/deceleration based on speed)                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Limit and Apply                                     │
│  • Limit to max current                                     │
│  • Set duty cycle target                                    │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 11: Cruise Control Mode (PID)

```
┌─────────────────────────────────────────────────────────────┐
│              apply_cruise()                                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Initialize PID Controller                           │
│  (on first call or mode change)                             │
│  • Reset error, integral, derivative                        │
│  • Set target speed from assist level                       │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate PID Error                                 │
│  Error = Target Speed - Actual Speed                         │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate PID Terms                                 │
│  • Proportional: KP × error                                 │
│  • Integral: KI × sum of errors                            │
│  • Derivative: KD × (error - last_error)                   │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Calculate Control Output                            │
│  Output = P + I + D                                         │
│  Limited to positive values and max limit                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Map Output to Duty Cycle                            │
│  Duty cycle = map(output, 0, max, 0, PWM_MAX)               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Set Control Parameters                              │
│  • Current target = max                                     │
│  • Duty cycle target = mapped value                         │
│  • Ramp up/down = cruise specific values                   │
└─────────────────────────────────────────────────────────────┘
```

---

## Key Variables and Constants

### Motor Control Variables
- `ui8_g_duty_cycle`: Actual duty cycle applied to motor (0-255)
- `ui8_controller_duty_cycle_target`: Target duty cycle from assist mode
- `ui8_adc_battery_current_target`: Target battery current in ADC steps
- `ui8_controller_adc_battery_current_target`: Target current sent to controller
- `ui16_motor_speed_erps`: Motor speed in electrical revolutions per second

### Ramp Control
- `ui8_controller_duty_cycle_ramp_up_inverse_step`: Ramp up rate (higher = slower)
- `ui8_controller_duty_cycle_ramp_down_inverse_step`: Ramp down rate (higher = slower)

### Field Weakening
- `ui8_field_weakening_enabled`: Field weakening active flag
- `ui8_fw_hall_counter_offset`: Current field weakening angle offset
- `ui8_fw_hall_counter_offset_max`: Maximum field weakening offset
- `MOTOR_SPEED_FIELD_WEAKENING_MIN`: Minimum speed for field weakening (490 ERPS)

### Safety Limits
- `ADC_10_BIT_BATTERY_CURRENT_MAX`: Maximum battery current (18A default)
- `PWM_DUTY_CYCLE_MAX`: Maximum duty cycle (255)
- `PWM_DUTY_CYCLE_STARTUP`: Startup duty cycle (30)
- `ERPS_SPEED_OF_MOTOR_REENABLING`: Max speed to re-enable motor

---

## Summary

The motor control system operates in several layers:

1. **High-Level Control** (ebike_app.c): Determines assist mode, calculates power/current targets
2. **Low-Level Control** (motor.c): PWM interrupt handler manages duty cycle ramping and field weakening
3. **Safety Systems**: Multiple checks prevent unsafe operation
4. **Protection Systems**: Speed limiting, back-EMF protection, overcurrent protection

The system uses a cascaded control approach:
- Assist modes calculate **battery current targets** based on rider input
- Current targets are converted to **duty cycle targets**
- The PWM interrupt handler **ramps duty cycle** smoothly to targets
- **Field weakening** extends speed range when at maximum duty cycle

