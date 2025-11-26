# Motor Check Safety System - Diagrams

## Overview
The motor check system protects users against unwanted motor rotation that could turn pedals due to software or hardware bugs.

---

## Diagram 1: Overall System Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    ebike_app_controller()                    │
│                    (called every 25ms)                       │
└────────────────────────┬──────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              Calculate Motor Speed (ERPS)                    │
│         ui16_motor_speed_erps = f(ui16_hall_counter_total)  │
└────────────────────────┬──────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              check_system()                                  │
│              (called every 100ms)                            │
└────────────────────────┬──────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│         Motor Check Logic (Lines 1836-1851)                  │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 2: Motor Check Condition Logic

```
┌─────────────────────────────────────────────────────────────┐
│                    Motor Check Trigger                       │
└─────────────────────────────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────┐
│ Condition 1:     │          │ Condition 2:     │
│ Motor Speed      │    AND   │ Mode Check       │
│ > 20 ERPS        │          │                  │
└──────────────────┘          └──────────────────┘
                                       │
                    ┌──────────────────┼──────────────────┐
                    │                  │                  │
                    ▼                  ▼                  ▼
         ┌──────────────────┐  ┌──────────────────┐
         │ Assist = OFF      │  │ Torque Mode OR   │
         │ (ALL modes)       │  │ Cadence Mode    │
         └──────────────────┘  └──────────────────┘
                    │                  │
                    └────────┬─────────┘
                             │
                             ▼
                    ┌──────────────────┐
                    │ Condition 3:     │
                    │ Both Targets = 0│
                    │ (AND, not OR)   │
                    └──────────────────┘
                             │
                             ▼
                    ┌──────────────────┐
                    │ Condition 4:     │
                    │ Pedal Cadence    │
                    │ < 5 RPM          │
                    └──────────────────┘
                             │
                             ▼
                    ┌──────────────────┐
                    │ Increment Timer  │
                    │ (ui8_motor_check_ │
                    │  goes_alone_timer)│
                    └──────────────────┘
```

---

## Diagram 3: Mode Coverage Matrix

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Motor Check Coverage                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  Assist Level: OFF                                                    │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ ✓ POWER_ASSIST_MODE      ✓ TORQUE_ASSIST_MODE              │    │
│  │ ✓ CADENCE_ASSIST_MODE    ✓ eMTB_ASSIST_MODE                │    │
│  │ ✓ HYBRID_ASSIST_MODE     ✓ CRUISE_MODE                     │    │
│  │ ✓ WALK_ASSIST_MODE       ✓ TORQUE_SENSOR_CALIBRATION_MODE  │    │
│  └─────────────────────────────────────────────────────────────┘    │
│  ALL MODES CHECKED WHEN ASSIST = OFF                                  │
│                                                                       │
│  Assist Level: ON (ECO, TOUR, SPORT, TURBO)                          │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ ✓ POWER_ASSIST_MODE      ✓ TORQUE_ASSIST_MODE              │    │
│  │ ✓ CADENCE_ASSIST_MODE    ✓ eMTB_ASSIST_MODE                │    │
│  │ ✓ HYBRID_ASSIST_MODE     ✗ CRUISE_MODE                     │    │
│  │ ✗ WALK_ASSIST_MODE       ✗ TORQUE_SENSOR_CALIBRATION_MODE   │    │
│  └─────────────────────────────────────────────────────────────┘    │
│  Only torque-based and cadence modes checked when assist is ON       │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Diagram 4: State Machine - Timer Logic

```
                    ┌──────────────┐
                    │   START      │
                    │ Timer = 0    │
                    └──────┬───────┘
                           │
                           ▼
         ┌─────────────────────────────────┐
         │  Check Conditions Met?          │
         └──────┬──────────────────┬───────┘
                │ YES               │ NO
                ▼                   ▼
    ┌──────────────────┐    ┌──────────────────┐
    │ Increment Timer  │    │ Reset Timer = 0  │
    │ timer++          │    │                  │
    └──────┬───────────┘    └────────┬─────────┘
           │                         │
           ▼                         │
    ┌──────────────────┐             │
    │ Timer > 60?      │             │
    │ (6.0 seconds)    │             │
    └──────┬───────────┘             │
           │                         │
      ┌────┴────┐                    │
      │ YES     │ NO                 │
      ▼         ▼                     │
┌──────────┐   │                     │
│ SET ERROR│   │                     │
│ ERROR_   │   │                     │
│ MOTOR_   │   │                     │
│ CHECK    │   └─────────────────────┘
└──────────┘         │
      │              │
      └──────────────┘
           │
           ▼
    ┌──────────────────┐
    │ Motor Disabled   │
    │ (Safety)         │
    └──────────────────┘
```

---

## Diagram 5: Condition Details - Before vs After Fix

### BEFORE FIX (Incorrect Logic)

```
┌─────────────────────────────────────────────────────────────┐
│                    OLD CONDITION (WRONG)                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Motor Speed > 20 ERPS                                       │
│  AND                                                          │
│  (Torque Mode OR Cadence Mode)  ← Only these modes           │
│  AND                                                          │
│  (Current = 0 OR Duty = 0)      ← WRONG: OR instead of AND  │
│  AND                                                          │
│  Pedal Cadence < 5 RPM                                       │
│                                                               │
│  ❌ Problem: Doesn't check when assist = OFF                 │
│  ❌ Problem: OR logic allows false positives                  │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### AFTER FIX (Correct Logic)

```
┌─────────────────────────────────────────────────────────────┐
│                    NEW CONDITION (CORRECT)                   │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Motor Speed > 20 ERPS                                       │
│  AND                                                          │
│  (Assist = OFF OR Torque Mode OR Cadence Mode)  ← All modes  │
│  AND                                                          │
│  (Current = 0 AND Duty = 0)      ← CORRECT: Both must be 0  │
│  AND                                                          │
│  Pedal Cadence < 5 RPM                                       │
│                                                               │
│  ✅ Always checks when assist = OFF (safety critical)       │
│  ✅ AND logic prevents false positives                       │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 6: Safety Protection Scenarios

```
┌─────────────────────────────────────────────────────────────┐
│              Scenario Analysis                                │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Scenario 1: Motor rotating, assist OFF, no pedaling          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Motor ERPS: 25 (> 20) ✓                             │   │
│  │ Assist Level: OFF ✓                                 │   │
│  │ Current Target: 0 ✓                                 │   │
│  │ Duty Target: 0 ✓                                    │   │
│  │ Pedal Cadence: 0 (< 5) ✓                            │   │
│  │ → TIMER INCREMENTS → ERROR AFTER 6 SECONDS          │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
│  Scenario 2: Motor rotating, assist ON, no command, no pedals │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Motor ERPS: 25 (> 20) ✓                             │   │
│  │ Mode: TORQUE_ASSIST_MODE ✓                          │   │
│  │ Current Target: 0 ✓                                 │   │
│  │ Duty Target: 0 ✓                                    │   │
│  │ Pedal Cadence: 0 (< 5) ✓                            │   │
│  │ → TIMER INCREMENTS → ERROR AFTER 6 SECONDS          │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
│  Scenario 3: Motor rotating, assist ON, command active        │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Motor ERPS: 25 (> 20) ✓                             │   │
│  │ Mode: TORQUE_ASSIST_MODE ✓                          │   │
│  │ Current Target: 50 ✗ (not 0)                        │   │
│  │ Duty Target: 100 ✗ (not 0)                           │   │
│  │ → TIMER RESETS (normal operation)                    │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
│  Scenario 4: Motor rotating, assist ON, pedals moving         │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Motor ERPS: 25 (> 20) ✓                             │   │
│  │ Mode: TORQUE_ASSIST_MODE ✓                          │   │
│  │ Current Target: 0 ✓                                 │   │
│  │ Duty Target: 0 ✓                                    │   │
│  │ Pedal Cadence: 15 RPM ✗ (>= 5)                     │   │
│  │ → TIMER RESETS (pedals are moving, normal)           │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## Diagram 7: Code Structure Flow

```
check_system() [called every 100ms]
│
├─► Determine Riding Mode Type
│   │
│   ├─► ui8_riding_torque_mode = 1
│   │   └─► (POWER/TORQUE/HYBRID/eMTB) AND no throttle
│   │
│   └─► ui8_riding_torque_mode = 0
│       └─► Other modes or throttle active
│
├─► Motor Check Logic
│   │
│   ├─► Condition Check:
│   │   ├─► ui16_motor_speed_erps > 20 ERPS?
│   │   ├─► (Assist = OFF) OR (Torque Mode) OR (Cadence Mode)?
│   │   ├─► (Current = 0) AND (Duty = 0)?
│   │   └─► Pedal Cadence < 5 RPM?
│   │
│   ├─► If ALL conditions met:
│   │   └─► ui8_motor_check_goes_alone_timer++
│   │
│   └─► If ANY condition NOT met:
│       └─► ui8_motor_check_goes_alone_timer = 0
│
└─► Error Check
    │
    └─► If timer > 60 (6 seconds):
        └─► ui8_system_state = ERROR_MOTOR_CHECK
            └─► Motor will be disabled (see line 617)
```

---

## Diagram 8: Timing Diagram

```
Time (100ms intervals)
│
0ms    100ms   200ms   300ms   ...   6000ms
│       │       │       │              │
│       │       │       │              │
▼       ▼       ▼       ▼              ▼
┌───┐  ┌───┐  ┌───┐  ┌───┐          ┌───┐
│ 0 │→ │ 1 │→ │ 2 │→ │ 3 │→ ... → │60 │→ ERROR
└───┘  └───┘  └───┘  └───┘          └───┘
Timer increments if conditions met continuously

If ANY condition fails:
┌───┐  ┌───┐  ┌───┐
│ 5 │→ │ 0 │  │ 0 │  (Timer resets)
└───┘  └───┘  └───┘
```

---

## Key Constants

```
MOTOR_CHECK_ERPS_THRESHOLD = 20 ERPS
  └─► Motor must be rotating faster than ~60 RPM to trigger check

MOTOR_CHECK_TIME_GOES_ALONE_TRESHOLD = 60 (100ms intervals)
  └─► 60 × 100ms = 6.0 seconds of continuous unwanted rotation

Pedal Cadence Threshold = 5 RPM
  └─► Pedals must be essentially stopped (< 5 RPM)
```

---

## Summary

The motor check system provides **defense in depth** protection:

1. **Continuous Monitoring**: Checks every 100ms
2. **Comprehensive Coverage**: All modes when assist is OFF
3. **Strict Conditions**: Requires BOTH targets to be zero (AND logic)
4. **Time-Based Detection**: 6 seconds of continuous unwanted rotation triggers error
5. **Automatic Protection**: Error state automatically disables motor

This ensures that if a software or hardware bug causes unwanted motor rotation, the system will detect and stop it within 6 seconds, preventing the motor from turning the pedals.

