# Control Flow: ui8_adc_battery_current_target and ui8_duty_cycle_target
## When ADD_MINIMUM_POWER is defined

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ebike_app_controller() [called every 25ms]              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│              ebike_control_motor() [main control function]                 │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Line 482-483: RESET                                                         │
│  ui8_adc_battery_current_target = 0                                         │
│  ui8_duty_cycle_target = 0                                                   │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Line 503-512: Riding Mode Selection                                         │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │ apply_power_assist()                                                  │  │
│  │ apply_torque_assist()                                                 │  │
│  │ apply_cadence_assist()                                                │  │
│  │ apply_emtb_assist()                                                   │  │
│  │ apply_hybrid_assist()                                                 │  │
│  │ apply_cruise()                                                        │  │
│  │ apply_walk_assist()                                                   │  │
│  │                                                                       │  │
│  │ These functions modify:                                               │  │
│  │ • ui8_adc_battery_current_target (multiple locations)                 │  │
│  │ • ui8_duty_cycle_target (multiple locations)                          │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Line 515-521: Optional ADC Function                                         │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │ apply_throttle() OR apply_temperature_limiting()                     │  │
│  │                                                                       │  │
│  │ May modify:                                                          │  │
│  │ • ui8_adc_battery_current_target (line 1450, 1485)                   │  │
│  │ • ui8_duty_cycle_target (line 1454)                                   │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Line 524: apply_speed_limit()                                               │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │ Line 1576: ui8_adc_battery_current_target = map_ui16(...)            │  │
│  │                                                                       │  │
│  │ Line 1583: if (ui16_wheel_speed_x10 > speed_limit_high)              │  │
│  │   Line 1587: #if defined DEBUG_BUILD || defined ADD_MINIMUM_POWER     │  │
│  │                                                                       │  │
│  │     ╔═══════════════════════════════════════════════════════════╗   │  │
│  │     ║  ADD_MINIMUM_POWER CODE BLOCK (when speed limit exceeded) ║   │  │
│  │     ╚═══════════════════════════════════════════════════════════╝   │  │
│  │                                                                       │  │
│  │     Line 1604: ui8_duty_cycle_target = map_ui8(...)                  │  │
│  │                [MODIFIED - based on assist level]                    │  │
│  │                                                                       │  │
│  │     Line 1615: Calculate ui8_max_allowed_current_for_duty             │  │
│  │                                                                       │  │
│  │     Line 1617: ui8_adc_battery_current_target =                       │  │
│  │                ui8_max_allowed_current_for_duty                       │  │
│  │                [MODIFIED - to match duty cycle]                       │  │
│  │                                                                       │  │
│  │     Line 1628: ui8_adc_battery_current_target =                       │  │
│  │                SPEED_LIMIT_MAX_CURRENT_ADC                            │  │
│  │                [MODIFIED - LAST TIME before application]              │  │
│  │                                                                       │  │
│  │   #endif                                                              │  │
│  │   #endif                                                              │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Line 530-533: apply_back_emf_protection() [if ADD_BACK_EMF_PROTECTION]    │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │ Line 1535: ui8_duty_cycle_target = ui8_min_duty_for_motor_speed       │  │
│  │           [May increase duty cycle, never decrease]                   │  │
│  │                                                                       │  │
│  │ Line 1563: ui8_duty_cycle_target = ui8_regen_duty_cycle               │  │
│  │           [May increase if regenerative current detected]             │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Line 560-575: Safety Checks                                                │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │ If error conditions:                                                  │  │
│  │   ui8_controller_adc_battery_current_target = 0                       │  │
│  │   ui8_controller_duty_cycle_target = 0                                │  │
│  │                                                                       │  │
│  │ Else:                                                                 │  │
│  │   Line 586-587: Limit ui8_adc_battery_current_target                  │  │
│  │                 if > ui8_adc_battery_current_max                      │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  Line 607: APPLY TO CONTROLLER (Final Assignment)                           │
│  ui8_controller_adc_battery_current_target = ui8_adc_battery_current_target │
│                                                                              │
│  Line 610: APPLY TO CONTROLLER (Final Assignment)                           │
│  ui8_controller_duty_cycle_target = ui8_duty_cycle_target                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Summary of Modifications with ADD_MINIMUM_POWER

### ui8_duty_cycle_target modifications in cycle:
1. **Line 483**: Reset to 0
2. **Riding mode functions** (lines 806, 870, 915, 989, 1082, 1220, 1350, 1454): Modified based on assist mode
3. **Line 1604** (ADD_MINIMUM_POWER): Modified based on assist level when speed limit exceeded
4. **Line 1535** (apply_back_emf_protection): May be increased (never decreased)
5. **Line 1563** (apply_back_emf_protection): May be increased if regenerative current detected
6. **Line 610**: Applied to controller (final)

### ui8_adc_battery_current_target modifications in cycle:
1. **Line 482**: Reset to 0
2. **Riding mode functions** (multiple locations): Modified based on assist mode
3. **Line 1450** (apply_throttle): May be modified
4. **Line 1485** (apply_temperature_limiting): May be modified
5. **Line 1576** (apply_speed_limit): Modified based on wheel speed
6. **Line 1617** (ADD_MINIMUM_POWER): Modified to match duty cycle limit
7. **Line 1628** (ADD_MINIMUM_POWER): **LAST MODIFICATION** - Limited to SPEED_LIMIT_MAX_CURRENT_ADC
8. **Line 586-587**: Safety limit check
9. **Line 607**: Applied to controller (final)

## Key Points:
- **Line 1604**: First modification of `ui8_duty_cycle_target` in ADD_MINIMUM_POWER block
- **Line 1617**: First modification of `ui8_adc_battery_current_target` in ADD_MINIMUM_POWER block
- **Line 1628**: **LAST modification** of `ui8_adc_battery_current_target` before application
- **Line 1535/1563**: `ui8_duty_cycle_target` may be increased further by back-EMF protection
- **Line 607/610**: Final assignments to controller variables

