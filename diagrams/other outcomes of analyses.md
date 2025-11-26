# Startup boost implementation details
_Exported on 3/10/2026 at 14:41:56 GMT+1 from Cursor (2.6.14)_

---

**User**

explain how "startup boost" is implemented

---

**Cursor**

Let me look at the startup boost implementation in the codebase.

Now let me also check the `CADENCE` and `SPEED` enum definitions:

Here is a complete explanation of how **startup boost** is implemented:

---

## Overview

Startup boost is an optional feature that artificially amplifies the measured pedal torque signal (`ui16_adc_pedal_torque_delta`) at low cadence / low speed, giving the motor extra assistance when the rider first starts pedaling from a standstill. It only applies in **Power Assist** mode.

---

## Configuration (`config.h`)

| Constant | Default value | Meaning |
|---|---|---|
| `STARTUP_BOOST_ON_STARTUP` | `0` | Whether startup boost is enabled when the controller powers on |
| `STARTUP_BOOST_TORQUE_FACTOR` | `250` | Percentage amplification factor at cadence = 0 RPM (250 → +250% boost) |
| `STARTUP_BOOST_CADENCE_STEP` | `10` | Rate at which the factor decays per cadence step (in fixed-point, ~10/256 per RPM) |
| `STARTUP_BOOST_AT_ZERO` | `0` | Trigger mode: `CADENCE` (0) = always active when pedaling, `SPEED` (1) = only at zero wheel speed |

---

## Initialization — Building the Factor Lookup Table

At startup, a 120-entry array `ui16_startup_boost_factor_array[]` is pre-computed, where the index is the pedal cadence in RPM:

```391:401:src/ebike_app.c
// make startup boost array
ui16_startup_boost_factor_array[0] = STARTUP_BOOST_TORQUE_FACTOR;

uint8_t ui8_i;
for (ui8_i = 1; ui8_i < 120; ui8_i++)
{
	// Fixed startup boost factor array (to reach zero)
	uint32_t ui32_temp = (uint32_t)((ui16_startup_boost_factor_array[ui8_i - 1] << 8)
		- (ui16_startup_boost_factor_array[ui8_i - 1] * STARTUP_BOOST_CADENCE_STEP));
	ui16_startup_boost_factor_array[ui8_i] = ui32_temp >> 8;
}
```

Each entry is computed as: `factor[i] = factor[i-1] × (1 - CADENCE_STEP/256)`, yielding a geometrically decreasing curve. With `CADENCE_STEP = 10`, the factor multiplies by `(256-10)/256 ≈ 0.961` per RPM, so the boost fades to near-zero around 60–80 RPM.

---

## Runtime Logic — `apply_startup_boost()`

Called from `apply_power_assist()` on every control cycle when startup boost is enabled:

```688:708:src/ebike_app.c
static void apply_startup_boost(void)
{
	// startup boost mode
	switch (ui8_startup_boost_at_zero) {
		case CADENCE:
			ui8_startup_boost_flag = 1;
			break;
		case SPEED:
			if (ui16_wheel_speed_x10 == 0U) {
				ui8_startup_boost_flag = 1;
			}
			else if (ui8_pedal_cadence_RPM > 45) {
				ui8_startup_boost_flag = 0;
			}
			break;
	}
	// pedal torque delta & startup boost
	if (ui8_startup_boost_flag) {
		uint32_t ui32_temp = ((uint32_t)(ui16_adc_pedal_torque_delta * ui16_startup_boost_factor_array[ui8_pedal_cadence_RPM])) / 100;
		ui16_adc_pedal_torque_delta += (uint16_t) ui32_temp;
	}
}
```

**Two trigger modes:**
- **`CADENCE` (0):** Boost flag is always set (boost is always active when the function is called).
- **`SPEED` (1):** Boost flag is set only while wheel speed is zero; it is cleared once cadence exceeds 45 RPM (hysteresis to avoid toggling).

**Torque amplification:**
The current pedal torque delta is multiplied by the pre-computed factor for the current cadence, and the result is **added back** to the torque delta:

```
ui16_adc_pedal_torque_delta += (ui16_adc_pedal_torque_delta × factor[cadence]) / 100
```

At cadence = 0, this effectively multiplies the torque by `1 + 2.5 = 3.5×` (factor = 250 → +250%). The boost smoothly decreases as cadence rises, vanishing entirely by ~80 RPM.

---

## Integration point

The boost-amplified torque delta then flows into the standard power assist calculation, which computes motor current target from pedal torque × cadence × assist level multiplier:

```745:752:src/ebike_app.c
// startup boost
if (m_configuration_variables.ui8_startup_boost_enabled) {
	apply_startup_boost();
}

if ((ui8_pedal_cadence_RPM > 0U)||(ui8_startup_assist_adc_battery_current_target)) {
	// calculate torque on pedals + torque startup boost
	uint32_t ui32_pedal_torque_x100 = (uint32_t)(ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100);
```

---

## Enable/Disable at runtime

The feature can be toggled at runtime via the display button menu (mapped to the ECO mode display slot), and the state is saved to / restored from EEPROM (`ADDRESS_STARTUP_BOOST_ON_STARTUP`).

---

**User**

in cadence mode, which STARTUP_BOOST_CADENCE_STEP is to be set such that the boost stops at around 40 rpm

---

**Cursor**

The factor array is a geometric series: `factor[n] = STARTUP_BOOST_TORQUE_FACTOR × ((256 - CADENCE_STEP) / 256)^n`

Since the torque addition is `torque_delta × factor[cadence] / 100`, the boost is effectively gone when `factor[40]` drops below ~1–5 (i.e. less than 1–5% extra torque).

**Formula to solve for CADENCE_STEP:**

`factor[40] = 250 × ((256 - step) / 256)^40`

Setting the target threshold and solving for `step`:

| Target factor at 40 RPM | CADENCE_STEP needed |
|---|---|
| ~50 (+50% still boosting) | 10 (current default — fades slowly) |
| ~13 (+13%) | 18 |
| ~5 (+5%, barely noticeable) | **25** |
| ~2 (+2%) | 30 |
| <1 (essentially zero) | 34 |

**Recommendation: set `STARTUP_BOOST_CADENCE_STEP` to 25–30.**

- At **25**, the boost is down to about +4% at 40 RPM — perceptually gone.
- At **30**, it's down to about +1.7% — mathematically near zero.

In `config.h`:

```c
#define STARTUP_BOOST_CADENCE_STEP 25
```

Note that the geometric series never reaches exactly zero, but at step=25 the boost contribution at 40 RPM is already negligible (<5%), and it's fully imperceptible in practice. If you want a cleaner cutoff, step=30 virtually eliminates it by 40 RPM.

---

**User**

assuming boost is enabled, check if the choice of STARTUP_BOOST_TORQUE_FACTOR can bypass the power- and current limiters such that the controller can get damaged

---

**Cursor**

Let me trace the full signal path from the boost through to the final motor current command, checking all limiters along the way.

Now let me check the data type widths on the STM8 target and the max value of `ui16_adc_pedal_torque_delta`:

Now I have all the pieces to give a complete answer. Let me trace the full signal chain and check for overflow risk on the STM8's 16-bit `int`.

---

## Analysis: Can `STARTUP_BOOST_TORQUE_FACTOR` bypass the limiters?

### Signal path from boost to motor current

```
ui16_adc_pedal_torque_delta  ──(×factor/100)──▶  boosted delta
       │
       ▼
ui32_pedal_torque_x100  =  delta × ui8_pedal_torque_per_10_bit_ADC_step_x100
       │
       ▼
ui32_power_assist_x100  =  torque × cadence × assist_multiplier
       │
       ▼
ui32_battery_current_target_x100  =  power / voltage
       │
       ▼
ui16_adc_battery_current_target  (uint16_t)
       │
       ▼ clamped ──── [1] assist function clamp (line 784)
ui8_adc_battery_current_target  ≤  ui8_adc_battery_current_max
       │
       ▼ clamped ──── [2] apply_controls() clamp (line 592)
ui8_controller_adc_battery_current_target  ≤  ui8_adc_battery_current_max
       │
       ▼ clamped ──── [3] hard hardware constant clamp (line 584)
ui8_adc_battery_current_max  ≤  ADC_10_BIT_BATTERY_CURRENT_MAX  (= 18 A)
       │
       ▼ hardware OC interrupt ── [4] assembly overcurrent check (line 549)
actual ADC current  vs.  ui8_adc_battery_overcurrent  (= 18 A + 8 A = 26 A)
```

**Conclusion: The boost factor cannot bypass the limiters.** It only inflates the current *request*; the request is clamped by three independent software layers and one hardware interrupt before reaching the motor controller. No matter how large the factor, the commanded current is bounded.

---

### Integer overflow risk in the boost calculation itself

The multiplication at line 706 is on the STM8 with SDCC, where `int` is **16-bit**:

```706:707:src/ebike_app.c
uint32_t ui32_temp = ((uint32_t)(ui16_adc_pedal_torque_delta * ui16_startup_boost_factor_array[ui8_pedal_cadence_RPM])) / 100;
ui16_adc_pedal_torque_delta += (uint16_t) ui32_temp;
```

Both operands are `uint16_t`, so the multiplication is computed as **16-bit unsigned int** and only then cast to `uint32_t`. If the product exceeds 65 535, it wraps modulo 65 536 — **before** the cast.

With `PEDAL_TORQUE_ADC_RANGE = 300 − 150 = 150` (from `config.h`), the overflow threshold is:

```
factor_overflow_limit = 65535 / delta_max = 65535 / 150 ≈ 436
```

| `STARTUP_BOOST_TORQUE_FACTOR` | Result |
|---|---|
| ≤ 436 | No overflow, boost applied correctly |
| > 436 | 16-bit overflow → boost wraps to a **smaller** (incorrect) value |
| Default = 250 | Safe (150 × 250 = 37 500, no overflow) |

**Important:** the overflow produces *less* boost than intended — not more — so it is fail-safe from a damage perspective. However, at factors above 436 the boost behaviour becomes unpredictable (non-linear wrap-around).

---

### Summary

| Concern | Safe? | Reason |
|---|---|---|
| Factor bypasses current limiter | **No risk** | 3-layer software clamp + HW OC interrupt downstream |
| Factor bypasses power limiter | **No risk** | `ui8_adc_battery_current_max` is the min of current and power limits |
| Integer overflow at high factors | **Fail-safe** | Overflow reduces boost, never amplifies past uint16 wrap |
| Aggressive saturation behaviour | **Mechanical concern only** | At any factor > ~50, the controller saturates to max current on first pedal touch; electronics are protected but drivetrain sees an abrupt full-power jerk |

**Recommendation:** keep `STARTUP_BOOST_TORQUE_FACTOR` well below **436** to avoid the overflow bug. A value of 250–300 is the practical maximum for reliable, overflow-free operation.

---

**User**

how is BATTERY_CURRENT_MAX used, and guess why ADC_10_BIT_BATTERY_CURRENT_MAX is fixed to 18A and is not related to BATTERY_CURRENT_MAX 

---

**Cursor**

Now I have the complete picture. Here is a thorough answer:

---

## How `BATTERY_CURRENT_MAX` is used

`BATTERY_CURRENT_MAX` (from `config.h`, default 13 A) is the **user-configurable soft current limit**. It travels through the following chain:

**1. Stored in EEPROM as default** (`eeprom.c` line 19), loaded at boot into `m_configuration_variables.ui8_battery_current_max` (in Amps).

**2. Converted to ADC steps** at initialisation (`ebike_app_init`):

```408:410:src/ebike_app.c
// calculate max adc battery current from the received battery current limit
ui8_adc_battery_current_max_temp_1 = (uint8_t)((uint16_t)(m_configuration_variables.ui8_battery_current_max * 100U)
    / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100);
```

With `BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100 = 16` (= 0.16 A/step × 100), this is simply `amps / 0.16`. For 13 A → 81 ADC steps.

**3. Also used to scale the motor phase current limit** proportionally:

```421:422:src/ebike_app.c
uint16_t ui16_temp = ui8_adc_battery_current_max_temp_1 * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
ui8_adc_motor_phase_current_max = (uint8_t)(ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX);
```

**4. Updated each display packet** as the minimum of the current limit and the power limit back-calculated to a current at the current battery voltage:

```3005:3010:src/ebike_app.c
// current limit with power limit
ui8_adc_battery_current_max_temp_2 = (uint8_t)((uint32_t)(ui32_adc_battery_power_max_x1000_array[...]
    / ui16_battery_voltage_filtered_x1000));

// set max battery current
ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max_temp_1, ui8_adc_battery_current_max_temp_2);
```

**5. Applied as the clamp** in every assist mode (power, torque, eMTB, throttle, …) and once more centrally in `apply_controls()`.

---

## Why `ADC_10_BIT_BATTERY_CURRENT_MAX` is fixed at 18 A and independent of `BATTERY_CURRENT_MAX`

These are **two completely different concepts** operating at different layers:

| | `BATTERY_CURRENT_MAX` / `ui8_battery_current_max` | `ADC_10_BIT_BATTERY_CURRENT_MAX` |
|---|---|---|
| **Nature** | User preference / ride tuning | Hardware safety ceiling |
| **Typical value** | 10–16 A (whatever the user wants) | 18 A (fixed in `main.h`) |
| **Where enforced** | In every assist algorithm | In `apply_controls()` as absolute backstop, and in the assembly overcurrent ISR |
| **Can be changed?** | Yes, via EEPROM / display | Yes, but requires a firmware recompile and knowledge of hardware limits |

The comment in `main.h` tells the real story:

```210:217:src/main.h
/*---------------------------------------------------------
 NOTE: regarding ADC battery current max

 This is the maximum current in ADC steps that the motor
 will be able to draw from the battery. A higher value
 will give higher torque figures but the limit of the
 controller is 16 A and it should not be exceeded.
 ---------------------------------------------------------*/
```

The **hardware reason** it is not derived from `BATTERY_CURRENT_MAX`: the TSDZ2 controller's MOSFETs and current-sense shunt resistor have a physical maximum safe current. That limit is a property of the **PCB and silicon**, not of the battery or user preference. Setting it to 18 A (while noting 16 A is the true hardware limit — the 2 A extra is a brief transient headroom before the overcurrent interrupt fires) protects the FETs regardless of what any user-configurable parameter says.

The design intent is therefore a **two-layer defence**:

```
User configures BATTERY_CURRENT_MAX (e.g. 13 A)
         │
         ▼
ui8_adc_battery_current_max_temp_1  ←  soft limit (per user wish + power limit)
         │
         ▼ clamped by
ADC_10_BIT_BATTERY_CURRENT_MAX (18 A)  ←  absolute hardware ceiling, compile-time constant
         │
         ▼ hardware OC interrupt fires at
ADC_10_BIT_BATTERY_CURRENT_MAX + ADC_10_BIT_BATTERY_EXTRACURRENT (18 + 8 = 26 A)
```

If `ADC_10_BIT_BATTERY_CURRENT_MAX` were simply set to `BATTERY_CURRENT_MAX / 0.16`, a user who enters an unrealistically large current limit (e.g. 25 A) would raise the hardware ceiling too, defeating the protection entirely. By keeping it a separate compile-time constant in `main.h`, only a developer who consciously edits `main.h` and recompiles can raise it — not a casual configuration change.

---

**User**

what happens when the overcurrent interrupt fires

---

**Cursor**

Now I have the complete picture. Here is the full sequence:

---

## What happens when overcurrent is detected

Note: despite the heading, this is **not a hardware interrupt**. It is a polled software check that runs every PWM cycle (~19 kHz), but it reads the ADC register directly in assembly to guarantee consistent (non-overrun) data.

---

### Step 1 — ADC sample read in assembly (each PWM cycle)

```544:563:src/ebike_app.c
if (ui8_battery_overcurrent_delay > 0) {	// overcurrent error enabled
    __asm
        mov _ui8_error_battery_overcurrent+0, #0
        ld a, 0x53eb // ADC1->DB5RL
        cp a, _ui8_adc_battery_overcurrent
        jrc 00011$
        mov _ui8_error_battery_overcurrent+0, #ERROR_BATTERY_OVERCURRENT
    00011$:
    __endasm;
    if (ui8_error_battery_overcurrent != 0U) {
        ui8_error_battery_overcurrent_counter++;
    }
    else {
        ui8_error_battery_overcurrent_counter = 0;
    }
    if (ui8_error_battery_overcurrent_counter >= ui8_battery_overcurrent_delay) {
        ui8_system_state = ui8_error_battery_overcurrent;
    }
}
```

The assembly reads `ADC1->DB5RL` directly (battery current ADC channel) and compares it against `ui8_adc_battery_overcurrent` (= `ui8_adc_battery_current_max + ADC_10_BIT_BATTERY_EXTRACURRENT` = 18 A + 8 A = 26 A). If the measured current ≥ threshold, `ui8_error_battery_overcurrent` is set to `ERROR_BATTERY_OVERCURRENT` (7); otherwise it is cleared.

**Assembly is used** because regular C code risks reading the ADC at the same moment the hardware updates it ("conversion overrun"), which would give a corrupt value for this safety-critical check.

---

### Step 2 — Debounce counter (`OVERCURRENT_DELAY = 5` cycles)

The error must persist for **5 consecutive PWM cycles** before it escalates to a system fault. With `PWM_CYCLES_SECOND = 16,000,000 / (420 × 2) ≈ 19,047 Hz`, this is a debounce window of:

`5 / 19,047 ≈ 0.26 ms`

One clean sample resets the counter to zero. This prevents a single ADC glitch from shutting down the motor.

---

### Step 3 — System state latches to `ERROR_BATTERY_OVERCURRENT`

Once the counter reaches the threshold, `ui8_system_state` is set permanently to `ERROR_BATTERY_OVERCURRENT`. There is **no automatic recovery** — the state stays latched until a power cycle.

---

### Step 4 — Motor current and duty cycle forced to zero

On the very next pass through `apply_controls()`:

```566:581:src/ebike_app.c
if ((ui8_brake_state)
  || ...
  || (ui8_system_state == ERROR_BATTERY_OVERCURRENT)
  || ...) {
    ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
    ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
    ui8_controller_adc_battery_current_target = 0;
    ui8_controller_duty_cycle_target = 0;
}
```

Current target → 0, duty cycle target → 0. The motor is commanded to coast immediately.

---

### Step 5 — PWM outputs physically disabled

```619:631:src/ebike_app.c
if (ui8_motor_enabled
    && (...
        || (ui8_system_state == ERROR_BATTERY_OVERCURRENT)
        || ...)) {
    ui8_motor_enabled = 0;
    motor_disable_pwm();
}
```

`motor_disable_pwm()` calls `TIM1_OC1Init / OC2Init / OC3Init` with `TIM1_OUTPUTSTATE_DISABLE` and `TIM1_OUTPUTNSTATE_DISABLE` for all three phases:

```1080:1089:src/motor.c
void motor_disable_pwm() {
    TIM1_OC1Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_DISABLE, TIM1_OUTPUTNSTATE_DISABLE, ...);
    TIM1_OC2Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_DISABLE, TIM1_OUTPUTNSTATE_DISABLE, ...);
    TIM1_OC3Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_DISABLE, TIM1_OUTPUTNSTATE_DISABLE, ...);
}
```

All six MOSFET gate signals are disabled — both high-side and low-side for each phase — completely cutting drive to the motor.

---

### Step 6 — Error displayed to the rider

The fault code `ERROR_BATTERY_OVERCURRENT` (7 = E07) is sent to the display. For the XH18 display it is shown as blinking **E04**; on VLCD5/VLCD6 it shows as **E07**:

```3284:3288:src/ebike_app.c
else if (ui8_display_fault_code == ERROR_BATTERY_OVERCURRENT) {
    // instead of E07, display blinking E04
    if (ui8_default_flash_state) {
        ui8_tx_buffer[5] = 4;
    }
}
```

---

### Summary timeline

```
Actual current ≥ 26 A (18 A + 8 A margin)
       │
       ▼  repeated for 5 PWM cycles (~0.26 ms debounce)
ui8_system_state = ERROR_BATTERY_OVERCURRENT  ← permanent latch
       │
       ├──▶  current target = 0, duty cycle target = 0
       │
       ├──▶  motor_disable_pwm() — all 6 MOSFET gates disabled
       │
       └──▶  display shows E07 / blinking E04
             (no automatic recovery — power cycle required)
```


---

**User**

check for a simple way to reset the ERROR_BATTERY_OVERCURRENT  condition upon reaching speed of zero 

---

**Cursor**

Now I have a complete picture of the code. Here's my plan:

---

**Current behavior:**
- Lines 544–563: Each PWM cycle, the ADC battery current is read via inline assembler. If it exceeds `ui8_adc_battery_overcurrent`, a counter increments. When the counter reaches `ui8_battery_overcurrent_delay`, `ui8_system_state` is set to `ERROR_BATTERY_OVERCURRENT`.
- `ui8_system_state` is **never reset** back to `NO_ERROR` anywhere in the code — the error is permanent until power-off.

**Proposed fix (minimal, 4 lines):**

Right after the overcurrent detection block (line 564), add a reset condition:

```c
// reset overcurrent error when bike has stopped
if ((ui8_system_state == ERROR_BATTERY_OVERCURRENT) && (ui16_wheel_speed_x10 == 0U)) {
    ui8_system_state = NO_ERROR;
    ui8_error_battery_overcurrent_counter = 0;
}
```

This inserts between the overcurrent check (line 564) and the "reset control parameters" safety block (line 566).

---

**User**

Implement the plan as specified, it is attached for your reference. Do NOT edit the plan file itself.

To-do's from the plan have already been created. Do not create them again. Mark them as in_progress as you work, starting with the first one. Don't stop until you have completed all the to-dos.

---

**Cursor**

The change is in place and looks correct. Let me mark the todo as complete.

Done. The inserted block is at lines 566–570 of [`src/ebike_app.c`](src/ebike_app.c):

```566:570:src/ebike_app.c
	// reset overcurrent error when bike has stopped
	if ((ui8_system_state == ERROR_BATTERY_OVERCURRENT) && (ui16_wheel_speed_x10 == 0U)) {
		ui8_system_state = NO_ERROR;
		ui8_error_battery_overcurrent_counter = 0;
	}
```

It sits between the overcurrent detection block and the "reset control parameters" safety block, so the flow is:
1. Overcurrent is detected and `ui8_system_state` is set to `ERROR_BATTERY_OVERCURRENT` as before.
2. **New:** If that error state is active and `ui16_wheel_speed_x10` drops to 0, both the system state and the accumulator counter are cleared.
3. The full `ui8_battery_overcurrent_delay` must be elapsed again before the error can re-trigger, preventing immediate re-assertion.

---

MOTOR_ACCELERATION is used to configure the default "ramp-up" rate (acceleration response) for the motor controller.

Here is how it flows:

Initialization: In src/ebike_app.c (lines 321–325), MOTOR_ACCELERATION (0–100%) is mapped to a hardware parameter called ui8_duty_cycle_ramp_up_inverse_step_default.

A higher MOTOR_ACCELERATION results in a smaller "inverse step" (faster ramp-up).
A lower MOTOR_ACCELERATION results in a larger "inverse step" (slower ramp-up).
Runtime adjustment: In set_motor_ramp() (lines 663–674), this default value is used as the starting point (the slowest allowed acceleration) for low speeds (<4kph) or low cadences (<20rpm). As speed or cadence increases, the system automatically makes the ramp-up faster (smaller step value), transitioning from your configured "comfort" acceleration to the maximum hardware acceleration.

In short: It sets the "softness" of the motor engagement at low speeds. Higher values make the bike feel snappier off the line; lower values make it smoother and gentler.