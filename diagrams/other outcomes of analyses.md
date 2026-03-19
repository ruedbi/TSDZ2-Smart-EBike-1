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

---

# Smooth Start Logic Explanation
The Smooth Start feature is designed to prevent sudden power surges when starting from a standstill. It works by gradually ramping up the pedal torque signal (and thus motor power) over a set period of time, rather than applying full power instantly.

1. How it works
The logic is executed every 25 milliseconds (40 times per second) and follows this sequence:

Reset Condition: When the bike is completely stopped (0 RPM cadence AND 0 wheel speed), a counter is reset to a starting value determined by your setting.
Ramp Up: As soon as you start pedaling (or the wheel moves), the counter begins counting down to zero.
Scaling: While the counter is counting down, the pedal torque signal is scaled linearly from **0
% to 100%**: * Start: 0% torque passed through. * During: Torque ramps up progressively (e.g. 10%, 20%, 30%...). * End: 100% torque passed through (full power).

2. How SMOOTH_START_SET_PERCENT affects it
SMOOTH_START_SET_PERCENT directly controls the duration of this ramp-up phase. The value (default 35%) is mapped to a counter value, which determines how many 25ms cycles it takes to reach full power.

Higher Value (e.g., 100%): Sets a higher starting counter. This means the countdown takes longer, resulting in a slower, smoother ramp-up.
Lower Value (e.g., 0%): Sets a lower starting counter. The countdown finishes very quickly (or instantly), resulting in a sharper, more immediate power delivery.
Minimum Limit: There is a hardcoded minimum limit (SMOOTH_START_RAMP_MIN = 30), preventing the ramp-up from being instant (0ms) to ensure at least some basic smoothing (~0.75 seconds minimum ramp).
In summary: SMOOTH_START_SET_PERCENT controls the time it takes to go from 0% to 100% power when starting from a stop. A higher percentage means a longer, gentler start.

Based on the code analysis, the Smooth Start feature is applied in the following assist modes:

Torque Assist Mode (TORQUE_ASSIST_MODE)
Hybrid Assist Mode (HYBRID_ASSIST_MODE)
Cadence Assist Mode (
CADENCE_ASSIST_MODE)

It is NOT applied in:

Power Assist Mode (POWER_ASSIST_MODE)
eMTB Assist Mode (eMTB_ASSIST_MODE)
Cruise Mode
Walk Assist Mode
Important Distinction for Cadence Mode
While Torque and Hybrid modes strictly follow your configured SMOOTH_START_SET_PERCENT, the Cadence mode behaves differently:

It enforces a minimum smoothness (maximum speed of ramp-up).
If your configured smooth start is faster than the default (shorter ramp time), Cadence mode will ignore it and use the slower default ramp (SMOOTH_START_RAMP_DEFAULT).
This effectively means Cadence mode always enforces a slow, gentle start (approx. 4 seconds ramp-up) even if you try to configure it for a fast start, likely to prevent jerky behavior since Cadence mode applies power based on rotation speed rather than pedal pressure.

---

**User**

@src/main.h:218-222 what is the use of these current max values and when and for how long are they in use instead of BATTERY_CURRENT_MAX

---

**Cursor**

The hardcoded values in `src/main.h` (lines 218-222) serve two distinct purposes depending on the state of the bike:

### 1. As an Absolute Safety Ceiling (Always Active)
*   **`ADC_10_BIT_BATTERY_CURRENT_MAX` (112 ≈ 18A):** This is the hard upper limit for the controller. Even if you configure `BATTERY_CURRENT_MAX` in `config.h` to something higher (e.g., 20A or 25A), the firmware will forcibly cap it at **18A** (112 ADC steps) in the main control loop. You cannot exceed this value without changing the code.
*   **`ADC_10_BIT_BATTERY_EXTRACURRENT` (50 ≈ 8A):** This defines the "trip point" for the overcurrent error. It is added to the active current limit. So, if your limit is 18A, the system will shut down with an overcurrent error if it sees **

26A** (18A + 8A).
*   **`ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX` (187 ≈ 30A):** This is the hard safety limit for the motor phase current. It prevents hardware damage by capping the phase current at ~30A regardless of other settings.

### 2. As the "Default" Behavior (Startup Phase)
These values are used as the **active settings** during the startup phase, **ignoring your `config.h` settings** temporarily.

*   **When:** From the moment you power on the controller.
*   **How Long:** Until the controller successfully receives the first data packet from the display (LCD).
    *   **Normal Operation:** This typically takes **0.5 to 2 seconds** (display boot + communication handshake). During this brief window, the bike allows up to 18A, even if you configured it for 13A.
    *   **No Display / Communication Failure:** If you run the bike without a display or if the cable is broken, the controller **will never update** to your configured values. It will permanently run with the hardcoded **18A limit** and the defaults defined here.

**Summary:** Your configured `BATTERY_CURRENT_MAX` (e.g., 13A) is only applied **after** the display finishes booting and talks to the controller. Before that moment, the code blindly uses the 18A limit defined in these lines.

---

**User**

does that mean that after the display is up, the phase current can never exceed BATTERY_CURRENT_MAX ?

---

**Cursor**

No, that is **not correct**. The phase current **can** and usually **does** exceed `BATTERY_CURRENT_MAX`, especially at low speeds and high torque.

Here is the breakdown:

1.  **Phase Current vs. Battery Current:**
    *   **Battery Current:** The current flowing from the battery to the controller. This is what you configure with `BATTERY_CURRENT_MAX` (e.g., 13A).
    *   **Phase Current:** The current circulating in the motor windings. At low speeds, the controller acts like a transformer (buck converter), converting high voltage/low current (from battery) into low voltage/high current (to motor).
    *   **Result:** You can have 13A drawn from the battery but 20A or more flowing through the motor phases.

2.  **How the Limit is Calculated:**
    The code explicitly scales the allowed phase current based on your battery current setting.
    In `src/ebike_app.c` (lines 422-423):
    ```c
    // Calculate phase current limit proportional to battery current limit
    uint16_t ui16_temp = ui8_adc_battery_current_max_temp_1 * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
    ui8_adc_motor_phase_current_max = (uint8_t)(ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX);
    ```
    
    *   `ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX` = 187 (approx. **30A**)
    *   `ADC_10_BIT_BATTERY_CURRENT_MAX` = 112 (approx. **18A**)
    *   **Ratio:** 30A / 18A ≈ **1.67**

    **Example:**
    If you set `BATTERY_CURRENT_MAX` to **13A**:
    *   Battery Limit = 13A
    *   Phase Current Limit = 13A × 1.67 ≈ **21.7A**

**Conclusion:**
Even after the display is up and your 13A battery limit is active, the motor phase current is allowed to go up to **~21.7A**. This is intentional and necessary for providing good torque at low speeds (like starting on a hill) without exceeding the battery's power rating. The hard ceiling of **30A** (187 ADC steps) is the absolute hardware safety limit that is never exceeded.

---

**User**

show in the code where that transformation gets configured

---

**Cursor**

The transformation scaling is configured in `src/ebike_app.c` inside the function `uart_receive_package`.

This function processes the configuration data received from the display (or uses defaults if no display is connected) and calculates the runtime limits.

```c:src/ebike_app.c
// ... inside uart_receive_package ...

// 1. First, it calculates the user's requested battery current limit in ADC steps
// based on the configuration variable (e.g. 13A)
ui8_adc_battery_current_max_temp_1 = (uint8_t)((uint16_t)(m_configuration_variables.ui8_battery_current_max * 100U)
    / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100);

// ...

// 2. Then, it calculates the Motor Phase Current Limit
// It scales it proportionally: (User Battery Limit / Hardcoded Battery Limit) * Hardcoded Phase Limit
uint16_t ui16_temp = ui8_adc_battery_current_max_temp_1 * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
ui8_adc_motor_phase_current_max = (uint8_t)(ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX);
```

**Where:**
*   `ui8_adc_battery_current_max_temp_1` is your **13A** (converted to ADC steps).
*   `ADC_10_BIT_BATTERY_CURRENT_MAX` is the hardcoded **18A** (112 steps).
*   `ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX` is the hardcoded **30A** (187 steps).

So the calculation effectively sets the phase limit to:
`Phase Limit = 13A * (30A / 18A) = 21.6A`

This variable `ui8_adc_motor_phase_current_max` is then used by the motor control loop (in `src/motor.c`, not shown here but implied) to cap the phase current in real-time.

---

**User**

which max. values can ui8_adc_motor_phase_current reach?

---

**Cursor**

The variable `ui8_adc_motor_phase_current` is an 8-bit unsigned integer, so mathematically it can reach a maximum value of **255**.

In the context of the firmware:

1.  **ADC Steps:** 1 ADC step ≈ 0.16 Amps.
2.  **Max Value (255):** 255 * 0.16A ≈ **40.8 Amps**.

The code explicitly caps this value at 255 if the calculation overflows (lines 660-661 in the C-commented section and lines 716-718 in the assembly section), meaning the software cannot represent a phase current higher than **~40.8A**.

However, in practice, the `ui8_adc_motor_phase_current_max` (the safety limit we discussed earlier, capped at 187 or ~30A) acts as a control threshold. If `ui8_adc_motor_phase_current` exceeds that limit (e.g., goes above 30A), the controller immediately ramps down the duty cycle (power) to bring it back under the limit.

---

**User**

looks like ui8_adc_battery_overcurrent is not used at all, right?

---

**Cursor**

It **is used**, but in a specific way:

1.  **Usage:** It is used in the inline assembly block (lines 542–549 of `src/ebike_app.c`) to check for **battery overcurrent**.
    *   `ld a, 0x53eb` loads the current battery current ADC value (from `ADC1->DB5RL`).
    *   `cp a, _ui8_adc_battery_overcurrent` compares the measured current against the overcurrent limit.
    *   If the measured current is higher than the limit (`ui8_adc_battery_overcurrent`), it sets `ui8_error_battery_overcurrent` to `ERROR_BATTERY_OVERCURRENT`.

2.  **Logic:**
    *   If the error persists for `ui8_battery_overcurrent_delay` cycles (lines 551–559), the system state transitions to `ERROR_BATTERY_OVERCURRENT`.
    *   This shuts down the motor (lines 626 and 632) until the error clears or the bike stops (lines 563–566).

So, `ui8_adc_battery_overcurrent` acts as the **trip threshold** for the software-based overcurrent protection. It is updated dynamically (line 591) based on the current limit + an extra buffer (`ADC_10_BIT_BATTERY_EXTRACURRENT`).

---

**User**

@src/main.h:218-220 what is the purpose of these values vs. BATTERY_CURRENT_MAX

---

**Cursor**

The values in `src/main.h` act as **absolute safety limits (hard ceilings)** and **emergency trip thresholds**, whereas `BATTERY_CURRENT_MAX` in `config.h` is your **normal operational target**.

Here is how they differ in purpose:

### 1. `BATTERY_CURRENT_MAX` (e.g., 13A)
*   **Purpose:** This is your **target limit**. During normal riding, the motor controller continuously adjusts the motor power (PWM duty cycle) to ensure the current drawn from the battery *smoothly* stays at or below this value. 
*   **Location:** Defined in `config.h` and sent by the display.

### 2. `ADC_10_BIT_BATTERY_CURRENT_MAX` (112 = 18A)
*   **Purpose:** This is the **absolute safety ceiling**. 
*   **How it works:** Even if you set `BATTERY_CURRENT_MAX` to 20A or 25A in your configuration, the software will forcibly cap it at 18A (112 ADC steps). This protects the TSDZ2 hardware from melting. 
*   **Fallback:** It also serves as the default limit for the first few seconds when you turn the bike on, before the display has successfully transmitted your `config.h` settings.

### 3. `ADC_10_BIT_BATTERY_EXTRACURRENT` (50 = 8A)
*   **Purpose:** This defines the **emergency shutdown margin** (Overcurrent Protection).
*   **How it works:** While `BATTERY_CURRENT_MAX` tells the motor to *slowly reduce power* if it reaches 13A, spikes can still happen. The controller calculates an emergency trip point:
    `Trip Point = Active Limit + ADC_10_BIT_BATTERY_EXTRACURRENT`
*   If the current hits this trip point (e.g., 13A + 8A = 21A), the controller stops trying to regulate power smoothly and **instantly cuts off the motor entirely**, throwing an `ERROR_BATTERY_OVERCURRENT` (E07).

**In Summary:**
*   **`BATTERY_CURRENT_MAX`:** "Try to keep the current below this."
*   **`ADC_10_BIT_BATTERY_CURRENT_MAX`:** "Never let the active limit go above 18A, no matter what the user configures."
*   **`ADC_10_BIT_BATTERY_EXTRACURRENT`:** "If the current suddenly spikes 8A *above* the active limit, instantly kill the motor to prevent a fire."

---

**User**

@src/main.h:222-223 how is that value used and what is the reaction on exceeding this value

---

**Cursor**

The constant `ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX` (187, which corresponds to roughly 30 amps) represents the **absolute maximum hardware limit** for the motor phase current. 

### How it is used

1.  **Scaling the Target Limit:** 
    It is used in `src/ebike_app.c` to proportionally scale the motor phase current limit based on your configured battery current limit. 
    ```c
    // Phase Limit = (User Battery Limit * 187) / 112
    uint16_t ui16_temp = ui8_adc_battery_current_max_temp_1 * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
    ui8_adc_motor_phase_current_max = (uint8_t)(ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX);
    ```
2.  **Absolute Safety Cap:**
    Immediately after the above calculation, the code enforces that the resulting phase current limit can *never* exceed 187:
    ```c
    if (ui8_adc_motor_phase_current_max > ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX) {
        ui8_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
    }
    ```

### Reaction on Exceeding This Value

The active limit (`ui8_adc_motor_phase_current_max`) is constantly monitored inside the high-frequency PWM control loop in `src/motor.c`.

If the currently measured motor phase current (`ui8_adc_motor_phase_current`) exceeds this maximum limit, the reaction is to **smoothly but immediately ramp down the motor power**:

```c:src/motor.c (lines 805-827)
// check if to decrease, increase or maintain duty cycle
if ( // ... other conditions ...
  || (ui8_adc_motor_phase_current > ui8_adc_motor_phase_current_max) // <--- CHECK IS HERE
  // ...
   ) {
    // ...
    // ramp down duty cycle
    if (++ui8_counter_duty_cycle_ramp_down > ui8_controller_duty_cycle_ramp_down_inverse_step) {
        ui8_counter_duty_cycle_ramp_down = 0;
        // decrement field weakening angle if set or duty cycle if not
        if (ui8_fw_hall_counter_offset > 0) {
            ui8_fw_hall_counter_offset--;
        }
        else if (ui8_g_duty_cycle > 0) {
            ui8_g_duty_cycle--; // <--- REDUCES POWER TO THE MOTOR
        }
    }
}
```

**In summary:** Unlike the *battery overcurrent* (which triggers an E07 error and shuts off the motor entirely), exceeding the *phase current limit* is treated as a normal control boundary. The controller simply reduces the PWM duty cycle (backing off the throttle) until the phase current drops back down below the limit. This prevents the motor coils from burning up while climbing steep hills at low speeds.

---

**User**

what happens when ADC_10_BIT_BATTERY_CURRENT_MAX gets exceeded

---

**Cursor**

When `ADC_10_BIT_BATTERY_CURRENT_MAX` (the hardware safety limit) gets exceeded by the user's configuration, the software prevents the configured value from being used.

Here is the exact sequence of what happens:

1.  **Capping the Configuration:**
    Every 25ms in `ebike_app_controller()`, the code checks if your configured current limit (`ui8_adc_battery_current_max`) is higher than the absolute safety limit (`ADC_10_BIT_BATTERY_CURRENT_MAX`). If it is, it forcibly overwrites your configured limit with the safety limit:
    ```c:src/ebike_app.c (lines 585-588)
    // limit max current if higher than configured hardware limit (safety)
    if (ui8_adc_battery_current_max > ADC_10_BIT_BATTERY_CURRENT_MAX) {
        ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
    }
    ```

2.  **Capping the Target Current:**
    Because the max limit is now capped, any calculations that try to draw more current (like pedaling hard in Turbo mode) will also be capped. The requested target current is forced to stay at or below this new ceiling:
    ```c:src/ebike_app.c (lines 593-596)
    // limit target current if higher than max value (safety)
    if (ui8_adc_battery_current_target > ui8_adc_battery_current_max) {
        ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    }
    ```

3.  **Motor Control Response:**
    This target (`ui8_adc_battery_current_target`) is then passed down to the high-frequency motor control loop (`src/motor.c`). If the *actual measured battery current* exceeds this target, the controller automatically reduces the motor power (PWM duty cycle) to bring the current back down:
    ```c:src/motor.c (lines 805-827)
    if ( ...
      || (ui8_controller_adc_battery_current_target < ui8_adc_battery_current_filtered)
      // ...
    ) {
        // ... ramp down duty cycle (reduce power)
    }
    ```

**In summary:** If you configure a `BATTERY_CURRENT_MAX` that translates to more than `ADC_10_BIT_BATTERY_CURRENT_MAX` (18 Amps), the system simply ignores the excess. It will act exactly as if you had configured it for 18 Amps. The motor will smoothly limit its power draw to stay at or below 18 Amps, and no error is thrown—it just refuses to give you more power than the hardware limit allows.

---

**User**

why are ui8_adc_battery_current_max  and ui8_adc_battery_current_target  no fixed values

---

**Cursor**

These two variables are dynamic because they represent the **active state** of the controller, which changes constantly as you ride. Here is a detailed explanation of why each is not a fixed value:

### 1. `ui8_adc_battery_current_target` (The "Throttle" Signal)
This is the core variable that tells the motor how much power to deliver right now. It is never fixed because it is constantly recalculated (every 25ms) based on:
*   **Your pedaling effort:** In Torque Mode, if you push harder on the pedals, the target goes up.
*   **Your cadence:** In Cadence Mode, faster pedaling increases the target.
*   **Assist Level:** Eco mode calculates a much lower target than Turbo mode for the exact same pedal pressure.
*   **Walk Assist / Cruise Control:** These modes calculate their own specific targets to maintain a steady speed.

In short, `ui8_adc_battery_current_target` is the active output of the pedal assist algorithms. It goes from 0 (coasting) up to whatever limit is currently active.

### 2. `ui8_adc_battery_current_max` (The Active Limit)
While `BATTERY_CURRENT_MAX` in `config.h` is a fixed number (e.g., 13A), the *active* max limit (`ui8_adc_battery_current_max`) can change dynamically while riding due to several software features:

*   **Street Mode vs. Offroad Mode (Power Limits):**
    If you have `STREET_MODE_POWER_LIMIT_ENABLED` active, the system calculates a max current limit dynamically based on your battery voltage to ensure you don't exceed the legal wattage (e.g., 250W or 500W). Since battery voltage drops as you discharge it, the current limit must *increase* to maintain the same wattage limit, meaning this max value is constantly shifting.
*   **Temperature Limiting:**
    If `ENABLE_TEMPERATURE_LIMIT` is active and the motor gets too hot, the software gradually lowers `ui8_adc_battery_current_max` to restrict power and let the motor cool down.
*   **Startup / Walk Assist Restrictions:**
    Walk assist enforces its own strict maximum current (`WALK_ASSIST_ADC_BATTERY_CURRENT_MAX`) which is usually much lower than your normal riding limit.

**To summarize:**
*   `BATTERY_CURRENT_MAX` (config) = The absolute maximum you *want* to allow.
*   `ui8_adc_battery_current_max` (dynamic) = The actual limit applied *right now* (can be lower if the motor is hot, or if street mode power limits are active).
*   `ui8_adc_battery_current_target` (dynamic) = How much power the motor should deliver *this exact millisecond* based on how hard you are pedaling.

---

**User**

that basically means that the power is managed on the input side of the controller, not on the motor side, right

---

**Cursor**

Yes, your understanding is fundamentally correct!

The primary "throttle" or power management in the TSDZ2 firmware is driven by the **input side (battery current)**, while the **motor side (phase current)** acts mostly as a safety limiter.

Here is why it is designed this way:

### 1. Battery Side = Power Control (The "Target")
The firmware calculates how much assist to give you based on your pedaling (torque or cadence). It translates this effort into a **Target Battery Current** (`ui8_adc_battery_current_target`). 

Why use the battery side for this? Because battery voltage is relatively stable. 
*   `Power (Watts) = Battery Voltage × Battery Current`
*   By controlling the battery current, the controller is directly controlling the **total power** being fed into the system. If you want 250 Watts of assist, the controller knows exactly how much battery current to draw to give you 250 Watts.

### 2. Motor Side = Hardware Protection (The "Ceiling")
The **Motor Phase Current** is completely different from the battery current, especially at low speeds. The controller acts like a transformer (buck converter). 
*   If you are climbing a steep hill at 5 km/h, the motor needs high torque (which requires high phase current). 
*   The controller might draw only **10 Amps** from the battery (input), but transform that into **25 Amps** going through the motor coils (output). 

If the firmware *only* managed the input side, drawing 15 Amps from the battery at near-zero speeds could result in 50+ Amps in the motor coils, which would melt the copper wires or strip the nylon gears.

Therefore, the motor side is used as a **hard ceiling**. The code constantly monitors `ui8_adc_motor_phase_current`. It doesn't use it to decide how much assist you *should* get, but if that value hits the safety cap (`ui8_adc_motor_phase_current_max`, e.g., 30A), it overrides your pedaling request and forces the power down to protect the hardware.

### Summary of the Controller's Logic
1.  **Input (Battery):** "Based on how hard the rider is pedaling, I want to draw **X Amps** from the battery to give them the right amount of power."
2.  **Output (Motor):** "Wait, before I do that, let me make sure that drawing X Amps won't cause the motor phase current to exceed **Y Amps**, otherwise I'll melt the motor."

---

**User**

@src/main.h:224-251 @src/common.h:50-55 review the code

---

**Cursor**

Here is a review of your code changes. You've correctly understood the relationship between the different variables, but there are **three important issues** regarding precision, trip-point scaling, and a side-effect on motor torque that you should address.

### 1. Inaccuracy in the `* 6` Conversion Factor
You are multiplying your Amp values by `6` to get ADC steps. However, `BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100` is defined as `16` (meaning 1 step = 0.16 Amps). 
The exact conversion factor is `1 / 0.16 = 6.25`. 

By using `6`, you are under-calculating the limits by about 4%:
* Your `BATTERY_CURRENT_SOFT_LIMIT` of 16A becomes `16 * 6 = 96 steps`. 
* `96 steps * 0.16A = 15.36 Amps` (not 16 Amps).

**Recommendation:** Use `* 100 / 16` to perfectly match the firmware's internal math.
```c
#define ADC_10_BIT_BATTERY_CURRENT_MAX ((BATTERY_CURRENT_SOFT_LIMIT * 100) / 16)
```

### 2. The Overcurrent Trip Point (Emergency Margin)
In `ebike_app.c`, the trip threshold is calculated dynamically by adding the margin to the **active** current limit:
`ui8_adc_battery_overcurrent = ui8_adc_battery_current_max + ADC_10_BIT_BATTERY_EXTRACURRENT`

In your code, you defined the margin based on `BATTERY_CURRENT_MAX` (13A):
`Margin = 22A - 13A = 9A`

Because the margin is a fixed **9 Amps**, if your LCD (or soft limit) allows the active current to reach `BATTERY_CURRENT_SOFT_LIMIT` (16A), the trip point will float up to **25 Amps** (`16A + 9A`), which defeats your intended `HARD_LIMIT` of 22A.

**Recommendation:** Calculate the margin relative to the soft limit so that the trip point never exceeds 22A, even if the bike is operating at the absolute maximum soft limit (16A).
```c
#define ADC_10_BIT_BATTERY_EXTRACURRENT (((BATTERY_CURRENT_HARD_LIMIT - BATTERY_CURRENT_SOFT_LIMIT) * 100) / 16)
```

### 3. Unintended Loss of Torque (Motor Phase Current Scaling)
This is the most critical side effect. In `ebike_app.c`, the motor phase current limit is dynamically scaled using the ratio of your hard limits:
`Phase Limit = User Configured Battery Limit * (PHASE_CURRENT_MAX / BATTERY_CURRENT_MAX)`

*   **Original Firmware:** `13A * (30A / 18A) = 21.6A` Phase Current
*   **Your Code:** `13A * (20A / 16A) = 16.25A` Phase Current

Because you changed the ratio from **1.66** to **1.25**, the controller will push significantly less current into the motor coils at low speeds. **You will experience a noticeable loss of torque** when starting from a standstill or climbing steep hills. 

**Recommendation:** If your goal is to protect the motor by capping it at 20A, your code will safely achieve that. However, if you didn't intend to weaken the low-speed torque profile at 13A, you may want to rethink the `PHASE_CURRENT_LIMIT`. Setting it to `22` or `24` would restore a ratio closer to 1.5, bringing the low-speed torque back up while still being much safer than the original 30A limit.

### Suggested Corrected Code (`src/main.h`)

```c
#if defined (LIMIT_CURRENTS) // ruedbi

#define PHASE_CURRENT_HARD_LIMIT 25 // Amps

// Convert Amps to ADC steps perfectly (1 step = 0.16A)
#undef ADC_10_BIT_BATTERY_CURRENT_MAX
#define ADC_10_BIT_BATTERY_CURRENT_MAX ((BATTERY_CURRENT_SOFT_LIMIT * 100) / 16)

// Margin is calculated relative to the Soft Limit to ensure the Hard Limit is never exceeded
#undef ADC_10_BIT_BATTERY_EXTRACURRENT
#define ADC_10_BIT_BATTERY_EXTRACURRENT (((BATTERY_CURRENT_HARD_LIMIT - BATTERY_CURRENT_SOFT_LIMIT) * 100) / 16)

#undef ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX ((PHASE_CURRENT_LIMIT * 100) / 16)

// Sanity check macros (using exact math to avoid truncation errors in #if checks)
#if (BATTERY_CURRENT_MAX * 100 / 16) > ADC_10_BIT_BATTERY_CURRENT_MAX
#error "BATTERY_CURRENT_MAX exceeded BATTERY_CURRENT_SOFT_LIMIT"
#endif

#if BATTERY_CURRENT_SOFT_LIMIT >= BATTERY_CURRENT_HARD_LIMIT
#error "BATTERY_CURRENT_SOFT_LIMIT must be strictly less than BATTERY_CURRENT_HARD_LIMIT"
#endif

#if PHASE_CURRENT_LIMIT > PHASE_CURRENT_HARD_LIMIT
#error "PHASE_CURRENT_LIMIT exceeded PHASE_CURRENT_HARD_LIMIT"
#endif

#endif // LIMIT_CURRENTS
```

---

**User**

at which rate are battery- and phase current measured and at which rate are they adjusted

---

**Cursor**

Based on the codebase analysis, here is how the currents are measured and adjusted:

### 1. The Measurement Rate (High Frequency)
Both the **Battery Current** and the **Motor Phase Current** are measured and calculated inside the core motor PWM interrupt loop.

*   **Frequency:** They are measured at the exact same frequency as the PWM switching frequency, which is either **18 kHz** or **19 kHz** (defined by `PWM_FREQ`). 
*   **Time interval:** This means the hardware reads the ADC (Analog-to-Digital Converter) and updates the current values every **~55 microseconds** (`1/18000` seconds).
*   **Where in code:** This happens inside the `TIM1_UPD_OVF_TRG_BRK_IRQHandler` function in `src/motor.c` (around line 640+). The battery current is read directly from the ADC, and the phase current is instantly mathematically derived from the battery current and the current PWM duty cycle.

### 2. The Adjustment Rate (Two-Tiered)

The firmware adjusts these currents at two completely different rates:

#### A. The "Target" Calculation Rate (Low Frequency)
The overall "Target" for the battery current (`ui8_adc_battery_current_target`)—which is based on your pedaling torque, cadence, and assist level—is calculated in the main application loop.

*   **Frequency:** Every **25 milliseconds** (40 Hz).
*   **Where in code:** Inside `ebike_app_controller()` in `src/ebike_app.c`, which is triggered by the main loop in `src/main.c`.
*   **What it does:** Every 25ms, the system looks at the rider's inputs and decides, "Okay, for the next 25ms, the goal is to draw exactly X Amps."

#### B. The Motor Control Adjustment Rate (High Frequency)
While the *goal* is updated every 25ms, the actual *adjustment* of power (to reach that goal and respect safety limits) happens inside the same high-frequency PWM interrupt where the measurements are taken.

*   **Frequency:** Up to **18 kHz / 19 kHz** (every ~55 microseconds).
*   **Where in code:** Also in the `TIM1_UPD_OVF_TRG_BRK_IRQHandler` in `src/motor.c` (around line 800+).
*   **What it does:** Every 55 microseconds, the code checks:
    1. Is the actual battery current lower/higher than the 25ms target?
    2. Is the actual phase current exceeding the hard safety limit?
    *   If power needs to go up or down, it increments or decrements the PWM duty cycle right then and there.

**Summary:** 
*   **Measured:** Every ~55 microseconds (18-19 kHz).
*   **Rider Input Evaluated:** Every 25 milliseconds (40 Hz).
*   **Power Adjusted (Duty Cycle changed):** Evaluated every ~55 microseconds, ensuring the motor responds instantly to limit-breaches while smoothly ramping up to the rider's 25ms pedaling targets.

---

**User**

for BATTERY_CURRENT_MAX values in the range 10..15, create a table of max battery- and phase currents

---

**Cursor**

Based on the firmware's standard integer math, here is a table showing exactly how different `BATTERY_CURRENT_MAX` settings between 10A and 15A translate into internal ADC steps and actual motor phase current limits.

### How the math works in the C code:
1. **Battery Limit (ADC Steps):** `(Configured_Amps * 100) / 16` (decimals are truncated/rounded down).
2. **Phase Limit (ADC Steps):** `(Battery_ADC_Steps * 187) / 112` (decimals are truncated).
3. **Actual Phase Amps:** `Phase_ADC_Steps * 0.16`

### The Current Limits Table

| Configured `BATTERY_CURRENT_MAX` | Target Battery Limit (ADC Steps) | Calculated Phase Limit (ADC Steps) | **Max Motor Phase Current (Amps)** |
| :---: | :---: | :---: | :---: |
| **10 A** | 62 | 103 | **16.48 A** |
| **11 A** | 68 | 113 | **18.08 A** |
| **12 A** | 75 | 125 | **20.00 A** |
| **13 A** | 81 | 135 | **21.60 A** |
| **14 A** | 87 | 145 | **23.20 A** |
| **15 A** | 93 | 155 | **24.80 A** |

*Note: The hardcoded absolute safety limits in the firmware are 112 steps (18.0A) for the battery and 187 steps (29.92A) for the motor phases. None of the values in the 10-15A range hit these safety ceilings, so these calculated limits are what the ones actually applied while riding.*

