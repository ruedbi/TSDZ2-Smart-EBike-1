// clang-format off
/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef COMMON_COMMON_H_
#define COMMON_COMMON_H_

#include <stdint.h>

// riding modes
#define OFF_MODE                                  0
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define HYBRID_ASSIST_MODE						  5
#define CRUISE_MODE                               6
#define WALK_ASSIST_MODE                          7
#define TORQUE_SENSOR_CALIBRATION_MODE            8

// walk assist
//#define WALK_ASSIST_THRESHOLD_SPEED_X10           70  // 70 -> 7.0 kph, this is the maximum speed limit from which walk assist can be activated

// cruise
//#define CRUISE_THRESHOLD_SPEED_X10                90  // 90 -> 9.0 kph, this is the minimum speed limit from which cruise can be activated

// optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2

uint16_t map_ui16(uint16_t in, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
uint8_t map_ui8(uint8_t in, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);
uint8_t ui8_max(uint8_t value_a, uint8_t value_b);
uint8_t ui8_min(uint8_t value_a, uint8_t value_b);
uint16_t filter(uint16_t ui16_new_value, uint16_t ui16_old_value, uint8_t ui8_alpha);
void crc16(uint8_t ui8_data, uint16_t *ui16_crc);

#if ENABLE_VLCD5
#define ENABLE_DZ40MINI_AS_VLCD5 1
// #define RELEASE_BUILD
// #define DEBUG_BUILD
#define SCALE_WHEEL_SPEED_TIME_IN_OFFROAD_MODE

// activate with my own limits:
// #define LIMIT_CURRENTS
#define BATTERY_CURRENT_SOFT_LIMIT 16 // Amps
#define BATTERY_CURRENT_HARD_LIMIT 22 // Amps, must exceed soft limit by some margin
#define PHASE_CURRENT_LIMIT 20 // Amps; OSF default: 30A


// Speed-dependent motor phase current limit (see ebike_app.c)
#define PHASE_CURRENT_SPEED_LIMIT_ENABLED 1
#if PHASE_CURRENT_SPEED_LIMIT_ENABLED
// Wheel speed km/h * 10 (same units as ui16_wheel_speed_x10)
#define PHASE_CURRENT_SPEED_LOW_X10 80
#define PHASE_CURRENT_SPEED_MEDIUM_X10 150
// Phase current ADC limit above speed medium; ~0.16 A per step = 16A
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_PROTECT 100
#endif // PHASE_CURRENT_SPEED_LIMIT_ENABLED

#endif // ENABLE_VLCD5

#endif /* COMMON_COMMON_H_ */
