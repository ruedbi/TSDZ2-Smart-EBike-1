/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */
// clang-format off

 #ifndef EEPROM_H_
 #define EEPROM_H_
 
 #include "main.h"
 
 #define EEPROM_BASE_ADDRESS                                 0x4000
 
 #define ADDRESS_KEY                                     0 + EEPROM_BASE_ADDRESS
 #define ADDRESS_BATTERY_CURRENT_MAX                     1 + EEPROM_BASE_ADDRESS
 #define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0       2 + EEPROM_BASE_ADDRESS
 #define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1       3 + EEPROM_BASE_ADDRESS
 #define ADDRESS_WHEEL_PERIMETER_0                       4 + EEPROM_BASE_ADDRESS
 #define ADDRESS_WHEEL_PERIMETER_1                       5 + EEPROM_BASE_ADDRESS
 // for oem display
 #define ADDRESS_STARTUP_ASSIST_ENABLED					EEPROM_BASE_ADDRESS + 6
 #define ADDRESS_TORQUE_SENSOR_ESTIMATED					EEPROM_BASE_ADDRESS + 7
 #define ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100	EEPROM_BASE_ADDRESS + 8
 #define ADDRESS_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION EEPROM_BASE_ADDRESS + 9
 #define ADDRESS_MOTOR_ASSISTANCE_WITH_ERROR_ENABLED		EEPROM_BASE_ADDRESS + 10
 #define ADDRESS_BATTERY_SOC								EEPROM_BASE_ADDRESS + 11																
 #define ADDRESS_SET_PARAMETER_ON_STARTUP				EEPROM_BASE_ADDRESS + 12
 #define ADDRESS_STREET_MODE_ON_STARTUP					EEPROM_BASE_ADDRESS + 13
 #define ADDRESS_RIDING_MODE_ON_STARTUP					EEPROM_BASE_ADDRESS + 14
 #define ADDRESS_LIGHTS_CONFIGURATION_ON_STARTUP			EEPROM_BASE_ADDRESS + 15
 #define ADDRESS_STARTUP_BOOST_ON_STARTUP				EEPROM_BASE_ADDRESS + 16
 #define ADDRESS_ENABLE_AUTO_DATA_DISPLAY				EEPROM_BASE_ADDRESS + 17
 #define ADDRESS_SOC_PERCENT_CALC						EEPROM_BASE_ADDRESS + 18
 #define ADDRESS_TORQUE_SENSOR_ADV_ON_STARTUP			EEPROM_BASE_ADDRESS + 19
 #define ADDRESS_CONSUMED_WH_X10_0						EEPROM_BASE_ADDRESS + 20
 #define ADDRESS_CONSUMED_WH_X10_1						EEPROM_BASE_ADDRESS + 21
 #define ADDRESS_CONSUMED_WH_X10_2						EEPROM_BASE_ADDRESS + 22
#define ADDRESS_CONSUMED_WH_X10_3						EEPROM_BASE_ADDRESS + 23
// unloaded battery voltage (x10 V) recorded at the last regular power-off, 16-bit little-endian;
// persisted as part of the block snapshot and used at startup for battery-change detection
#define ADDRESS_BATTERY_VOLTAGE_AT_SHUTDOWN_X10_0		EEPROM_BASE_ADDRESS + 24
#define ADDRESS_BATTERY_VOLTAGE_AT_SHUTDOWN_X10_1		EEPROM_BASE_ADDRESS + 25
#define EEPROM_BYTES_STORED                             20

// Block-based power-off persistence: the data EEPROM is treated as fixed-size blocks of
// FLASH_BLOCK_SIZE bytes, each programmed in a single FLASH_ProgramBlock() cycle.
// Block 0 holds the authoritative live configuration that is read at startup; block 1 holds
// the snapshot written at power-off. On boot a valid block 1 is promoted into block 0.
#define EEPROM_LIVE_BLOCK								0
#define EEPROM_SHUTDOWN_BLOCK							1
// Offset of the CRC byte inside a block: the last byte holds the CRC-8 of all preceding bytes.
#define ADDRESS_BLOCK_CRC								(EEPROM_BASE_ADDRESS + FLASH_BLOCK_SIZE - 1)
 #define EEPROM_BYTES_STORED_OEM_DISPLAY					13
 #define EEPROM_BYTES_INIT_OEM_DISPLAY					EEPROM_BYTES_STORED - EEPROM_BYTES_STORED_OEM_DISPLAY
 
 
 // system
 #define DEFAULT_VALUE_KEY     204
 #define SET_TO_DEFAULT        0
 #define READ_FROM_MEMORY      1
 #define WRITE_TO_MEMORY       2
 
void EEPROM_init(void);

void EEPROM_controller(uint8_t ui8_operation, uint8_t ui8_byte_init);

uint32_t EEPROM_read_consumed_wh_x10(void);
/// Writes the consumed watt-hours x10 value; unlocks and re-locks the data EEPROM.
void EEPROM_write_consumed_wh_x10(uint32_t ui32_value);

/// Programs one FLASH_BLOCK_SIZE block of the data EEPROM in a single FLASH_ProgramBlock()
/// cycle. The last byte of \p ui8_buffer is overwritten with the CRC-8 of all preceding
/// bytes before programming, so the stored block is self-checking. \p ui8_buffer must point
/// to a RAM array of at least FLASH_BLOCK_SIZE bytes; the function unlocks and re-locks the
/// data EEPROM itself.
void EEPROM_write_block_with_crc(uint8_t ui8_block_index, uint8_t *ui8_buffer);

/// Builds the current configuration snapshot (settings + battery SOC + consumed Wh) and
/// writes it to the shutdown block in a single block-programming cycle. Called from the
/// power-off ISR (interrupts disabled) and from the periodic mid-ride save; Wh bytes
/// come from the main-loop latch, not from a 32-bit add in this path.
void EEPROM_save_shutdown_snapshot(void);

/// Reads the unloaded battery voltage (x10 V) recorded in the live block at the last regular
/// power-off. A blank STM8 data EEPROM reads 0x00, so 0 means "uninitialized" and the
/// battery-change detector treats it as "accept the next full battery once".
uint16_t EEPROM_read_battery_voltage_at_shutdown_x10(void);

#endif /* EEPROM_H_ */
