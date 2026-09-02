// clang-format off
/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

 #include <stdint.h>
 #include "stm8s.h"
 #include "stm8s_flash.h"
 #include "main.h"
 #include "common.h"
 #include "eeprom.h"
 #include "ebike_app.h"
 #include "motor.h"
 #include "flash_ram.h"
 
 static const uint8_t ui8_default_array[EEPROM_BYTES_STORED] = 
 {
   DEFAULT_VALUE_KEY,							// 0 + EEPROM_BASE_ADDRESS (Array index)
   BATTERY_CURRENT_MAX,							// 1 + EEPROM_BASE_ADDRESS
   BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,			// 2 + EEPROM_BASE_ADDRESS
   BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,			// 3 + EEPROM_BASE_ADDRESS
   WHEEL_PERIMETER_0,							// 4 + EEPROM_BASE_ADDRESS
   WHEEL_PERIMETER_1,							// 5 + EEPROM_BASE_ADDRESS
   // for oem display
   STARTUP_ASSIST_ENABLED,						// 6 + EEPROM_BASE_ADDRESS
   TORQUE_SENSOR_ESTIMATED,						// 7 + EEPROM_BASE_ADDRESS
   PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100,		// 8 + EEPROM_BASE_ADDRESS
   MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION,		// 9 + EEPROM_BASE_ADDRESS
   ASSISTANCE_WITH_ERROR_ENABLED,				// 10 + EEPROM_BASE_ADDRESS
   BATTERY_SOC,									// 11 + EEPROM_BASE_ADDRESS
   ENABLE_SET_PARAMETER_ON_STARTUP,				// 12 + EEPROM_BASE_ADDRESS
   ENABLE_STREET_MODE_ON_STARTUP,				// 13 + EEPROM_BASE_ADDRESS
   RIDING_MODE_ON_STARTUP,						// 14 + EEPROM_BASE_ADDRESS
   LIGHTS_CONFIGURATION_ON_STARTUP,				// 15 + EEPROM_BASE_ADDRESS
   STARTUP_BOOST_ON_STARTUP,						// 16 + EEPROM_BASE_ADDRESS
   ENABLE_AUTO_DATA_DISPLAY,						// 17 + EEPROM_BASE_ADDRESS
   SOC_PERCENT_CALC,								// 18 + EEPROM_BASE_ADDRESS
   TORQUE_SENSOR_ADV_ON_STARTUP					// 19 + EEPROM_BASE_ADDRESS
 };
 
static uint8_t ui8_error_number = 0;

/// Shared 128-byte image for block program/promote. File-scope so the PWM ISR
/// save path does not put a FLASH_BLOCK_SIZE array on SDCC overlay RAM (nested
/// ISRs share that overlay and would corrupt the snapshot while it is built).
static uint8_t ui8_eeprom_block_buffer[FLASH_BLOCK_SIZE];

// Computes the CRC-8 (polynomial 0x07, initial value 0x00) over the first \p ui8_length bytes.
static uint8_t EEPROM_crc8(const uint8_t *ui8_data, uint8_t ui8_length);
// Builds the 128-byte live-configuration block image (settings + SOC + consumed Wh) in \p ui8_buffer.
static void EEPROM_build_live_block(uint8_t *ui8_buffer);
// Promotes a valid shutdown snapshot (block 1) into the live block (block 0) at startup.
static uint8_t EEPROM_promote_shutdown_block(void);

void EEPROM_init(void)
{
   volatile uint32_t ui32_delay_counter = 0;

#ifdef COPY_TO_RAM
   // relocate the block-program routine into RAM before any block write (the promote
   // below may write the live block)
   flash_ram_init();
#endif

   // deinitialize EEPROM
   FLASH_DeInit();
   
   // time delay
   for (ui32_delay_counter = 0; ui32_delay_counter < 160000; ++ui32_delay_counter) {}
   
   // select and set programming time mode
   FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD); // standard programming (erase and write) time mode
   //FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG); // fast programming (write only) time mode
   
  // time delay
  for (ui32_delay_counter = 0; ui32_delay_counter < 160000; ++ui32_delay_counter) {}
  
  // if the last power-off wrote a valid snapshot block, promote it into the live block before
  // anything is read, so the key check and READ_FROM_MEMORY below see the persisted values
  EEPROM_promote_shutdown_block();
  
  // read key
  volatile uint8_t ui8_saved_key = FLASH_ReadByte(ADDRESS_KEY);
   
   // check if key is valid
   if (ui8_saved_key != DEFAULT_VALUE_KEY)
   {
     // set to default values
     EEPROM_controller(SET_TO_DEFAULT, 0);
   }
   
   // read from EEPROM
   EEPROM_controller(READ_FROM_MEMORY, 0);
 }
 
 
 
 void EEPROM_controller(uint8_t ui8_operation, uint8_t ui8_byte_init)
 {
   struct_configuration_variables *p_configuration_variables;
   p_configuration_variables = get_configuration_variables();
   
   uint8_t ui8_array[EEPROM_BYTES_STORED];
   uint8_t ui8_temp;
   uint16_t ui16_temp;
   uint8_t ui8_i;
 
   // unlock memory
   FLASH_Unlock(FLASH_MEMTYPE_DATA);
   
   // wait until data EEPROM area unlocked flag is set
   while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) {}
   
   // select EEPROM operation
   switch (ui8_operation)
   {
     
     
     /********************************************************************************************************************************************************/
     
     
     case SET_TO_DEFAULT:
     
       // write array of variables to EEPROM, write key last
       for (ui8_i = EEPROM_BYTES_STORED; ui8_i > 0; ui8_i--)
       {
         // get address
         uint32_t ui32_default_address = (uint32_t) ui8_i - 1 + EEPROM_BASE_ADDRESS;
         
         // get value
         uint8_t ui8_default_variable_value = ui8_default_array[ui8_i - 1];
         
         // write variable value to EEPROM
         FLASH_ProgramByte(ui32_default_address, ui8_default_variable_value);
         
         // wait until end of programming (write or erase operation) flag is set
         while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET) {}
         
         // read value from EEPROM for validation
         volatile uint8_t ui8_saved_default_value = FLASH_ReadByte(ui32_default_address);
         
         // if write was not successful, rewrite
         if (ui8_saved_default_value != ui8_default_variable_value)
                 {
                         // limit errors number
                         ui8_error_number += 1;
                         if(ui8_error_number > 3)
                                 ui8_display_fault_code = ERROR_WRITE_EEPROM;
                         else
                                 ui8_i = EEPROM_BYTES_STORED;
                 }
       }
       
     break;
     
     
     /********************************************************************************************************************************************************/
     
     
     case READ_FROM_MEMORY:
       
       //p_configuration_variables->ui8_motor_power_x10 = FLASH_ReadByte(ADDRESS_MOTOR_POWER_X10); // NOT USED
       p_configuration_variables->ui8_battery_current_max = FLASH_ReadByte(ADDRESS_BATTERY_CURRENT_MAX);
           
       ui16_temp = FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
       ui8_temp = FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
       ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
       p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;
       
       ui16_temp = FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_0);
       ui8_temp = FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_1);
       ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
       p_configuration_variables->ui16_wheel_perimeter = ui16_temp;
 
       p_configuration_variables->ui8_startup_assist_enabled = FLASH_ReadByte(ADDRESS_STARTUP_ASSIST_ENABLED);
 
       p_configuration_variables->ui8_torque_sensor_estimated = FLASH_ReadByte(ADDRESS_TORQUE_SENSOR_ESTIMATED);
       
       p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_est_x100 = FLASH_ReadByte(ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100);
       // for oem display
           p_configuration_variables->ui8_assist_without_pedal_rotation_enabled = FLASH_ReadByte(ADDRESS_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION);
          
           p_configuration_variables->ui8_assist_with_error_enabled = FLASH_ReadByte(ADDRESS_MOTOR_ASSISTANCE_WITH_ERROR_ENABLED);
           p_configuration_variables->ui8_battery_SOC_percentage_8b = FLASH_ReadByte(ADDRESS_BATTERY_SOC);
           p_configuration_variables->ui8_set_parameter_enabled = FLASH_ReadByte(ADDRESS_SET_PARAMETER_ON_STARTUP);
           p_configuration_variables->ui8_street_mode_enabled = FLASH_ReadByte(ADDRESS_STREET_MODE_ON_STARTUP);
           p_configuration_variables->ui8_riding_mode = FLASH_ReadByte(ADDRESS_RIDING_MODE_ON_STARTUP);
           p_configuration_variables->ui8_lights_configuration = FLASH_ReadByte(ADDRESS_LIGHTS_CONFIGURATION_ON_STARTUP);
           p_configuration_variables->ui8_startup_boost_enabled = FLASH_ReadByte(ADDRESS_STARTUP_BOOST_ON_STARTUP);
           p_configuration_variables->ui8_auto_display_data_enabled = FLASH_ReadByte(ADDRESS_ENABLE_AUTO_DATA_DISPLAY);
       p_configuration_variables->ui8_soc_percent_calculation = FLASH_ReadByte(ADDRESS_SOC_PERCENT_CALC);
           
           p_configuration_variables->ui8_torque_sensor_adv_enabled = FLASH_ReadByte(ADDRESS_TORQUE_SENSOR_ADV_ON_STARTUP);
           
     break;
     
     
     /********************************************************************************************************************************************************/
     
     
     case WRITE_TO_MEMORY:
     
       ui8_array[0] = DEFAULT_VALUE_KEY;
     
       //ui8_array[ADDRESS_MOTOR_POWER_X10 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_power_x10; // NOT USED
       ui8_array[ADDRESS_BATTERY_CURRENT_MAX - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_current_max;
                         
       ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
       ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
       
       ui8_array[ADDRESS_WHEEL_PERIMETER_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_wheel_perimeter & 255;
       ui8_array[ADDRESS_WHEEL_PERIMETER_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;
       
       ui8_array[ADDRESS_STARTUP_ASSIST_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_startup_assist_enabled;
       
       ui8_array[ADDRESS_TORQUE_SENSOR_ESTIMATED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_torque_sensor_estimated;
       
       ui8_array[ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_est_x100;
       // for oem display
           ui8_array[ADDRESS_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_assist_without_pedal_rotation_enabled;
           
           ui8_array[ADDRESS_MOTOR_ASSISTANCE_WITH_ERROR_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_assist_with_error_enabled;
           ui8_array[ADDRESS_BATTERY_SOC - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_SOC_percentage_8b;
           ui8_array[ADDRESS_SET_PARAMETER_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_set_parameter_enabled;
           ui8_array[ADDRESS_STREET_MODE_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_enabled;
           ui8_array[ADDRESS_RIDING_MODE_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_riding_mode;
           ui8_array[ADDRESS_LIGHTS_CONFIGURATION_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_lights_configuration;
           ui8_array[ADDRESS_STARTUP_BOOST_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_startup_boost_enabled;
           ui8_array[ADDRESS_ENABLE_AUTO_DATA_DISPLAY - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_auto_display_data_enabled;
           ui8_array[ADDRESS_SOC_PERCENT_CALC - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_soc_percent_calculation;
           ui8_array[ADDRESS_TORQUE_SENSOR_ADV_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_torque_sensor_adv_enabled;
           
       // write array of variables to EEPROM
       for (ui8_i = EEPROM_BYTES_STORED; ui8_i > ui8_byte_init; ui8_i--)
       {
         // get address
         uint32_t ui32_address = (uint32_t) ui8_i - 1 + EEPROM_BASE_ADDRESS;
         
         // get value
         uint8_t ui8_variable_value = ui8_array[ui8_i - 1];
         
         // write variable value to EEPROM
         FLASH_ProgramByte(ui32_address, ui8_variable_value);
         
         // wait until end of programming (write or erase operation) flag is set
         while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET) {}
         
         // read value from EEPROM for validation
         volatile uint8_t ui8_saved_value = FLASH_ReadByte(ui32_address);
         
         // if write was not successful, rewrite
                 if (ui8_saved_value != ui8_variable_value)
                 {
                         // limit errors number
                         ui8_error_number += 1;
                         if(ui8_error_number > 3)
                                 // E09 (E08 blinking for XH18)
                                 ui8_display_fault_code = ERROR_WRITE_EEPROM;
                         else
                                 ui8_i = EEPROM_BYTES_STORED;
                 }
       }
       
     break;
   }
   
   // lock memory
   FLASH_Lock(FLASH_MEMTYPE_DATA);
 }
 
/// Programs the 4 consumed watt-hours x10 bytes, assuming the data EEPROM is
/// already unlocked. Shared by the locked and unlocked public entry points so
/// the actual byte programming logic exists in only one place.
static void EEPROM_program_consumed_wh_x10(uint32_t ui32_value) {
    uint8_t ui8_i;
    uint32_t ui32_address;

    // store the 32-bit value little-endian across 4 consecutive EEPROM bytes
    for (ui8_i = 0; ui8_i < 4; ui8_i++) {
        ui32_address = (uint32_t)ADDRESS_CONSUMED_WH_X10_0 + ui8_i;
        FLASH_ProgramByte(ui32_address, (uint8_t)(ui32_value >> (ui8_i * 8)));
        // wait until end of programming flag is set before writing the next byte
        while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET) {
        }
    }
}

void EEPROM_write_consumed_wh_x10(uint32_t ui32_value) {
    // standalone caller: data EEPROM is locked, so unlock it here first
    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    // wait until data EEPROM area unlocked flag is set
    while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) {
    }

    EEPROM_program_consumed_wh_x10(ui32_value);

    // lock memory
    FLASH_Lock(FLASH_MEMTYPE_DATA);
}

uint32_t EEPROM_read_consumed_wh_x10(void) {
        uint32_t ui32_value;
    
        ui32_value = (uint32_t)FLASH_ReadByte(ADDRESS_CONSUMED_WH_X10_0);
        ui32_value |= (uint32_t)FLASH_ReadByte(ADDRESS_CONSUMED_WH_X10_1) << 8;
        ui32_value |= (uint32_t)FLASH_ReadByte(ADDRESS_CONSUMED_WH_X10_2) << 16;
        ui32_value |= (uint32_t)FLASH_ReadByte(ADDRESS_CONSUMED_WH_X10_3) << 24;
    
        if (ui32_value == 0xFFFFFFFFUL) {
            return 0;
        }
    
        return ui32_value;
}

static void EEPROM_program_odometer_meters(uint32_t ui32_value) {
    uint8_t ui8_i;
    uint32_t ui32_address;

    // store the 32-bit value little-endian across 4 consecutive EEPROM bytes
    for (ui8_i = 0; ui8_i < 4; ui8_i++) {
        ui32_address = (uint32_t)ADDRESS_ODOMETER_METERS_0 + ui8_i;
        FLASH_ProgramByte(ui32_address, (uint8_t)(ui32_value >> (ui8_i * 8)));
        // wait until end of programming flag is set before writing the next byte
        while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET) {
        }
    }
}

void EEPROM_write_odometer_meters(uint32_t ui32_value) {
    // standalone caller: data EEPROM is locked, so unlock it here first
    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    // wait until data EEPROM area unlocked flag is set
    while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) {
    }

    EEPROM_program_odometer_meters(ui32_value);

    // lock memory
    FLASH_Lock(FLASH_MEMTYPE_DATA);
}

uint32_t EEPROM_read_odometer_meters(void) {
    uint32_t ui32_value;

    ui32_value = (uint32_t)FLASH_ReadByte(ADDRESS_ODOMETER_METERS_0);
    ui32_value |= (uint32_t)FLASH_ReadByte(ADDRESS_ODOMETER_METERS_1) << 8;
    ui32_value |= (uint32_t)FLASH_ReadByte(ADDRESS_ODOMETER_METERS_2) << 16;
    ui32_value |= (uint32_t)FLASH_ReadByte(ADDRESS_ODOMETER_METERS_3) << 24;

    if (ui32_value == 0xFFFFFFFFUL) {
        return 0;
    }

    return ui32_value;
}

static uint8_t EEPROM_crc8(const uint8_t *ui8_data, uint8_t ui8_length) {
    uint8_t ui8_crc = 0x00;
    uint8_t ui8_i;
    uint8_t ui8_bit;

    // standard bitwise CRC-8 with polynomial 0x07 (MSB-first), no input/output reflection
    for (ui8_i = 0; ui8_i < ui8_length; ui8_i++) {
        ui8_crc ^= ui8_data[ui8_i];
        for (ui8_bit = 0; ui8_bit < 8; ui8_bit++) {
            if (ui8_crc & 0x80) {
                ui8_crc = (uint8_t)((ui8_crc << 1) ^ 0x07);
            } else {
                ui8_crc = (uint8_t)(ui8_crc << 1);
            }
        }
    }

    return ui8_crc;
}

void EEPROM_write_block_with_crc(uint8_t ui8_block_index, uint8_t *ui8_buffer) {
    // append the CRC-8 of all data bytes as the last byte of the block so the stored block
    // can be validated on the next startup before it is trusted
    ui8_buffer[FLASH_BLOCK_SIZE - 1] = EEPROM_crc8(ui8_buffer, FLASH_BLOCK_SIZE - 1);

    // unlock memory
    FLASH_Unlock(FLASH_MEMTYPE_DATA);

    // wait until data EEPROM area unlocked flag is set
    while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) {}

    // program the whole block in a single cycle (standard mode erases then writes the block);
    // the 128 source bytes are latched from the RAM buffer in one operation
#ifdef COPY_TO_RAM
    // run the byte-latching loop from RAM; the destination address is computed here
    // (in flash) so the relocated routine itself stays call-free and position independent
    flash_program_block_ram_ptr(
        (uint8_t *)(FLASH_DATA_START_PHYSICAL_ADDRESS + ((uint16_t)ui8_block_index * FLASH_BLOCK_SIZE)),
        ui8_buffer);
#else
    FLASH_ProgramBlock((uint16_t)ui8_block_index, FLASH_MEMTYPE_DATA, FLASH_PROGRAMMODE_STANDARD, ui8_buffer);
#endif

    // wait until end of programming (write or erase operation) flag is set
    while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET) {}

    // lock memory
    FLASH_Lock(FLASH_MEMTYPE_DATA);
}

static void EEPROM_build_live_block(uint8_t *ui8_buffer) {
    struct_configuration_variables *p_configuration_variables;
    uint8_t ui8_i;

    p_configuration_variables = get_configuration_variables();

    // start from a fully zeroed block so unused bytes (and the CRC byte) are deterministic
    for (ui8_i = 0; ui8_i < FLASH_BLOCK_SIZE; ui8_i++) {
        ui8_buffer[ui8_i] = 0;
    }

    // key marks the block as written and is checked before the block is trusted at startup
    ui8_buffer[ADDRESS_KEY - EEPROM_BASE_ADDRESS] = DEFAULT_VALUE_KEY;

    ui8_buffer[ADDRESS_BATTERY_CURRENT_MAX - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_current_max;

    ui8_buffer[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
    ui8_buffer[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;

    ui8_buffer[ADDRESS_WHEEL_PERIMETER_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_wheel_perimeter & 255;
    ui8_buffer[ADDRESS_WHEEL_PERIMETER_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;

    ui8_buffer[ADDRESS_STARTUP_ASSIST_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_startup_assist_enabled;
    ui8_buffer[ADDRESS_TORQUE_SENSOR_ESTIMATED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_torque_sensor_estimated;
    ui8_buffer[ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_est_x100;
    ui8_buffer[ADDRESS_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_assist_without_pedal_rotation_enabled;
    ui8_buffer[ADDRESS_MOTOR_ASSISTANCE_WITH_ERROR_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_assist_with_error_enabled;
    // battery SOC at power-off: the value to be restored on the next startup
    ui8_buffer[ADDRESS_BATTERY_SOC - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_SOC_percentage_8b;
    ui8_buffer[ADDRESS_SET_PARAMETER_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_set_parameter_enabled;
    ui8_buffer[ADDRESS_STREET_MODE_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_enabled;
    ui8_buffer[ADDRESS_RIDING_MODE_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_riding_mode;
    ui8_buffer[ADDRESS_LIGHTS_CONFIGURATION_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_lights_configuration;
    ui8_buffer[ADDRESS_STARTUP_BOOST_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_startup_boost_enabled;
    ui8_buffer[ADDRESS_ENABLE_AUTO_DATA_DISPLAY - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_auto_display_data_enabled;
    ui8_buffer[ADDRESS_SOC_PERCENT_CALC - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_soc_percent_calculation;
    ui8_buffer[ADDRESS_TORQUE_SENSOR_ADV_ON_STARTUP - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_torque_sensor_adv_enabled;

    // consumed watt-hours x10 at power-off: copy the main-loop latch byte-wise so
    // this ISR-reachable path never performs a 32-bit add on SDCC overlay RAM
    ui8_buffer[ADDRESS_CONSUMED_WH_X10_0 - EEPROM_BASE_ADDRESS] = ui8_consumed_wh_x10_for_shutdown_save[0];
    ui8_buffer[ADDRESS_CONSUMED_WH_X10_1 - EEPROM_BASE_ADDRESS] = ui8_consumed_wh_x10_for_shutdown_save[1];
    ui8_buffer[ADDRESS_CONSUMED_WH_X10_2 - EEPROM_BASE_ADDRESS] = ui8_consumed_wh_x10_for_shutdown_save[2];
    ui8_buffer[ADDRESS_CONSUMED_WH_X10_3 - EEPROM_BASE_ADDRESS] = ui8_consumed_wh_x10_for_shutdown_save[3];

    // unloaded battery voltage x10 at power-off, stored little-endian across 2 bytes; the next
    // startup uses it to detect whether the battery was charged or swapped while powered off
    ui8_buffer[ADDRESS_BATTERY_VOLTAGE_AT_SHUTDOWN_X10_0 - EEPROM_BASE_ADDRESS] = (uint8_t)(ui16_battery_voltage_filtered_x10_for_shutdown_save & 0xFF);
    ui8_buffer[ADDRESS_BATTERY_VOLTAGE_AT_SHUTDOWN_X10_1 - EEPROM_BASE_ADDRESS] = (uint8_t)((ui16_battery_voltage_filtered_x10_for_shutdown_save >> 8) & 0xFF);

    // travelled distance (odometer in meters) at power-off: copy the main-loop latch byte-wise
    ui8_buffer[ADDRESS_ODOMETER_METERS_0 - EEPROM_BASE_ADDRESS] = ui8_odometer_meters_for_shutdown_save[0];
    ui8_buffer[ADDRESS_ODOMETER_METERS_1 - EEPROM_BASE_ADDRESS] = ui8_odometer_meters_for_shutdown_save[1];
    ui8_buffer[ADDRESS_ODOMETER_METERS_2 - EEPROM_BASE_ADDRESS] = ui8_odometer_meters_for_shutdown_save[2];
    ui8_buffer[ADDRESS_ODOMETER_METERS_3 - EEPROM_BASE_ADDRESS] = ui8_odometer_meters_for_shutdown_save[3];
}

void EEPROM_save_shutdown_snapshot(void) {
    // assemble the current configuration + dynamic values, then persist it as the shutdown
    // snapshot in a single block-programming cycle
    EEPROM_build_live_block(ui8_eeprom_block_buffer);
    EEPROM_write_block_with_crc(EEPROM_SHUTDOWN_BLOCK, ui8_eeprom_block_buffer);
}

uint16_t EEPROM_read_battery_voltage_at_shutdown_x10(void) {
    uint16_t ui16_value;

    ui16_value = (uint16_t)FLASH_ReadByte(ADDRESS_BATTERY_VOLTAGE_AT_SHUTDOWN_X10_0);
    ui16_value |= (uint16_t)FLASH_ReadByte(ADDRESS_BATTERY_VOLTAGE_AT_SHUTDOWN_X10_1) << 8;

    // a blank STM8 data EEPROM reads 0x00, so 0 is the intended "uninitialized" sentinel
    return ui16_value;
}

static uint8_t EEPROM_promote_shutdown_block(void) {
    uint32_t ui32_shutdown_block_address;
    uint8_t ui8_i;

    // the shutdown block starts one block above the live block
    ui32_shutdown_block_address = (uint32_t)EEPROM_BASE_ADDRESS + FLASH_BLOCK_SIZE;

    // read the whole shutdown block into RAM
    for (ui8_i = 0; ui8_i < FLASH_BLOCK_SIZE; ui8_i++) {
        ui8_eeprom_block_buffer[ui8_i] = FLASH_ReadByte(ui32_shutdown_block_address + ui8_i);
    }

    // reject a blank/never-written block: a freshly flashed data EEPROM reads 0x00, so the
    // key (which is non-zero for a valid block) catches that case before the CRC is trusted
    if (ui8_eeprom_block_buffer[ADDRESS_KEY - EEPROM_BASE_ADDRESS] != DEFAULT_VALUE_KEY) {
        return 0;
    }

    // reject a corrupted / partially written block
    if (EEPROM_crc8(ui8_eeprom_block_buffer, FLASH_BLOCK_SIZE - 1) != ui8_eeprom_block_buffer[FLASH_BLOCK_SIZE - 1]) {
        return 0;
    }

    // snapshot is valid: overwrite the live block with it so the normal read path picks it up
    EEPROM_write_block_with_crc(EEPROM_LIVE_BLOCK, ui8_eeprom_block_buffer);

    return 1;
}

    