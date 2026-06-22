/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Released under the GPL License, Version 3
 */
// clang-format off

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_flash.h"
#include "common.h"
#include "flash_ram.h"

#ifdef COPY_TO_RAM

// Maximum number of bytes reserved in RAM for the relocated block-program routine.
// The actual routine is only a few dozen bytes; verify against the linker symbol
// l_RAM_SEG (see flash_ram_init) and the .map file if the routine is changed.
#define FLASH_RAM_ROUTINE_SIZE 96

// RAM-resident copy of flash_program_block_ram() filled once at startup.
static uint8_t ui8_flash_ram_routine[FLASH_RAM_ROUTINE_SIZE];

// Byte length of the RAM_SEG code section, loaded from the linker symbol l_RAM_SEG.
// Module-global so the inline assembly can address it as _ui16_ram_seg_len.
static uint16_t ui16_ram_seg_len = 0;

// Points at the RAM copy of the routine once flash_ram_init() has run.
void (*flash_program_block_ram_ptr)(uint8_t *ui8_dest, uint8_t *ui8_source) = 0;

// Copies the RAM_SEG routine from flash into the RAM buffer and arms the call
// pointer. Must be called once at startup before any block write is attempted.
void flash_ram_init(void) {
    uint16_t ui16_i;
    // start address where the routine is linked in flash; source of the copy
    uint8_t *ui8_src = (uint8_t *)flash_program_block_ram;

    // l_RAM_SEG is the linker-provided byte length of the RAM_SEG area, which holds
    // exactly flash_program_block_ram (it is the only function in its module)
    __asm
        ldw x, #l_RAM_SEG
        ldw _ui16_ram_seg_len, x
    __endasm;

    // relocate the routine into RAM
    for (ui16_i = 0; ui16_i < ui16_ram_seg_len; ui16_i++) {
        ui8_flash_ram_routine[ui16_i] = ui8_src[ui16_i];
    }

    // from now on the routine is invoked from RAM through this pointer
    flash_program_block_ram_ptr = (void (*)(uint8_t *, uint8_t *))ui8_flash_ram_routine;
}

#endif // COPY_TO_RAM
