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

// This translation unit contains ONLY the block-program routine. SDCC's
// "#pragma codeseg" sets the code segment for the whole module, so the routine is
// kept in its own file to land entirely in the RAM_SEG section. The linker symbol
// l_RAM_SEG (used by flash_ram_init) then equals exactly this routine's length.
#pragma codeseg RAM_SEG

// Routine that must execute from RAM: while the data-EEPROM byte latches are being
// loaded the CPU cannot fetch instructions from flash. Contains no calls to other
// functions, so the copied bytes are position independent (only relative branches
// plus absolute IO/data accesses). The destination address is computed by the caller
// (in flash) and passed in, keeping this routine self-contained.
void flash_program_block_ram(uint8_t *ui8_dest, uint8_t *ui8_source) {
    uint8_t ui8_count;

    // select standard block programming mode
    FLASH->CR2 |= FLASH_CR2_PRG;
    FLASH->NCR2 &= (uint8_t)(~FLASH_NCR2_NPRG);

    // latch all block bytes in a single uninterrupted cycle; this loop is the part
    // that must run from RAM
    for (ui8_count = 0; ui8_count < FLASH_BLOCK_SIZE; ui8_count++) {
        ui8_dest[ui8_count] = ui8_source[ui8_count];
    }
}

#endif // COPY_TO_RAM
