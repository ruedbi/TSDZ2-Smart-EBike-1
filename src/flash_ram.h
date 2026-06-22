/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Released under the GPL License, Version 3
 */
// clang-format off

#ifndef FLASH_RAM_H_
#define FLASH_RAM_H_

#include <stdint.h>
#include "common.h"

#ifdef COPY_TO_RAM

/// Block-program routine as linked in flash (in its own RAM_SEG section). Defined in
/// flash_ram_routine.c; used as the source address for the runtime copy into RAM.
/// Do not call directly once relocated; invoke through flash_program_block_ram_ptr.
void flash_program_block_ram(uint8_t *ui8_dest, uint8_t *ui8_source);

/// Function pointer to the RAM-resident block-program routine. Valid only after
/// flash_ram_init() has run. Latches FLASH_BLOCK_SIZE bytes from \p ui8_source into
/// the data-EEPROM block starting at \p ui8_dest, executing entirely from RAM.
extern void (*flash_program_block_ram_ptr)(uint8_t *ui8_dest, uint8_t *ui8_source);

/// Relocates the block-program routine into RAM and arms flash_program_block_ram_ptr.
/// Must be called once at startup before any data-EEPROM block write.
void flash_ram_init(void);

#endif // COPY_TO_RAM

#endif // FLASH_RAM_H_
