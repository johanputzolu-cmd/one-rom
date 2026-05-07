// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License
//
// Flash erase critical section for NV storage support.
//
// MUST be compiled with -fPIC -fno-plt.  The function is copied into a RAM
// slot at runtime and called via function pointer, so that it can execute
// while XIP is disabled during the erase sequence.
//
// Self-contained by design: no globals, no static data, no external calls
// beyond those passed as parameters.  Verify with objdump that no absolute
// addresses appear in the output.

#include <stdint.h>
#include "flash_erase.h"

// RP2350 flash geometry
#define FLASH_BLOCK_SIZE      (65536u) // 64KB — largest erase granularity
#define FLASH_BLOCK_ERASE_CMD (0xD8u)  // SPI block erase command

__attribute__((section(".flash_erase_fn"), noinline))
void flash_erase_critical(
    flash_exit_xip_fn_t             exit_xip,
    flash_range_erase_fn_t          range_erase,
    flash_flush_cache_fn_t          flush_cache,
    flash_select_xip_read_mode_fn_t select_xip,
    uint32_t                        flash_offs,
    uint32_t                        size,
    uint8_t                         clkdiv
) {
    __asm volatile ("cpsid i");
    exit_xip();
    range_erase(flash_offs, size, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
    select_xip(3u, clkdiv);
    flush_cache();
    __asm volatile ("cpsie i");
}