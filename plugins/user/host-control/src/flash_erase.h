// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License

#if !defined(FLASH_ERASE_H)
#define FLASH_ERASE_H

#include <stdint.h>

typedef void (*flash_exit_xip_fn_t)(void);
typedef void (*flash_range_erase_fn_t)(uint32_t offs, uint32_t count, uint32_t block_size, uint8_t block_cmd);
typedef void (*flash_flush_cache_fn_t)(void);
typedef void (*flash_select_xip_read_mode_fn_t)(uint8_t mode, uint8_t clkdiv);
typedef void (*flash_range_program_fn_t)(uint32_t offs, const uint8_t *data, uint32_t count);
typedef void (*connect_internal_flash_fn_t)(void);

typedef void (*nv_flash_erase_critical_fn_t)(
    flash_exit_xip_fn_t             exit_xip,
    flash_range_erase_fn_t          range_erase,
    flash_flush_cache_fn_t          flush_cache,
    flash_select_xip_read_mode_fn_t select_xip,
    uint32_t                        flash_offs,
    uint32_t                        size,
    uint8_t                         clkdiv
);

void flash_erase_critical(
    flash_exit_xip_fn_t             exit_xip,
    flash_range_erase_fn_t          range_erase,
    flash_flush_cache_fn_t          flush_cache,
    flash_select_xip_read_mode_fn_t select_xip,
    uint32_t                        flash_offs,
    uint32_t                        size,
    uint8_t                         clkdiv
);

#endif // FLASH_ERASE_H