// One ROM utils

// Copyright (C) 2025 Piers Finlayson <piers@piers.rocks>
//
// MIT License

#include "include.h"
#include "roms.h"

//
// Logging functions
//

#if defined(BOOT_LOGGING)
extern uint32_t _sdrr_runtime_info_start;
extern uint32_t _ram_rom_image_start[];
// Logging function to output various debug information via RTT
void log_init(void) {
    LOG(log_divider);
    LOG("%s v%d.%d.%d.%d %s", product, sdrr_info.major_version, sdrr_info.minor_version, sdrr_info.patch_version, sdrr_info.build_number, project_url);
    LOG("%s %s", copyright, author);
#if defined(DEBUG_BUILD)
    LOG("Built: %s (DEBUG)", sdrr_info.build_date);
#else // !DEBUG_BUILD
    LOG("Built: %s", sdrr_info.build_date);
#endif // DEBUG_BUILD
    LOG("Commit: %s", sdrr_info.commit);

    LOG("ROM: %d pin", sdrr_info.pins->chip_pins);
    LOG("USB: %s", sdrr_info.extra->usb_dfu ? "Y" : "N");

    // This refers to dropping in DFU/BOOTSEL mode when all the image select
    // jumpers are closed, and is disabled by default.
    if (sdrr_info.bootloader_capable) {
        LOG("Sel boot: %s", enabled);
    } else {
        DEBUG("Sel boot: %s", disabled);
    }

    if (sdrr_info.status_led_enabled) {
        DEBUG("LED: enabled - P%s:%d",
            port_names[sdrr_info.pins->status_port],
            sdrr_info.pins->status);
    } else {
        DEBUG("LED: disabled");
    }

    DEBUG("sdrr_info: 0x%08X", (uint32_t)(uintptr_t)&sdrr_info);
    DEBUG("sdrr_extra_info: 0x%08X", (uint32_t)(uintptr_t)sdrr_info.extra);
    DEBUG("RAM ROM table: 0x%08X", (uint32_t)(uintptr_t)&_ram_rom_image_start);
    DEBUG("sdrr_runtime_info: 0x%08X", (uint32_t)(uintptr_t)sdrr_info.extra->runtime_info);
    DEBUG("RTT CB: 0x%08X", (uint32_t)(uintptr_t)sdrr_info.extra->rtt);
    DEBUG(log_divider);
    DEBUG("RT Ice Freq: 0x%04X", sdrr_runtime_info.ice_freq);
    DEBUG("RT Fire Freq: 0x%04X", sdrr_runtime_info.fire_freq);
    DEBUG("RT Overclock Enabled: 0x%02X", sdrr_runtime_info.overclock_enabled);
    DEBUG("RT Status LED Enabled: 0x%02X", sdrr_runtime_info.status_led_enabled);
    DEBUG("RT SWD Enabled: 0x%02X", sdrr_runtime_info.swd_enabled);
    DEBUG("RT PIO mode: %s", sdrr_runtime_info.fire_serve_mode == FIRE_SERVE_PIO ? "Y" : "N");

    LOG(log_divider);
    platform_logging();

#if defined(C_MAIN_LOOP)
    LOG("C main loop: enabled");
#endif // C_MAIN_LOOP

    DEBUG(log_divider);
    
    // Data pins
    DEBUG("D[0-7]: P%s:%d,%d,%d,%d,%d,%d,%d,%d", 
        port_names[sdrr_info.pins->data_port],
        sdrr_info.pins->data[0], sdrr_info.pins->data[1], sdrr_info.pins->data[2], sdrr_info.pins->data[3],
        sdrr_info.pins->data[4], sdrr_info.pins->data[5], sdrr_info.pins->data[6], sdrr_info.pins->data[7]);
    if (sdrr_info.pins->data2[0] != 0xFF) {
        DEBUG("D[8-15]: P%s:%d,%d,%d,%d,%d,%d,%d,%d", 
            port_names[sdrr_info.pins->data_port],
            sdrr_info.pins->data2[0], sdrr_info.pins->data2[1], sdrr_info.pins->data2[2], sdrr_info.pins->data2[3],
            sdrr_info.pins->data2[4], sdrr_info.pins->data2[5], sdrr_info.pins->data2[6], sdrr_info.pins->data2[7]);
    }
    
    // Address pins
    DEBUG("A[0-15]: P%s:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
        port_names[sdrr_info.pins->addr_port],
        sdrr_info.pins->addr[0], sdrr_info.pins->addr[1], sdrr_info.pins->addr[2], sdrr_info.pins->addr[3],
        sdrr_info.pins->addr[4], sdrr_info.pins->addr[5], sdrr_info.pins->addr[6], sdrr_info.pins->addr[7],
        sdrr_info.pins->addr[8], sdrr_info.pins->addr[9], sdrr_info.pins->addr[10], sdrr_info.pins->addr[11],
        sdrr_info.pins->addr[12], sdrr_info.pins->addr[13], sdrr_info.pins->addr[14], sdrr_info.pins->addr[15]);
    if (sdrr_info.pins->addr2[0] != 0xFF) {
        DEBUG("A[16-31]: P%s:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
            port_names[sdrr_info.pins->addr_port],
            sdrr_info.pins->addr2[0], sdrr_info.pins->addr2[1], sdrr_info.pins->addr2[2], sdrr_info.pins->addr2[3],
            sdrr_info.pins->addr2[4], sdrr_info.pins->addr2[5], sdrr_info.pins->addr2[6], sdrr_info.pins->addr2[7],
            sdrr_info.pins->addr2[8], sdrr_info.pins->addr2[9], sdrr_info.pins->addr2[10], sdrr_info.pins->addr2[11],
            sdrr_info.pins->addr2[12], sdrr_info.pins->addr2[13], sdrr_info.pins->addr2[14], sdrr_info.pins->addr2[15]);
    }
        
    // Chip select pins
    DEBUG("CS: P%s:%d,%d,%d,%d,%d X1: P%s:%d X2: P%s:%d", 
        port_names[sdrr_info.pins->cs_port], sdrr_info.pins->cs1, sdrr_info.pins->cs2, sdrr_info.pins->cs3,
        sdrr_info.pins->ce, sdrr_info.pins->oe,
        port_names[sdrr_info.pins->cs_port], sdrr_info.pins->x1, port_names[sdrr_info.pins->cs_port], sdrr_info.pins->x2);
    
    // Select and status pins
    DEBUG("Sel: P%s:%d,%d,%d,%d,%d,%d,%d", port_names[sdrr_info.pins->sel_port], 
        sdrr_info.pins->sel[0], sdrr_info.pins->sel[1], 
        sdrr_info.pins->sel[2], sdrr_info.pins->sel[3],
        sdrr_info.pins->sel[4], sdrr_info.pins->sel[5],
        sdrr_info.pins->sel[6]);
    DEBUG("LED pin: P%s:%d", port_names[sdrr_info.pins->status_port], sdrr_info.pins->status);
    if (sdrr_info.extra->usb_dfu) {
        DEBUG("VBUS: P%s:%d", 
            port_names[sdrr_info.extra->usb_port],
            sdrr_info.extra->vbus_pin);
    }

#if !defined(EXECUTE_FROM_RAM)
    DEBUG("Execute from: %s", flash);
#else // EXECUTE_FROM_RAM
    LOG("Execute from: %s", ram);
#endif // EXECUTE_FROM_RAM

    LOG(log_divider);
}

void log_roms(const onerom_metadata_header_t *metadata_header) {

    uint8_t extra_info = metadata_header->rom_sets[0].extra_info;
#if defined(DEBUG_LOGGING)
    if (extra_info == 1) {
        DEBUG("ROM sets: v0.6.0+");
    } else {
        DEBUG("ROM sets: pre-v0.6.0");
    }
#endif // DEBUG_LOGGING

    LOG("# of ROM sets: %d", metadata_header->rom_set_count);

    // Need to cope with two different sizes of sdrr_rom_set_t structure
    size_t stride = (extra_info == 1) ? sizeof(sdrr_rom_set_t) : 16;
    uint8_t *base = (uint8_t *)metadata_header->rom_sets;

    for (uint8_t ii = 0; ii < metadata_header->rom_set_count; ii++) {
        const sdrr_rom_set_t *set = (const sdrr_rom_set_t *)(base + (stride * ii));

        LOG("Set #%d: %d ROM(s), size: %d bytes", ii, set->rom_count, set->size);
        
#if defined(DEBUG_LOGGING)
        for (uint8_t jj = 0; jj < set->rom_count; jj++) {
            const sdrr_rom_info_t *rom = set->roms[jj];
            const char *rom_type_str = chip_type_strings[rom->rom_type];

            DEBUG("  Chip #%d: %s, %s",
                jj, rom->filename,
                rom_type_str);
        }
#endif // DEBUG_LOGGING
    }
}
#endif // BOOT_LOGGING

#if !(TEST_BUILD)
void __attribute__((noinline)) do_log_v(const char* msg, va_list* args) {
    SEGGER_RTT_vprintf(0, msg, args);
    SEGGER_RTT_printf(0, "\n");
}

void do_err_log_prefix() {
    SEGGER_RTT_printf(0, "ERROR: ");
}

#if defined(DEBUG_LOGGING)
void do_debug_log_prefix() {
    SEGGER_RTT_printf(0, "DBG: ");
}
#endif // DEBUG_LOGGING

#if defined(BOOT_LOGGING)
void __attribute__((noinline)) do_log(const char* msg, ...) {
    va_list args;
    va_start(args, msg);
    do_log_v(msg, &args);
    va_end(args);
}

void __attribute__((noinline)) err_log(const char* msg, ...) {
    do_err_log_prefix();
    va_list args;
    va_start(args, msg);
    do_log_v(msg, &args);
    va_end(args);
}
#endif // BOOT_LOGGING
#endif // !TEST_BUILD

//
// Functions to handle copying functions to and executing them from RAM
//

#if defined(EXECUTE_FROM_RAM)

// Copies a function from flash to RAM
void copy_func_to_ram(void (*fn)(void), uint32_t ram_addr, size_t size) {
    // Copy the function to RAM
    memcpy((void*)ram_addr, (void*)((uint32_t)fn & ~1), size);
}

void execute_ram_func(uint32_t ram_addr) {
    // Execute the function in RAM
    void (*ram_func)() = (void(*)(void))(ram_addr | 1);
    ram_func();
}

#endif // EXECUTE_FROM_RAM

// Simple delay function
void delay(volatile uint32_t count) {
    while(count--);
}
