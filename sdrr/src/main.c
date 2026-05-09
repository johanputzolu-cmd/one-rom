// One ROM Main startup code (clock and GPIO initialisation)

// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License

#include "include.h"

#if !defined(RP235X) && !defined(STM32F4)
    #error "Unsupported MCU line - please define RP235X or STM32F4"
#endif // !RP2350 && !STM32F4

const char sdrr_build_date[] = __DATE__ " " __TIME__;

#if !defined(TEST_BUILD)
#define SECTION_SDRR_RUNTIME_INFO __attribute__((section(".sdrr_runtime_info")))
#else // TEST_BUILD
#define SECTION_SDRR_RUNTIME_INFO
#endif // TEST_BUILD
sdrr_runtime_info_t sdrr_runtime_info SECTION_SDRR_RUNTIME_INFO = {
    .magic = {'s', 'd', 'r', 'r'},  // Lower case to distinguish from firmware magic
    .runtime_info_size = sizeof(sdrr_runtime_info_t),
    .image_sel = 0xFF,
    .rom_set_index = 0xFF,
    .count_rom_access = 0x00,
    .access_count = 0xFFFFFFFF,
    .rom_table = NULL,
    .rom_table_size = 0,
    .bootloader_entry = 0,
#if defined(OVERCLOCK) && (OVERCLOCK == 1)
    .overclock_enabled = 1,
#else // !OVERCLOCK
    .overclock_enabled = 0,
#endif // OVERCLOCK
    .status_led_enabled = 0,  // updated from sdrr_info in vector.c
    .swd_enabled = 0,  // updated from sdrr_info in vector.c
    .fire_vreg = FIRE_VREG_STOCK,
    .ice_freq = ICE_FREQ_NONE,
    .fire_freq = FIRE_FREQ_NONE,
    .sysclk_mhz = TARGET_FREQ_MHZ,
#if defined(RP_PIO) && (RP_PIO == 1)
    .fire_serve_mode = FIRE_SERVE_PIO,
#else // !RP_PIO
    .fire_serve_mode = FIRE_SERVE_CPU,
#endif // RP_PIO
    .bit_mode = BIT_MODE_8,
    .rom_dma_copy = 1,
    .num_data_pins = 8,
#if defined(FORCE_16_BIT) && (FORCE_16_BIT == 1)
    .force_16_bit = 1,
#else // !FORCE_16_BIT
    .force_16_bit = 0,
#endif // FORCE_16_BIT
    .peri_en = 0,
    .system_plugin_context = NULL,
    .user_plugin_context = NULL,
    .timer0_irq_0_handler = NULL,
    .usbctrl_irq_handler = NULL,
    .limp_mode = LIMP_MODE_NONE,
    .piorom_config = {
        // This is properly set up at runtime in piorom.c
        .invert_cs = {0, 0, 0, 0},
        .num_cs_pins = 0,
        .cs_base_pin = 0,
        .data_base_pin = 0,
        .num_data_pins = 0,
        .addr_base_pin = 0,
        .num_addr_pins = 0,
        .addr_read_irq = 0,
        .addr_read_delay = 0,
        .cs_active_delay = 0,
        .cs_inactive_delay = 0,
        .no_dma = 0,
        .byte_pin = 0,
        .a_minus_1_pin = 0,
        .a_minus_1_signal_pin = 0,
        .force_16_bit = 0,
        .rom_type = INVALID_CHIP_TYPE,
        .rom_table_addr = 0,
        .addr_reader_read_clkdiv_int = 0,
        .addr_reader_read_clkdiv_frac = 0,
        .pad2 = 0,
        .a_minus_1_clkdiv_int = 0,
        .a_minus_1_clkdiv_frac = 0,
        .pad3 = 0,
        .data_io_clkdiv_int = 0,
        .data_io_clkdiv_frac = 0,
        .pad4 = 0,
        .data_out_clkdiv_int = 0,
        .data_out_clkdiv_frac = 0,
        .pad5 = 0,
        .contiguous_cs_pins = 0,
        .multi_rom_mode = 0,
        .bit_mode = 0,
        .pad6 = 0,
        .cs_pin_2nd_match = 0,
    },
};

// This function checks the state of the image select pins, and returns an
// integer value, as if the sel pins control bit 0, 1, 2, 3, etc in order of
// that integer.  The first sel pin in the array is bit 0, the second bit 1, 
// etc.
uint32_t check_sel_pins(uint32_t *sel_mask) {
    uint32_t num_sel_pins, sel_value;
    uint64_t orig_sel_mask, gpio_value, sel_flip_bits;

    // Setup the pins first.  Do this first to allow any pull-ups to settle
    // before reading.
    num_sel_pins = setup_sel_pins(&orig_sel_mask, &sel_flip_bits);
    if (num_sel_pins == 0) {
        DEBUG("No image select pins");
        disable_sel_pins();
        *sel_mask = 0;
        return 0;
    }

    // Read the actual GPIO value, masked appropriately
    gpio_value = get_sel_value(orig_sel_mask, sel_flip_bits);

    // SEGGER doesn't cope well with 64-bit values, so split into two 32-bit
    // parts for logging
    DEBUG("Sel GPIO val: 0x%08lX%08lX #: %lu mask 0x%08lX%08lX", 
        (uint32_t)(gpio_value >> 32), (uint32_t)gpio_value,
        (unsigned long)num_sel_pins,
        (uint32_t)(orig_sel_mask >> 32), (uint32_t)orig_sel_mask);

    disable_sel_pins();

    // Now turn the GPIO value into a SEL value, with the bits consecutive
    // starting from bit 0, based on which pin the SEL value is.  At the same
    // time we have to update sel_mask, to match.  This gives us an integer
    // which can be used as an index into the rom set
    *sel_mask = 0;
    sel_value = 0;
    for (int ii = 0; ii < MAX_IMG_SEL_PINS; ii++) {
        uint8_t pin = sdrr_info.pins->sel[ii];
        if (pin < MAX_USED_GPIOS) {
            if (gpio_value & (1ULL << pin)) {
                sel_value |= (1 << ii);
            }
            *sel_mask |= (1ULL << ii);
        }
    }

    DEBUG("Sel value: 0x%08X mask: 0x%08X", sel_value, *sel_mask);

    // Store the value of the pins in sdrr_runtime_info
    sdrr_runtime_info.image_sel = sel_value;

    return sel_value;
}

// Check whether we shoud enter the device's bootloader and, if so, enter it.
// This is indicated via jumping SEL0, SEL1, and SEL2 - PB0-2.  These are all
// pulled up to enter the bootloader.  STM32F4 variant from rev E onwards also
// include PB7 as the most significant bit.
//
// This must be done before we set up the PLL, peripheral clocks, etc, as
// those must be disabled for the bootloader.
//
// As this checks the sel pins, cache and return the result, so we don't need
// to re-check it later.
void check_enter_bootloader(uint32_t sel_pins, uint32_t sel_mask) {
    LOG("Checking whether to enter bootloader");

    if ((sel_mask) && ((sel_pins & sel_mask) == sel_mask)) {
        // SEL pins are all high - enter the bootloader
        LOG("Entering bootloader");

        // Pause to allow the log to be received
        for (volatile int ii = 0; ii < 1000000; ii++);

        enter_bootloader();
    }

    return;
}

// Check the metadata is present
uint8_t metadata_present(const sdrr_info_t *info) {
    onerom_metadata_header_t *metadata = (onerom_metadata_header_t *)info->metadata_header;
    uint8_t present = 1;

    // Check for magic in metadata
    for (int ii = 0; ii < 16; ii++) {
        if (metadata->magic[ii] != "ONEROM_METADATA"[ii]) {
            present = 0;
            break;
        }
    }

    if (present) {
        if (metadata->version == 1) {
            LOG("Metadata v%d", metadata->version);
        } else {
            ERR("Unsupported metadata version: %d", metadata->version);
            present = 0;
        }
    } else {
        LOG("Valid metadata header not found");
    }

    return present;
}

#if !defined(TEST_BUILD)
void limp_mode(limp_mode_pattern_t pattern) {
    LOG("Limp mode %d", pattern);

    sdrr_runtime_info.limp_mode = pattern;

    uint32_t on_time, off_time;

    if (!sdrr_runtime_info.status_led_enabled && sdrr_info.status_led_enabled) {
        DEBUG("Enabled status LED");
        setup_status_led();
    }

    if (pattern >= NUM_LIMP_MODE_PATTERNS) {
        DEBUG("!!! Invalid limp mode pattern");
        pattern = LIMP_MODE_NONE;
    }

    on_time = limp_mode_patterns[pattern].on_time;
    off_time = limp_mode_patterns[pattern].off_time;

    while (1) {
        blink_pattern(on_time, off_time, 1);
    }
}
#endif // !TEST_BUILD

// Read in firmware overrides from the selected ROM set, if present (0.6.0+)
// and modify sdrr_runtime_info accordingly.
void process_firmware_overrides(
    sdrr_runtime_info_t *runtime_info,
    const sdrr_rom_set_t *set
) {
    if (set->extra_info == 1) {
        const onerom_firmware_overrides_t *overrides = set->firmware_overrides;
        if ((overrides != NULL) && (overrides != (void*)0xFFFFFFFF)) {
#if defined(STM32F4)
            if (overrides->override_present[0] & (1 << 0)) {
                runtime_info->ice_freq = overrides->ice_freq;
                LOG("ICE freq override: %d", runtime_info->ice_freq);
            }
            if (overrides->override_present[0] & (1 << 1)) {
                runtime_info->overclock_enabled = overrides->override_value[0] & (1 << 0) ? 1 : 0;
                LOG("ICE overclock override: %d", runtime_info->overclock_enabled);
            }
#endif
#if defined(RP235X)
            if (overrides->override_present[0] & (1 << 2)) {
                runtime_info->fire_freq = overrides->fire_freq;
                LOG("Fire freq override: %d", runtime_info->fire_freq);
            }
            if (overrides->override_present[0] & (1 << 3)) {
                runtime_info->overclock_enabled = overrides->override_value[0] & (1 << 1) ? 1 : 0;
                LOG("Fire overclock override: %d", runtime_info->overclock_enabled);
            }
            if (overrides->override_present[0] & (1 << 4)) {
                runtime_info->fire_vreg = overrides->fire_vreg;
                LOG("Fire VREG override: %d", runtime_info->fire_vreg);
            }
#endif
            if (overrides->override_present[0] & (1 << 5)) {
                runtime_info->status_led_enabled = overrides->override_value[0] & (1 << 2) ? 1 : 0;
                LOG("Status LED override: %d", runtime_info->status_led_enabled);
            }
            if (overrides->override_present[0] & (1 << 6)) {
                runtime_info->swd_enabled = overrides->override_value[0] & (1 << 3) ? 1 : 0;
                LOG("SWD enabled override: %d", runtime_info->swd_enabled);
            }
#if defined(RP235X)
            if (overrides->override_present[0] & (1 << 7)) {
                runtime_info->fire_serve_mode = overrides->override_value[0] & (1 << 4) ? FIRE_SERVE_PIO : FIRE_SERVE_CPU;
                LOG("Fire serve mode override: %d", runtime_info->fire_serve_mode);
                if ((runtime_info->fire_serve_mode == FIRE_SERVE_CPU) && (sdrr_info.pins->chip_pins != 24)) {
                    ERR("Serve mode CPU is only supported on One ROM 24");
                    limp_mode(LIMP_MODE_INVALID_CONFIG);
                }
            }
            if (overrides->override_present[1] & (1 << 0)) {
                runtime_info->rom_dma_copy = overrides->override_value[0] & (1 << 5) ? 1 : 0;
                LOG("Fire ROM DMA preload override: %d", runtime_info->rom_dma_copy);
            }
            if (overrides->override_present[1] & (1 << 1)) {
                runtime_info->force_16_bit = overrides->override_value[0] & (1 << 6) ? 1 : 0;
                LOG("Fire Force 16 bit mode override: %d", runtime_info->force_16_bit);
            }
#endif
        }
    }
    else if (set->extra_info == 0) {
        LOG("No extra info in ROM set - no overrides present");
    } else {
        ERR("Unsupported extra_info value in ROM set: %d", set->extra_info);
    }
}

uint8_t check_plugin_valid(
    const ora_plugin_header_t *header,
    const ora_plugin_type_t expected_type,
    uint8_t index
) {
    if (header->magic != ORA_PLUGIN_MAGIC) {
        ERR("ORA badmagic 0x%08x", header->magic);
        return 0;
    }
    if (header->api_version != ORA_PLUGIN_VERSION_1) {
        ERR("ORA version 0x%08x", header->api_version);
        return 0;
    }
    if (header->plugin_type != expected_type) {
        ERR("ORA type %d, expected %d", header->plugin_type, expected_type);
        return 0;
    }

    // A plugin is expected to be located at 0x10010000, 0x10020000, etc based
    // on the specific ROM set it is.
    uint32_t expected_launch_region = (0x1001 + index) << 16;
    uint32_t entry_addr = (uint32_t)(uintptr_t)header->entry;
    if ((entry_addr & ~expected_launch_region) >= 0x10000) {
        ERR("ORA 0x%08x vs ep 0x%08x", entry_addr, expected_launch_region);
        return 0;
    }

    uint16_t min_fw_major = header->min_fw_major_version;
    uint16_t min_fw_minor = header->min_fw_minor_version;
    uint16_t min_fw_patch = header->min_fw_patch_version;
    if (min_fw_major > sdrr_info.major_version ||
        (min_fw_major == sdrr_info.major_version && min_fw_minor > sdrr_info.minor_version) ||
        (min_fw_major == sdrr_info.major_version && min_fw_minor == sdrr_info.minor_version && min_fw_patch > sdrr_info.patch_version)) {
        ERR("ORA reqd v%d.%d.%d vs v%d.%d.%d",
            min_fw_major, min_fw_minor, min_fw_patch,
            sdrr_info.major_version, sdrr_info.minor_version, sdrr_info.patch_version);
        return 0;
    }

    return 1;
}

uint8_t initial_plugin_parse(uint8_t *disable_vbus_det) {
    uint8_t plugins = 0;
    *disable_vbus_det = 0;

    if (sdrr_info.metadata_header->rom_set_count >= 1) {
        const sdrr_rom_set_t *set = &sdrr_info.metadata_header->rom_sets[0];
        if (set->roms[0]->rom_type == CHIP_TYPE_SYSTEM_PLUGIN) {
            const ora_plugin_header_t *header = (ora_plugin_header_t *)(uintptr_t)(set->data);
            if (check_plugin_valid(header, ORA_PLUGIN_TYPE_SYSTEM, 0)) {
                *disable_vbus_det = header->overrides1 & ORA_OVERRIDE1_DISABLE_VBUS_DETECT ? 1 : 0;
                LOG("Valid plugin detected, disable_vbus_det=%d", *disable_vbus_det);
            }

            // Have system plugin (1)
            plugins |= 1;
        } else {
            DEBUG("ROM is not a plugin, skipping plugin parsing");
        }

        if (sdrr_info.metadata_header->rom_set_count > 1) {
            const sdrr_rom_set_t *other_set = &sdrr_info.metadata_header->rom_sets[1];
            if (other_set->roms[0]->rom_type == CHIP_TYPE_USER_PLUGIN) {
                if (plugins & 0x01) {
                    // Have user plugin (2) so check it
                    const ora_plugin_header_t *header = (ora_plugin_header_t *)(uintptr_t)(set->data);
                    if (check_plugin_valid(header, ORA_PLUGIN_TYPE_USER, 1)) {
                        *disable_vbus_det = header->overrides1 & ORA_OVERRIDE1_DISABLE_VBUS_DETECT ? 1 : 0;
                        LOG("Valid plugin detected, disable_vbus_det=%d", *disable_vbus_det);
                    }
                    DEBUG("User plugin");
                    plugins |= 2;
                } else {
                    ERR("Ignoring user plugin without system plugin");
                }
            }
        }
    } else {
        DEBUG("No ROM sets defined, skipping plugin parsing");
    }

    return plugins;
}

// Needs to do the following:
// - Set up the clock to 68.8Mhz
// - Set up GPIO ports A, B and C to inputs
// - Load the selected ROM image into RAM for faster access
// - Run the main loop, from RAM
//
// Startup needs to be a small number of hundreds of ms, so it's complete and
// the main loop is running before the other hardware is accessing the ROM.
//
// The hardware takes around 200us to power up, then maybe 200us for the PLL to
// lock, in clock_init().  The rest of time we have for our code.
//
// preload_rom_image is likely to take the longest, as it is copying an 8KB
// ROM image to RAM, and having to deal with the internal complexity of
// remapping the data to the bit ordering we need, and to skip bit 3 (and use
// bit 14 instead).
#if !defined(TEST_BUILD)
int main(void) {
#else // TEST_BUILD
int firmware_main(void) {
#endif // TEST_BUILD
    // Platform specific initialization
    platform_specific_init();

    // Initialize GPIOs.  Do it now before checking bootloader mode.
    DEBUG("Init GPIO");
    setup_gpio();

    // Enable logging.  Done after GPIO setup, so SWD pins are configured.
    if (sdrr_info.boot_logging_enabled) {
        LOG_INIT();
    }

    // Do initial plugin parsing.  The system plugin can potentially override
    // USB DFU support, which is why we do it now.
    //
    // We have to do this on STM32F4 even though plugins aren't supported,
    // as we need to know if should skip any plugins when dealing with the image
    // sel pins.
    uint8_t disable_vbus_det = 0;
    uint8_t plugins = 0;
    DEBUG("Initial plugin parse");
    plugins = initial_plugin_parse(&disable_vbus_det);
#if defined(STM32F4)
    // Never disable VBUS detect on STM32F4 as the plugin won't run 
    disable_vbus_det = 0;
#endif // STM32F4

    // Set up VBUS detect interrupt.  Done next, so we can enter DFU mode as 
    // soon as USB plugged in
    if (sdrr_info.extra->usb_dfu) {
        if (!disable_vbus_det) {
            DEBUG("Init VBUS det");
            setup_vbus_interrupt();
        } else {
            LOG("Plugin disabled VBUS detect");
        }
    }

    // Read image select pin values - we need this to check whether to enter
    // bootloader mode if they are all 1.
    uint32_t sel_mask, sel_pins;
    sel_pins = check_sel_pins(&sel_mask);

    // Now check whether to enter bootloader mode
    if (sdrr_info.bootloader_capable) {
        check_enter_bootloader(sel_pins, sel_mask);
    }
    
    // Now get the rom set from the image select pins.  We do this before
    // setting up the clock, in case there's any clock configuration overrides
    // to be applied from the selected ROM set.
    const sdrr_rom_set_t *set = NULL;
    uint8_t md = metadata_present(&sdrr_info);
#if defined(BOOT_LOGGING)
    if (md) {
        log_roms(sdrr_info.metadata_header);
    }
#endif
    uint8_t num_plugins = plugins & 1 ? 1 : 0;
    num_plugins += plugins & 2 ? 1 : 0;
    if (md && (sdrr_info.metadata_header->rom_set_count > num_plugins)) {
        // Find out if extra_info is set to 1 on the first set.  If it is, the
        // set structures are 0.6.0+ and are the size of sdrr_rom_set_t (64
        // bytes), otherwise they are pre 0.6.0 (probably created by an old
        // Studio/wasm version) and hence 16 bytes.
        uint8_t extra_info = sdrr_info.metadata_header->rom_sets[0].extra_info;
        DEBUG("First ROM set extra_info = %d", extra_info);
        if (extra_info == 1) {
            DEBUG("ROM sets are 0.6.0+ format");
        } else {
            DEBUG("ROM sets are pre-0.6.0 format");
        }
        uint8_t *base = (uint8_t *)sdrr_info.metadata_header->rom_sets;
        size_t stride = (extra_info == 1) ? sizeof(sdrr_rom_set_t) : 16;

        // Figure out what ROM set to use
        sdrr_runtime_info.rom_set_index = get_rom_set_index(sel_pins, sel_mask, plugins);

        // Get pointer to the selected ROM set
        set = (const sdrr_rom_set_t *)(base + (stride * sdrr_runtime_info.rom_set_index));

        // Now process any firmware overrides from the selected ROM set.
        // This should be safe because the first thing that does is check that
        // extra_info is 1.
        process_firmware_overrides(&sdrr_runtime_info, set);
    } else if (!md) {
        LOG("No metadata");
    } else {
        ERR("No ROM sets");
    }

    // Initialize clock
    DEBUG("Init clock");
    setup_clock();

#if !defined(TIMER_TEST) && !defined(TOGGLE_PA4)
    if (set != NULL) {
        // Set up the ROM table
        if (sdrr_info.preload_image_to_ram) {
            sdrr_runtime_info.rom_table = preload_rom_image(&sdrr_runtime_info, set);
        } else {
            // If we are not preloading the ROM image, we need to set up the
            // rom_table to point to the flash location of the ROM image.
            sdrr_runtime_info.rom_table = (void *)&(set->data[0]);
        }
        sdrr_runtime_info.rom_table_size = set->size;
    }
#endif // !TIMER_TEST && !TOGGLE_PA4

    // Startup MCO after preloading the ROM - this allows us to test (with a
    // scope), how long the startup takes.
    if (sdrr_info.mco_enabled) {
        DEBUG("Init MCO");
        setup_mco();
    }

    // Setup status LED up now, so we don't need to call the function from the
    // main loop - which might be running from RAM.
    if (sdrr_runtime_info.status_led_enabled) {
        DEBUG("Init LED");
        setup_status_led();
    }

    // If we're in ROM mode, and no ROM set is selected, enter limp mode
    if (set == NULL) {
        // Brief blink pattern to indicate no ROM being served.  Stays off for
        // a fifth of the time as it is on.  Exact timings depend on clock
        // speed.  At 100MHz this is roughly 0.5s on 2.5s off.
        limp_mode(LIMP_MODE_NO_ROMS);
    }

    // Do final checks before entering the main loop
    check_config(&sdrr_info, &sdrr_runtime_info, set);

    // Startup - from a stable 5V supply to here - takes:
    // - ~3ms    F411 100MHz BOOT_LOGGING=1
    // - ~1.5ms  F411 100MHz BOOT_LOGGING=0

// Check for incompatible options
#if defined(EXECUTE_FROM_RAM) && defined(XIP_CACHE_WARM)
#error "EXECUTE_FROM_RAM and XIP_CACHE_WARM cannot be defined at the same time"
#endif

#if !defined(PRELOAD_TO_RAM)
#if defined(EXECUTE_FROM_RAM)
// PRELOAD_TO_RAM is for the ROM image, EXECUTE_FROM_RAM is main_loop()
#error "PRELOAD_TO_RAM must be defined when EXECUTE_FROM_RAM is enabled"
#endif // EXECUT_FROM_RAM
#if defined(XIP_CACHE_WARM)
#error "XIP_CACHE_WARM cannot be defined when EXECUTE_FROM_RAM is enabled"
#endif // XIP_CACHE_WARM
#endif // PRELOAD_TO_RAM

#if !defined(EXECUTE_FROM_RAM) && !defined(XIP_CACHE_WARM)
    // Execute the main_loop
#if !defined(MAIN_LOOP_LOGGING)
    LOG("Start main loop");
#endif // !MAIN_LOOP_LOGGING
    //XIP_QMI_M0_TIMING &= ~0x04;
    //XIP_QMI_M0_TIMING |= 0x01;
    main_loop(&sdrr_info, &sdrr_runtime_info, set);
    return 0;
#endif

#if defined(EXECUTE_FROM_RAM) || defined(XIP_CACHE_WARM)
    // We need to set up a copy of some of sdrr_info and linked to data, in
    // order for main_loop() to be able to access it.  If we don't do this,
    // main_loop() will try to access the original sdrr_info, which is in
    // flash, and it will use relative addressing, which won't work when 
    // executing from RAM, or is sub-optimal, if using XIP cache pinning.

    // Set up addresses to copy sdrr_info and related data to

    // These come from the linker
    extern uint8_t _sdrr_info_ram_start[];
    extern uint8_t _sdrr_info_ram_end[];

    // The _addresses_ of the linker variables are the locations we're
    // interested in
    uint8_t *sdrr_info_ram_start = &_sdrr_info_ram_start[0];
    uint8_t *sdrr_info_ram_end = &_sdrr_info_ram_end[0];
    uint32_t ram_size = sdrr_info_ram_end - sdrr_info_ram_start;
    uint32_t required_size = sizeof(sdrr_info_t) + sizeof(sdrr_pins_t) + sizeof(sdrr_rom_set_t);
    DEBUG("RAM start: 0x%08X, end: 0x%08X", (unsigned int)sdrr_info_ram_start, (unsigned int)sdrr_info_ram_end);
    DEBUG("RAM size: 0x%08X bytes, required size: 0x%08X bytes", ram_size, required_size);
    if (required_size > ram_size) {
        ERR("Not enough RAM for sdrr_info and related data");
    }
    // Continue anyway :-|

    // Copy sdrr_info to RAM
    uint8_t *ptr = sdrr_info_ram_start;
    sdrr_info_t *info = (sdrr_info_t *)ptr;
    memcpy(info, &sdrr_info, sizeof(sdrr_info_t));
    DEBUG("Copied sdrr_info to RAM at 0x%08X", (uint32_t)info);
    ptr += sizeof(sdrr_info_t);

    // Copy the pins and update sdrr_info which points to pins
    sdrr_pins_t *pins = (sdrr_pins_t *)ptr;
    memcpy(pins, sdrr_info.pins, sizeof(sdrr_pins_t));
    DEBUG("Copied sdrr_pins to RAM at 0x%08X", (uint32_t)pins);
    info->pins = pins;
    ptr += sizeof(sdrr_pins_t);

    // Copy the rom_set to RAM
    sdrr_rom_set_t *rom_set = (sdrr_rom_set_t *)ptr;
    memcpy(rom_set, set, sizeof(sdrr_rom_set_t));
    DEBUG("Copied sdrr_rom_set to RAM at 0x%08X", (uint32_t)rom_set);
    ptr += sizeof(sdrr_rom_set_t);
#endif // EXECUTE_FROM_RAM || XIP_CACHE_WARM

#if defined(XIP_CACHE_WARM)
    // Start and end of main_loop section in FLASH - these are variables
    // from the linker effectively located at these locations on flash, so we
    // need to use & to get the actual addresses.
    extern uint32_t _main_loop_start;
    //extern uint32_t _main_loop_end;

    // Get as addresses
    uint32_t main_loop_start_addr = (uint32_t)&_main_loop_start;
    uint32_t main_loop_end_addr = (uint32_t)&_main_loop_end;

    // Get offset from start of flash main_loop() is located at, and its
    // length
    uint32_t offset = main_loop_start_addr - FLASH_BASE;
    uint32_t length = main_loop_end_addr - main_loop_start_addr;

    // "Read" the main_loop so it gets loads into the cache 
    volatile uint32_t *code_ptr = (volatile uint32_t *)main_loop_start_addr;
    for (uint32_t ii = 0; ii < length; ii += 4) {
        volatile uint32_t dummy = code_ptr[ii/4];
        (void)dummy;
    }

    DEBUG("Warming 0x%08X bytes from 0x%08X, offset: 0x%08X", length, main_loop_start_addr, offset);

    LOG("Finished warming up main_loop %d bytes in XIP cache", length);
    // Execute the main_loop
#if !defined(MAIN_LOOP_LOGGING)
    LOG("Start main loop - logging ends");
#endif // !MAIN_LOOP_LOGGING
    main_loop(info, rom_set);
#endif // XIP_CACHE_WARM

#if defined(EXECUTE_FROM_RAM)
    // The main loop function was copied to RAM in the ResetHandler
    extern uint32_t _ram_func_start;
    void (*ram_func)(const sdrr_info_t *, const sdrr_rom_set_t *set) = (void(*)(const sdrr_info_t *, const sdrr_rom_set_t *set))((uint32_t)&_ram_func_start | 1);
    LOG("Executing main_loop from RAM at 0x%08X", (uint32_t)ram_func);
#if !defined(MAIN_LOOP_LOGGING)
    LOG("Start main loop - logging ends");
#endif // !MAIN_LOOP_LOGGING
    ram_func(info, rom_set);
#endif // EXECUTE_FROM_RAM

    ERR("Unreachable code reached - main_loop() returned or never executed");

    return 0;
}
