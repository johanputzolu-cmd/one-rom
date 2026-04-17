// Copyright (C) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT License
//
// RBCP (ROM Bus Control Protocol) device-side plugin for One ROM.
//
// Implements the full RBCP device role, supporting both Command mode and
// Command-Response mode as defined by the RBCP specification.
//
// Knock sequence: "!RBCP!" (6 bytes, matched against A0-A7 only).
//
// In Command mode each knock initiates a single-command session with no
// back-channel.  In Command-Response mode a session spans from the knock
// through ENTER_CMD_RESP to an explicit exit command, with all responses
// written into the configured back-channel region of the active RAM slot.

#include <stdint.h>
#include <stdbool.h>
#include "plugin.h"

// ---------------------------------------------------------------------------
// Plugin header
// ---------------------------------------------------------------------------

ORA_DEFINE_USER_PLUGIN(
    rbcp_main,
    0, 1, 0, 0,   // plugin version 0.1.0.0
    0, 6, 9       // minimum firmware version 0.6.9
);

// ---------------------------------------------------------------------------
// Ring buffer
// ---------------------------------------------------------------------------

#define RING_ENTRIES_LOG2  6u                               // 64 entries
#define RING_MASK          ((1u << RING_ENTRIES_LOG2) - 1u)

ORA_RING_BUF_DECLARE(ring_buf, RING_ENTRIES_LOG2);

// ---------------------------------------------------------------------------
// Knock sequence: "!RBCP!"
// ---------------------------------------------------------------------------

#define KNOCK_LEN  6u
static const uint32_t s_knock_seq[KNOCK_LEN] = {
    '!', 'R', 'B', 'C', 'P', '!'
};

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

#define RBCP_DEFAULT_COMPLETE   ((uint8_t)0xAAu)
#define RBCP_DEFAULT_STATUS_OK  ((uint8_t)0xCCu)

// Response header byte offsets within the back-channel region
#define HDR_LAST_CMD_GROUP  0u
#define HDR_LAST_CMD_CMD    1u
#define HDR_TOKEN_LSB       2u
#define HDR_TOKEN_MSB       3u
#define HDR_PROGRESS        4u
#define HDR_RESPONSE        5u
#define HDR_RESERVED_0      6u
#define HDR_RESERVED_1      7u
#define HDR_SIZE            8u

// Command groups
#define GRP_CONTROL  0x00u
#define GRP_READ     0x01u
#define GRP_MODIFY   0x02u

// Control commands
#define CMD_NOP                  0x00u
#define CMD_CONFIG_CMD_RESP      0x01u
#define CMD_ENTER_CMD_RESP       0x02u
#define CMD_EXIT_CMD_RESP_ACK    0x03u
#define CMD_EXIT_CMD_RESP_SILENT 0x04u
#define CMD_SWITCH_AND_EXIT      0x05u

// Read commands
#define CMD_GET_FLASH_SLOT_INFO  0x00u
#define CMD_GET_RAM_SLOT_INFO    0x01u

// Modify commands
#define CMD_SLOT_POKE    0x00u
#define CMD_SWITCH_SLOT  0x01u
#define CMD_LOAD_SLOT    0x02u

// Data section sizes in bytes, indexed by RBCP size_id.
// IDs 0x00-0x0D are protocol-defined; IDs >= 0x0E are reserved or
// implementation-specific and are rejected by this implementation.
static const uint32_t s_size_table[] = {
    0u, 8u, 16u, 32u, 64u, 128u, 256u, 512u,
    1024u, 2048u, 4096u, 8192u, 16384u, 32768u
};
#define SIZE_TABLE_LEN  ((uint8_t)(sizeof(s_size_table) / sizeof(s_size_table[0])))

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  location_id;   // location table index (0x00 or 0x01)
    uint8_t  size_id;       // size table index
    uint8_t  complete;      // protocol "complete" byte value
    uint8_t  status_ok;     // protocol "status-OK" byte value
    uint32_t slot_offset;   // offset of back-channel region within the RAM slot
    uint32_t data_size;     // data section size in bytes
} rbcp_cfg_t;

typedef struct {
    bool       active;       // true while in Command-Response mode
    rbcp_cfg_t cfg;
    uint8_t    token_lsb;
    uint8_t    token_msb;
    uint8_t    active_slot;  // RAM slot in which the back-channel is established;
                             // valid only while active is true.  Cached here so
                             // that back-channel writes always target the correct
                             // slot without repeated get_active_ram_slot() calls,
                             // and to ensure correctness if firmware slot-tracking
                             // state changes unexpectedly after session entry.
} rbcp_state_t;

static rbcp_state_t s_state;

// ---------------------------------------------------------------------------
// API function pointers (populated at plugin entry)
// ---------------------------------------------------------------------------

static ora_log_fn_t                         s_log;
static ora_demangle_addr_fn_t               s_demangle;
static ora_reprogram_ram_rom_slot_fn_t      s_reprogram;
static ora_get_ram_slot_info_fn_t           s_get_ram_slot_info;
static ora_get_ram_slot_count_fn_t          s_get_ram_slot_count;
static ora_get_active_ram_slot_fn_t         s_get_active_ram_slot;
static ora_set_active_ram_slot_fn_t         s_set_active_ram_slot;
static ora_get_flash_slot_count_fn_t        s_get_flash_slot_count;
static ora_get_flash_slot_info_fn_t         s_get_flash_slot_info;
static ora_copy_flash_slot_to_ram_slot_fn_t s_copy_flash_to_ram;

// ---------------------------------------------------------------------------
// Ring buffer read helpers
// ---------------------------------------------------------------------------

static volatile uint32_t * volatile *s_write_pos_ptr; // pointer to DMA write pointer (volatile: DMA hardware updates the register)
static uint32_t            s_read_idx;      // our read index, kept masked

// Advance our read index to the current DMA write position, discarding any
// buffered entries.  Called immediately after wait_for_knock() returns so
// that ring_read_byte() begins consuming from the correct position.  Placed
// at that point — before the demangle calls on the preamble — to prevent
// argument bytes arriving in the demangle window from being silently lost.
// Not called on the wait_for_knock() error path because wait_for_knock()
// itself re-snapshots the DMA write position on each entry.
static void ring_sync(void) {
    s_read_idx = (uint32_t)(*s_write_pos_ptr - ring_buf) & RING_MASK;
}

// Block until the next CS-active address capture is available, then return
// its A0-A7 as a logical byte.  Entries where CS is inactive are skipped.
//
// WARNING: This function blocks indefinitely.  If the host resets or crashes
// mid-command while the plugin is waiting for an argument byte, this function
// will never return and recovery requires a device power cycle.  A future API
// extension providing a timeout or cancellation mechanism would allow a
// cleaner recovery path.
static uint8_t ring_read_byte(void) {
    for (;;) {
        uint32_t write_idx = (uint32_t)(*s_write_pos_ptr - ring_buf) & RING_MASK;
        if (s_read_idx == write_idx) {
            continue;
        }
        uint32_t phys = ring_buf[s_read_idx];
        s_read_idx = (s_read_idx + 1u) & RING_MASK;

        uint32_t logical;
        if (s_demangle(phys, &logical, 1) == ORA_RESULT_OK) {
            return (uint8_t)(logical & 0xFFu);
        }
        // CS inactive or demangle error: discard and try next entry
    }
}

// ---------------------------------------------------------------------------
// Back-channel write helpers
// ---------------------------------------------------------------------------

static inline uint8_t pending_val(void) {
    return (uint8_t)(~s_state.cfg.complete);
}

static inline uint8_t failed_val(void) {
    return (uint8_t)(~s_state.cfg.status_ok);
}

// Write one byte into the response header at the given header-relative offset.
static void hdr_write(uint8_t slot, uint32_t hdr_offset, uint8_t val) {
    if (s_reprogram(slot, s_state.cfg.slot_offset + hdr_offset,
                    &val, 1u, 1u) != ORA_RESULT_OK) {
        s_log("RBCP: hdr_write failed at offset %u", (unsigned)hdr_offset);
        // No recovery is possible: if the back-channel write fails, the host
        // will poll indefinitely.  Logging is the only meaningful action.
    }
}

// Write bytes into the data section at the given data-section-relative offset.
// Writes are clamped to the configured data section size.
static void data_write(uint8_t slot, uint32_t data_offset,
                       const uint8_t *buf, uint32_t len) {
    if (data_offset >= s_state.cfg.data_size) return;
    if (data_offset + len > s_state.cfg.data_size) {
        len = s_state.cfg.data_size - data_offset;
    }
    if (s_reprogram(slot,
                    s_state.cfg.slot_offset + HDR_SIZE + data_offset,
                    buf, len, 1u) != ORA_RESULT_OK) {
        s_log("RBCP: data_write failed at data offset %u", (unsigned)data_offset);
    }
}

// ---------------------------------------------------------------------------
// Command processing sequence helpers
// ---------------------------------------------------------------------------

// Steps 1-3: set progress=pending, increment token, update last_cmd.
static void cmd_begin(uint8_t slot, uint8_t group, uint8_t cmd) {
    hdr_write(slot, HDR_PROGRESS, pending_val());
    s_state.token_lsb++;
    if (s_state.token_lsb == 0u) s_state.token_msb++;
    hdr_write(slot, HDR_TOKEN_LSB, s_state.token_lsb);
    hdr_write(slot, HDR_TOKEN_MSB, s_state.token_msb);
    hdr_write(slot, HDR_LAST_CMD_GROUP, group);
    hdr_write(slot, HDR_LAST_CMD_CMD, cmd);
}

// Steps 5-6: write response field then set progress=complete.
static void cmd_end(uint8_t slot, bool ok) {
    hdr_write(slot, HDR_RESPONSE, ok ? s_state.cfg.status_ok : failed_val());
    hdr_write(slot, HDR_PROGRESS, s_state.cfg.complete);
}

// ---------------------------------------------------------------------------
// Back-channel region setup
// ---------------------------------------------------------------------------

// Compute the slot byte offset of the back-channel region from the location
// table index and size table index.
static ora_result_t compute_region_offset(uint8_t location_id, uint8_t size_id,
                                           uint8_t active_slot,
                                           uint32_t *offset_out) {
    if (location_id == 0x00u) {
        *offset_out = 0u;
        return ORA_RESULT_OK;
    }
    if (location_id == 0x01u) {
        uint32_t addr, slot_size;
        ora_result_t r = s_get_ram_slot_info(active_slot, &addr, &slot_size, NULL);
        if (r != ORA_RESULT_OK) return r;
        uint32_t data_size = (size_id < SIZE_TABLE_LEN) ? s_size_table[size_id] : 0u;
        *offset_out = slot_size - (HDR_SIZE + data_size);
        return ORA_RESULT_OK;
    }
    // location IDs 0x02-0x7F reserved; 0x80-0xFF implementation-specific:
    // not supported by this implementation.
    return ORA_RESULT_INVALID_ARG;
}

// Zero-initialise the response header in the back-channel region.
static void init_back_channel(uint8_t slot) {
    static const uint8_t zeros[HDR_SIZE] = {0u};
    if (s_reprogram(slot, s_state.cfg.slot_offset,
                    zeros, HDR_SIZE, 1u) != ORA_RESULT_OK) {
        s_log("RBCP: init_back_channel failed for slot %u", (unsigned)slot);
    }
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

// Zero n bytes starting at p.  Uses a volatile pointer so the compiler does
// not recognise the loop as a memset pattern and emit a library call —
// there is no C runtime available in the plugin environment.
static void zero_bytes(uint8_t *p, uint8_t n) {
    volatile uint8_t *vp = p;
    while (n--) *vp++ = 0u;
}

// ---------------------------------------------------------------------------
// Command handlers — Control group (0x00)
// ---------------------------------------------------------------------------

static bool exec_nop(void) {
    return true;
}

// Configures command-response mode parameters.  May be called in either
// command or command-response mode.
//
// Note: slot_offset is NOT recomputed here even if called mid-session.
// New location and size parameters take effect only at the next
// ENTER_CMD_RESP.  Recomputing slot_offset mid-session would invalidate the
// host's current polling location, which is more dangerous than deferring.
static bool exec_config_cmd_resp(void) {
    uint8_t location  = ring_read_byte();
    uint8_t size_id   = ring_read_byte();
    uint8_t complete  = ring_read_byte();
    uint8_t status_ok = ring_read_byte();

    if (size_id >= SIZE_TABLE_LEN) return false;
    if (location > 0x01u)          return false;

    s_state.cfg.location_id = location;
    s_state.cfg.size_id     = size_id;
    s_state.cfg.complete    = complete;
    s_state.cfg.status_ok   = status_ok;
    s_state.cfg.data_size   = s_size_table[size_id];
    return true;
}

// Transitions into Command-Response mode.  Queries and caches the active RAM
// slot, initialises the back-channel region, and resets the token.  If
// already active, succeeds immediately (idempotent).
//
// The active slot is cached in s_state.active_slot for the lifetime of the
// session.  All subsequent back-channel writes use the cached value, removing
// the need for repeated get_active_ram_slot() calls and ensuring a valid slot
// is always available.  The cache is updated by CMD_SWITCH_SLOT if the host
// switches slot mid-session; CMD_SWITCH_AND_EXIT does not update it as that
// command exits silently with no writes to the new slot.
static bool exec_enter_cmd_resp(void) {
    if (s_state.active) return true;

    uint8_t active_slot;
    if (s_get_active_ram_slot(&active_slot) != ORA_RESULT_OK) {
        s_log("RBCP: ENTER_CMD_RESP failed: no active slot");
        return false;
    }

    uint32_t offset;
    if (compute_region_offset(s_state.cfg.location_id, s_state.cfg.size_id,
                              active_slot, &offset) != ORA_RESULT_OK) {
        return false;
    }
    s_state.cfg.slot_offset = offset;
    s_state.token_lsb       = 0u;
    s_state.token_msb       = 0u;
    s_state.active_slot     = active_slot;
    init_back_channel(active_slot);
    s_state.active = true;
    return true;
}

// ---------------------------------------------------------------------------
// Command handlers — Read group (0x01)
// ---------------------------------------------------------------------------

static bool exec_get_flash_slot_info(uint8_t slot) {
    uint8_t total = s_get_flash_slot_count(ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS);

    // If the data section can't hold even the 4-byte preamble, write what fits.
    if (s_state.cfg.data_size < 4u) {
        uint8_t preamble[4] = { total, 0u, 0u, 0u };
        data_write(slot, 0u, preamble, s_state.cfg.data_size);
        return true;
    }

    uint32_t space       = s_state.cfg.data_size - 4u;
    uint8_t  whole_count = (uint8_t)(space / 32u);
    if (whole_count > total) whole_count = total;

    // A partial record follows complete records if there are more slots to
    // report and at least one byte of space remains after the whole records.
    uint32_t partial_bytes = space - ((uint32_t)whole_count * 32u);
    uint8_t  partial_flag  = (whole_count < total && partial_bytes > 0u) ? 0x01u : 0x00u;

    uint8_t preamble[4] = { total, whole_count, partial_flag, 0u };
    data_write(slot, 0u, preamble, 4u);

    uint32_t data_off      = 4u;
    uint8_t  slots_to_emit = whole_count + (partial_flag ? 1u : 0u);

    for (uint8_t i = 0u; i < slots_to_emit; i++) {
        const char *name     = NULL;
        uint32_t    rom_type = 0xFFu;
        s_get_flash_slot_info(i, ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS,
                              &name, &rom_type, NULL);

        // Build a 32-byte record: 1 byte rom_type, 31 bytes name (zero-padded).
        uint8_t record[32];
        record[0] = (uint8_t)(rom_type & 0xFFu);
        zero_bytes(&record[1], 31u);
        if (name != NULL) {
            uint8_t nlen = 0u;
            while (nlen < 31u && name[nlen] != '\0') nlen++;
            for (uint8_t j = 0u; j < nlen; j++) record[1u + j] = (uint8_t)name[j];
        }

        // Whole records are 32 bytes; the trailing partial record is however
        // many bytes remain in the data section.
        uint32_t bytes = (i < whole_count) ? 32u : partial_bytes;
        data_write(slot, data_off, record, bytes);
        data_off += bytes;
    }

    return true;
}

static bool exec_get_ram_slot_info(uint8_t slot) {
    uint8_t  total    = s_get_ram_slot_count();
    uint32_t rom_type = 0xFFu;

    // slot is s_state.active_slot — both the back-channel destination and
    // the index of the RAM slot currently being served.  Query its ROM type
    // via the extended get_ram_slot_info API.
    s_get_ram_slot_info(slot, NULL, NULL, &rom_type);

    uint8_t resp[4] = { total, slot, (uint8_t)(rom_type & 0xFFu), 0u };
    data_write(slot, 0u, resp, 4u);
    return true;
}

// ---------------------------------------------------------------------------
// Command handlers — Modify group (0x02)
// ---------------------------------------------------------------------------

static bool exec_slot_poke(void) {
    uint8_t target = ring_read_byte();
    uint8_t a0     = ring_read_byte();
    uint8_t a1     = ring_read_byte();
    uint8_t a2     = ring_read_byte();
    uint8_t byte   = ring_read_byte();

    uint32_t addr = (uint32_t)a0
                  | ((uint32_t)a1 << 8u)
                  | ((uint32_t)a2 << 16u);
    return (s_reprogram(target, addr, &byte, 1u, 1u) == ORA_RESULT_OK);
}

static bool exec_switch_slot(void) {
    uint8_t target = ring_read_byte();
    return (s_set_active_ram_slot(target) == ORA_RESULT_OK);
}

static bool exec_load_slot(void) {
    uint8_t ram_slot   = ring_read_byte();
    uint8_t flash_slot = ring_read_byte();
    return (s_copy_flash_to_ram(flash_slot,
                                ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS,
                                ram_slot, 0u) == ORA_RESULT_OK);
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

// Dispatch one command.  Reads argument bytes from the ring buffer.
// Uses s_state.active_slot for all back-channel writes.
//
// exit_silent_out: set to true for commands that must not trigger cmd_end
//                  (EXIT_CMD_RESP_SILENT, SWITCH_AND_EXIT).
// terminate_out:   set to true when Command-Response mode should end after
//                  this command.
//
// Returns the ok/fail result for cmd_end.
static bool dispatch(uint8_t group, uint8_t cmd,
                     bool *exit_silent_out, bool *terminate_out) {
    *exit_silent_out = false;
    *terminate_out   = false;
    bool ok = false;

    switch (group) {

    case GRP_CONTROL:
        switch (cmd) {
        case CMD_NOP:
            ok = exec_nop();
            break;
        case CMD_CONFIG_CMD_RESP:
            ok = exec_config_cmd_resp();
            break;
        case CMD_ENTER_CMD_RESP:
            ok = exec_enter_cmd_resp();
            break;
        case CMD_EXIT_CMD_RESP_ACK:
            // The device completes the full command processing sequence
            // (including progress=complete) before exiting.  cmd_end is
            // called by the caller; s_state.active is cleared here so that
            // run_command knows to terminate the inner loop.
            s_state.active = false;
            *terminate_out = true;
            ok = true;
            break;
        case CMD_EXIT_CMD_RESP_SILENT:
            s_state.active   = false;
            *exit_silent_out = true;
            *terminate_out   = true;
            ok = true;
            break;
        case CMD_SWITCH_AND_EXIT:
            // Activate specified slot then exit silently.
            // active_slot cache is NOT updated: this command exits with no
            // back-channel writes to the new slot, so the cached value is
            // irrelevant for the remainder of the session.
            ok               = exec_switch_slot();
            s_state.active   = false;
            *exit_silent_out = true;
            *terminate_out   = true;
            break;
        default:
            // Unknown command: no args consumed.  This will desync the
            // session; the best the host can do is re-knock.
            ok = false;
            break;
        }
        break;

    case GRP_READ:
        switch (cmd) {
        case CMD_GET_FLASH_SLOT_INFO:
            ok = exec_get_flash_slot_info(s_state.active_slot);
            break;
        case CMD_GET_RAM_SLOT_INFO:
            ok = exec_get_ram_slot_info(s_state.active_slot);
            break;
        default:
            ok = false;
            break;
        }
        break;

    case GRP_MODIFY:
        switch (cmd) {
        case CMD_SLOT_POKE:
            ok = exec_slot_poke();
            break;
        case CMD_SWITCH_SLOT:
            ok = exec_switch_slot();
            if (ok) {
                // Update the cached active slot so subsequent back-channel
                // writes in this session target the new slot's SRAM region.
                // get_active_ram_slot() should not fail here since
                // set_active_ram_slot() just succeeded.
                uint8_t new_slot;
                if (s_get_active_ram_slot(&new_slot) == ORA_RESULT_OK) {
                    s_state.active_slot = new_slot;
                } else {
                    s_log("RBCP: active slot desync after SWITCH_SLOT");
                }
            }
            break;
        case CMD_LOAD_SLOT:
            ok = exec_load_slot();
            break;
        default:
            ok = false;
            break;
        }
        break;

    default:
        ok = false;
        break;
    }

    return ok;
}

// ---------------------------------------------------------------------------
// Session execution
// ---------------------------------------------------------------------------

// Execute one command.  Handles the full command processing sequence:
// cmd_begin -> dispatch -> cmd_end, in accordance with the RBCP spec.
//
// All back-channel writes use s_state.active_slot, which is populated at
// ENTER_CMD_RESP and updated by CMD_SWITCH_SLOT.
//
// Special case: ENTER_CMD_RESP arrives while the device is in Command mode
// (was_active=false) but transitions to Command-Response mode.  In this case
// cmd_begin and cmd_end are called after the transition so that the host can
// poll the back-channel to confirm entry.
//
// Returns true if Command-Response mode is active after this command, meaning
// the caller should continue reading commands from the ring buffer rather than
// waiting for the next knock.
static bool run_command(uint8_t group, uint8_t cmd) {
    bool was_active = s_state.active;
    bool exit_silent, terminate;

    if (was_active) {
        cmd_begin(s_state.active_slot, group, cmd);
    }

    bool ok = dispatch(group, cmd, &exit_silent, &terminate);

    bool now_active = s_state.active;

    if (was_active && !exit_silent) {
        // Normal Command-Response mode: complete the processing sequence.
        // s_state.active_slot may have been updated by CMD_SWITCH_SLOT
        // inside dispatch, so use the current cached value.
        cmd_end(s_state.active_slot, ok);
    } else if (!was_active && now_active) {
        // ENTER_CMD_RESP: the device has just transitioned into
        // Command-Response mode.  s_state.active_slot is now valid.
        // Write the initial response so the host can confirm entry by
        // polling the token and progress fields.
        cmd_begin(s_state.active_slot, group, cmd);
        cmd_end(s_state.active_slot, ok);
    }
    // Command mode (!was_active && !now_active): no back-channel, nothing to write.
    // EXIT_CMD_RESP_SILENT / SWITCH_AND_EXIT (exit_silent=true): no header update.

    return now_active;
}

// ---------------------------------------------------------------------------
// Plugin entry point
// ---------------------------------------------------------------------------

void rbcp_main(
    ora_lookup_fn_t         ora_lookup_fn,
    ora_plugin_type_t       plugin_type,
    const ora_entry_args_t *entry_args
) {
    (void)plugin_type;
    (void)entry_args;

    // Retrieve API function pointers
    s_log                  = ora_lookup_fn(ORA_ID_LOG);
    s_demangle             = ora_lookup_fn(ORA_ID_DEMANGLE_ADDR);
    s_reprogram            = ora_lookup_fn(ORA_ID_REPROGRAM_RAM_ROM_SLOT);
    s_get_ram_slot_info    = ora_lookup_fn(ORA_ID_GET_RAM_SLOT_INFO);
    s_get_ram_slot_count   = ora_lookup_fn(ORA_ID_GET_RAM_SLOT_COUNT);
    s_get_active_ram_slot  = ora_lookup_fn(ORA_ID_GET_ACTIVE_RAM_SLOT);
    s_set_active_ram_slot  = ora_lookup_fn(ORA_ID_SET_ACTIVE_RAM_SLOT);
    s_get_flash_slot_count = ora_lookup_fn(ORA_ID_GET_FLASH_SLOT_COUNT);
    s_get_flash_slot_info  = ora_lookup_fn(ORA_ID_GET_FLASH_SLOT_INFO);
    s_copy_flash_to_ram    = ora_lookup_fn(ORA_ID_COPY_FLASH_SLOT_TO_RAM_SLOT);

    ora_setup_address_monitor_fn_t              setup_address_monitor =
        ora_lookup_fn(ORA_ID_SETUP_ADDRESS_MONITOR);
    ora_start_address_monitor_fn_t              start_address_monitor =
        ora_lookup_fn(ORA_ID_START_ADDRESS_MONITOR);
    ora_get_address_monitor_ring_write_pos_fn_t get_write_pos =
        ora_lookup_fn(ORA_ID_GET_ADDRESS_MONITOR_RING_WRITE_POS);
    ora_init_knock_fn_t     init_knock     = ora_lookup_fn(ORA_ID_INIT_KNOCK);
    ora_wait_for_knock_fn_t wait_for_knock = ora_lookup_fn(ORA_ID_WAIT_FOR_KNOCK);

    s_log("RBCP plugin starting");

    // Initialise device state with protocol defaults
    s_state.active              = false;
    s_state.active_slot         = 0u;
    s_state.cfg.location_id     = 0x00u;
    s_state.cfg.size_id         = 0x00u;
    s_state.cfg.complete        = RBCP_DEFAULT_COMPLETE;
    s_state.cfg.status_ok       = RBCP_DEFAULT_STATUS_OK;
    s_state.cfg.slot_offset     = 0u;
    s_state.cfg.data_size       = 0u;
    s_state.token_lsb           = 0u;
    s_state.token_msb           = 0u;

    // Set up address monitor in control mode so the plugin can modify the
    // ROM image being served (required for back-channel writes).
    if (setup_address_monitor(ring_buf, RING_ENTRIES_LOG2,
                              ORA_MONITOR_MODE_CONTROL, NULL) != ORA_RESULT_OK) {
        s_log("RBCP: address monitor setup failed");
        return;
    }

    // Retrieve and cache the DMA write pointer location.  This is called once
    // at init time; the returned pointer is valid for the lifetime of the monitor.
    s_write_pos_ptr = get_write_pos();
    if (s_write_pos_ptr == NULL) {
        s_log("RBCP: failed to get ring buffer write position");
        return;
    }

    // Pre-compute knock sequence match values before starting the monitor.
    ORA_KNOCK_DECLARE(knock, KNOCK_LEN);
    if (init_knock(s_knock_seq, KNOCK_LEN, 8u, knock) != ORA_RESULT_OK) {
        s_log("RBCP: knock init failed");
        return;
    }

    // Begin capturing address bus activity.
    start_address_monitor();

    s_log("RBCP: ready, awaiting knock");

    // Main loop — each outer iteration begins with a knock.
    for (;;) {
        // Collect GROUP and CMD as the two payload bytes immediately following
        // the knock.  wait_for_knock blocks until both are captured in the
        // ring buffer.  Payload entries are raw physical GPIO captures.
        uint32_t preamble[2];
        if (wait_for_knock(knock, ring_buf, RING_ENTRIES_LOG2,
                           ORA_WAIT_FOR_KNOCK_FLAG_DEBOUNCE_CS,
                           preamble, 2u) != ORA_RESULT_OK) {
            continue;
        }

        // Sync our ring read index to the current DMA write position
        // immediately after the knock, before any other processing.  Argument
        // bytes for this command arrive as new DMA entries after this point,
        // and ring_read_byte() will block until each one is available.
        // Placed here — before the demangle calls below — so that no argument
        // bytes can be lost in the window between wait_for_knock() returning
        // and ring_sync() executing.  Not called on the continue path above
        // because wait_for_knock() re-snapshots the DMA write position on
        // each entry.
        ring_sync();

        // Demangle GROUP and CMD from physical captures to logical addresses.
        // ORA_WAIT_FOR_KNOCK_FLAG_DEBOUNCE_CS guarantees payload entries are
        // CS-active, so check_control_pins=0 is correct here.
        uint32_t logical;
        if (s_demangle(preamble[0], &logical, 0) != ORA_RESULT_OK) continue;
        uint8_t group = (uint8_t)(logical & 0xFFu);

        if (s_demangle(preamble[1], &logical, 0) != ORA_RESULT_OK) continue;
        uint8_t cmd = (uint8_t)(logical & 0xFFu);

        // Execute the command.  In Command mode the session ends here and we
        // return to the top of the outer loop to await the next knock.
        // ENTER_CMD_RESP transitions into Command-Response mode, in which
        // case run_command returns true and we enter the inner loop.
        if (!run_command(group, cmd)) {
            continue;
        }

        // Command-Response mode inner loop: read commands directly from the
        // ring buffer without a knock between them.
        while (s_state.active) {
            group = ring_read_byte();
            cmd   = ring_read_byte();
            run_command(group, cmd);
        }

        // Fell out of Command-Response mode (EXIT or SWITCH_AND_EXIT).
        // Return to outer loop to await the next knock.
    }
}