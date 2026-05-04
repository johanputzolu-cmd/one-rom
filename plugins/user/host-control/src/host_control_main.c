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
//
// To change between 8, 16 and 32 bit ring buffer entries change:
// - RING_DATA_SIZE
// - RING_BUF_TYPE
// - ORA_RING_BUF_DECLARE_*BIT to match RING_DATA_SIZE
// ---------------------------------------------------------------------------

#define RING_ENTRIES_LOG2   6u                               // 64 entries
#define RING_DATA_SIZE      32u                              // 32 bits per entry
#define RING_MASK           ((1u << RING_ENTRIES_LOG2) - 1u)
#define RING_BUF_TYPE       uint32_t                         // 32 bit entries
_Static_assert(sizeof(RING_BUF_TYPE) * 8 == RING_DATA_SIZE, "RING_BUF_TYPE must match RING_DATA_SIZE");

ORA_RING_BUF_DECLARE_32BIT(ring_buf, RING_ENTRIES_LOG2);

#define RING_BUF_CUR_READ_INDEX()   s_read_idx
#define RING_BUF_ADV_READ_INDEX()   s_read_idx = (s_read_idx + 1u) & RING_MASK
#define RING_BUF_UPDATE_READ_INDEX(X) \
    s_read_idx = (uint32_t)((volatile RING_BUF_TYPE *)(X) - \
                            (volatile RING_BUF_TYPE *)ring_buf) & RING_MASK
#define RING_BUF_RESET_READ_INDEX() RING_BUF_UPDATE_READ_INDEX(*s_write_pos_ptr)
#define RING_BUF_CUR_WRITE_INDEX() \
    ((uint32_t)((volatile RING_BUF_TYPE *)*s_write_pos_ptr - \
                (volatile RING_BUF_TYPE *)ring_buf) & RING_MASK)
#define RING_BUF_GET_ENTRY(X) ((volatile RING_BUF_TYPE *)ring_buf)[(X)]

// Statics used to track ring buffer state.
static volatile uint32_t * volatile *s_write_pos_ptr; // pointer to DMA write pointer (volatile: DMA hardware updates the register)
static uint32_t            s_read_idx;      // our read index, kept masked

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

#define RBCP_PROTOCOL_VERSION_MAJOR 0u
#define RBCP_PROTOCOL_VERSION_MINOR 1u
#define RBCP_PROTOCOL_VERSION_PATCH 0u
const uint8_t protocol_version[4] = {
    RBCP_PROTOCOL_VERSION_MAJOR,
    RBCP_PROTOCOL_VERSION_MINOR,
    RBCP_PROTOCOL_VERSION_PATCH,
    0u
};

#define RBCP_DEFAULT_COMPLETE   ((uint8_t)0xBBu)
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
#define GRP_RESET    0xAAu

// Control commands
#define CMD_NOP                         0x00u
#define CMD_ENTER_CMD_RESP              0x01u
#define CMD_EXIT_CMD_RESP_ACK           0x02u
#define CMD_EXIT_CMD_RESP_SILENT        0x03u
#define CMD_SWITCH_AND_EXIT             0x04u

// Read commands
#define CMD_GET_FLASH_FLASH_SLOT_COUNT  0x00u
#define CMD_GET_FLASH_SLOT_INFO         0x01u
#define CMD_GET_FLASH_SLOT_INFO_ALL     0x02u
#define CMD_GET_RAM_SLOT_INFO_ALL       0x03u
#define CMD_GET_DEVICE_TYPE             0x04u
#define CMD_GET_DEVICE_VERSION          0x05u
#define CMD_GET_PROTOCOL_VERSION        0x06u
#define CMD_SLOT_PEEK                   0x07u

// Modify commands
#define CMD_SLOT_POKE                   0x00u
#define CMD_SWITCH_SLOT                 0x01u
#define CMD_LOAD_SLOT                   0x02u
#define CMD_SLOT_POKE_ALL_BYTE          0x03u

// Reset commands
#define CMD_RBCP_RESET                  0xAAu

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t command_page;  // command page filter during command-response mode
    uint8_t  complete;      // protocol "complete" byte value
    uint8_t  status_ok;     // protocol "status-OK" byte value
    uint32_t region_offset; // offset of back-channel region within the RAM slot
    uint32_t region_end;    // end address (exclusive) of the back-channel region within the RAM slot;
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
static ora_get_chip_size_from_type_fn_t     s_get_chip_size;
static ora_get_device_version_fn_t          s_get_device_version;
static ora_map_addr_to_phys_fn_t            s_map_addr_to_phys;
static ora_map_data_to_phys_fn_t            s_map_data_to_phys;
static ora_demangle_data_fn_t               s_demangle_data;

// ---------------------------------------------------------------------------
// Ring buffer read helpers
// ---------------------------------------------------------------------------

// Block until the next CS-active address capture is available, then return
// its A0-A7 as a logical byte.  Entries where CS is inactive are skipped.
// In command-response mode, entries whose upper address bits do not match
// the configured command page are also skipped.
//
// WARNING: This function blocks indefinitely.  If the host resets or crashes
// mid-command while the plugin is waiting for an argument byte, this function
// will never return and recovery requires a device power cycle.  A future API
// extension providing a timeout or cancellation mechanism would allow a
// cleaner recovery path.
static uint8_t ring_read_byte(void) {
    for (;;) {
        if (RING_BUF_CUR_READ_INDEX() == RING_BUF_CUR_WRITE_INDEX()) {
            // No new byte to read, yet.
            continue;
        }
        uint32_t phys = (uint32_t)RING_BUF_GET_ENTRY(s_read_idx);
        RING_BUF_ADV_READ_INDEX();

        uint32_t logical;
        if (s_demangle(phys, &logical, 1) == ORA_RESULT_OK) {
            if (s_state.active &&
                ((logical >> 8u) != (uint32_t)s_state.cfg.command_page)) {
                // Ignore accesses that do not match the command page.
                continue;
            }
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
// These reset_ring arg is intended to be used when update progress->complete,
// to ensure we collect as few new bytes as possible after the host potentially
// sees completion, before we can start processing them. 
static void hdr_write(uint8_t slot, uint32_t hdr_offset, uint8_t val, bool reset_ring) {
    // Do this directly, rather than s_reprogram, for speed
    uint32_t phys_addr = s_map_addr_to_phys(s_state.cfg.region_offset + hdr_offset);
    uint8_t phys_data = s_map_data_to_phys(val);
    uint32_t slot_base, slot_size;
    if (s_get_ram_slot_info(slot, &slot_base, &slot_size, NULL) != ORA_RESULT_OK) {
        s_log("RBCP: hdr_write failed: get_ram_slot_info error");
        return;
    }
    if (phys_addr >= slot_size) {
        s_log("RBCP: hdr_write failed: phys_addr out of bounds (hdr_offset=%u)", (unsigned)hdr_offset);
        return;
    }
    // Reset ring read pointer immediately before writing the byte
    if (reset_ring) {
        RING_BUF_RESET_READ_INDEX();
    }
    ((volatile uint8_t *)slot_base)[phys_addr] = phys_data;
}

// Read one byte from the back-channel region at the given header-relative offset.
static ora_result_t hdr_read(uint8_t slot, uint32_t hdr_offset, uint8_t *val_out) {
    uint32_t slot_base, slot_size;
    if (s_get_ram_slot_info(slot, &slot_base, &slot_size, NULL) != ORA_RESULT_OK) {
        s_log("RBCP: hdr_read failed: get_ram_slot_info error");
        return ORA_RESULT_INVALID_SLOT;
    }
    uint32_t phys_offset = s_map_addr_to_phys(s_state.cfg.region_offset + hdr_offset);
    uint8_t raw = ((const uint8_t *)slot_base)[phys_offset];
    if (s_demangle_data(raw, val_out) != ORA_RESULT_OK) {
        s_log("RBCP: hdr_read failed: demangle error at hdr_offset %u", (unsigned)hdr_offset);
        return ORA_RESULT_ERROR;
    }
    return ORA_RESULT_OK;
}

// Write bytes into the data section at the given data-section-relative offset.
// Writes are clamped to the configured data section size.
static void data_write(
    uint8_t slot,
    uint32_t data_offset,
    const uint8_t *buf,
    uint32_t len
) {
    if (data_offset >= s_state.cfg.data_size) return;
    if (data_offset + len > s_state.cfg.data_size) {
        len = s_state.cfg.data_size - data_offset;
    }
    if (s_reprogram(slot,
                    s_state.cfg.region_offset + HDR_SIZE + data_offset,
                    buf, len, 1u) != ORA_RESULT_OK) {
        s_log("RBCP: data_write failed at data offset %u", (unsigned)data_offset);
    }
}

// ---------------------------------------------------------------------------
// Command processing sequence helpers
// ---------------------------------------------------------------------------

// Steps 1-3: set progress=pending, increment token, update last_cmd.
static void cmd_begin(uint8_t slot, uint8_t group, uint8_t cmd) {
    hdr_write(slot, HDR_PROGRESS, pending_val(), false);
    s_state.token_lsb++;
    if (s_state.token_lsb == 0u) s_state.token_msb++;
    hdr_write(slot, HDR_TOKEN_LSB, s_state.token_lsb, false);
    hdr_write(slot, HDR_TOKEN_MSB, s_state.token_msb, false);
    hdr_write(slot, HDR_LAST_CMD_GROUP, group, false);
    hdr_write(slot, HDR_LAST_CMD_CMD, cmd, false);
}

// Steps 5-6: write response field then set progress=complete.
static void cmd_end(uint8_t slot, bool ok) {
    // First update the status byte.
    hdr_write(slot, HDR_RESPONSE, ok ? s_state.cfg.status_ok : failed_val(), false);

    // Now, set progress to complete, which must be the last step.
    hdr_write(slot, HDR_PROGRESS, s_state.cfg.complete, true);
}

// ---------------------------------------------------------------------------
// Back-channel region setup
// ---------------------------------------------------------------------------

// Zero-initialise the response header in the back-channel region.
static void init_back_channel(uint8_t slot) {
    static const uint8_t zeros[HDR_SIZE] = {0u};
    if (s_reprogram(slot, s_state.cfg.region_offset,
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

static void init_rbcp(bool reset_slot_info) {
    // Initialise device state with protocol defaults
    s_state.active              = false;
    s_state.active_slot         = 0u;
    s_state.cfg.command_page    = 0u;
    s_state.cfg.complete        = RBCP_DEFAULT_COMPLETE;
    s_state.cfg.status_ok       = RBCP_DEFAULT_STATUS_OK;
    s_state.cfg.region_offset   = 0u;
    s_state.cfg.region_end      = 0u;
    s_state.cfg.data_size       = 0u;
    s_state.token_lsb           = 0u;
    s_state.token_msb           = 0u;
    s_read_idx                  = 0u;
    if (reset_slot_info) {
        s_state.active_slot         = 0u;
    }
}

// ---------------------------------------------------------------------------
// Command handlers — Control group (0x00)
// ---------------------------------------------------------------------------

static bool exec_nop(void) {
    return true;
}

// Configures command-response mode parameters and enters command-response mode.
// Reads 9 argument bytes.  Validates all arguments before making any state
// changes.  Silent-discard conditions (as defined by the spec) are logged but
// produce no back-channel response.  Failure conditions are also logged; since
// ENTER_CMD_RESP is only valid in command mode, there is no back-channel for
// the device to write a failure response to — both outcomes are invisible to
// the host, which detects failure by the token not incrementing.
static bool exec_enter_cmd_resp(void) {
    uint8_t cp_lo     = ring_read_byte();  // A0: command page LSB
    uint8_t cp_hi     = ring_read_byte();  // A1: command page MSB
    uint8_t bc_a0     = ring_read_byte();  // A2: back-channel start address byte 0
    uint8_t bc_a1     = ring_read_byte();  // A3: back-channel start address byte 1
    uint8_t bc_a2     = ring_read_byte();  // A4: back-channel start address byte 2
    uint8_t bc_sz_lo  = ring_read_byte();  // A5: back-channel size LSB
    uint8_t bc_sz_hi  = ring_read_byte();  // A6: back-channel size MSB
    uint8_t complete  = ring_read_byte(); // A7
    uint8_t status_ok = ring_read_byte(); // A8

    uint16_t command_page  = (uint16_t)cp_lo | ((uint16_t)cp_hi << 8u);
    uint32_t region_offset = (uint32_t)bc_a0
                           | ((uint32_t)bc_a1 << 8u)
                           | ((uint32_t)bc_a2 << 16u);
    uint16_t region_size   = (uint16_t)bc_sz_lo | ((uint16_t)bc_sz_hi << 8u);

    if (s_state.active) {
        s_log("ENTER_CMD_RESP failed: already in command-response mode");
        return false;
    }
    if (complete == 0xAAu || status_ok == 0xAAu) {
        s_log("ENTER_CMD_RESP discarded: complete or status_ok is 0xAA");
        return false;
    }
    if (region_offset & 0x3u) {
        s_log("ENTER_CMD_RESP discarded: back-channel start address not 4-byte aligned");
        return false;
    }
    if ((uint32_t)region_size < HDR_SIZE) {
        s_log("ENTER_CMD_RESP failed: back-channel size too small to hold header");
        return false;
    }

    uint8_t active_slot;
    if (s_get_active_ram_slot(&active_slot) != ORA_RESULT_OK) {
        s_log("ENTER_CMD_RESP failed: no active slot");
        return false;
    }
    uint32_t slot_size;
    if (s_get_ram_slot_info(active_slot, NULL, &slot_size, NULL) != ORA_RESULT_OK) {
        s_log("ENTER_CMD_RESP failed: get_ram_slot_info error");
        return false;
    }
    if (((uint32_t)command_page << 8u) >= slot_size) {
        s_log("ENTER_CMD_RESP discarded: command page 0x%04X out of range for slot size %u",
              (unsigned)command_page, (unsigned)slot_size);
        return false;
    }
    uint32_t region_end = region_offset + (uint32_t)region_size;
    if (region_end > slot_size) {
        s_log("ENTER_CMD_RESP failed: back-channel region exceeds slot size");
        return false;
    }

    // Commit region_offset early so hdr_read can use it to locate the token.
    s_state.cfg.region_offset = region_offset;

    // The token must start from the value already in the back-channel region.
    if (hdr_read(active_slot, HDR_TOKEN_LSB, &s_state.token_lsb) != ORA_RESULT_OK ||
        hdr_read(active_slot, HDR_TOKEN_MSB, &s_state.token_msb) != ORA_RESULT_OK) {
        s_log("ENTER_CMD_RESP failed: could not read existing token");
        return false;
    }
    s_log("ENTER_CMD_RESP: cp=0x%04X ro=%u rsz=%u cplt=0x%02X stok=0x%02X token=0x%02X%02X",
          (unsigned)command_page, (unsigned)region_offset, (unsigned)region_size,
          complete, status_ok, s_state.token_msb, s_state.token_lsb);

    s_state.cfg.command_page  = command_page;
    s_state.cfg.complete      = complete;
    s_state.cfg.status_ok     = status_ok;
    s_state.cfg.region_end    = region_end;
    s_state.cfg.data_size     = (uint32_t)region_size - HDR_SIZE;
    s_state.active_slot       = active_slot;
    init_back_channel(active_slot);
    s_state.active = true;

    s_log("ENTER_CMD_RESP succeeded: as=%u, ro=%u, re=%u",
          (unsigned)active_slot, (unsigned)s_state.cfg.region_offset,
          (unsigned)s_state.cfg.region_end);
    return true;
}

// ---------------------------------------------------------------------------
// Command handlers — Read group (0x01)
// ---------------------------------------------------------------------------

static bool exec_get_flash_slot_count(void) {
    uint8_t count = s_get_flash_slot_count(ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS);
    uint8_t resp[1] = { count };
    data_write(s_state.active_slot, 0u, resp, 1u);
    s_log("GET_FLASH_SLOT_COUNT: count=%u", (unsigned)count);
    return true;
}

static bool get_flash_slot_info(
    uint8_t flash_slot,
    uint8_t record[32]
) {
    const char *name = NULL;
    uint32_t rom_type = 0xFFu;
    if (s_get_flash_slot_info(
            flash_slot, 
            ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS,
            &name,
            &rom_type,
            NULL
        ) != ORA_RESULT_OK) {
        s_log("GET_FLASH_SLOT_INFO failed: invalid flash_slot %u", (unsigned)flash_slot);
        return false;
    }

    record[0] = (uint8_t)(rom_type & 0xFFu);
    zero_bytes(&record[1], 31u);
    if (name != NULL) {
        uint8_t nlen = 0u;
        while (nlen < 30u && name[nlen] != '\0') nlen++;
        for (uint8_t j = 0u; j < nlen; j++) record[1u + j] = (uint8_t)name[j];
    }
    return true;
}

static bool exec_get_flash_slot_info(uint8_t ram_slot) {
    uint8_t flash_slot = ring_read_byte();

    s_log("GET_FLASH_SLOT_INFO: flash_slot=%u", (unsigned)flash_slot);

    if (flash_slot == 0xAA) {
        s_log("GET_FLASH_SLOT_INFO failed: flash_slot value 0xAA is reserved");
        return false;
    }

    uint32_t space = s_state.cfg.data_size;
    if (space < 32u) {
        s_log("GET_FLASH_SLOT_INFO failed: data section too small");
        return false;
    }

    uint8_t record[32];
    if (!get_flash_slot_info(flash_slot, record)) {
        return false;
    }

    data_write(ram_slot, 0, record, 32u);
    return true;
}

static bool exec_get_flash_slot_info_all(uint8_t slot) {
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
        if (!get_flash_slot_info(i, record)) {
            return false;
        }

        // Whole records are 32 bytes; the trailing partial record is however
        // many bytes remain in the data section.
        uint32_t bytes = (i < whole_count) ? 32u : partial_bytes;
        if (i == whole_count) {
            // Partial record: force null terminator at the truncation point.
            record[partial_bytes - 1] = 0x00u;
        }
        data_write(slot, data_off, record, bytes);
        data_off += bytes;
    }

    return true;
}

static bool exec_get_ram_slot_info_all(uint8_t slot) {
    s_log("GET_RAM_SLOT_INFO_ALL: slot=%u", (unsigned)slot);
    uint8_t  total    = s_get_ram_slot_count();
    uint32_t rom_type = 0xFFu;

    // slot is s_state.active_slot — both the back-channel destination and
    // the index of the RAM slot currently being served.  Query its ROM type
    // via the extended get_ram_slot_info API.
    s_get_ram_slot_info(slot, NULL, NULL, &rom_type);

    uint8_t resp[4] = { total, slot, (uint8_t)(rom_type & 0xFFu), 0u };
    data_write(slot, 0u, resp, 4u);
    s_log("GET_RAM_SLOT_INFO: tot=%u sl=%u rt=0x%02X", (unsigned)total, (unsigned)slot, (unsigned)rom_type);
    return true;
}

static bool exec_get_device_type(void) {
    uint8_t device_type[24];
    zero_bytes((uint8_t *)device_type, sizeof(device_type));
    device_type[0] = 'O';
    device_type[1] = 'n';
    device_type[2] = 'e';
    device_type[3] = ' ';
    device_type[4] = 'R';
    device_type[5] = 'O';
    device_type[6] = 'M';
    data_write(s_state.active_slot, 0u, device_type, sizeof(device_type));
    s_log("GET_DEVICE_TYPE: dt=%s", device_type);
    return true;
}

static bool exec_get_device_version(void) {
    uint8_t device_version[24];
    zero_bytes(device_version, sizeof(device_version));
    if (s_get_device_version(device_version, sizeof(device_version)) != ORA_RESULT_OK) {
        s_log("GET_DEVICE_VERSION failed");
        return false;
    }
    data_write(s_state.active_slot, 0u, device_version, sizeof(device_version));
    s_log("GET_DEVICE_VERSION: ver=%s", device_version);
    return true;
}

static bool exec_get_protocol_version(void) {
    data_write(s_state.active_slot, 0u, protocol_version, sizeof(protocol_version));
    s_log("GET_PROTOCOL_VERSION: ver=%u.%u.%u", (unsigned)protocol_version[0], (unsigned)protocol_version[1], (unsigned)protocol_version[2]);
    return true;
}

static bool exec_slot_peek(void) {
    uint8_t a0     = ring_read_byte();
    uint8_t a1     = ring_read_byte();
    uint8_t a2     = ring_read_byte();
    uint8_t count  = ring_read_byte();
    uint8_t target = ring_read_byte();

    s_log("SLOT_PEEK: tgt=%u a0=0x%02X a1=0x%02X a2=0x%02X ct=%u",
          (unsigned)target, (unsigned)a0, (unsigned)a1, (unsigned)a2, (unsigned)count);

    if (target == 0xAAu) {
        s_log("SLOT_PEEK failed: target value 0xAA is reserved");
        return false;
    }

    uint32_t addr       = (uint32_t)a0
                        | ((uint32_t)a1 << 8u)
                        | ((uint32_t)a2 << 16u);
    uint32_t byte_count = (count == 0u) ? 256u : (uint32_t)count;

    if (byte_count > s_state.cfg.data_size) {
        s_log("SLOT_PEEK failed: ct %u vs data size %u",
              (unsigned)byte_count, (unsigned)s_state.cfg.data_size);
        return false;
    }

    uint32_t slot_base, slot_size;
    if (s_get_ram_slot_info(target, &slot_base, &slot_size, NULL) != ORA_RESULT_OK) {
        s_log("SLOT_PEEK failed: tgt slot %u", (unsigned)target);
        return false;
    }

    if (addr + byte_count > slot_size) {
        s_log("SLOT_PEEK failed: read range exceeds slot size");
        return false;
    }

    // Write the requested data in chunks, small enough to not blow the stack
    const uint32_t buf_size = 32u;
    uint8_t  buf[buf_size];
    uint32_t remaining  = byte_count;
    uint32_t data_off   = 0u;

    while (remaining > 0u) {
        uint32_t chunk = (remaining > buf_size) ? buf_size : remaining;
        for (uint32_t i = 0u; i < chunk; i++) {
            uint32_t phys_offset = s_map_addr_to_phys(addr + data_off + i);
            uint8_t  raw         = ((const uint8_t *)slot_base)[phys_offset];
            if (s_demangle_data(raw, &buf[i]) != ORA_RESULT_OK) {
                s_log("SLOT_PEEK failed: demangle error at offset %u",
                      (unsigned)(data_off + i));
                return false;
            }
        }
        data_write(s_state.active_slot, data_off, buf, chunk);
        data_off  += chunk;
        remaining -= chunk;
    }

    s_log("SLOT_PEEK: slot=%u addr=0x%06X count=%u",
          (unsigned)target, (unsigned)addr, (unsigned)byte_count);
    return true;
}

// ---------------------------------------------------------------------------
// Command handlers — Modify group (0x02)
// ---------------------------------------------------------------------------

static bool exec_slot_poke(void) {
    uint8_t a0     = ring_read_byte();
    uint8_t a1     = ring_read_byte();
    uint8_t a2     = ring_read_byte();
    uint8_t byte   = ring_read_byte();
    uint8_t target = ring_read_byte();

    if (target == 0xAAu) {
        s_log("SLOT_POKE failed: target value 0xAA is reserved");
        return false;
    }

    uint32_t addr = (uint32_t)a0
                  | ((uint32_t)a1 << 8u)
                  | ((uint32_t)a2 << 16u);
    return (s_reprogram(target, addr, &byte, 1u, 1u) == ORA_RESULT_OK);
}

static bool exec_switch_slot(void) {
    uint8_t target = ring_read_byte();

    if (target == 0xAAu) {
        s_log("SWITCH_SLOT failed: target value 0xAA is reserved");
        return false;
    }

    s_log("SWITCH_SLOT: target=%u", (unsigned)target);
    return (s_set_active_ram_slot(target) == ORA_RESULT_OK);
}

static bool exec_load_slot(void) {
    uint8_t ram_slot   = ring_read_byte();
    uint8_t flash_slot = ring_read_byte();

    s_log("LOAD_SLOT: ram_slot=%u flash_slot=%u", (unsigned)ram_slot, (unsigned)flash_slot);

    if ((ram_slot == 0xAAu) || (flash_slot == 0xAAu)) {
        s_log("LOAD_SLOT failed: slot value 0xAA is reserved");
        return false;
    }

    ora_result_t rc = s_copy_flash_to_ram(
        flash_slot,
        ORA_FLASH_SLOT_FLAG_EXCLUDE_PLUGINS,
        ram_slot,
        0u
    );
    if (rc != ORA_RESULT_OK) {
        s_log("LOAD_SLOT failed: copy_flash_to_ram error %d", (int)rc);
        return false;
    }
    return true;
}

static bool exec_slot_poke_all_byte(void) {
    uint8_t byte     = ring_read_byte();
    uint8_t target = ring_read_byte();

    if (target == 0xAAu) {
        s_log("SLOT_POKE_ALL_BYTE failed: target value 0xAA is reserved");
        return false;
    }

    uint32_t rom_type = 0xFFu;
    if (s_get_ram_slot_info(target, NULL, NULL, &rom_type) != ORA_RESULT_OK) {
        s_log("SLOT_POKE failed: invalid target slot %u", (unsigned)target);
        return false;
    }
    uint32_t chip_size = s_get_chip_size(rom_type);
    if (chip_size == 0u) {
        s_log("SLOT_POKE failed: invalid ROM type 0x%02X", (unsigned)rom_type);
        return false;
    }

    for (uint32_t i = 0u; i < chip_size; i++) {
        if (s_reprogram(target, i, &byte, 1u, 1u) != ORA_RESULT_OK) {
            s_log("SLOT_POKE failed: reprogram error at offset %u", (unsigned)i);
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

// Dispatch one command.  Reads argument bytes from the ring buffer.
// Uses s_state.active_slot for all back-channel writes.
//
// exit_silent_out: set to true for commands that must not trigger cmd_end
//                  (EXIT_CMD_RESP_SILENT, SWITCH_AND_EXIT).
//
// Returns the ok/fail result for cmd_end.
static bool dispatch(
    uint8_t group,
    uint8_t cmd,
    bool *exit_silent_out
) {
    *exit_silent_out = false;
    bool ok = false;

    switch (group) {
        case GRP_CONTROL:
            switch (cmd) {
                case CMD_NOP:
                    ok = exec_nop();
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
                    ok = true;
                    break;
                case CMD_EXIT_CMD_RESP_SILENT:
                    s_state.active   = false;
                    *exit_silent_out = true;
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
                case CMD_GET_FLASH_FLASH_SLOT_COUNT:
                    ok = exec_get_flash_slot_count();
                    break;
                case CMD_GET_FLASH_SLOT_INFO:
                    ok = exec_get_flash_slot_info(s_state.active_slot);
                    break;
                case CMD_GET_FLASH_SLOT_INFO_ALL:
                    ok = exec_get_flash_slot_info_all(s_state.active_slot);
                    break;
                case CMD_GET_RAM_SLOT_INFO_ALL:
                    ok = exec_get_ram_slot_info_all(s_state.active_slot);
                    break;
                case CMD_GET_DEVICE_TYPE:
                    ok = exec_get_device_type();
                    break;
                case CMD_GET_DEVICE_VERSION:
                    ok = exec_get_device_version();
                    break;
                case CMD_GET_PROTOCOL_VERSION:
                    ok = exec_get_protocol_version();
                    break;
                case CMD_SLOT_PEEK:
                    ok = exec_slot_peek();
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
                case CMD_SLOT_POKE_ALL_BYTE:
                    ok = exec_slot_poke_all_byte();
                    break;
                default:
                    ok = false;
                    break;
            }
            break;

        case GRP_RESET:
            switch (cmd) {
                case CMD_RBCP_RESET:
                    init_rbcp(false);
                    s_state.active = false; // Unecessary as init_rbcp sets this.
                    *exit_silent_out = true;
                    s_log("RBCP_RESET: state reset, active_slot=%u", (unsigned)s_state.active_slot);
                    ok = true;
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

    if (!ok) {
        s_log("CMD g=0x%02x c=0x%02x failed", group, cmd);
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
    bool exit_silent;

    if (was_active) {
        cmd_begin(s_state.active_slot, group, cmd);
    }

    bool ok = dispatch(group, cmd, &exit_silent);

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
    s_get_chip_size        = ora_lookup_fn(ORA_ID_GET_CHIP_SIZE_FROM_TYPE);
    s_get_device_version   = ora_lookup_fn(ORA_ID_GET_DEVICE_VERSION);
    s_map_addr_to_phys     = ora_lookup_fn(ORA_ID_MAP_ADDR_TO_PHYS);
    s_map_data_to_phys     = ora_lookup_fn(ORA_ID_MAP_DATA_TO_PHYS);
    s_demangle_data        = ora_lookup_fn(ORA_ID_DEMANGLE_DATA);

    ora_setup_address_monitor_fn_t              setup_address_monitor =
        ora_lookup_fn(ORA_ID_SETUP_ADDRESS_MONITOR);
    ora_start_address_monitor_fn_t              start_address_monitor =
        ora_lookup_fn(ORA_ID_START_ADDRESS_MONITOR);
    ora_get_address_monitor_ring_write_pos_fn_t get_write_pos =
        ora_lookup_fn(ORA_ID_GET_ADDRESS_MONITOR_RING_WRITE_POS);
    ora_init_knock_fn_t     init_knock     = ora_lookup_fn(ORA_ID_INIT_KNOCK);
    ora_wait_for_knock_fn_t wait_for_knock = ora_lookup_fn(ORA_ID_WAIT_FOR_KNOCK);

    s_log("RBCP plugin starting");

    init_rbcp(true);

    // Set up address monitor in control mode so the plugin can modify the
    // ROM image being served (required for back-channel writes).
    ora_result_t rc = setup_address_monitor(
        ring_buf,
        RING_ENTRIES_LOG2,
        ORA_MONITOR_MODE_CONTROL,
        RING_DATA_SIZE,
        NULL
    );
    if (rc != ORA_RESULT_OK) {
        s_log("RBCP: address monitor setup failed %d", rc);
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
    if (init_knock(
            s_knock_seq,
            KNOCK_LEN,
            8u,
            RING_DATA_SIZE,
            knock
        ) != ORA_RESULT_OK) {
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
        //
        // We always call wait_for_knock with start_pos = NULL so that method
        // starts listening from the current write position of the DMA channel.
        // This means that some items may be discarded, but is safer from
        // assuming we can restart from wherever we just read from - because
        // enough time may have passed to mean that the buffer has wrapped,
        // which could cause inconsistent data to be read.
        volatile uint32_t *next_read;
        uint32_t preamble[2];
        if (wait_for_knock(knock, ring_buf, RING_ENTRIES_LOG2,
                           ORA_WAIT_FOR_KNOCK_FLAG_DEBOUNCE_CS,
                           preamble, 2u, NULL, &next_read) != ORA_RESULT_OK) {
            continue;
        }

        // WARNING.  There must be NO logging between detecting a knock and
        // consuming any argument bytes (in cmd/resp mode) or at all (in cmd 
        // mode) as otherwise the ring buffer may fill with log entries and
        // overwrite some before the code gets a chance to read them.
        //
        // That means no logging here until after the command itself reads
        // any further argument bytes.

        RING_BUF_UPDATE_READ_INDEX(next_read);

        // Demangle GROUP and CMD from physical captures to logical addresses.
        // ORA_WAIT_FOR_KNOCK_FLAG_DEBOUNCE_CS guarantees payload entries are
        // CS-active, so check_control_pins=0 is correct here.
        uint32_t logical;
        if (s_demangle(preamble[0], &logical, 0) != ORA_RESULT_OK) continue;
        uint8_t group = (uint8_t)(logical & 0xFFu);

        if (s_demangle(preamble[1], &logical, 0) != ORA_RESULT_OK) continue;
        uint8_t cmd = (uint8_t)(logical & 0xFFu);

        // Execute the command.  In command mode run_command returns false, so
        // this executes once.  In command-response mode (until it exits) it
        // returns true, so we loop, reading the next command directly from the
        // ring buffer without waiting for another knock.
        while (run_command(group, cmd)) {
            group = ring_read_byte();
            cmd   = ring_read_byte();
            // run_command (via cmd_end, via hdr_write) discards any reads
            // until the point it signals completion, unless the command was
            // a silent exit, in which case it doesn't - but the
            // wait_for_knock handles that.
        }

        // Command-Response mode exited (or Command-mode session ended).
        // Return to outer loop to await the next knock.
    }
}