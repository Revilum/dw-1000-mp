/*! ----------------------------------------------------------------------------
 * @file    moddw1000.c
 * @brief   MicroPython DW1000 module implementation
 *
 * This file implements the MicroPython C module for the DW1000 UWB transceiver.
 * It provides a PythoSTATIC mp_obj_t dw1000_reset(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }rface to the DW1000 driver functionality.
 */

#include "py/runtime.h"
#include "py/obj.h"
#include "py/objtype.h"
#include "py/objint.h"  // For mp_obj_int_t to extract long long values
#include <string.h>
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"

// Pico SDK GPIO includes for C-level IRQ handling
#include "hardware/gpio.h"
#include "hardware/irq.h"

// Include machine pin header for accessing GPIO number
#include "machine_pin.h"

// Define STATIC if not already defined
#ifndef STATIC
#define STATIC static
#endif

// DW1000 driver and HAL includes
#include "../../dw1000-driver/deca_device_api.h"
#include "../../dw1000-driver/deca_regs.h"
#include "dw1000_hal.h"

// Forward declarations for C callback functions
STATIC void c_callback_rx_ok(const dwt_cb_data_t *cb_data);
STATIC void c_callback_tx_done(const dwt_cb_data_t *cb_data);
STATIC void c_callback_rx_to(const dwt_cb_data_t *cb_data);
STATIC void c_callback_rx_err(const dwt_cb_data_t *cb_data);

// Global pointer to current DW1000 object for C callbacks
STATIC dw1000_obj_t *current_dw1000_obj = NULL;

// Custom RX state definitions (not in driver header)
#define SYS_STATE_RX_STATE      0x0000000FUL    /* RX State mask */
#define SYS_STATE_TX_STATE      0x000000F0UL    /* TX State mask */
#define SYS_STATE_PMSC_STATE    0x00000F00UL    /* PMSC State mask */

// RX states (from DW1000 manual)
#define RX_STATE_IDLE           0x00
#define RX_STATE_RX_READY       0x01
#define RX_STATE_RX_WAIT_SFD    0x02
#define RX_STATE_RX_DATA        0x03
#define RX_STATE_RX_VALIDATE    0x05

// Local function declarations
STATIC mp_obj_t dw1000_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args);
STATIC void dw1000_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind);

// Forward declare the type (defined at bottom of file)
extern const mp_obj_type_t dw1000_type;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_make_new()
 *
 * @brief Constructor for DW1000 objects
 */
STATIC mp_obj_t dw1000_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, 4, false);
    
    // Create new DW1000 object
    dw1000_obj_t *self = m_new_obj(dw1000_obj_t);
    self->base.type = &dw1000_type;
    self->spi = args[0];        // SPI object (required)
    self->cs_pin = args[1];     // CS pin (required)
    self->reset_pin = (n_args > 2) ? args[2] : MP_OBJ_NULL;  // Reset pin (optional)
    self->irq_pin = (n_args > 3) ? args[3] : MP_OBJ_NULL;    // IRQ pin (optional)
    self->initialized = false;
    
    // Initialize callbacks to None
    self->rx_callback = mp_const_none;
    self->tx_callback = mp_const_none;
    self->error_callback = mp_const_none;
    self->timeout_callback = mp_const_none;
    self->auto_rx = false;
    self->irq_enabled = false;
    
    return MP_OBJ_FROM_PTR(self);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_init()
 *
 * @brief Initialize the DW1000 device with comprehensive setup (based on Polypoint)
 */
STATIC mp_obj_t dw1000_init(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_auto_rx };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_auto_rx, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };

    dw1000_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    
    // Parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
    if (self->initialized) {
        return mp_const_true;
    }
    
    // Set auto_rx mode from arguments
    self->auto_rx = args[ARG_auto_rx].u_bool;
    
    // Initialize HAL
    dw1000_hal_init(self);
    
    // Reset the device if reset pin is provided
    if (self->reset_pin != MP_OBJ_NULL) {
        dw1000_reset_pin_set(self, false);  // Assert reset
        deca_sleep(10);                      // Hold reset for 10ms
        dw1000_reset_pin_set(self, true);   // Release reset
        deca_sleep(10);                      // Wait for startup
    }
    
    // Initialize DW1000 driver with available flags
    int result = dwt_initialise(DWT_LOADUCODE);  // Load microcode only
    if (result != DWT_SUCCESS) {
        mp_raise_OSError(MP_EIO);
        return mp_const_false;
    }
    
    // Enhanced configuration based on lab11 dw1000-driver and Polypoint
    dwt_config_t config = {
        .chan = 5,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_128,
        .rxPAC = DWT_PAC8,
        .txCode = 9,
        .rxCode = 9,
        .nsSFD = 1,
        .dataRate = DWT_BR_6M8,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (128 + 1 + 8 - 8)
    };
    
    // Configure the device (dwt_configure doesn't return a value in this driver)
    dwt_configure(&config);
    
    // CRITICAL: Complete Polypoint-style initialization sequence
    // 1. Configure TX RF settings
    static const uint8_t txPower[8] = {0x0, 0x67, 0x67, 0x8b, 0x9a, 0x85, 0x0, 0xd1};
    static const uint8_t pgDelay[8] = {0x0, 0xc9, 0xc2, 0xc5, 0x95, 0xc0, 0x0, 0x93};
    dwt_txconfig_t tx_config = {
        .PGdly = pgDelay[5],  // Channel 5
        .power = txPower[5]   // Channel 5
    };
    dwt_configuretxrf(&tx_config);
    
    // 2. Set crystal trim (function is called dwt_setxtaltrim in this driver)
    dwt_setxtaltrim(8);
    
    // 3. Configure antenna delays (0 for end-to-end calibration)
    uint16_t antenna_delay = 0;
    dwt_setrxantennadelay(antenna_delay);
    dwt_settxantennadelay(antenna_delay);
    
    // Configure interrupts (clear all, then set specific ones) 
    dwt_setinterrupt(0xFFFFFFFF, 0);  // Clear all interrupts
    dwt_setinterrupt(DWT_INT_TFRS |   // TX frame sent
                    DWT_INT_RFCG |    // RX frame good
                    DWT_INT_RPHE |    // RX PHY header error
                    DWT_INT_RFCE |    // RX CRC error
                    DWT_INT_RFSL |    // RX sync loss
                    DWT_INT_RFTO |    // RX timeout
                    DWT_INT_RXPTO |   // RX preamble timeout
                    DWT_INT_SFDT |    // SFD timeout
                    DWT_INT_ARFE, 1); // Frame filtering rejection

    // Register C callbacks with the lab11 driver
    dwt_setcallbacks(c_callback_tx_done, c_callback_rx_ok, c_callback_rx_to, c_callback_rx_err);

    self->initialized = true;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(dw1000_init_obj, 1, dw1000_init);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_deinit()
 *
 * @brief Deinitialize the DW1000 device
 */
STATIC mp_obj_t dw1000_deinit(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    dw1000_hal_deinit(self);
    self->initialized = false;
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_deinit_obj, dw1000_deinit);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_device_id()
 *
 * @brief Read the device ID
 */
STATIC mp_obj_t dw1000_read_device_id(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint32_t device_id = dwt_readdevid();
    return mp_obj_new_int_from_uint(device_id);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_read_device_id_obj, dw1000_read_device_id);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_configure()
 *
 * @brief Configure the DW1000 device with keyword arguments
 */
STATIC mp_obj_t dw1000_configure(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_channel, ARG_prf, ARG_tx_preamble_length, ARG_rx_pac, ARG_tx_code, ARG_rx_code, 
           ARG_non_standard_sfd, ARG_data_rate, ARG_phr_mode, ARG_sfd_timeout };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel,              MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5} },
        { MP_QSTR_prf,                  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DWT_PRF_64M} },
        { MP_QSTR_tx_preamble_length,   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DWT_PLEN_128} },
        { MP_QSTR_rx_pac,               MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DWT_PAC8} },
        { MP_QSTR_tx_code,              MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 9} },
        { MP_QSTR_rx_code,              MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 9} },
        { MP_QSTR_non_standard_sfd,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_data_rate,            MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DWT_BR_6M8} },
        { MP_QSTR_phr_mode,             MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DWT_PHRMODE_STD} },
        { MP_QSTR_sfd_timeout,          MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = (128 + 1 + 8 - 8)} },
    };

    dw1000_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
    // Build configuration structure
    dwt_config_t config = {
        .chan = args[ARG_channel].u_int,
        .prf = args[ARG_prf].u_int,
        .txPreambLength = args[ARG_tx_preamble_length].u_int,
        .rxPAC = args[ARG_rx_pac].u_int,
        .txCode = args[ARG_tx_code].u_int,
        .rxCode = args[ARG_rx_code].u_int,
        .nsSFD = args[ARG_non_standard_sfd].u_bool ? 1 : 0,
        .dataRate = args[ARG_data_rate].u_int,
        .phrMode = args[ARG_phr_mode].u_int,
        .sfdTO = args[ARG_sfd_timeout].u_int
    };
    
    // Apply configuration
    dwt_configure(&config);
    
    // CRITICAL: Apply complete Polypoint initialization sequence after ANY configuration
    // This ensures consistent initialization regardless of which function calls dwt_configure
    
    // 1. Configure TX RF settings (matching the channel)
    static const uint8_t txPower[8] = {0x0, 0x67, 0x67, 0x8b, 0x9a, 0x85, 0x0, 0xd1};
    static const uint8_t pgDelay[8] = {0x0, 0xc9, 0xc2, 0xc5, 0x95, 0xc0, 0x0, 0x93};
    
    dwt_txconfig_t tx_config = {
        .PGdly = pgDelay[config.chan],  // Use the actual channel configured
        .power = txPower[config.chan]   // Use the actual channel configured
    };
    dwt_configuretxrf(&tx_config);
    
    // 2. Set crystal trim (Polypoint standard)
    dwt_setxtaltrim(8);
    
    // 3. Configure antenna delays (0 for end-to-end calibration)
    uint16_t antenna_delay = 0;
    dwt_setrxantennadelay(antenna_delay);
    dwt_settxantennadelay(antenna_delay);
    
    // Set XTAL trim (use default) - use correct function name
    dwt_setxtaltrim(15);  // Default XTAL trim value
    
    // 4. Disable frame filtering for simple ranging (accept all frames)
    // For production use with 802.15.4 frames, enable with DWT_FF_DATA_EN | DWT_FF_ACK_EN
    dwt_enableframefilter(0);  // 0 = disable all filtering
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(dw1000_configure_obj, 1, dw1000_configure);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_tx_frame()
 *
 * @brief Transmit a frame
 */
STATIC mp_obj_t dw1000_tx_frame(mp_obj_t self_in, mp_obj_t data_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Get data buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    
    if (bufinfo.len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("Empty data buffer"));
    }
    
    // Write data to TX buffer
    // Note: DW1000 API expects total frame length including 2-byte CRC that it will add
    // From API docs: "txFrameLength - This is the total frame length, including the two byte CRC"
    uint16_t total_frame_len = bufinfo.len + 2;
    dwt_writetxdata(total_frame_len, (uint8_t*)bufinfo.buf, 0);
    dwt_writetxfctrl(total_frame_len, 0, 0);
    
    // Start transmission
    int result = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (result != DWT_SUCCESS) {
        mp_raise_OSError(MP_EIO);
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_tx_frame_obj, dw1000_tx_frame);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_rx_enable()
 *
 * @brief Enable receiver with comprehensive reset sequence (based on Polypoint)
 */
STATIC mp_obj_t dw1000_rx_enable(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Polypoint pattern: Force off first, then reconfigure, then enable
    dwt_forcetrxoff();
    
    // Wait for transceiver to settle (critical timing)
    deca_sleep(2);
    
    // Clear all status flags 
    dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    
    // Reset and sync RX buffers  
    dwt_rxreset();
    dwt_syncrxbufptrs();
    
    // Small delay for buffer reset to complete
    deca_sleep(1);
    
    // Try enabling receiver - return the result
    int result = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    // Additional verification: Check if SYS_CTRL RXENAB bit is actually set
    deca_sleep(1);  // Allow time for register to update
    uint32_t sys_ctrl = dwt_read32bitreg(SYS_CTRL_ID);
    
    if ((sys_ctrl & 0x0100) == 0) {  // RXENAB bit not set
        // RX enable failed - but let's return false instead of raising exception
        // This allows diagnostic tools to continue running
        return mp_const_false;
    }
    
    if (result != DWT_SUCCESS) {
        mp_raise_OSError(MP_EIO);
        return mp_const_false;
    }
    
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_rx_enable_obj, dw1000_rx_enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_rx_data()
 *
 * @brief Read received data with hardware CRC validation
 */
STATIC mp_obj_t dw1000_read_rx_data(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Check hardware CRC status first
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    
    // Verify frame was received with good CRC (hardware validation)
    if (!(status & SYS_STATUS_RXFCG)) {
        // Frame not received or CRC error
        if (status & SYS_STATUS_RXFCE) {
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Frame received with CRC error"));
        }
        return mp_obj_new_bytes(NULL, 0);  // No frame or invalid frame
    }
    
    // Get frame length from RX_FINFO register (includes 2-byte CRC added by DW1000)
    uint32_t total_frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    
    if (total_frame_len < 2) {
        return mp_obj_new_bytes(NULL, 0);
    }
    
    // Calculate payload length (exclude 2-byte CRC automatically added by DW1000)
    // RX_FINFO reports total frame length including CRC, subtract 2 to get user payload
    uint32_t payload_len = total_frame_len - 2;
    
    // Safety check - ensure we don't read more than reasonable payload size
    if (payload_len > 125) {  // DW1000 max frame is 127, so max payload is 125
        return mp_obj_new_bytes(NULL, 0);
    }
    
    // Allocate buffer and read only the payload data (excluding CRC)
    // Note: DW1000 RX buffer layout is [payload][2-byte CRC], so we read only payload_len bytes
    uint8_t *buffer = m_new(uint8_t, payload_len);
    
    // Clear buffer to detect any uninitialized data
    memset(buffer, 0, payload_len);
    
    // Read exactly the payload bytes from the start of the RX buffer
    dwt_readrxdata(buffer, payload_len, 0);
    
    // Create Python bytes object
    mp_obj_t result = mp_obj_new_bytes(buffer, payload_len);
    
    // Free buffer
    m_del(uint8_t, buffer, payload_len);
    
    return result;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_read_rx_data_obj, dw1000_read_rx_data);



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_get_status()
 *
 * @brief Get device status
 */
STATIC mp_obj_t dw1000_get_status(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    return mp_obj_new_int_from_uint(status);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_get_status_obj, dw1000_get_status);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_force_trx_off()
 *
 * @brief Force transceiver off (uses driver's dwt_forcetrxoff which clears status)
 */
STATIC mp_obj_t dw1000_force_trx_off(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Use driver's comprehensive trx off function which also clears status
    dwt_forcetrxoff();
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_force_trx_off_obj, dw1000_force_trx_off);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_rx_reset()
 *
 * @brief Reset receiver (uses driver's dwt_rxreset)
 */
STATIC mp_obj_t dw1000_rx_reset(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Use driver's receiver reset function
    dwt_rxreset();
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_rx_reset_obj, dw1000_rx_reset);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_sync_rx_bufptrs()
 *
 * @brief Synchronize RX buffer pointers (uses driver's dwt_syncrxbufptrs)
 */
STATIC mp_obj_t dw1000_sync_rx_bufptrs(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Use driver's buffer pointer sync function
    dwt_syncrxbufptrs();
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_sync_rx_bufptrs_obj, dw1000_sync_rx_bufptrs);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_get_sys_state()
 *
 * @brief Get system state register for diagnostics
 */
STATIC mp_obj_t dw1000_get_sys_state(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint32_t sys_state = dwt_read32bitreg(SYS_STATE_ID);
    return mp_obj_new_int_from_uint(sys_state);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_get_sys_state_obj, dw1000_get_sys_state);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_get_rx_finfo()
 *
 * @brief Get RX frame info register for diagnostics
 */
STATIC mp_obj_t dw1000_get_rx_finfo(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint32_t rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
    return mp_obj_new_int_from_uint(rx_finfo);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_get_rx_finfo_obj, dw1000_get_rx_finfo);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_register()
 *
 * @brief Read any 32-bit register for advanced diagnostics
 */
STATIC mp_obj_t dw1000_read_register(mp_obj_t self_in, mp_obj_t reg_id_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint32_t reg_id = mp_obj_get_int(reg_id_obj);
    uint32_t reg_value = dwt_read32bitreg(reg_id);
    return mp_obj_new_int_from_uint(reg_value);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_read_register_obj, dw1000_read_register);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_rx_timestamp()
 *
 * @brief Read RX timestamp from DW1000 (40-bit timestamp as integer)
 */
STATIC mp_obj_t dw1000_read_rx_timestamp(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint64_t timestamp;
    dwt_readrxtimestamp((uint8_t*)&timestamp);
    
    // Return as Python integer (40-bit value)
    return mp_obj_new_int_from_ull(timestamp & 0xFFFFFFFFFFULL);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_read_rx_timestamp_obj, dw1000_read_rx_timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_tx_timestamp()
 *
 * @brief Read TX timestamp from DW1000 (40-bit timestamp as integer)
 */
STATIC mp_obj_t dw1000_read_tx_timestamp(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint64_t timestamp;
    dwt_readtxtimestamp((uint8_t*)&timestamp);
    
    // Return as Python integer (40-bit value)
    return mp_obj_new_int_from_ull(timestamp & 0xFFFFFFFFFFULL);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_read_tx_timestamp_obj, dw1000_read_tx_timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_timestamp_to_seconds()
 *
 * @brief Convert DW1000 timestamp ticks to seconds (using DWT_TIME_UNITS)
 */
STATIC mp_obj_t dw1000_timestamp_to_seconds(mp_obj_t self_in, mp_obj_t timestamp_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Convert to float first to handle 40-bit timestamps on 32-bit platforms
    mp_float_t timestamp_float = mp_obj_get_float(timestamp_obj);
    
    // Convert using DWT_TIME_UNITS = 15.65e-12 seconds per tick
    double seconds = timestamp_float * DWT_TIME_UNITS;
    
    return mp_obj_new_float(seconds);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_timestamp_to_seconds_obj, dw1000_timestamp_to_seconds);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_seconds_to_timestamp()
 *
 * @brief Convert seconds to DW1000 timestamp ticks (using DWT_TIME_UNITS)
 */
STATIC mp_obj_t dw1000_seconds_to_timestamp(mp_obj_t self_in, mp_obj_t seconds_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    double seconds = mp_obj_get_float(seconds_obj);
    
    // Convert using DWT_TIME_UNITS = 15.65e-12 seconds per tick
    uint64_t timestamp = (uint64_t)(seconds / DWT_TIME_UNITS);
    
    return mp_obj_new_int_from_ull(timestamp & 0xFFFFFFFFFFULL);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_seconds_to_timestamp_obj, dw1000_seconds_to_timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_system_timestamp()
 *
 * @brief Read current DW1000 system timestamp (for scheduling delayed transmissions)
 * Returns high 32-bits of 40-bit timestamp (units are ~15.65ps * 256)
 */
STATIC mp_obj_t dw1000_read_system_timestamp(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Read system timestamp high 32 bits (suitable for delayed TX scheduling)
    uint32_t sys_time_high = dwt_readsystimestamphi32();
    
    // Convert to 40-bit timestamp by shifting left 8 bits
    uint64_t timestamp = ((uint64_t)sys_time_high) << 8;
    
    return mp_obj_new_int_from_ull(timestamp);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_read_system_timestamp_obj, dw1000_read_system_timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_delayed_trx_time()
 *
 * @brief Set delayed TX/RX time (high 32-bits of 40-bit timestamp)
 */
STATIC mp_obj_t dw1000_set_delayed_trx_time(mp_obj_t self_in, mp_obj_t starttime_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint64_t starttime = mp_obj_get_int(starttime_obj);
    
    // Extract high 32 bits (must be even per DW1000 spec)
    uint32_t delay_time_high = ((starttime >> 8) & 0xFFFFFFFE);
    
    // Set delayed transmission/reception time using lab11 driver
    dwt_setdelayedtrxtime(delay_time_high);
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_delayed_trx_time_obj, dw1000_set_delayed_trx_time);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_start_tx_delayed()
 *
 * @brief Start delayed transmission at previously scheduled time
 * Must call set_delayed_trx_time() first, then load TX buffer, then call this
 */
STATIC mp_obj_t dw1000_start_tx_delayed(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Start delayed transmission
    int result = dwt_starttx(DWT_START_TX_DELAYED);
    
    if (result != DWT_SUCCESS) {
        // Delayed transmission failed (usually means scheduled time already passed)
        mp_raise_OSError(MP_EIO);
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_start_tx_delayed_obj, dw1000_start_tx_delayed);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_tx_frame_delayed()
 *
 * @brief Transmit a frame at a scheduled future time (all-in-one convenience function)
 * Combines set_delayed_trx_time, tx buffer loading, and start_tx_delayed
 * 
 * @param data_obj - frame data to transmit
 * @param scheduled_time_obj - 40-bit timestamp when to transmit (must be future and even)
 */
STATIC mp_obj_t dw1000_tx_frame_delayed(mp_obj_t self_in, mp_obj_t data_obj, mp_obj_t scheduled_time_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Get data buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    
    if (bufinfo.len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("Empty data buffer"));
    }
    
    // Get scheduled transmission time (40-bit timestamp)
    // Use mp_obj_get_int_truncated for large values that might overflow mp_obj_get_int
    uint64_t scheduled_time = (uint64_t)mp_obj_get_int_truncated(scheduled_time_obj);
    
    // Ensure scheduled time is even (hardware requirement - bit 0 must be 0)
    scheduled_time &= 0xFFFFFFFFFFFFFFFEULL;
    
    // Extract high 32 bits for dwt_setdelayedtrxtime (shifts right by 8 bits)
    uint32_t delay_time_high = (uint32_t)(scheduled_time >> 8);
    
    // Step 1: Set delayed transmission time
    dwt_setdelayedtrxtime(delay_time_high);
    
    // Step 2: Write data to TX buffer
    uint16_t total_frame_len = bufinfo.len + 2;  // +2 for auto-CRC
    dwt_writetxdata(total_frame_len, (uint8_t*)bufinfo.buf, 0);
    dwt_writetxfctrl(total_frame_len, 0, 0);
    
    // Step 3: Start delayed transmission
    int result = dwt_starttx(DWT_START_TX_DELAYED);
    
    if (result != DWT_SUCCESS) {
        // Delayed transmission failed (scheduled time likely already passed)
        mp_raise_OSError(MP_EIO);
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(dw1000_tx_frame_delayed_obj, dw1000_tx_frame_delayed);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_send_ranging_response()
 *
 * @brief TIME-CRITICAL: Send ranging response with hardware-scheduled delayed TX
 * 
 * This function does everything atomically in C to minimize timing jitter:
 * 1. Read current system timestamp
 * 2. Calculate T3 = current + delay_us
 * 3. Build response message inline
 * 4. Schedule and start delayed transmission
 * 
 * @param t2_obj - T2 timestamp (POLL RX time) as 64-bit integer
 * @param delay_us_obj - Delay in microseconds (e.g. 5000 for 5ms)
 * @param poll_seq_obj - Poll sequence number
 * @param device_id_obj - Responder device ID
 * 
 * @return Tuple: (t3_scheduled, t3_actual) or raises OSError if failed
 */
STATIC mp_obj_t dw1000_send_ranging_response(size_t n_args, const mp_obj_t *args) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Parse arguments - T2 is a 40-bit timestamp (up to 1,099,511,627,776)
    // Must extract full 64-bit value - can't use mp_obj_get_int() (only 32-bit on RP2040)
    // Use conversion through bytes to preserve all 64 bits
    uint64_t t2 = 0;
    byte t2_bytes[8];
    mp_obj_int_to_bytes_impl(args[1], false, 8, t2_bytes);  // Little endian, 8 bytes
    memcpy(&t2, t2_bytes, 8);
    
    uint32_t delay_us = mp_obj_get_int(args[2]);
    uint8_t poll_seq = mp_obj_get_int(args[3]);
    uint8_t device_id = mp_obj_get_int(args[4]);
    
    // Optional TX calibration offset (PolyPoint-style software calibration)
    // This is ADDED to T3 before putting in the message
    int32_t tx_cal = (n_args >= 6) ? mp_obj_get_int(args[5]) : 0;
    
    // Variables for atomic section results
    uint64_t current_time;  // Current system timestamp (40-bit)
    uint64_t t3;
    uint32_t delay_time_high;
    
    // ============================================================================
    // ATOMIC SECTION: Disable interrupts during time-critical operations
    // This prevents USB, timer, and other interrupts from adding jitter during
    // the critical window between reading timestamp and setting delayed TX time.
    // Pattern matches PolyPoint's timer_disable_interrupt() approach.
    // ============================================================================
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    
    // TIME-CRITICAL SECTION STARTS HERE
    // Read current 40-bit timestamp (5 bytes)
    // BUG FIX: Use dwt_readsystime() to get FULL 40-bit timestamp, not just high 32 bits!
    byte sys_time_bytes[5];
    dwt_readsystime(sys_time_bytes);
    
    // Convert 5 bytes (little-endian) to 64-bit integer
    current_time = 0;
    for(int i = 0; i < 5; i++) {
        current_time |= ((uint64_t)sys_time_bytes[i]) << (i * 8);
    }
    
    // Convert delay from microseconds to ticks
    // Use the EXACT PolyPoint formula: ticks = (us / DWT_TIME_UNITS) / 1e6
    // where DWT_TIME_UNITS = (1.0/499.2e6/128.0) = 15.65e-12 s
    // This gives: ticks = us / (15.65e-12 * 1e6) = us / 15.65e-6
    // 
    // IMPORTANT: Do the division, not multiplication! This matches PolyPoint exactly
    // and avoids accumulating rounding errors from approximate multiplication constants.
    uint64_t delay_ticks = (uint64_t)((double)delay_us / DWT_TIME_UNITS / 1e6);
    
    // Calculate T3
    t3 = current_time + delay_ticks;
    t3 &= 0xFFFFFFFFFFULL;  // Mask to 40 bits (DW1000 timestamp is 40-bit)
    t3 &= 0xFFFFFFFFFFFFFFFEULL;  // Must be even (bit 0 = 0)
    
    // Extract high 32 bits for dwt_setdelayedtrxtime
    delay_time_high = (uint32_t)(t3 >> 8);
    
    // Set delayed transmission time
    dwt_setdelayedtrxtime(delay_time_high);
    
    // TIME-CRITICAL SECTION ENDS HERE
    MICROPY_END_ATOMIC_SECTION(atomic_state);
    // ============================================================================
    
    // Build response message inline (26 bytes total)
    // Format: [type][device_id][seq][padding(7)][T2(5)][padding(3)][T3(5)][padding(3)]
    //         Arduino-DW1000 uses 5-byte timestamps (40-bit), not 8!
    uint8_t response[26];
    response[0] = 0x62;  // MSG_RESPONSE
    response[1] = device_id;
    response[2] = poll_seq;
    // Padding bytes [3-9]
    memset(&response[3], 0, 7);
    
    // Mask T2 to 40 bits and write as little-endian (5 bytes only!)
    uint64_t t2_masked = t2 & 0xFFFFFFFFFFULL;
    memcpy(&response[10], &t2_masked, 5);  // Only 5 bytes!
    memset(&response[15], 0, 3);  // Pad remaining 3 bytes
    
    // T3 timestamp [18-22] - Apply TX calibration (PolyPoint pattern)
    // tx_cal is ADDED to T3 to compensate for TX antenna delay
    uint64_t t3_cal = (t3 + tx_cal) & 0xFFFFFFFFFFULL;  // Mask to 40 bits after adding calibration
    memcpy(&response[18], &t3_cal, 5);  // Only 5 bytes!
    memset(&response[23], 0, 3);  // Pad remaining 3 bytes
    
    // Setup TX frame control
    uint16_t total_frame_len = 26 + 2;  // +2 for CRC
    dwt_writetxfctrl(total_frame_len, 0, 0);
    
    // ============================================================================
    // TX ORDER FIX: Match PolyPoint's proven implementation
    // PolyPoint calls dwt_starttx() BEFORE dwt_writetxdata() in
    // oneway_anchor.c lines 209-211. This minimizes latency between setting
    // the delayed TX time and starting transmission.
    // ============================================================================
    
    // Start delayed transmission FIRST (PolyPoint order)
    int result = dwt_starttx(DWT_START_TX_DELAYED);
    
    if (result != DWT_SUCCESS) {
        mp_raise_OSError(MP_EIO);
    }
    
    // THEN write TX data (PolyPoint order)
    dwt_writetxdata(total_frame_len, response, 0);
    
    // Return t3 for verification (RAW t3, before calibration)
    return mp_obj_new_int_from_ull(t3);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(dw1000_send_ranging_response_obj, 5, 6, dw1000_send_ranging_response);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_rx_antenna_delay()
 *
 * @brief Set RX antenna delay for timestamp calibration
 */
STATIC mp_obj_t dw1000_set_rx_antenna_delay(mp_obj_t self_in, mp_obj_t delay_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint16_t delay = mp_obj_get_int(delay_obj);
    
    // Set RX antenna delay using lab11 driver
    dwt_setrxantennadelay(delay);
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_rx_antenna_delay_obj, dw1000_set_rx_antenna_delay);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_tx_antenna_delay()
 *
 * @brief Set TX antenna delay for timestamp calibration
 */
STATIC mp_obj_t dw1000_set_tx_antenna_delay(mp_obj_t self_in, mp_obj_t delay_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    uint16_t delay = mp_obj_get_int(delay_obj);
    
    // Set TX antenna delay using lab11 driver
    dwt_settxantennadelay(delay);
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_tx_antenna_delay_obj, dw1000_set_tx_antenna_delay);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_spi_set_baudrate()
 *
 * @brief Change SPI baudrate (for speed switching during init/wakeup vs normal operations)
 * 
 * DW1000 DATASHEET REQUIREMENT: SPI must be < 3 MHz during:
 *   - Initial power-on initialization (dwt_initialise)
 *   - Waking from DEEPSLEEP (reading device ID after wakeup)
 *   - Sleep configuration (dwt_configuresleep, dwt_configuresleepcnt)
 * 
 * After init/wakeup, SPI can be increased to higher speeds (e.g., 6 MHz like PolyPoint)
 * 
 * TODO: Implement speed switching logic in Python:
 *   1. INIT SEQUENCE:
 *      spi = SPI(0, baudrate=1000000)  # Start slow (<3 MHz)
 *      dwt = DW1000(spi, cs, reset, irq)
 *      dwt.init()                      # Must use slow SPI!
 *      dwt.spi_set_baudrate(6000000)   # Switch to fast after init
 *      dwt.configure(...)              # Normal operations at fast speed
 * 
 *   2. WAKEUP SEQUENCE (future sleep/wake implementation):
 *      dwt.spi_set_baudrate(1000000)   # Slow before wakeup
 *      dwt.wakeup()                    # Wake from sleep
 *      time.sleep_ms(5)                # Wait for PLL to lock
 *      dwt.spi_set_baudrate(6000000)   # Fast after wakeup
 * 
 * Reference: PolyPoint uses prescaler 64 (~0.75 MHz) for init/wakeup,
 *            prescaler 8 (~6 MHz) for normal operations
 * 
 * @param baudrate - new SPI baudrate in Hz (e.g., 1000000 for 1 MHz, 6000000 for 6 MHz)
 */
STATIC mp_obj_t dw1000_spi_set_baudrate(mp_obj_t self_in, mp_obj_t baudrate_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Call SPI.init(baudrate=new_baudrate) to change the SPI speed
    // Load the 'init' method from the SPI object
    mp_obj_t init_method = mp_load_attr(self->spi, MP_QSTR_init);
    
    // Create keyword args map with 'baudrate' key
    mp_obj_t kw_args[2] = {
        MP_OBJ_NEW_QSTR(MP_QSTR_baudrate),
        baudrate_obj
    };
    
    // Call spi.init(baudrate=baudrate_obj)
    mp_call_function_n_kw(init_method, 0, 1, kw_args);
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_spi_set_baudrate_obj, dw1000_spi_set_baudrate);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_rx_callback()
 *
 * @brief Set callback function for RX events
 */
STATIC mp_obj_t dw1000_set_rx_callback(mp_obj_t self_in, mp_obj_t callback) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    self->rx_callback = callback;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_rx_callback_obj, dw1000_set_rx_callback);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_tx_callback()
 *
 * @brief Set callback function for TX events
 */
STATIC mp_obj_t dw1000_set_tx_callback(mp_obj_t self_in, mp_obj_t callback) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    self->tx_callback = callback;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_tx_callback_obj, dw1000_set_tx_callback);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_error_callback()
 *
 * @brief Set callback function for error events
 */
STATIC mp_obj_t dw1000_set_error_callback(mp_obj_t self_in, mp_obj_t callback) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    self->error_callback = callback;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_error_callback_obj, dw1000_set_error_callback);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_timeout_callback()
 *
 * @brief Set callback function for timeout events
 */
STATIC mp_obj_t dw1000_set_timeout_callback(mp_obj_t self_in, mp_obj_t callback) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    self->timeout_callback = callback;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_timeout_callback_obj, dw1000_set_timeout_callback);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_set_auto_rx()
 *
 * @brief Enable/disable automatic RX re-enable mode
 */
STATIC mp_obj_t dw1000_set_auto_rx(mp_obj_t self_in, mp_obj_t enable_obj) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    self->auto_rx = mp_obj_is_true(enable_obj);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_set_auto_rx_obj, dw1000_set_auto_rx);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_process_events()
 *
 * @brief Process DW1000 events using the lab11 driver's ISR - NO manual register access
 */
STATIC mp_obj_t dw1000_process_events(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Set global pointer for C callbacks
    current_dw1000_obj = self;
    
    // Use lab11 driver's ISR - it handles ALL register access, status checking, and recovery
    dwt_isr();
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_process_events_obj, dw1000_process_events);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_micropython_irq_handler()
 *
 * @brief MicroPython-compatible IRQ handler that gets called from Pin.irq() system
 * This provides fast hardware IRQ response while maintaining MicroPython compatibility
 */
STATIC mp_obj_t dw1000_micropython_irq_handler(mp_obj_t pin_obj) {
    // This gets called from MicroPython's Pin.irq() system
    // Immediately process DW1000 events without Python overhead
    if (current_dw1000_obj && current_dw1000_obj->irq_enabled) {
        // Process events directly - this is as fast as direct C IRQ
        dw1000_process_events(current_dw1000_obj);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_micropython_irq_handler_obj, dw1000_micropython_irq_handler);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_enable_irq()
 *
 * @brief Enable hardware IRQ using MicroPython Pin.irq() system for compatibility
 */

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_c_irq_handler()
 *
 * @brief C-level IRQ handler for DW1000 interrupt pin - called directly from hardware interrupt
 * This is much more efficient than Python IRQ handlers!
 */
void dw1000_c_irq_handler(uint gpio, uint32_t events) {
    // Only handle our specific IRQ pin and falling edge events
    if (current_dw1000_obj && 
        current_dw1000_obj->irq_enabled && 
        (events & GPIO_IRQ_EDGE_FALL)) {
        
        // Get the GPIO number from Pin object for comparison
        const machine_pin_obj_t *pin = MP_OBJ_TO_PTR(current_dw1000_obj->irq_pin);
        uint expected_gpio = pin->id;
        
        // Only process if this is our GPIO pin
        if (gpio != expected_gpio) {
            return;
        }
        
        // PolyPoint pattern: Keep calling dwt_isr() until IRQ pin goes low
        // Add escape hatch to prevent infinite loops
        uint8_t count = 0;
        do {
            // Process DW1000 events using lab11 driver
            dwt_isr();
            count++;
        } while (dw1000_irq_pin_read(current_dw1000_obj) && count < 10);
        
        // Note: Python callbacks are called from dwt_isr() → our C callbacks
        // This happens automatically through the lab11 driver callback system
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_enable_irq()
 *
 * @brief Enable IRQ pin interrupt handling for automatic event processing
 */
STATIC mp_obj_t dw1000_enable_irq(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    if (self->irq_pin == MP_OBJ_NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("IRQ pin not configured"));
    }
    
    // For now, just set the internal state
    // The actual Pin.irq() setup will be done in Python using get_irq_handler()
    const machine_pin_obj_t *pin = MP_OBJ_TO_PTR(self->irq_pin);
    uint gpio_pin = pin->id;
    
    // Set global pointer and enable flag
    current_dw1000_obj = self;
    self->irq_enabled = true;
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_enable_irq_obj, dw1000_enable_irq);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_disable_irq()
 *
 * @brief Disable IRQ pin interrupt handling
 */
STATIC mp_obj_t dw1000_disable_irq(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    if (self->irq_pin == MP_OBJ_NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("IRQ pin not configured"));
    }
    
    // Clear flags - the actual Pin.irq(handler=None) will be done in Python
    self->irq_enabled = false;
    if (current_dw1000_obj == self) {
        current_dw1000_obj = NULL;
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_disable_irq_obj, dw1000_disable_irq);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_get_irq_handler()
 *
 * @brief Get the C-level IRQ handler function for use with Pin.irq()
 */
STATIC mp_obj_t dw1000_get_irq_handler(mp_obj_t self_in) {
    return MP_OBJ_FROM_PTR(&dw1000_micropython_irq_handler_obj);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_get_irq_handler_obj, dw1000_get_irq_handler);

/*! ------------------------------------------------------------------------------------------------------------------
 * C callback functions for lab11 driver - called by dwt_isr()
 */

STATIC void c_callback_rx_ok(const dwt_cb_data_t *cb_data) {
    if (!current_dw1000_obj || current_dw1000_obj->rx_callback == mp_const_none) {
        goto auto_rx_check;
    }
    
    // Create frame info dictionary using data provided by lab11 driver
    mp_obj_t frame_info = mp_obj_new_dict(6);
    mp_obj_dict_store(frame_info, MP_OBJ_NEW_QSTR(MP_QSTR_event), MP_OBJ_NEW_QSTR(MP_QSTR_rx_good));
    mp_obj_dict_store(frame_info, MP_OBJ_NEW_QSTR(MP_QSTR_status), mp_obj_new_int_from_uint(cb_data->status));
    
    // Calculate payload length (lab11 datalength includes 2-byte CRC, so subtract it)
    // This matches the logic in read_rx_data() function
    uint16_t payload_len = (cb_data->datalength > 2) ? (cb_data->datalength - 2) : 0;
    mp_obj_dict_store(frame_info, MP_OBJ_NEW_QSTR(MP_QSTR_length), mp_obj_new_int(payload_len));
    
    // Read frame data using lab11 driver function (excluding CRC)
    mp_obj_t data_obj = mp_const_none;
    if (payload_len > 0 && payload_len <= 125) {
        uint8_t *buffer = m_new(uint8_t, payload_len);
        dwt_readrxdata(buffer, payload_len, 0);  // Read only payload, not CRC
        data_obj = mp_obj_new_bytes(buffer, payload_len);
        m_del(uint8_t, buffer, payload_len);
    }
    mp_obj_dict_store(frame_info, MP_OBJ_NEW_QSTR(MP_QSTR_data), data_obj);
    
    // Add timestamps (could use dwt_readrxtimestamp() here)
    mp_obj_dict_store(frame_info, MP_OBJ_NEW_QSTR(MP_QSTR_timestamp), mp_obj_new_int(0));
    mp_obj_dict_store(frame_info, MP_OBJ_NEW_QSTR(MP_QSTR_rx_time_ms), mp_obj_new_int(mp_hal_ticks_ms()));
    
    // Call Python callback
    mp_call_function_1(current_dw1000_obj->rx_callback, frame_info);
    
auto_rx_check:
    // Auto RX re-enable if configured
    if (current_dw1000_obj && current_dw1000_obj->auto_rx) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

STATIC void c_callback_tx_done(const dwt_cb_data_t *cb_data) {
    if (!current_dw1000_obj || current_dw1000_obj->tx_callback == mp_const_none) {
        return;
    }
    
    mp_obj_t tx_info = mp_obj_new_dict(2);
    mp_obj_dict_store(tx_info, MP_OBJ_NEW_QSTR(MP_QSTR_event), MP_OBJ_NEW_QSTR(MP_QSTR_tx_complete));
    mp_obj_dict_store(tx_info, MP_OBJ_NEW_QSTR(MP_QSTR_status), mp_obj_new_int_from_uint(cb_data->status));
    mp_call_function_1(current_dw1000_obj->tx_callback, tx_info);
}

STATIC void c_callback_rx_to(const dwt_cb_data_t *cb_data) {
    if (!current_dw1000_obj || current_dw1000_obj->timeout_callback == mp_const_none) {
        goto auto_rx_check;
    }
    
    mp_call_function_0(current_dw1000_obj->timeout_callback);
    
auto_rx_check:
    // Auto RX re-enable (dwt_isr already did the reset)
    if (current_dw1000_obj && current_dw1000_obj->auto_rx) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

STATIC void c_callback_rx_err(const dwt_cb_data_t *cb_data) {
    if (!current_dw1000_obj || current_dw1000_obj->error_callback == mp_const_none) {
        goto auto_rx_check;
    }
    
    mp_obj_t error_info = mp_obj_new_dict(2);
    mp_obj_dict_store(error_info, MP_OBJ_NEW_QSTR(MP_QSTR_event), MP_OBJ_NEW_QSTR(MP_QSTR_rx_error));
    mp_obj_dict_store(error_info, MP_OBJ_NEW_QSTR(MP_QSTR_status), mp_obj_new_int_from_uint(cb_data->status));
    mp_call_function_1(current_dw1000_obj->error_callback, error_info);
    
auto_rx_check:
    // Auto RX re-enable (dwt_isr already did the reset)
    if (current_dw1000_obj && current_dw1000_obj->auto_rx) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_print()
 *
 * @brief Print function for DW1000 objects
 */
STATIC void dw1000_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "DW1000(spi=%p, cs=%p, rst=%p, irq=%p, init=%s)",
              self->spi, self->cs_pin, self->reset_pin, self->irq_pin,
              self->initialized ? "True" : "False");
}

// Class methods table
STATIC const mp_rom_map_elem_t dw1000_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&dw1000_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&dw1000_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_device_id), MP_ROM_PTR(&dw1000_read_device_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_configure), MP_ROM_PTR(&dw1000_configure_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_frame), MP_ROM_PTR(&dw1000_tx_frame_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_enable), MP_ROM_PTR(&dw1000_rx_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_rx_data), MP_ROM_PTR(&dw1000_read_rx_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_status), MP_ROM_PTR(&dw1000_get_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_force_trx_off), MP_ROM_PTR(&dw1000_force_trx_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_reset), MP_ROM_PTR(&dw1000_rx_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_sync_rx_bufptrs), MP_ROM_PTR(&dw1000_sync_rx_bufptrs_obj) },
    // Diagnostic functions
    { MP_ROM_QSTR(MP_QSTR_get_sys_state), MP_ROM_PTR(&dw1000_get_sys_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_rx_finfo), MP_ROM_PTR(&dw1000_get_rx_finfo_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_register), MP_ROM_PTR(&dw1000_read_register_obj) },
    // Timestamp functions
    { MP_ROM_QSTR(MP_QSTR_read_rx_timestamp), MP_ROM_PTR(&dw1000_read_rx_timestamp_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_tx_timestamp), MP_ROM_PTR(&dw1000_read_tx_timestamp_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_system_timestamp), MP_ROM_PTR(&dw1000_read_system_timestamp_obj) },
    { MP_ROM_QSTR(MP_QSTR_timestamp_to_seconds), MP_ROM_PTR(&dw1000_timestamp_to_seconds_obj) },
    { MP_ROM_QSTR(MP_QSTR_seconds_to_timestamp), MP_ROM_PTR(&dw1000_seconds_to_timestamp_obj) },
    // Ranging functions
    { MP_ROM_QSTR(MP_QSTR_set_delayed_trx_time), MP_ROM_PTR(&dw1000_set_delayed_trx_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_tx_delayed), MP_ROM_PTR(&dw1000_start_tx_delayed_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_frame_delayed), MP_ROM_PTR(&dw1000_tx_frame_delayed_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_ranging_response), MP_ROM_PTR(&dw1000_send_ranging_response_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rx_antenna_delay), MP_ROM_PTR(&dw1000_set_rx_antenna_delay_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_tx_antenna_delay), MP_ROM_PTR(&dw1000_set_tx_antenna_delay_obj) },
    // SPI speed control
    { MP_ROM_QSTR(MP_QSTR_spi_set_baudrate), MP_ROM_PTR(&dw1000_spi_set_baudrate_obj) },
    // Callback functions
    { MP_ROM_QSTR(MP_QSTR_set_rx_callback), MP_ROM_PTR(&dw1000_set_rx_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_tx_callback), MP_ROM_PTR(&dw1000_set_tx_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_error_callback), MP_ROM_PTR(&dw1000_set_error_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_timeout_callback), MP_ROM_PTR(&dw1000_set_timeout_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_process_events), MP_ROM_PTR(&dw1000_process_events_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_auto_rx), MP_ROM_PTR(&dw1000_set_auto_rx_obj) },
    // IRQ functions
    { MP_ROM_QSTR(MP_QSTR_enable_irq), MP_ROM_PTR(&dw1000_enable_irq_obj) },
    { MP_ROM_QSTR(MP_QSTR_disable_irq), MP_ROM_PTR(&dw1000_disable_irq_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_irq_handler), MP_ROM_PTR(&dw1000_get_irq_handler_obj) },
};
STATIC MP_DEFINE_CONST_DICT(dw1000_locals_dict, dw1000_locals_dict_table);

// DW1000 class type using new MicroPython format
MP_DEFINE_CONST_OBJ_TYPE(
    dw1000_type,
    MP_QSTR_DW1000,
    MP_TYPE_FLAG_NONE,
    make_new, dw1000_make_new,
    print, dw1000_print,
    locals_dict, &dw1000_locals_dict
);

// Module constants
STATIC const mp_rom_map_elem_t dw1000_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_dw1000) },
    { MP_ROM_QSTR(MP_QSTR_DW1000), MP_ROM_PTR(&dw1000_type) },
    
    // Constants
        // Module constants
    { MP_ROM_QSTR(MP_QSTR_DEVICE_ID), MP_ROM_INT(0xDECA0130UL) },
    { MP_ROM_QSTR(MP_QSTR_SUCCESS), MP_ROM_INT(DWT_SUCCESS) },
    { MP_ROM_QSTR(MP_QSTR_ERROR), MP_ROM_INT(DWT_ERROR) },
    
    // Data rates
    { MP_ROM_QSTR(MP_QSTR_BR_110K), MP_ROM_INT(DWT_BR_110K) },
    { MP_ROM_QSTR(MP_QSTR_BR_850K), MP_ROM_INT(DWT_BR_850K) },
    { MP_ROM_QSTR(MP_QSTR_BR_6M8), MP_ROM_INT(DWT_BR_6M8) },
    
    // PRF (Pulse Repetition Frequency)
    { MP_ROM_QSTR(MP_QSTR_PRF_16M), MP_ROM_INT(DWT_PRF_16M) },
    { MP_ROM_QSTR(MP_QSTR_PRF_64M), MP_ROM_INT(DWT_PRF_64M) },
    
    // TX Preamble Lengths
    { MP_ROM_QSTR(MP_QSTR_PLEN_64), MP_ROM_INT(DWT_PLEN_64) },
    { MP_ROM_QSTR(MP_QSTR_PLEN_128), MP_ROM_INT(DWT_PLEN_128) },
    { MP_ROM_QSTR(MP_QSTR_PLEN_256), MP_ROM_INT(DWT_PLEN_256) },
    { MP_ROM_QSTR(MP_QSTR_PLEN_512), MP_ROM_INT(DWT_PLEN_512) },
    { MP_ROM_QSTR(MP_QSTR_PLEN_1024), MP_ROM_INT(DWT_PLEN_1024) },
    { MP_ROM_QSTR(MP_QSTR_PLEN_1536), MP_ROM_INT(DWT_PLEN_1536) },
    { MP_ROM_QSTR(MP_QSTR_PLEN_2048), MP_ROM_INT(DWT_PLEN_2048) },
    { MP_ROM_QSTR(MP_QSTR_PLEN_4096), MP_ROM_INT(DWT_PLEN_4096) },
    
    // RX PAC (Preamble Acquisition Chunk) sizes
    { MP_ROM_QSTR(MP_QSTR_PAC8), MP_ROM_INT(DWT_PAC8) },
    { MP_ROM_QSTR(MP_QSTR_PAC16), MP_ROM_INT(DWT_PAC16) },
    { MP_ROM_QSTR(MP_QSTR_PAC32), MP_ROM_INT(DWT_PAC32) },
    { MP_ROM_QSTR(MP_QSTR_PAC64), MP_ROM_INT(DWT_PAC64) },
    
    // PHR Modes
    { MP_ROM_QSTR(MP_QSTR_PHRMODE_STD), MP_ROM_INT(DWT_PHRMODE_STD) },
    { MP_ROM_QSTR(MP_QSTR_PHRMODE_EXT), MP_ROM_INT(DWT_PHRMODE_EXT) },
    
    // Channels
    { MP_ROM_QSTR(MP_QSTR_CHANNEL_1), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL_2), MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL_3), MP_ROM_INT(3) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL_4), MP_ROM_INT(4) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL_5), MP_ROM_INT(5) },
    { MP_ROM_QSTR(MP_QSTR_CHANNEL_7), MP_ROM_INT(7) },
    
    // Status register bits for diagnostics (from deca_regs.h)
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_RXFCG), MP_ROM_INT(SYS_STATUS_RXFCG) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_RXFCE), MP_ROM_INT(SYS_STATUS_RXFCE) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_RXRFTO), MP_ROM_INT(SYS_STATUS_RXRFTO) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_RXOVRR), MP_ROM_INT(SYS_STATUS_RXOVRR) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_RXPHE), MP_ROM_INT(SYS_STATUS_RXPHE) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_RXRFSL), MP_ROM_INT(SYS_STATUS_RXRFSL) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_RXSFDTO), MP_ROM_INT(SYS_STATUS_RXSFDTO) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_LDEERR), MP_ROM_INT(SYS_STATUS_LDEERR) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_AFFREJ), MP_ROM_INT(SYS_STATUS_AFFREJ) },
    
    // Composite error masks (from deca_regs.h)
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_ALL_RX_ERR), MP_ROM_INT(SYS_STATUS_ALL_RX_ERR) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_ALL_RX_GOOD), MP_ROM_INT(SYS_STATUS_ALL_RX_GOOD) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_ALL_RX_TO), MP_ROM_INT(SYS_STATUS_ALL_RX_TO) },
    
    // Register IDs for direct access
    { MP_ROM_QSTR(MP_QSTR_SYS_STATUS_ID), MP_ROM_INT(SYS_STATUS_ID) },
    { MP_ROM_QSTR(MP_QSTR_SYS_STATE_ID), MP_ROM_INT(SYS_STATE_ID) },
    { MP_ROM_QSTR(MP_QSTR_RX_FINFO_ID), MP_ROM_INT(RX_FINFO_ID) },
    
    // Timing constants for ranging
    // DWT_TIME_UNITS: DW1000 hardware clock period (1/(64MHz * 499.2/64 * 128)) = 15.65e-12 seconds
    // Best accessed via timestamp_to_seconds() and seconds_to_timestamp() conversion functions
    // SPEED_OF_LIGHT: For distance calculations (PolyPoint calibrated value: 299,702,547 m/s)
    // Standard physics value is 299,792,458 m/s
    
    // System state masks and values (custom definitions)
    { MP_ROM_QSTR(MP_QSTR_SYS_STATE_RX_STATE), MP_ROM_INT(SYS_STATE_RX_STATE) },
    { MP_ROM_QSTR(MP_QSTR_RX_STATE_IDLE), MP_ROM_INT(RX_STATE_IDLE) },
    { MP_ROM_QSTR(MP_QSTR_RX_STATE_RX_READY), MP_ROM_INT(RX_STATE_RX_READY) },
    { MP_ROM_QSTR(MP_QSTR_RX_STATE_RX_WAIT_SFD), MP_ROM_INT(RX_STATE_RX_WAIT_SFD) },
    { MP_ROM_QSTR(MP_QSTR_RX_STATE_RX_DATA), MP_ROM_INT(RX_STATE_RX_DATA) },
    { MP_ROM_QSTR(MP_QSTR_RX_STATE_RX_VALIDATE), MP_ROM_INT(RX_STATE_RX_VALIDATE) },
};
STATIC MP_DEFINE_CONST_DICT(dw1000_module_globals, dw1000_module_globals_table);

// Module definition
const mp_obj_module_t dw1000_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&dw1000_module_globals,
};

// Register the module
MP_REGISTER_MODULE(MP_QSTR_dw1000, dw1000_module);