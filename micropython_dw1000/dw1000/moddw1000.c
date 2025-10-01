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
#include <string.h>
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"

// Define STATIC if not already defined
#ifndef STATIC
#define STATIC static
#endif

#include "dw1000_hal.h"

// Include the DW1000 driver
#include "../../dw1000-driver/deca_device_api.h"
#include "../../dw1000-driver/deca_regs.h"

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
    
    return MP_OBJ_FROM_PTR(self);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_init()
 *
 * @brief Initialize the DW1000 device with comprehensive setup (based on Polypoint)
 */
STATIC mp_obj_t dw1000_init(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (self->initialized) {
        return mp_const_true;
    }
    
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

    self->initialized = true;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_init_obj, dw1000_init);

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
 * @brief Configure the DW1000 device
 */
STATIC mp_obj_t dw1000_configure(size_t n_args, const mp_obj_t *args) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Default configuration (matching dw1000_init for consistency)
    dwt_config_t config = {
        .chan = 5,                  // Channel 5 (consistent with dw1000_init)
        .prf = DWT_PRF_64M,        // 64 MHz PRF
        .txPreambLength = DWT_PLEN_128, // 128 preamble length
        .rxPAC = DWT_PAC8,         // PAC size 8
        .txCode = 9,               // TX preamble code
        .rxCode = 9,               // RX preamble code
        .nsSFD = 1,                // Non-standard SFD (matching dw1000_init)
        .dataRate = DWT_BR_6M8,    // 6.8 Mbps
        .phrMode = DWT_PHRMODE_STD, // Standard PHR mode
        .sfdTO = (128 + 1 + 8 - 8) // SFD timeout (matching dw1000_init)
    };
    
    // Override with provided arguments if any
    if (n_args > 1) {
        // Parse configuration dictionary if provided
        if (mp_obj_is_type(args[1], &mp_type_dict)) {
            mp_obj_dict_t *dict = MP_OBJ_TO_PTR(args[1]);
            
            // Extract configuration parameters from dictionary
            mp_obj_t value = mp_obj_dict_get(MP_OBJ_FROM_PTR(dict), MP_OBJ_NEW_QSTR(MP_QSTR_channel));
            if (value != MP_OBJ_NULL) {
                config.chan = mp_obj_get_int(value);
            }
            value = mp_obj_dict_get(MP_OBJ_FROM_PTR(dict), MP_OBJ_NEW_QSTR(MP_QSTR_data_rate));
            if (value != MP_OBJ_NULL) {
                config.dataRate = mp_obj_get_int(value);
            }
            // Add more configuration parameters as needed
        }
    }
    
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
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(dw1000_configure_obj, 1, 2, dw1000_configure);

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
    uint32_t value = dwt_read32bitreg(reg_id);
    return mp_obj_new_int_from_uint(value);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dw1000_read_register_obj, dw1000_read_register);

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