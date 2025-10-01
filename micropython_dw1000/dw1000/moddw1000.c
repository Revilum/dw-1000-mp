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
 * @brief Initialize the DW1000 device
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
    
    // Initialize DW1000 driver
    int result = dwt_initialise(DWT_LOADUCODE);
    if (result != DWT_SUCCESS) {
        mp_raise_OSError(MP_EIO);
        return mp_const_false;
    }
    
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
    
    // Default configuration
    dwt_config_t config = {
        .chan = 2,                  // Channel 2
        .prf = DWT_PRF_64M,        // 64 MHz PRF
        .txPreambLength = DWT_PLEN_128, // 128 preamble length
        .rxPAC = DWT_PAC8,         // PAC size 8
        .txCode = 9,               // TX preamble code
        .rxCode = 9,               // RX preamble code
        .nsSFD = 0,                // Standard SFD
        .dataRate = DWT_BR_6M8,    // 6.8 Mbps
        .phrMode = DWT_PHRMODE_STD, // Standard PHR mode
        .sfdTO = (129 + 8 - 8)     // SFD timeout
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
    // Note: DW1000 functions expect total frame length including frame overhead
    // Trying 4-byte overhead based on receiver analysis
    uint16_t total_frame_len = bufinfo.len + 4;
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
 * @brief Enable receiver
 */
STATIC mp_obj_t dw1000_rx_enable(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Enable receiver
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(dw1000_rx_enable_obj, dw1000_rx_enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_read_rx_data()
 *
 * @brief Read received data
 */
STATIC mp_obj_t dw1000_read_rx_data(mp_obj_t self_in) {
    dw1000_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (!self->initialized) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("DW1000 not initialized"));
    }
    
    // Get frame length (includes frame overhead)
    uint32_t total_frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    
    if (total_frame_len < 4) {
        return mp_obj_new_bytes(NULL, 0);
    }
    
    // Calculate payload length (exclude frame overhead - trying 4 bytes)
    // UWB frames may have more overhead than just 2-byte CRC
    uint32_t payload_len = total_frame_len - 4;
    
    // Allocate buffer and read only the payload data
    uint8_t *buffer = m_new(uint8_t, payload_len);
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
    { MP_ROM_QSTR(MP_QSTR_DEVICE_ID), MP_ROM_INT(DWT_DEVICE_ID) },
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
};
STATIC MP_DEFINE_CONST_DICT(dw1000_module_globals, dw1000_module_globals_table);

// Module definition
const mp_obj_module_t dw1000_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&dw1000_module_globals,
};

// Register the module
MP_REGISTER_MODULE(MP_QSTR_dw1000, dw1000_module);