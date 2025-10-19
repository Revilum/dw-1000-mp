/*! ----------------------------------------------------------------------------
 * @file    dw1000_hal.h
 * @brief   MicroPython Hardware Abstraction Layer for DW1000 driver
 *
 * This file provides the platform-specific interface required by the 
 * DW1000 driver for MicroPython environments.
 */

#ifndef _DW1000_HAL_H_
#define _DW1000_HAL_H_

#include "py/runtime.h"
#include "py/obj.h"

#ifdef __cplusplus
extern "C" {
#endif

// Include the original DW1000 driver headers
#include "../../dw1000-driver/deca_types.h"
#include "../../dw1000-driver/deca_device_api.h"

// MicroPython-specific HAL functions  
typedef struct _dw1000_obj_t {
    mp_obj_base_t base;
    mp_obj_t spi;           // SPI object
    mp_obj_t cs_pin;        // Chip select pin
    mp_obj_t reset_pin;     // Reset pin (optional)
    mp_obj_t irq_pin;       // IRQ pin (optional)
    bool initialized;       // Initialization state
    
    // Callback function storage
    mp_obj_t rx_callback;      // RX frame callback
    mp_obj_t tx_callback;      // TX complete callback  
    mp_obj_t error_callback;   // Error callback
    mp_obj_t timeout_callback; // Timeout callback
    bool auto_rx;              // Auto RX re-enable mode
    bool irq_enabled;          // IRQ handler enabled state
} dw1000_obj_t;

// HAL function declarations - these implement the platform-specific functions
// required by the DW1000 driver

// SPI communication functions (required by driver)
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);

// Timing functions (required by driver)
void deca_sleep(unsigned int time_ms);

// Interrupt control functions (optional but recommended)
typedef int decaIrqStatus_t;
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

// HAL initialization and configuration
void dw1000_hal_init(dw1000_obj_t *dw1000);
void dw1000_hal_deinit(dw1000_obj_t *dw1000);

// Pin control functions
void dw1000_reset_pin_set(dw1000_obj_t *dw1000, bool state);
bool dw1000_irq_pin_read(dw1000_obj_t *dw1000);

// Global DW1000 object pointer for HAL functions
extern dw1000_obj_t *g_dw1000_obj;

#ifdef __cplusplus
}
#endif

#endif /* _DW1000_HAL_H_ */