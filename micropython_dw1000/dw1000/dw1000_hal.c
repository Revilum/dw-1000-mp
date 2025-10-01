/*! ----------------------------------------------------------------------------
 * @file    dw1000_hal.c
 * @brief   MicroPython Hardware Abstraction Layer implementation for DW1000
 *
 * This file implements the platform-specific functions required by the 
 * DW1000 driver for MicroPython environments.
 */

#include "dw1000_hal.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"
#include "machine_pin.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Need to include the DW1000 driver return codes
#include "../../dw1000-driver/deca_device_api.h"

// Global reference to the current DW1000 object
dw1000_obj_t *g_dw1000_obj = NULL;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn writetospi()
 *
 * @brief Platform-specific SPI write function for MicroPython
 * 
 * This function is called by the DW1000 driver to write data to the SPI bus.
 * It combines the header and body into a single SPI transaction.
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer) {
    if (g_dw1000_obj == NULL || g_dw1000_obj->spi == mp_const_none) {
        return DWT_ERROR;
    }
    
    // Get the SPI protocol structure
    mp_obj_base_t *spi_base = (mp_obj_base_t *)MP_OBJ_TO_PTR(g_dw1000_obj->spi);
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t *)MP_OBJ_TYPE_GET_SLOT(spi_base->type, protocol);
    
    if (spi_p == NULL || spi_p->transfer == NULL) {
        return DWT_ERROR;
    }
    
    // Assert CS (pull low)
    if (g_dw1000_obj->cs_pin != MP_OBJ_NULL) {
        mp_hal_pin_write(mp_hal_get_pin_obj(g_dw1000_obj->cs_pin), 0);
    }
    
    // Write header
    if (headerLength > 0 && headerBuffer != NULL) {
        spi_p->transfer(spi_base, headerLength, headerBuffer, NULL);
    }
    
    // Write body
    if (bodylength > 0 && bodyBuffer != NULL) {
        spi_p->transfer(spi_base, bodylength, bodyBuffer, NULL);
    }
    
    // Deassert CS (pull high)
    if (g_dw1000_obj->cs_pin != MP_OBJ_NULL) {
        mp_hal_pin_write(mp_hal_get_pin_obj(g_dw1000_obj->cs_pin), 1);
    }
    
    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn readfromspi()
 *
 * @brief Platform-specific SPI read function for MicroPython
 * 
 * This function is called by the DW1000 driver to read data from the SPI bus.
 * It first writes the header, then reads the specified amount of data.
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
    if (g_dw1000_obj == NULL || g_dw1000_obj->spi == mp_const_none) {
        return DWT_ERROR;
    }
    
    // Get the SPI protocol structure
    mp_obj_base_t *spi_base = (mp_obj_base_t *)MP_OBJ_TO_PTR(g_dw1000_obj->spi);
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t *)MP_OBJ_TYPE_GET_SLOT(spi_base->type, protocol);
    
    if (spi_p == NULL || spi_p->transfer == NULL) {
        return DWT_ERROR;
    }
    
    // Assert CS (pull low)
    if (g_dw1000_obj->cs_pin != MP_OBJ_NULL) {
        mp_hal_pin_write(mp_hal_get_pin_obj(g_dw1000_obj->cs_pin), 0);
    }
    
    // Send header (write only)
    if (headerLength > 0 && headerBuffer != NULL) {
        spi_p->transfer(spi_base, headerLength, headerBuffer, NULL);
    }
    
    // Read data
    if (readlength > 0 && readBuffer != NULL) {
        // For read operations, we send dummy bytes (0x00) and read the response
        uint8_t dummy_byte = 0x00;
        for (uint32_t i = 0; i < readlength; i++) {
            spi_p->transfer(spi_base, 1, &dummy_byte, &readBuffer[i]);
        }
    }
    
    // Deassert CS (pull high)
    if (g_dw1000_obj->cs_pin != MP_OBJ_NULL) {
        mp_hal_pin_write(mp_hal_get_pin_obj(g_dw1000_obj->cs_pin), 1);
    }
    
    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn deca_sleep()
 *
 * @brief Platform-specific delay function for MicroPython
 * 
 * This function provides the delay functionality required by the DW1000 driver.
 */
void deca_sleep(unsigned int time_ms) {
    mp_hal_delay_ms(time_ms);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn decamutexon()
 *
 * @brief Disable interrupts for critical sections
 * 
 * This function disables interrupts to protect critical sections in the DW1000 driver.
 */
decaIrqStatus_t decamutexon(void) {
    return mp_hal_quiet_timing_enter();
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn decamutexoff()
 *
 * @brief Re-enable interrupts after critical sections
 * 
 * This function re-enables interrupts after critical sections in the DW1000 driver.
 */
void decamutexoff(decaIrqStatus_t s) {
    mp_hal_quiet_timing_exit(s);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_hal_init()
 *
 * @brief Initialize the HAL for the given DW1000 object
 */
void dw1000_hal_init(dw1000_obj_t *dw1000) {
    g_dw1000_obj = dw1000;
    
    // Configure CS pin as output, initially high
    if (dw1000->cs_pin != MP_OBJ_NULL) {
        mp_hal_pin_config(mp_hal_get_pin_obj(dw1000->cs_pin), MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_PULL_NONE, 0);
        mp_hal_pin_write(mp_hal_get_pin_obj(dw1000->cs_pin), 1);
    }
    
    // Configure reset pin as output, initially high
    if (dw1000->reset_pin != MP_OBJ_NULL) {
        mp_hal_pin_config(mp_hal_get_pin_obj(dw1000->reset_pin), MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_PULL_NONE, 0);
        mp_hal_pin_write(mp_hal_get_pin_obj(dw1000->reset_pin), 1);
    }
    
    // Configure IRQ pin as input with pull-up
    if (dw1000->irq_pin != MP_OBJ_NULL) {
        mp_hal_pin_config(mp_hal_get_pin_obj(dw1000->irq_pin), MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_UP, 0);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_hal_deinit()
 *
 * @brief Deinitialize the HAL
 */
void dw1000_hal_deinit(dw1000_obj_t *dw1000) {
    g_dw1000_obj = NULL;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_reset_pin_set()
 *
 * @brief Control the reset pin
 */
void dw1000_reset_pin_set(dw1000_obj_t *dw1000, bool state) {
    if (dw1000->reset_pin != MP_OBJ_NULL) {
        mp_hal_pin_write(mp_hal_get_pin_obj(dw1000->reset_pin), state ? 1 : 0);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dw1000_irq_pin_read()
 *
 * @brief Read the IRQ pin state
 */
bool dw1000_irq_pin_read(dw1000_obj_t *dw1000) {
    if (dw1000->irq_pin != MP_OBJ_NULL) {
        return mp_hal_pin_read(mp_hal_get_pin_obj(dw1000->irq_pin));
    }
    return false;
}