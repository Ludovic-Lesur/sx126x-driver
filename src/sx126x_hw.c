/*
 * sx126x_hw.c
 *
 *  Created on: 14 sep. 2025
 *      Author: Ludo
 */

#include "sx126x_hw.h"

#ifndef SX126X_DRIVER_DISABLE_FLAGS_FILE
#include "sx126x_driver_flags.h"
#endif
#include "sx126x.h"
#include "types.h"

#ifndef SX126X_DRIVER_DISABLE

/*** SX126X HW functions ***/

/*******************************************************************/
SX126X_status_t __attribute__((weak)) SX126X_HW_init(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
SX126X_status_t __attribute__((weak)) SX126X_HW_de_init(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
SX126X_status_t __attribute__((weak)) SX126X_HW_set_nreset_gpio(uint8_t state) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    /* To be implemented */
    UNUSED(state);
    return status;
}

/*******************************************************************/
SX126X_status_t __attribute__((weak)) SX126X_HW_wait_busy_low(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
SX126X_status_t __attribute__((weak)) SX126X_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    /* To be implemented */
    UNUSED(tx_data);
    UNUSED(rx_data);
    UNUSED(transfer_size);
    return status;
}

/*******************************************************************/
SX126X_status_t __attribute__((weak)) SX126X_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    /* To be implemented */
    UNUSED(delay_ms);
    return status;
}

#endif /* SX126X_DRIVER_DISABLE */
