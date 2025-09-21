/*
 * sx126x_hw.h
 *
 *  Created on: 14 sep. 2025
 *      Author: Ludo
 */

#ifndef __SX126X_HW_H__
#define __SX126X_HW_H__

#ifndef SX126X_DRIVER_DISABLE_FLAGS_FILE
#include "sx126x_driver_flags.h"
#endif
#include "sx126x.h"
#include "types.h"

#ifndef SX126X_DRIVER_DISABLE

/*** SX126X HW functions ***/

/*!******************************************************************
 * \fn SX126X_status_t SX126X_HW_init(void)
 * \brief Init SX126X hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_HW_init(void);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_HW_de_init(void)
 * \brief Release SX126X hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_HW_de_init(void);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size)
 * \brief Transfer data to transceiver over SPI interface.
 * \param[in]   tx_data: Byte array to send.
 * \param[in]   transfer_size: Number of shorts to send and receive.
 * \param[out]  rx_data: Pointer to the received bytes.
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size);

#endif /* SX126X_DRIVER_DISABLE */

#endif /* __SX126X_HW_H__ */
