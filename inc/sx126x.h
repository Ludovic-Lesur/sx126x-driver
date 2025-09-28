/*
 * sx126x.h
 *
 *  Created on: 14 sep. 2025
 *      Author: Ludo
 */

#ifndef __SX126X_H__
#define __SX126X_H__

#ifndef SX126X_DRIVER_DISABLE_FLAGS_FILE
#include "sx126x_driver_flags.h"
#endif
#include "error.h"
#include "types.h"

/*** SX126X macros ***/

#define SX126X_EXIT_RESET_DELAY_MS              50
#define SX126X_CALIBRATION_DELAY_MS             4

#define SX126X_SYNC_WORD_MAXIMUM_SIZE_BYTES     8

#define SX126X_RAMP_UP_DELAY_DBPSK_100BPS       0x370F
#define SX126X_RAMP_UP_DELAY_DBPSK_600BPS       0x092F
#define SX126X_RAMP_DOWN_DELAY_DBPSK_100BPS     0x1D70
#define SX126X_RAMP_DOWN_DELAY_DBPSK_600BPS     0x04E1

/*** SX126X structures ***/

/*!******************************************************************
 * \enum SX126X_status_t
 * \brief SX126X driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    SX126X_SUCCESS = 0,
    SX126X_ERROR_NULL_PARAMETER,
    SX126X_ERROR_BUSY_TIMEOUT,
    SX126X_ERROR_REGULATION_MODE,
    SX126X_ERROR_OSCILLATOR,
    SX126X_ERROR_TCXO_VOLTAGE,
    SX126X_ERROR_TCXO_TIMEOUT,
    SX126X_ERROR_CALIBRATION_TIMEOUT,
    SX126X_ERROR_MODE,
    SX126X_ERROR_RF_FREQUENCY_OVERFLOW,
    SX126X_ERROR_RF_FREQUENCY_UNDERFLOW,
    SX126X_ERROR_MODULATION,
    SX126X_ERROR_MODULATION_SHAPING,
    SX126X_ERROR_MODULATION_NOT_SELECTED,
    SX126X_ERROR_BIT_RATE_OVERFLOW,
    SX126X_ERROR_BIT_RATE_UNDERFLOW,
    SX126X_ERROR_RX_BANDWIDTH,
    SX126X_ERROR_PREAMBLE_DETECTOR_LENGTH,
    SX126X_ERROR_SYNC_WORD_LENGTH,
    SX126X_ERROR_IRQ_INDEX,
    SX126X_ERROR_RF_OUTPUT_POWER_OVERFLOW,
    SX126X_ERROR_RF_OUTPUT_POWER_UNDERFLOW,
    SX126X_ERROR_PA_RAMP_TIME,
    SX126X_ERROR_LNA_MODE,
    SX126X_ERROR_RSSI_TYPE,
    SX126X_ERROR_RX_PAYLOAD_SIZE,
    // Low level drivers errors.
    SX126X_ERROR_BASE_SPI = ERROR_BASE_STEP,
    SX126X_ERROR_BASE_DELAY = (SX126X_ERROR_BASE_SPI + SX126X_DRIVER_SPI_ERROR_BASE_LAST),
    // Last base value.
    SX126X_ERROR_BASE_LAST = (SX126X_ERROR_BASE_DELAY + SX126X_DRIVER_DELAY_ERROR_BASE_LAST)
} SX126X_status_t;

#ifndef SX126X_DRIVER_DISABLE

/*!******************************************************************
 * \enum SX126X_regulation_mode_t
 * \brief SX126X power regulation modes.
 *******************************************************************/
typedef enum {
    SX126X_REGULATION_MODE_LDO = 0,
    SX126X_REGULATION_MODE_DCDC,
    SX126X_REGULATION_MODE_LAST
} SX126X_regulation_mode_t;

/*!******************************************************************
 * \enum SX126X_oscillator_t
 * \brief SX126X external oscillator type.
 *******************************************************************/
typedef enum {
    SX126X_OSCILLATOR_QUARTZ = 0x00,
    SX126X_OSCILLATOR_TCXO,
    SX126X_OSCILLATOR_LAST
} SX126X_oscillator_t;

/*!******************************************************************
 * \enum SX126X_tcxo_voltage_t
 * \brief SX126X TCXO control voltages list.
 *******************************************************************/
typedef enum {
    SX126X_TCXO_VOLTAGE_1V6 = 0,
    SX126X_TCXO_VOLTAGE_1V7,
    SX126X_TCXO_VOLTAGE_1V8,
    SX126X_TCXO_VOLTAGE_2V2,
    SX126X_TCXO_VOLTAGE_2V4,
    SX126X_TCXO_VOLTAGE_2V7,
    SX126X_TCXO_VOLTAGE_3V0,
    SX126X_TCXO_VOLTAGE_3V3,
    SX126X_TCXO_VOLTAGE_LAST
} SX126X_tcxo_voltage_t;

/*!******************************************************************
 * \enum SX126X_mode_t
 * \brief SX126X transceiver modes.
 *******************************************************************/
typedef enum {
    SX126X_MODE_SLEEP,
    SX126X_MODE_STANDBY_RC,
    SX126X_MODE_STANDBY_XOSC,
    SX126X_MODE_FS,
    SX126X_MODE_TX,
    SX126X_MODE_TX_CW,
    SX126X_MODE_RX,
    SX126X_MODE_LAST
} SX126X_mode_t;

/*!******************************************************************
 * \enum SX126X_modulation_t
 * \brief SX126X modulations list.
 *******************************************************************/
typedef enum {
    SX126X_MODULATION_GFSK,
    SX126X_MODULATION_BPSK,
    SX126X_MODULATION_LAST
} SX126X_modulation_t;

/*!******************************************************************
 * \enum SX126X_modulation_shaping_t
 * \brief SX126X modulations shaping list.
 *******************************************************************/
typedef enum {
    SX126X_MODULATION_SHAPING_NONE,
    SX126X_MODULATION_SHAPING_GAUSSIAN_BT_03,
    SX126X_MODULATION_SHAPING_GAUSSIAN_BT_05,
    SX126X_MODULATION_SHAPING_GAUSSIAN_BT_07,
    SX126X_MODULATION_SHAPING_GAUSSIAN_BT_1,
    SX126X_MODULATION_SHAPING_DBPSK,
    SX126X_MODULATION_SHAPING_LAST
} SX126X_modulation_shaping_t;

/*!******************************************************************
 * \enum SX126X_rxbw_mantissa_t
 * \brief SX126X RX bandwidth values.
 *******************************************************************/
typedef enum {
    SX126X_RXBW_4800HZ,
    SX126X_RXBW_5800HZ,
    SX126X_RXBW_7300HZ,
    SX126X_RXBW_9700HZ,
    SX126X_RXBW_11700HZ,
    SX126X_RXBW_14600HZ,
    SX126X_RXBW_19500HZ,
    SX126X_RXBW_23400HZ,
    SX126X_RXBW_29300HZ,
    SX126X_RXBW_39000HZ,
    SX126X_RXBW_46900HZ,
    SX126X_RXBW_58600HZ,
    SX126X_RXBW_78200HZ,
    SX126X_RXBW_93800HZ,
    SX126X_RXBW_117300HZ,
    SX126X_RXBW_156200HZ,
    SX126X_RXBW_187200HZ,
    SX126X_RXBW_234300HZ,
    SX126X_RXBW_312000HZ,
    SX126X_RXBW_373600HZ,
    SX126X_RXBW_467000HZ,
    SX126X_RXBW_LAST
} SX126X_rxbw_t;

/*!******************************************************************
 * \enum SX126X_irq_index_t
 * \brief SX126X internal interrupts list.
 *******************************************************************/
typedef enum {
    SX126X_IRQ_INDEX_TX_DONE = 0,
    SX126X_IRQ_INDEX_RX_DONE,
    SX126X_IRQ_INDEX_PREAMBLE_DETECTED,
    SX126X_IRQ_INDEX_SYNC_WORD_VALID,
    SX126X_IRQ_INDEX_HEADER_VALID,
    SX126X_IRQ_INDEX_HEADER_ERROR,
    SX126X_IRQ_INDEX_CRC_ERROR,
    SX126X_IRQ_INDEX_CAD_DONE,
    SX126X_IRQ_INDEX_CAD_DETECTED,
    SX126X_IRQ_INDEX_TIMEOUT,
    SX126X_IRQ_INDEX_LAST
} SX126X_irq_index_t;

/*!******************************************************************
 * \enum SX126X_preamble_detector_length_t
 * \brief SX126X preamble detector length list.
 *******************************************************************/
typedef enum {
    SX126X_PREAMBLE_DETECTOR_LENGTH_OFF = 0,
    SX126X_PREAMBLE_DETECTOR_LENGTH_8BITS,
    SX126X_PREAMBLE_DETECTOR_LENGTH_16BITS,
    SX126X_PREAMBLE_DETECTOR_LENGTH_24BITS,
    SX126X_PREAMBLE_DETECTOR_LENGTH_32BITS,
    SX126X_PREAMBLE_DETECTOR_LENGTH_LAST
} SX126X_preamble_detector_length_t;

/*!******************************************************************
 * \enum SX126X_pa_ramp_time_t
 * \brief SX126X power amplifier ramp time list.
 *******************************************************************/
typedef enum {
    SX126X_PA_RAMP_TIME_10U = 0,
    SX126X_PA_RAMP_TIME_20U,
    SX126X_PA_RAMP_TIME_40U,
    SX126X_PA_RAMP_TIME_80U,
    SX126X_PA_RAMP_TIME_200U,
    SX126X_PA_RAMP_TIME_800U,
    SX126X_PA_RAMP_TIME_1700U,
    SX126X_PA_RAMP_TIME_3400U,
    SX126X_PA_RAMP_TIME_LAST
} SX126X_pa_ramp_time_t;

/*!******************************************************************
 * \enum SX126X_lna_mode_t
 * \brief SX126X internal LNA modes list.
 *******************************************************************/
typedef enum {
    SX126X_LNA_MODE_NORMAL = 0,
    SX126X_LNA_MODE_BOOST,
    SX126X_LNA_MODE_LAST
} SX126X_lna_mode_t;

/*!******************************************************************
 * \enum SX126X_rssi_t
 * \brief SX126X RSSI measurement methods.
 *******************************************************************/
typedef enum {
    SX126X_RSSI_TYPE_INSTANTANEOUS,
    SX126X_RSSI_TYPE_SYNC_WORD,
    SX126X_RSSI_TYPE_AVERAGED,
    SX126X_RSSI_TYPE_LAST
} SX126X_rssi_t;

/*!******************************************************************
 * \struct SX126X_modulation_parameters_t
 * \brief SX126X modulation parameters structure.
 *******************************************************************/
typedef struct {
    SX126X_modulation_t modulation;
    SX126X_modulation_shaping_t modulation_shaping;
    uint32_t fsk_deviation_hz;
    uint32_t bit_rate_bps;
    SX126X_rxbw_t rx_bandwidth;
} SX126X_modulation_parameters_t;

/*!******************************************************************
 * \struct SX126X_gfsk_packet_parameters_t
 * \brief SX126X GFSK packet parameters structure.
 *******************************************************************/
typedef struct {
    uint16_t preamble_length_bits;
    SX126X_preamble_detector_length_t preamble_detector_length;
    uint8_t sync_word[SX126X_SYNC_WORD_MAXIMUM_SIZE_BYTES];
    uint8_t sync_word_length_bits;
    uint8_t payload_length_bytes;
} SX126X_gfsk_packet_parameters_t;

/*!******************************************************************
 * \struct SX126X_bpsk_packet_parameters_t
 * \brief SX126X BPSK packet parameters structure.
 *******************************************************************/
typedef struct {
    uint8_t payload_length_bytes;
    uint16_t payload_length_bits;
    uint16_t ramp_up_delay;
    uint16_t ramp_down_delay;
} SX126X_bpsk_packet_parameters_t;

/*** SX126X functions ***/

/*!******************************************************************
 * \fn SX126X_status_t SX126X_init(void)
 * \brief Init SX126X driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_init(void);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_de_init(void)
 * \brief Release SX126X driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_de_init(void);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_reset(uint8_t reset_enable)
 * \brief Control SX126X NRESET pin.
 * \param[in]   reset_enable: 0 to release chip, any other value to reset.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_reset(uint8_t reset_enable);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_get_device_errors(uint16_t* op_error)
 * \brief Read SX126X internal errors.
 * \param[in]   none
 * \param[out]  op_error: Pointer to the errors bitfield.
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_get_device_errors(uint16_t* op_error);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_clear_device_errors(void)
 * \brief Clear SX126X internal errors.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_clear_device_errors(void);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_regulation_mode(SX126X_regulation_mode_t regulation_mode)
 * \brief Set SX126X regulation mode.
 * \param[in]   regulation_mode: Power regulation mode to select.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_regulation_mode(SX126X_regulation_mode_t regulation_mode);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_oscillator(SX126X_oscillator_t oscillator, SX126X_tcxo_voltage_t tcxo_voltage, uint32_t tcxo_timeout_ms)
 * \brief Set SX126X oscillator type.
 * \param[in]   oscillator: Oscillator type.
 * \param[in]   tcxo_voltage: TCXO control voltage on DIO3.
 * \param[in]   tcxo_timeout_ms: TCXO startup timeout in ms.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_oscillator(SX126X_oscillator_t oscillator, SX126X_tcxo_voltage_t tcxo_voltage, uint32_t tcxo_timeout_ms);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_calibrate(uint16_t frequency_range_low_mhz, uint16_t frequency_range_high_mhz)
 * \brief Calibrate all SX126X internal blocks.
 * \param[in]   frequency_range_low_mhz: Minimum frequency of the operating band in MHz.
 * \param[in]   frequency_range_high_mhz: Maximum frequency of the operating band in MHz
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_calibrate(uint16_t frequency_range_low_mhz, uint16_t frequency_range_high_mhz);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_mode(SX126X_mode_t mode)
 * \brief Set SX126X state.
 * \param[in]   mode: New mode to switch to.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_mode(SX126X_mode_t mode);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_rf_frequency(uint32_t rf_frequency_hz)
 * \brief Set SX126X RF center frequency.
 * \param[in]   rf_frequency_hz: Center frequency to set in Hz.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_rf_frequency(uint32_t rf_frequency_hz);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_modulation(SX126X_modulation_parameters_t* modulation_parameters)
 * \brief Configure SX126X modulation scheme.
 * \param[in]   modulation_parameters: Pointer to the modulation parameters to set.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_modulation(SX126X_modulation_parameters_t* modulation_parameters);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_gfsk_packet(SX126X_gfsk_packet_parameters_t* packet_parameters)
 * \brief Set SX126X GFSK packet parameters.
 * \param[in]   packet_parameters: Pointer to the packet parameters structure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_gfsk_packet(SX126X_gfsk_packet_parameters_t* packet_parameters);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_bpsk_packet(SX126X_bpsk_packet_parameters_t* packet_parameters)
 * \brief Set SX126X BPSK packet parameters.
 * \param[in]   packet_parameters: Pointer to the packet parameters structure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_bpsk_packet(SX126X_bpsk_packet_parameters_t* packet_parameters);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_dio_irq_mask(uint16_t irq_mask, uint16_t irq_mask_dio1, uint16_t irq_mask_dio2, uint16_t irq_mask_dio3)
 * \brief Configure SX126X interrupts and GPIO.
 * \param[in]   irq_mask: Global interrupts mask.
 * \param[in]   irq_mask_dio1: Interrupts mask attached to DIO1.
 * \param[in]   irq_mask_dio2: Interrupts mask attached to DIO2.
 * \param[in]   irq_mask_dio3: Interrupts mask attached to DIO3.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_dio_irq_mask(uint16_t irq_mask, uint16_t irq_mask_dio1, uint16_t irq_mask_dio2, uint16_t irq_mask_dio3);

/*!******************************************************************
 * \fn SX126X_status_t SX126X_get_irq_flag(SX126X_irq_index_t irq_index, uint8_t* irq_flag)
 * \brief Read SX126X internal interrupt status.
 * \param[in]   irq_index: Interrupt index.
 * \param[out]  irq_flag: Pointer to bit that will contain interrupt status.
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_get_irq_flag(SX126X_irq_index_t irq_index, uint8_t* irq_flag);

#ifdef SX126X_DRIVER_TX_ENABLE
/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_rf_output_power(int8_t rf_output_power_dbm, SX126X_pa_ramp_time_t pa_ramp_time)
 * \brief Set SX126X RF output power.
 * \param[in]   rf_output_power_dbm: RF output power in dBm.
 * \param[in]   pa_ramp_time: PA ramp time.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_rf_output_power(int8_t rf_output_power_dbm, SX126X_pa_ramp_time_t pa_ramp_time);
#endif

#ifdef SX126X_DRIVER_TX_ENABLE
/*!******************************************************************
 * \fn SX126X_status_t SX126X_differential_encoding(uint8_t* data_in, uint8_t data_in_size_bytes, uint8_t* data_out, uint8_t* data_out_size_bytes, uint16_t* data_out_size_bits)
 * \brief Convert a payload buffer using differential encoding.
 * \param[in]   data_in: Input byte array.
 * \param[in]   data_in_size_bytes: Input data size in bytes.
 * \param[out]  data_out: Pointer to the output data.
 * \param[out]  data_out_size_bytes: Pointer to the output data size in bytes.
 * \param[out]  data_out_size_bits: Pointer to the output data size in bits.
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_differential_encoding(uint8_t* data_in, uint8_t data_in_size_bytes, uint8_t* data_out, uint8_t* data_out_size_bytes, uint16_t* data_out_size_bits);
#endif

#ifdef SX126X_DRIVER_TX_ENABLE
/*!******************************************************************
 * \fn SX126X_status_t SX126X_write_fifo(uint8_t* tx_data, uint8_t tx_data_size)
 * \brief Write SX126X TX buffer.
 * \param[in]   tx_data: Byte array to write in FIFO.
 * \param[in]   tx_data_size: Number of bytes to write.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_write_fifo(uint8_t* tx_data, uint8_t tx_data_size);
#endif

#ifdef SX126X_DRIVER_RX_ENABLE
/*!******************************************************************
 * \fn SX126X_status_t SX126X_set_lna_mode(SX126X_lna_mode_t lna_mode)
 * \brief Configure SX126X internal LNA.
 * \param[in]   lna_mode: Internal LNA mode.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_set_lna_mode(SX126X_lna_mode_t lna_mode);
#endif

#ifdef SX126X_DRIVER_RX_ENABLE
/*!******************************************************************
 * \fn SX126X_status_t SX126X_get_rssi(SX126X_rssi_t rssi_type, int16_t* rssi_dbm)
 * \brief Get SX126X RX RSSI.
 * \param[in]   rssi_type: RSSI type to read.
 * \param[out]  rssi_dbm: Pointer to signed 16-bits value that will contain the RSSI in dBm.
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_get_rssi(SX126X_rssi_t rssi_type, int16_t* rssi_dbm);
#endif

#ifdef SX126X_DRIVER_RX_ENABLE
/*!******************************************************************
 * \fn SX126X_status_t SX126X_read_fifo(uint8_t* rx_data, uint8_t rx_data_size)
 * \brief Read SX126X FIFO.
 * \param[in]   rx_data_size: Number of bytes to read.
 * \param[out]  rx_data: Byte array that will contain the RX FIFO bytes.
 * \retval      Function execution status.
 *******************************************************************/
SX126X_status_t SX126X_read_fifo(uint8_t* rx_data, uint8_t rx_data_size);
#endif

/*******************************************************************/
#define SX126X_exit_error(base) { ERROR_check_exit(sx126x_status, SX126X_SUCCESS, base) }

/*******************************************************************/
#define SX126X_stack_error(base) { ERROR_check_stack(sx126x_status, SX126X_SUCCESS, base) }

/*******************************************************************/
#define SX126X_stack_exit_error(base, code) { ERROR_check_stack_exit(sx126x_status, SX126X_SUCCESS, base, code) }

#endif /* SX126X_DRIVER_DISABLE */

#endif /* __SX126X_H__ */
