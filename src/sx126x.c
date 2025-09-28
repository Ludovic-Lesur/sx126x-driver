/*
 * sx126x.c
 *
 *  Created on: 14 sep. 2025
 *      Author: Ludo
 */

#include "sx126x.h"

#ifndef SX126X_DRIVER_DISABLE_FLAGS_FILE
#include "sx126x_driver_flags.h"
#endif
#include "sx126x_hw.h"
#include "types.h"

#ifndef SX126X_DRIVER_DISABLE

/*** SX126X local macros ***/

#define SX126X_REGISTER_ADDRESS_BPSK_PACKET 0x00F0
#define SX126X_REGISTER_ADDRESS_SYNC_WORD   0x06C0
#define SX126X_REGISTER_ADDRESS_RX_GAIN     0x08AC

#define SX126X_TCXO_TIMEOUT_DELAY_STEP_NS   15625
#define SX126X_TCXO_TIMEOUT_MIN_MS          1
#define SX126X_TCXO_TIMEOUT_MAX_MS          262000

#define SX126X_BPSK_PACKET_REGISTER_SIZE    6

#define SX126X_MODE_COMMAND_SIZE            4

#define SX126X_BIT_RATE_BPS_MIN             70
#define SX126X_BIT_RATE_BPS_MAX             500000

#define SX126X_RF_FREQUENCY_HZ_MIN          150000000
#define SX126X_RF_FREQUENCY_HZ_MAX          960000000

#define SX126X_IMAGE_CALIBRATION_STEP_MHZ   4

#ifdef SX126X_DRIVER_DEVICE_SX1262
#define SX126X_OUTPUT_POWER_MIN             (-9)
#define SX126X_OUTPUT_POWER_MAX             22
#else
#define SX126X_OUTPUT_POWER_MIN             (-17)
#define SX126X_OUTPUT_POWER_MAX             15
#endif

/*** SX126X local structures ***/

/*******************************************************************/
typedef enum {
    // NOP.
    SX126X_OP_CODE_NOP = 0x00,
    // Operational Modes Functions
    SX126X_OP_CODE_SET_SLEEP = 0x84,
    SX126X_OP_CODE_SET_STANDBY = 0x80,
    SX126X_OP_CODE_SET_FS = 0xC1,
    SX126X_OP_CODE_SET_TX = 0x83,
    SX126X_OP_CODE_SET_RX = 0x82,
    SX126X_OP_CODE_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_OP_CODE_SET_RX_DUTY_CYCLE = 0x94,
    SX126X_OP_CODE_SET_CAD = 0xC5,
    SX126X_OP_CODE_SET_TX_CONTINUOUS_WAVE = 0xD1,
    SX126X_OP_CODE_SET_TX_INFINITE_PREAMBLE = 0xD2,
    SX126X_OP_CODE_SET_REGULATOR_MODE = 0x96,
    SX126X_OP_CODE_CALIBRATE = 0x89,
    SX126X_OP_CODE_CALIBRATE_IMAGE = 0x98,
    SX126X_OP_CODE_SET_PA_CONFIG = 0x95,
    SX126X_OP_CODE_SET_RX_TX_FALLBACK_MODE = 0x93,
    // Registers and buffer Access
    SX126X_OP_CODE_WRITE_REGISTER = 0x0D,
    SX126X_OP_CODE_READ_REGISTER = 0x1D,
    SX126X_OP_CODE_WRITE_BUFFER = 0x0E,
    SX126X_OP_CODE_READ_BUFFER = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_OP_CODE_SET_DIO_IRQ_PARAMS = 0x08,
    SX126X_OP_CODE_GET_IRQ_STATUS = 0x12,
    SX126X_OP_CODE_CLEAR_IRQ_STATUS = 0x02,
    SX126X_OP_CODE_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_OP_CODE_SET_DIO3_AS_TCXO_CTRL = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_OP_CODE_SET_RF_FREQUENCY = 0x86,
    SX126X_OP_CODE_SET_PACKET_TYPE = 0x8A,
    SX126X_OP_CODE_GET_PACKET_TYPE = 0x11,
    SX126X_OP_CODE_SET_TX_PARAMS = 0x8E,
    SX126X_OP_CODE_SET_MODULATION_PARAMS = 0x8B,
    SX126X_OP_CODE_SET_PACKET_PARAMS = 0x8C,
    SX126X_OP_CODE_SET_CAD_PARAMS = 0x88,
    SX126X_OP_CODE_SET_BUFFER_BASE_ADDRESS = 0x8F,
    SX126X_OP_CODE_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_OP_CODE_GET_STATUS = 0xC0,
    SX126X_OP_CODE_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_OP_CODE_GET_PACKET_STATUS = 0x14,
    SX126X_OP_CODE_GET_RSSI_INST = 0x15,
    SX126X_OP_CODE_GET_STATS = 0x10,
    SX126X_OP_CODE_RESET_STATS = 0x00,
    // Miscellaneous
    SX126X_OP_CODE_GET_DEVICE_ERRORS = 0x17,
    SX126X_OP_CODE_CLEAR_DEVICE_ERRORS = 0x07,
} SX126X_op_code_t;

/*******************************************************************/
typedef enum {
    // Operational Modes Functions
    SX126X_COMMAND_SIZE_SET_SLEEP = 2,
    SX126X_COMMAND_SIZE_SET_STANDBY = 2,
    SX126X_COMMAND_SIZE_SET_FS = 1,
    SX126X_COMMAND_SIZE_SET_TX = 4,
    SX126X_COMMAND_SIZE_SET_RX = 4,
    SX126X_COMMAND_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    SX126X_COMMAND_SIZE_SET_RX_DUTY_CYCLE = 7,
    SX126X_COMMAND_SIZE_SET_CAD = 1,
    SX126X_COMMAND_SIZE_SET_TX_CONTINUOUS_WAVE = 1,
    SX126X_COMMAND_SIZE_SET_TX_INFINITE_PREAMBLE = 1,
    SX126X_COMMAND_SIZE_SET_REGULATOR_MODE = 2,
    SX126X_COMMAND_SIZE_CALIBRATE = 2,
    SX126X_COMMAND_SIZE_CALIBRATE_IMAGE = 3,
    SX126X_COMMAND_SIZE_SET_PA_CONFIG = 5,
    SX126X_COMMAND_SIZE_SET_RX_TX_FALLBACK_MODE = 2,
    // Registers and buffer Access
    SX126X_COMMAND_SIZE_WRITE_REGISTER = 4,
    SX126X_COMMAND_SIZE_READ_REGISTER = 5,
    SX126X_COMMAND_SIZE_WRITE_BUFFER = 2,
    SX126X_COMMAND_SIZE_READ_BUFFER = 2,
    // DIO and IRQ Control Functions
    SX126X_COMMAND_SIZE_SET_DIO_IRQ_PARAMS = 9,
    SX126X_COMMAND_SIZE_GET_IRQ_STATUS = 4,
    SX126X_COMMAND_SIZE_CLEAR_IRQ_STATUS = 3,
    SX126X_COMMAND_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    SX126X_COMMAND_SIZE_SET_DIO3_AS_TCXO_CTRL = 5,
    // RF Modulation and Packet-Related Functions
    SX126X_COMMAND_SIZE_SET_RF_FREQUENCY = 5,
    SX126X_COMMAND_SIZE_SET_PACKET_TYPE = 2,
    SX126X_COMMAND_SIZE_GET_PACKET_TYPE = 2,
    SX126X_COMMAND_SIZE_SET_TX_PARAMS = 3,
    SX126X_COMMAND_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    SX126X_COMMAND_SIZE_SET_MODULATION_PARAMS_BPSK = 5,
    SX126X_COMMAND_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    SX126X_COMMAND_SIZE_SET_PACKET_PARAMS_GFSK = 10,
    SX126X_COMMAND_SIZE_SET_PACKET_PARAMS_BPSK = 2,
    SX126X_COMMAND_SIZE_SET_PACKET_PARAMS_LORA = 7,
    SX126X_COMMAND_SIZE_SET_CAD_PARAMS = 8,
    SX126X_COMMAND_SIZE_SET_BUFFER_BASE_ADDRESS = 3,
    SX126X_COMMAND_SIZE_SET_LORA_SYMB_NUM_TIMEOUT = 2,
    // Communication Status Information
    SX126X_COMMAND_SIZE_GET_STATUS = 1,
    SX126X_COMMAND_SIZE_GET_RX_BUFFER_STATUS = 4,
    SX126X_COMMAND_SIZE_GET_PACKET_STATUS = 5,
    SX126X_COMMAND_SIZE_GET_RSSI_INST = 3,
    SX126X_COMMAND_SIZE_GET_STATS = 2,
    SX126X_COMMAND_SIZE_RESET_STATS = 7,
    // Miscellaneous
    SX126X_COMMAND_SIZE_GET_DEVICE_ERRORS = 4,
    SX126X_COMMAND_SIZE_CLEAR_DEVICE_ERRORS = 3,
} SX126X_command_size_t;

/*******************************************************************/
typedef struct {
    int8_t power;
    uint8_t pa_duty_cycle;
#ifdef SX126X_DRIVER_DEVICE_SX1262
    uint8_t hp_max;
#endif
} SX126X_pa_config_t;

/*** SX126X local global variables ***/

#ifdef SX126X_DRIVER_TX_ENABLE
static const SX126X_pa_config_t SX126X_PA_CONFIG[SX126X_OUTPUT_POWER_MAX - SX126X_OUTPUT_POWER_MIN + 1] = {
#ifdef SX126X_DRIVER_DEVICE_SX1262
    { .power = 3,  .hp_max = 0x01, .pa_duty_cycle = 0x01 },
    { .power = 4,  .hp_max = 0x01, .pa_duty_cycle = 0x01 },
    { .power = 7,  .hp_max = 0x01, .pa_duty_cycle = 0x00 },
    { .power = 7,  .hp_max = 0x01, .pa_duty_cycle = 0x01 },
    { .power = 9,  .hp_max = 0x01, .pa_duty_cycle = 0x04 },
    { .power = 10, .hp_max = 0x01, .pa_duty_cycle = 0x00 },
    { .power = 8,  .hp_max = 0x01, .pa_duty_cycle = 0x03 },
    { .power = 12, .hp_max = 0x01, .pa_duty_cycle = 0x00 },
    { .power = 15, .hp_max = 0x01, .pa_duty_cycle = 0x00 },
    { .power = 14, .hp_max = 0x01, .pa_duty_cycle = 0x01 },
    { .power = 20, .hp_max = 0x01, .pa_duty_cycle = 0x00 },
    { .power = 19, .hp_max = 0x01, .pa_duty_cycle = 0x02 },
    { .power = 20, .hp_max = 0x01, .pa_duty_cycle = 0x01 },
    { .power = 21, .hp_max = 0x01, .pa_duty_cycle = 0x00 },
    { .power = 18, .hp_max = 0x02, .pa_duty_cycle = 0x00 },
    { .power = 22, .hp_max = 0x01, .pa_duty_cycle = 0x00 },
    { .power = 22, .hp_max = 0x01, .pa_duty_cycle = 0x01 },
    { .power = 22, .hp_max = 0x01, .pa_duty_cycle = 0x02 },
    { .power = 22, .hp_max = 0x01, .pa_duty_cycle = 0x03 },
    { .power = 22, .hp_max = 0x01, .pa_duty_cycle = 0x04 },
    { .power = 22, .hp_max = 0x02, .pa_duty_cycle = 0x00 },
    { .power = 22, .hp_max = 0x02, .pa_duty_cycle = 0x01 },
    { .power = 22, .hp_max = 0x02, .pa_duty_cycle = 0x02 },
    { .power = 22, .hp_max = 0x02, .pa_duty_cycle = 0x03 },
    { .power = 22, .hp_max = 0x03, .pa_duty_cycle = 0x01 },
    { .power = 22, .hp_max = 0x03, .pa_duty_cycle = 0x02 },
    { .power = 22, .hp_max = 0x05, .pa_duty_cycle = 0x00 },
    { .power = 22, .hp_max = 0x05, .pa_duty_cycle = 0x01 },
    { .power = 22, .hp_max = 0x04, .pa_duty_cycle = 0x04 },
    { .power = 22, .hp_max = 0x06, .pa_duty_cycle = 0x03 },
    { .power = 22, .hp_max = 0x07, .pa_duty_cycle = 0x03 },
    { .power = 22, .hp_max = 0x07, .pa_duty_cycle = 0x04 },
#else
    { .power = -15, .pa_duty_cycle = 0x01 },
    { .power = -13, .pa_duty_cycle = 0x00 },
    { .power = -12, .pa_duty_cycle = 0x00 },
    { .power = -12, .pa_duty_cycle = 0x01 },
    { .power = -12, .pa_duty_cycle = 0x03 },
    { .power = -10, .pa_duty_cycle = 0x02 },
    { .power = -10, .pa_duty_cycle = 0x04 },
    { .power = -7,  .pa_duty_cycle = 0x00 },
    { .power = -8,  .pa_duty_cycle = 0x03 },
    { .power = -7,  .pa_duty_cycle = 0x03 },
    { .power = -5,  .pa_duty_cycle = 0x02 },
    { .power = -4,  .pa_duty_cycle = 0x02 },
    { .power = -3,  .pa_duty_cycle = 0x02 },
    { .power = 0,   .pa_duty_cycle = 0x00 },
    { .power = 0,   .pa_duty_cycle = 0x01 },
    { .power = 0,   .pa_duty_cycle = 0x02 },
    { .power = 3,   .pa_duty_cycle = 0x00 },
    { .power = 3,   .pa_duty_cycle = 0x01 },
    { .power = 4,   .pa_duty_cycle = 0x01 },
    { .power = 6,   .pa_duty_cycle = 0x00 },
    { .power = 6,   .pa_duty_cycle = 0x01 },
    { .power = 7,   .pa_duty_cycle = 0x01 },
    { .power = 8,   .pa_duty_cycle = 0x01 },
    { .power = 9,   .pa_duty_cycle = 0x01 },
    { .power = 10,  .pa_duty_cycle = 0x01 },
    { .power = 13,  .pa_duty_cycle = 0x00 },
    { .power = 14,  .pa_duty_cycle = 0x00 },
    { .power = 13,  .pa_duty_cycle = 0x01 },
    { .power = 14,  .pa_duty_cycle = 0x01 },
    { .power = 14,  .pa_duty_cycle = 0x02 },
    { .power = 14,  .pa_duty_cycle = 0x04 },
    { .power = 14,  .pa_duty_cycle = 0x05 },
    { .power = 14,  .pa_duty_cycle = 0x07 },
#endif
};
#endif

#ifdef SX126X_DRIVER_RX_ENABLE
static const uint8_t SX126X_RXBW[SX126X_RXBW_LAST] = { 0x1F, 0x17, 0x0F, 0x1E, 0x16, 0x0E, 0x1D, 0x15, 0x0D, 0x1C, 0x14, 0x0C, 0x1B, 0x13, 0x0B, 0x1A, 0x12, 0x0A, 0x19, 0x11, 0x09 };
#endif

/*** SX126X local functions ***/

/*******************************************************************/
static SX126X_status_t _SX126X_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    // Wait for BUSY line to be low.
    status = SX126X_HW_wait_busy_low();
    if (status != SX126X_SUCCESS) {
        status = SX126X_ERROR_BUSY_TIMEOUT;
        goto errors;
    }
    // Perform SPI transfer.
    status = SX126X_HW_spi_write_read_8(tx_data, rx_data, transfer_size);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static SX126X_status_t _SX126X_write_register(uint16_t register_address, uint8_t value) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_WRITE_REGISTER] = { SX126X_OP_CODE_WRITE_REGISTER, (uint8_t) (register_address >> 8), (uint8_t) (register_address >> 0), value };
    uint8_t data[SX126X_COMMAND_SIZE_WRITE_REGISTER];
    // Write access sequence.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_WRITE_REGISTER);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

#if (0)
/*******************************************************************/
static SX126X_status_t _SX126X_read_register(uint16_t register_address, uint8_t* value) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_READ_REGISTER] = {
        SX126X_OP_CODE_READ_REGISTER,
        (uint8_t) (register_address >> 8),
        (uint8_t) (register_address >> 0),
        SX126X_OP_CODE_NOP,
        SX126X_OP_CODE_NOP
    };
    uint8_t data[SX126X_COMMAND_SIZE_READ_REGISTER];
    // Read access sequence.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_READ_REGISTER);
    if (status != SX126X_SUCCESS) goto errors;
    // Update value.
    (*value) = data[SX126X_COMMAND_SIZE_READ_REGISTER - 1];
errors:
    return status;
}
#endif

/*** SX126X functions ***/

/*******************************************************************/
SX126X_status_t SX126X_init(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    // Init hardware interface.
    status = SX126X_HW_init();
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_de_init(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    // Release hardware interface.
    status = SX126X_HW_de_init();
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_reset(uint8_t reset_enable) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    // Check enable.
    if (reset_enable == 0) {
        // Put NRESET high.
        status = SX126X_HW_set_nreset_gpio(1);
        if (status != SX126X_SUCCESS) goto errors;
        // Wait for reset time.
        status = SX126X_HW_delay_milliseconds(SX126X_EXIT_RESET_DELAY_MS);
        if (status != SX126X_SUCCESS) goto errors;
    }
    else {
        // Put NRESET low.
        status = SX126X_HW_set_nreset_gpio(0);
        if (status != SX126X_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_get_device_errors(uint16_t* op_error) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_GET_DEVICE_ERRORS] = { SX126X_OP_CODE_GET_DEVICE_ERRORS, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP };
    uint8_t data[SX126X_COMMAND_SIZE_GET_DEVICE_ERRORS];
    // Check parameter.
    if (op_error == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_REGULATOR_MODE);
    if (status != SX126X_SUCCESS) goto errors;
    // Update data.
    (*op_error) = (uint16_t) ((data[2] << 8) + data[3]);
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_clear_device_errors(void) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_CLEAR_DEVICE_ERRORS] = { SX126X_OP_CODE_CLEAR_DEVICE_ERRORS, 0x00, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_CLEAR_DEVICE_ERRORS];
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_CLEAR_DEVICE_ERRORS);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_regulation_mode(SX126X_regulation_mode_t regulation_mode) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_SET_REGULATOR_MODE] = { SX126X_OP_CODE_SET_REGULATOR_MODE, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_SET_REGULATOR_MODE];
    // Check regulation mode.
    switch (regulation_mode) {
    case SX126X_REGULATION_MODE_LDO:
        break;
    case SX126X_REGULATION_MODE_DCDC:
        command[1] = 0x01;
        break;
    default:
        status = SX126X_ERROR_REGULATION_MODE;
        goto errors;
    }
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_REGULATOR_MODE);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_oscillator(SX126X_oscillator_t oscillator, SX126X_tcxo_voltage_t tcxo_voltage, uint32_t tcxo_timeout_ms) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_SET_DIO3_AS_TCXO_CTRL] = { SX126X_OP_CODE_SET_DIO3_AS_TCXO_CTRL, 0x00, 0x00, 0x00, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_SET_DIO3_AS_TCXO_CTRL];
    uint64_t tmp_u64 = 0;
    // Check oscillator.
    switch (oscillator) {
    case SX126X_OSCILLATOR_QUARTZ:
        // Nothing to do.
        break;
    case SX126X_OSCILLATOR_TCXO:
        // Check parameters.
        if (tcxo_voltage >= SX126X_TCXO_VOLTAGE_LAST) {
            status = SX126X_ERROR_TCXO_VOLTAGE;
            goto errors;
        }
        if ((tcxo_timeout_ms < SX126X_TCXO_TIMEOUT_MIN_MS) || (tcxo_timeout_ms > SX126X_TCXO_TIMEOUT_MAX_MS)) {
            status = SX126X_ERROR_TCXO_TIMEOUT;
            goto errors;
        }
        // Compute timeout.
        tmp_u64 = (((uint64_t) tcxo_timeout_ms) * 1000000);
        tmp_u64 /= ((uint64_t) SX126X_TCXO_TIMEOUT_DELAY_STEP_NS);
        // Compute parameters.
        command[1] = (uint8_t) (tcxo_voltage);
        command[2] = (uint8_t) (tmp_u64 >> 16);
        command[3] = (uint8_t) (tmp_u64 >> 8);
        command[4] = (uint8_t) (tmp_u64 >> 0);
        // Send command.
        status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_DIO3_AS_TCXO_CTRL);
        if (status != SX126X_SUCCESS) goto errors;
        break;
    default:
        status = SX126X_ERROR_OSCILLATOR;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_calibrate(uint16_t frequency_range_low_mhz, uint16_t frequency_range_high_mhz) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_CALIBRATE_IMAGE] = { SX126X_OP_CODE_CALIBRATE, 0x7F, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_CALIBRATE_IMAGE];
    uint8_t image_cal_freq1 = 0;
    uint8_t image_cal_freq2 = 0;
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_CALIBRATE);
    if (status != SX126X_SUCCESS) goto errors;
    // Wait for calibration to complete.
    status = SX126X_HW_wait_busy_low();
    if (status != SX126X_SUCCESS) {
        status = SX126X_ERROR_CALIBRATION_TIMEOUT;
        goto errors;
    }
    // Calibrate image according to requested frequency.
    image_cal_freq1 = (uint8_t) (frequency_range_low_mhz / SX126X_IMAGE_CALIBRATION_STEP_MHZ);
    image_cal_freq2 = (uint8_t) ((frequency_range_high_mhz + SX126X_IMAGE_CALIBRATION_STEP_MHZ - 1) / (SX126X_IMAGE_CALIBRATION_STEP_MHZ));
    command[0] = SX126X_OP_CODE_CALIBRATE_IMAGE;
    command[1] = image_cal_freq1;
    command[2] = image_cal_freq2;
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_CALIBRATE_IMAGE);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_mode(SX126X_mode_t mode) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_MODE_COMMAND_SIZE] = { SX126X_OP_CODE_NOP, 0x00, 0x00, 0x00 };
    uint8_t data[SX126X_MODE_COMMAND_SIZE];
    uint8_t command_size = 0;
    // Compute register.
    switch (mode) {
    case SX126X_MODE_SLEEP:
        command[0] = SX126X_OP_CODE_SET_SLEEP;
        command_size = SX126X_COMMAND_SIZE_SET_SLEEP;
        break;
    case SX126X_MODE_STANDBY_RC:
        command[0] = SX126X_OP_CODE_SET_STANDBY;
        command_size = SX126X_COMMAND_SIZE_SET_STANDBY;
        break;
    case SX126X_MODE_STANDBY_XOSC:
        command[0] = SX126X_OP_CODE_SET_STANDBY;
        command[1] = 0x01;
        command_size = SX126X_COMMAND_SIZE_SET_STANDBY;
        break;
    case SX126X_MODE_FS:
        command[0] = SX126X_OP_CODE_SET_FS;
        command_size = SX126X_COMMAND_SIZE_SET_FS;
        break;
    case SX126X_MODE_TX:
        command[0] = SX126X_OP_CODE_SET_TX;
        command_size = SX126X_COMMAND_SIZE_SET_TX;
        break;
    case SX126X_MODE_TX_CW:
        command[0] = SX126X_OP_CODE_SET_TX_CONTINUOUS_WAVE;
        command_size = SX126X_COMMAND_SIZE_SET_TX_CONTINUOUS_WAVE;
        break;
    case SX126X_MODE_RX:
        command[0] = SX126X_OP_CODE_SET_RX;
        command_size = SX126X_COMMAND_SIZE_SET_RX;
        break;
    default:
        status = SX126X_ERROR_MODE;
        goto errors;
    }
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, command_size);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_rf_frequency(uint32_t rf_frequency_hz) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_SET_RF_FREQUENCY] = { SX126X_OP_CODE_SET_RF_FREQUENCY, 0x00, 0x00, 0x00, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_SET_RF_FREQUENCY];
    uint64_t rf_freq = 0;
    // Check frequency range.
    if (rf_frequency_hz > SX126X_RF_FREQUENCY_HZ_MAX) {
        status = SX126X_ERROR_RF_FREQUENCY_OVERFLOW;
        goto errors;
    }
    if (rf_frequency_hz < SX126X_RF_FREQUENCY_HZ_MIN) {
        status = SX126X_ERROR_RF_FREQUENCY_UNDERFLOW;
        goto errors;
    }
    // Compute register.
    rf_freq = (((uint64_t) rf_frequency_hz) << 25);
    rf_freq /= ((uint64_t) SX126X_DRIVER_FXOSC_HZ);
    // Set command.
    command[1] = (uint8_t) (rf_freq >> 24);
    command[2] = (uint8_t) (rf_freq >> 16);
    command[3] = (uint8_t) (rf_freq >> 8);
    command[4] = (uint8_t) (rf_freq >> 0);
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_RF_FREQUENCY);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_modulation(SX126X_modulation_parameters_t* modulation_parameters) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_SET_MODULATION_PARAMS_GFSK] = { SX126X_OP_CODE_SET_PACKET_TYPE, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_SET_MODULATION_PARAMS_GFSK];
    uint64_t tmp_u64 = 0;
    // Check parameter.
    if (modulation_parameters == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Packet type.
    switch (modulation_parameters->modulation) {
    case SX126X_MODULATION_GFSK:
        command[1] = 0x00;
        break;
    case SX126X_MODULATION_BPSK:
        command[1] = 0x02;
        break;
    default:
        status = SX126X_ERROR_MODULATION;
        goto errors;
    }
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_PACKET_TYPE);
    if (status != SX126X_SUCCESS) goto errors;
    // Modulation parameters.
    command[0] = SX126X_OP_CODE_SET_MODULATION_PARAMS;
    // Bit rate.
    if ((modulation_parameters->bit_rate_bps) > SX126X_BIT_RATE_BPS_MAX) {
        status = SX126X_ERROR_BIT_RATE_OVERFLOW;
        goto errors;
    }
    if ((modulation_parameters->bit_rate_bps) < SX126X_BIT_RATE_BPS_MIN) {
        status = SX126X_ERROR_BIT_RATE_UNDERFLOW;
        goto errors;
    }
    tmp_u64 = (((uint64_t) SX126X_DRIVER_FXOSC_HZ) << 5);
    tmp_u64 /= ((uint64_t) (modulation_parameters->bit_rate_bps));
    command[1] = (uint8_t) (tmp_u64 >> 16);
    command[2] = (uint8_t) (tmp_u64 >> 8);
    command[3] = (uint8_t) (tmp_u64 >> 0);
    // Modulation shaping.
    switch (modulation_parameters->modulation_shaping) {
    case SX126X_MODULATION_SHAPING_NONE:
        command[4] = 0x00;
        break;
    case SX126X_MODULATION_SHAPING_GAUSSIAN_BT_03:
        command[4] = 0x08;
        break;
    case SX126X_MODULATION_SHAPING_GAUSSIAN_BT_05:
        command[4] = 0x09;
        break;
    case SX126X_MODULATION_SHAPING_GAUSSIAN_BT_07:
        command[4] = 0x0A;
        break;
    case SX126X_MODULATION_SHAPING_GAUSSIAN_BT_1:
        command[4] = 0x0B;
        break;
    case SX126X_MODULATION_SHAPING_DBPSK:
        command[4] = 0x16;
        break;
    default:
        status = SX126X_ERROR_MODULATION_SHAPING;
        goto errors;
    }
    if (modulation_parameters->rx_bandwidth >= SX126X_RXBW_LAST) {
        status = SX126X_ERROR_RX_BANDWIDTH;
        goto errors;
    }
#ifdef SX126X_DRIVER_RX_ENABLE
    command[5] = SX126X_RXBW[modulation_parameters->rx_bandwidth];
#endif
    // Deviation.
    tmp_u64 = ((uint64_t) (modulation_parameters->fsk_deviation_hz)) * ((uint64_t) SX126X_DRIVER_FXOSC_HZ);
    tmp_u64 <<= 25;
    command[6] = (uint8_t) (tmp_u64 >> 16);
    command[7] = (uint8_t) (tmp_u64 >> 8);
    command[8] = (uint8_t) (tmp_u64 >> 0);
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_MODULATION_PARAMS_GFSK);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_gfsk_packet(SX126X_gfsk_packet_parameters_t* packet_parameters) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_SET_PACKET_PARAMS_GFSK] = { SX126X_OP_CODE_SET_PACKET_PARAMS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_SET_PACKET_PARAMS_GFSK];
    uint8_t idx = 0;
    // Check parameter.
    if (packet_parameters == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Preamble length.
    command[1] = (uint8_t) ((packet_parameters->preamble_length_bits) >> 8);
    command[2] = (uint8_t) ((packet_parameters->preamble_length_bits) >> 0);
    // Preamble detector length.
    switch (packet_parameters->preamble_detector_length) {
    case SX126X_PREAMBLE_DETECTOR_LENGTH_OFF:
        command[3] = 0x00;
        break;
    case SX126X_PREAMBLE_DETECTOR_LENGTH_8BITS:
        command[3] = 0x04;
        break;
    case SX126X_PREAMBLE_DETECTOR_LENGTH_16BITS:
        command[3] = 0x05;
        break;
    case SX126X_PREAMBLE_DETECTOR_LENGTH_24BITS:
        command[3] = 0x06;
        break;
    case SX126X_PREAMBLE_DETECTOR_LENGTH_32BITS:
        command[3] = 0x07;
        break;
    default:
        status = SX126X_ERROR_PREAMBLE_DETECTOR_LENGTH;
        goto errors;
    }
    // Sync word length.
    if ((packet_parameters->sync_word_length_bits) > (SX126X_SYNC_WORD_MAXIMUM_SIZE_BYTES << 3)) {
        status = SX126X_ERROR_SYNC_WORD_LENGTH;
        goto errors;
    }
    command[4] = (packet_parameters->sync_word_length_bits);
    // Sync word.
    for (idx = 0; idx < SX126X_SYNC_WORD_MAXIMUM_SIZE_BYTES; idx++) {
        status = _SX126X_write_register((SX126X_REGISTER_ADDRESS_SYNC_WORD + idx), (packet_parameters->sync_word[idx]));
        if (status != SX126X_SUCCESS) goto errors;
    }
    // Payload length.
    command[7] = (packet_parameters->payload_length_bytes);
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_PACKET_PARAMS_GFSK);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_bpsk_packet(SX126X_bpsk_packet_parameters_t* packet_parameters) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_BPSK_PACKET_REGISTER_SIZE] = { SX126X_OP_CODE_SET_PACKET_PARAMS, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t data[SX126X_BPSK_PACKET_REGISTER_SIZE];
    uint8_t idx = 0;
    // Check parameter.
    if (packet_parameters == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Payload length.
    command[1] = (packet_parameters->payload_length_bytes);
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_PACKET_PARAMS_BPSK);
    if (status != SX126X_SUCCESS) goto errors;
    // Specific registers for BPSK.
    command[0] = (uint8_t) ((packet_parameters->ramp_up_delay) >> 8);
    command[1] = (uint8_t) ((packet_parameters->ramp_up_delay) >> 0);
    command[2] = (uint8_t) ((packet_parameters->ramp_down_delay) >> 8);
    command[3] = (uint8_t) ((packet_parameters->ramp_down_delay) >> 0);
    command[4] = (uint8_t) ((packet_parameters->payload_length_bits) >> 8);
    command[5] = (uint8_t) ((packet_parameters->payload_length_bits) >> 0);
    // Write registers.
    for (idx = 0; idx < SX126X_BPSK_PACKET_REGISTER_SIZE; idx++) {
        status = _SX126X_write_register((SX126X_REGISTER_ADDRESS_BPSK_PACKET + idx), command[idx]);
        if (status != SX126X_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_set_dio_irq_mask(uint16_t irq_mask, uint16_t irq_mask_dio1, uint16_t irq_mask_dio2, uint16_t irq_mask_dio3) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_SET_DIO_IRQ_PARAMS] = { SX126X_OP_CODE_SET_DIO_IRQ_PARAMS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t data[SX126X_COMMAND_SIZE_SET_DIO_IRQ_PARAMS];
    // Global interrupts mask.
    command[1] = (uint8_t) (irq_mask >> 8);
    command[2] = (uint8_t) (irq_mask >> 0);
    // DIO1.
    command[3] = (uint8_t) (irq_mask_dio1 >> 8);
    command[4] = (uint8_t) (irq_mask_dio1 >> 0);
    // DIO2.
    command[5] = (uint8_t) (irq_mask_dio2 >> 8);
    command[6] = (uint8_t) (irq_mask_dio2 >> 0);
    // DIO3.
    command[7] = (uint8_t) (irq_mask_dio3 >> 8);
    command[8] = (uint8_t) (irq_mask_dio3 >> 0);
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_SET_DIO_IRQ_PARAMS);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
SX126X_status_t SX126X_get_irq_flag(SX126X_irq_index_t irq_index, uint8_t* irq_flag) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_GET_IRQ_STATUS] = { SX126X_OP_CODE_GET_IRQ_STATUS, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP };
    uint8_t data[SX126X_COMMAND_SIZE_GET_IRQ_STATUS];
    // Check parameters.
    if (irq_index >= SX126X_IRQ_INDEX_LAST) {
        status = SX126X_ERROR_IRQ_INDEX;
        goto errors;
    }
    if (irq_flag == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_GET_IRQ_STATUS);
    if (status != SX126X_SUCCESS) goto errors;
    // Read bit.
    (*irq_flag) = ((data[3 - (irq_index >> 3)] >> (irq_index % 8)) & 0x01);
errors:
    return status;
}

#ifdef SX126X_DRIVER_TX_ENABLE
/*******************************************************************/
SX126X_status_t SX126X_set_rf_output_power(int8_t rf_output_power_dbm, SX126X_pa_ramp_time_t pa_ramp_time) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
#ifdef SX126X_DRIVER_DEVICE_SX1262
    uint8_t command_pa_config[SX126X_COMMAND_SIZE_SET_PA_CONFIG] = { SX126X_OP_CODE_SET_PA_CONFIG, 0x00, 0x00, 0x00, 0x01 };
#else
    uint8_t command_pa_config[SX126X_COMMAND_SIZE_SET_PA_CONFIG] = { SX126X_OP_CODE_SET_PA_CONFIG, 0x00, 0x00, 0x01, 0x01 };
#endif
    uint8_t data_pa_config[SX126X_COMMAND_SIZE_SET_PA_CONFIG];
    uint8_t command_tx_params[SX126X_COMMAND_SIZE_SET_TX_PARAMS] = { SX126X_OP_CODE_SET_TX_PARAMS, 0x00, 0x00 };
    uint8_t data_tx_params[SX126X_COMMAND_SIZE_SET_TX_PARAMS];
    uint8_t lut_index = 0;
    // Check parameters.
    if (rf_output_power_dbm > SX126X_OUTPUT_POWER_MAX) {
        status = SX126X_ERROR_RF_OUTPUT_POWER_OVERFLOW;
        goto errors;
    }
    if (rf_output_power_dbm < SX126X_OUTPUT_POWER_MIN) {
        status = SX126X_ERROR_RF_OUTPUT_POWER_UNDERFLOW;
        goto errors;
    }
    if (pa_ramp_time >= SX126X_PA_RAMP_TIME_LAST) {
        status = SX126X_ERROR_PA_RAMP_TIME;
        goto errors;
    }
    // Compute LUT index.
    lut_index = (uint8_t) (rf_output_power_dbm - SX126X_OUTPUT_POWER_MIN);
    // Compute PA parameters.
    command_pa_config[1] = SX126X_PA_CONFIG[lut_index].pa_duty_cycle;
#ifdef SX126X_DRIVER_DEVICE_SX1262
    command_pa_config[2] = SX126X_PA_CONFIG[lut_index].hp_max;
#endif
    // Compute TX parameters.
    command_tx_params[1] = (uint8_t) (SX126X_PA_CONFIG[lut_index].power);
    command_tx_params[2] = (uint8_t) (pa_ramp_time);
    // Send commands.
    status = _SX126X_spi_write_read_8(command_pa_config, data_pa_config, SX126X_COMMAND_SIZE_SET_PA_CONFIG);
    if (status != SX126X_SUCCESS) goto errors;
    status = _SX126X_spi_write_read_8(command_tx_params, data_tx_params, SX126X_COMMAND_SIZE_SET_TX_PARAMS);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX126X_DRIVER_TX_ENABLE
/*******************************************************************/
SX126X_status_t SX126X_differential_encoding(uint8_t* data_in, uint8_t data_in_size_bytes, uint8_t* data_out, uint8_t* data_out_size_bytes, uint16_t* data_out_size_bits) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t in_byte = 0;
    uint8_t out_byte = 0;
    uint16_t data_in_size_bits = (uint16_t) ((data_in_size_bytes << 3) + 2);
    int16_t data_in_bytes_count = data_in_size_bytes;
    uint8_t current = 0;
    uint8_t idx = 0;
    // Check parameters.
    if ((data_in == NULL) || (data_out == NULL) || (data_out_size_bytes == NULL) || (data_out_size_bits == NULL)) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    in_byte = (*data_in++);
    // Process full bytes
    while (--data_in_bytes_count >= 0) {
        for (idx = 0; idx < 8; ++idx) {
            out_byte = (out_byte << 1) | current;
            if ((in_byte & 0x80) == 0) {
                current = current ^ 0x01;
            }
            in_byte <<= 1;
        }
        in_byte = (*data_in++);
        *data_out++ = out_byte;
    }
    // Process remaining bits
    for (idx = 0; idx < (data_in_size_bits & 0x07); ++idx) {
        out_byte = (out_byte << 1) | current;
        if ((in_byte & 0x80) == 0) {
            current = current ^ 0x01;
        }
        in_byte <<= 1;
    }
    // Process last data bit
    out_byte = (out_byte << 1) | current;
    if ((data_in_size_bits & 0x07) == 0x07) {
        *data_out++ = out_byte;
    }
    // Add duplicate bit and store
    out_byte = (out_byte << 1) | current;
    (*data_out) = out_byte << (7 - ((data_in_size_bits + 1) & 0x07));
    // Update output sizes.
    (*data_out_size_bytes) = (uint8_t) ((data_in_size_bits + 7) >> 3);
    (*data_out_size_bits) = data_in_size_bits;
errors:
    return status;
}
#endif

#ifdef SX126X_DRIVER_TX_ENABLE
/*******************************************************************/
SX126X_status_t SX126X_write_fifo(uint8_t* tx_data, uint8_t tx_data_size) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_WRITE_BUFFER + 255];
    uint8_t data[SX126X_COMMAND_SIZE_WRITE_BUFFER + 255];
    uint8_t idx = 0;
    // Check parameters.
    if (tx_data == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Operation code.
    command[0] = SX126X_OP_CODE_WRITE_BUFFER;
    // Offset.
    command[1] = 0;
    // Data.
    for (idx = 0; idx < tx_data_size; idx ++) {
        command[SX126X_COMMAND_SIZE_WRITE_BUFFER + idx] = tx_data[idx];
    }
    // Send command.
    status = _SX126X_spi_write_read_8(command, data, (SX126X_COMMAND_SIZE_WRITE_BUFFER + tx_data_size));
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX126X_DRIVER_RX_ENABLE
/*******************************************************************/
SX126X_status_t SX126X_set_lna_mode(SX126X_lna_mode_t lna_mode) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t reg_value = 0x00;
    // Compute register.
    switch (lna_mode) {
    case SX126X_LNA_MODE_NORMAL:
        reg_value = 0x94;
        break;
    case SX126X_LNA_MODE_BOOST:
        reg_value = 0x96;
        break;
    default:
        status = SX126X_ERROR_LNA_MODE;
        goto errors;
    }
    // Program register.
    status = _SX126X_write_register(SX126X_REGISTER_ADDRESS_RX_GAIN, reg_value);
    if (status != SX126X_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#ifdef SX126X_DRIVER_RX_ENABLE
/*******************************************************************/
SX126X_status_t SX126X_get_rssi(SX126X_rssi_t rssi_type, int16_t* rssi_dbm) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command[SX126X_COMMAND_SIZE_GET_PACKET_STATUS] = { SX126X_OP_CODE_GET_PACKET_STATUS, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP };
    uint8_t data[SX126X_COMMAND_SIZE_GET_PACKET_STATUS];
    uint8_t rssi_raw = 0;
    // Check parameters.
    if (rssi_type >= SX126X_RSSI_TYPE_LAST) {
        status = SX126X_ERROR_RSSI_TYPE;
        goto errors;
    }
    if (rssi_dbm == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (rssi_type == SX126X_RSSI_TYPE_INSTANTANEOUS) {
        // Update operation code.
        command[0] = SX126X_OP_CODE_GET_RSSI_INST;
        // Send command.
        status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_GET_RSSI_INST);
        if (status != SX126X_SUCCESS) goto errors;
        // Extract raw value.
        rssi_raw = data[2];
    }
    else {
        // Send command.
        status = _SX126X_spi_write_read_8(command, data, SX126X_COMMAND_SIZE_GET_PACKET_STATUS);
        if (status != SX126X_SUCCESS) goto errors;
        // Extract raw value.
        rssi_raw = (rssi_type == SX126X_RSSI_TYPE_SYNC_WORD) ? data[3] : data[4];
    }
    // Compute RSSI.
    (*rssi_dbm) = (-1) * ((int16_t) (rssi_raw >> 1));
errors:
    return status;
}
#endif

#ifdef SX126X_DRIVER_RX_ENABLE
/*******************************************************************/
SX126X_status_t SX126X_read_fifo(uint8_t* rx_data, uint8_t rx_data_size) {
    // Local variables.
    SX126X_status_t status = SX126X_SUCCESS;
    uint8_t command_rx_buffer_status[SX126X_COMMAND_SIZE_GET_RX_BUFFER_STATUS] = { SX126X_OP_CODE_GET_RX_BUFFER_STATUS, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP, SX126X_OP_CODE_NOP };
    uint8_t data_rx_buffer_status[SX126X_COMMAND_SIZE_GET_RX_BUFFER_STATUS];
    uint8_t command_fifo[SX126X_COMMAND_SIZE_READ_BUFFER + 255];
    uint8_t data_fifo[SX126X_COMMAND_SIZE_READ_BUFFER + 255];
    uint8_t idx = 0;
    // Check parameters.
    if (rx_data == NULL) {
        status = SX126X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Get RX buffer status.
    status = _SX126X_spi_write_read_8(command_rx_buffer_status, data_rx_buffer_status, SX126X_COMMAND_SIZE_GET_RX_BUFFER_STATUS);
    if (status != SX126X_SUCCESS) goto errors;
    // Check received payload length.
    if (rx_data_size > data_rx_buffer_status[2]) {
        status = SX126X_ERROR_RX_PAYLOAD_SIZE;
        goto errors;
    }
    // Build FIFO command.
    command_fifo[0] = SX126X_OP_CODE_READ_BUFFER;
    command_fifo[1] = data_rx_buffer_status[3];
    for (idx = 0; idx < rx_data_size; idx ++) {
        command_fifo[SX126X_COMMAND_SIZE_WRITE_BUFFER + idx] = SX126X_OP_CODE_NOP;
    }
    // Send command.
    status = _SX126X_spi_write_read_8(command_fifo, data_fifo, (SX126X_COMMAND_SIZE_READ_BUFFER + rx_data_size));
    if (status != SX126X_SUCCESS) goto errors;
    // Update data.
    for (idx = 0; idx < rx_data_size; idx ++) {
        rx_data[idx] = data_fifo[SX126X_COMMAND_SIZE_READ_BUFFER + 1 + idx];
    }
errors:
    return status;
}
#endif

#endif /* SX126X_DRIVER_DISABLE */
