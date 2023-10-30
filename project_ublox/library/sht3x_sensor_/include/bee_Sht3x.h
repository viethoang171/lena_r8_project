/**
 * @file bee_Sht3x.h
 * @author nguyen__viet_hoang
 * @date 20 August 2023
 * @brief module SHT3x with API, init, I2C protocol to read data sensor
 */
#ifndef __SHT3x_H__
#define __SHT3x_H__

#define I2C_FREQ_80K 80000
#define I2C_FREQ_100K 100000
#define I2C_FREQ_400K 400000
#define I2C_FREQ_500K 500000
#define I2C_FREQ_600K 600000
#define I2C_FREQ_800K 800000
#define I2C_FREQ_1000K 1000000
#define I2C_FREQ_1300K 1300000

#define SHT3x_STATUS_CMD 0xF32D
#define SHT3x_CLEAR_STATUS_CMD 0x3041
#define SHT3x_RESET_CMD 0x30A2
#define SHT3x_FETCH_DATA_CMD 0xE000
#define SHT3x_HEATER_OFF_CMD 0x3066

#define TIME_TO_TICKS(ms) (1 + ((ms) + (portTICK_PERIOD_MS - 1) + portTICK_PERIOD_MS / 2) / portTICK_PERIOD_MS)

#define SHT3x_MEAS_DURATION_REP_HIGH 15
#define SHT3x_MEAS_DURATION_REP_MEDIUM 6
#define SHT3x_MEAS_DURATION_REP_LOW 4

// definition of possible I2C slave addresses
#define SHT3x_ADDR_1 0x44 // ADDR pin connected to GND/VSS (default)
#define SHT3x_ADDR_2 0x45 // ADDR pin connected to VDD

// // definition of error codes
#define SHT3x_OK 0
#define SHT3x_NOK -1

#define SHT3x_I2C_ERROR_MASK 0x000f
#define SHT3x_DRV_ERROR_MASK 0xfff0

// error codes for I2C interface ORed with SHT3x error codes
#define SHT3x_I2C_READ_FAILED 1
#define SHT3x_I2C_SEND_CMD_FAILED 2
#define SHT3x_I2C_BUSY 3

// SHT3x driver error codes OR ed with error codes for I2C interface
#define SHT3x_MEAS_NOT_STARTED (1 << 8)
#define SHT3x_MEAS_ALREADY_RUNNING (2 << 8)
#define SHT3x_MEAS_STILL_RUNNING (3 << 8)
#define SHT3x_READ_RAW_DATA_FAILED (4 << 8)

#define SHT3x_SEND_MEAS_CMD_FAILED (5 << 8)
#define SHT3x_SEND_RESET_CMD_FAILED (6 << 8)
#define SHT3x_SEND_STATUS_CMD_FAILED (7 << 8)
#define SHT3x_SEND_FETCH_CMD_FAILED (8 << 8)

#define SHT3x_WRONG_CRC_TEMPERATURE (9 << 8)
#define SHT3x_WRONG_CRC_HUMIDITY (10 << 8)

#define SHT3x_RAW_DATA_SIZE 6

#define I2C_ACK_VAL 0x0
#define I2C_NACK_VAL 0x1

/**
 * @brief	raw data type
 */
typedef uint8_t sht3x_raw_data_t[SHT3x_RAW_DATA_SIZE];

/**
 * @brief   possible measurement modes
 */
typedef enum
{
    sht3x_single_shot = 0, // one single measurement
    sht3x_periodic_05mps,  // periodic with 0.5 measurements per second (mps)
    sht3x_periodic_1mps,   // periodic with   1 measurements per second (mps)
    sht3x_periodic_2mps,   // periodic with   2 measurements per second (mps)
    sht3x_periodic_4mps,   // periodic with   4 measurements per second (mps)
    sht3x_periodic_10mps   // periodic with  10 measurements per second (mps)
} sht3x_mode_t;

/**
 * @brief   possible repeatability modes
 */
typedef enum
{
    sht3x_high = 0,
    sht3x_medium,
    sht3x_low
} sht3x_repeat_t;

/**
 * @brief 	SHT3x sensor device data structure type
 */
typedef struct
{

    uint32_t error_code; // combined error codes

    uint8_t bus;  // I2C bus at which sensor is connected
    uint8_t addr; // I2C slave address of the sensor

    sht3x_mode_t mode;            // used measurement mode
    sht3x_repeat_t repeatability; // used repeatability

    bool meas_started;        // indicates whether measurement started
    uint32_t meas_start_time; // measurement start time in us
    bool meas_first;          // first measurement in periodic mode

} sht3x_sensor_t;

void sht3x_start(void);
#endif /* __SHT3x_H__ */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/