/**
 * @file bee_Sht3x.c
 * @author nguyen__viet_hoang
 * @date 20 August 2023
 * @brief module SHT3x with API, init, I2C protocol to read data sensor. some mode to read
 */

/* -- use following constants to define the example mode ----------- */

// #define SINGLE_SHOT_LOW_LEVEL
// #define SINGLE_SHOT_HIGH_LEVEL

/* -- includes ----------------------------------------------------- */
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "bee_Sht3x.h"

/* -- platform dependent definitions ------------------------------- */

#define TASK_STACK_DEPTH 2048

// I2C interface defintions for ESP32 and ESP8266
#define I2C_BUS GPIO_NUM_0
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_FREQ I2C_FREQ_100K

const uint16_t SHT3x_MEASURE_CMD[6][3] = {
    {0x2400, 0x240b, 0x2416},  // [SINGLE_SHOT][H,M,L] without clock stretching
    {0x2032, 0x2024, 0x202f},  // [PERIODIC_05][H,M,L]
    {0x2130, 0x2126, 0x212d},  // [PERIODIC_1 ][H,M,L]
    {0x2236, 0x2220, 0x222b},  // [PERIODIC_2 ][H,M,L]
    {0x2234, 0x2322, 0x2329},  // [PERIODIC_4 ][H,M,L]
    {0x2737, 0x2721, 0x272a}}; // [PERIODIC_10][H,M,L]

// due to the fact that ticks can be smaller than portTICK_PERIOD_MS, one and
// a half tick period added to the duration to be sure that waiting time for
// the results is long enough

// measurement durations in us
const uint16_t SHT3x_MEAS_DURATION_US[3] = {SHT3x_MEAS_DURATION_REP_HIGH * 1000,
                                            SHT3x_MEAS_DURATION_REP_MEDIUM * 1000,
                                            SHT3x_MEAS_DURATION_REP_LOW * 1000};

// measurement durations in RTOS ticks
const uint8_t SHT3x_MEAS_DURATION_TICKS[3] = {TIME_TO_TICKS(SHT3x_MEAS_DURATION_REP_HIGH),
                                              TIME_TO_TICKS(SHT3x_MEAS_DURATION_REP_MEDIUM),
                                              TIME_TO_TICKS(SHT3x_MEAS_DURATION_REP_LOW)};
extern float f_Sht3x_temp;
extern float f_Sht3x_humi;

// esp-open-rtos I2C interface wrapper

static void i2c_init(int bus, gpio_num_t scl, gpio_num_t sda, uint32_t freq)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(bus, &conf);

    i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
}

static int i2c_slave_write(uint8_t bus, uint8_t addr, const uint8_t *reg,
                           uint8_t *data, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    if (reg)
        i2c_master_write_byte(cmd, *reg, true);
    if (data)
        i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

static int i2c_slave_read(uint8_t bus, uint8_t addr, const uint8_t *reg,
                          uint8_t *data, uint32_t len)
{
    if (len == 0)
        return true;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (reg)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, *reg, true);
        if (!data)
            i2c_master_stop(cmd);
    }
    if (data)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        if (len > 1)
            i2c_master_read(cmd, data, len - 1, I2C_ACK_VAL);
        i2c_master_read_byte(cmd, data + len - 1, I2C_NACK_VAL);
        i2c_master_stop(cmd);
    }
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

/* -- user tasks --------------------------------------------------- */
static uint32_t sdk_system_get_time();
static bool sht3x_send_command(sht3x_sensor_t *dev, uint16_t cmd);
static bool sht3x_get_status(sht3x_sensor_t *dev, uint16_t *status);
static bool sht3x_read_data(sht3x_sensor_t *dev, uint8_t *data, uint32_t len);
static bool sht3x_is_measuring(sht3x_sensor_t *dev);
static bool sht3x_reset(sht3x_sensor_t *dev);
static bool sht3x_get_raw_data(sht3x_sensor_t *dev, sht3x_raw_data_t raw_data);
static bool sht3x_compute_values(sht3x_raw_data_t raw_data, float *temperature, float *humidity);
static uint8_t crc8(uint8_t data[], int len);

static uint8_t sht3x_get_measurement_duration(sht3x_repeat_t repeat)
{
    return SHT3x_MEAS_DURATION_TICKS[repeat]; // in RTOS ticks
}

static bool sht3x_start_measurement(sht3x_sensor_t *dev, sht3x_mode_t mode, sht3x_repeat_t repeat)
{
    if (!dev)
        return false;

    dev->error_code = SHT3x_OK;
    dev->mode = mode;
    dev->repeatability = repeat;

    // start measurement according to selected mode and return an duration estimate
    if (!sht3x_send_command(dev, SHT3x_MEASURE_CMD[mode][repeat]))
    {
        // error_dev("could not send start measurment command", __FUNCTION__, dev);
        dev->error_code |= SHT3x_SEND_MEAS_CMD_FAILED;
        return false;
    }

    dev->meas_start_time = sdk_system_get_time();
    dev->meas_started = true;
    dev->meas_first = true;

    return true;
}

static bool sht3x_get_results(sht3x_sensor_t *dev, float *temperature, float *humidity)
{
    if (!dev || (!temperature && !humidity))
        return false;

    sht3x_raw_data_t raw_data;

    if (!sht3x_get_raw_data(dev, raw_data))
        return false;

    return sht3x_compute_values(raw_data, temperature, humidity);
}

static sht3x_sensor_t *sht3x_init_sensor(uint8_t bus, uint8_t addr)
{
    sht3x_sensor_t *dev;

    if ((dev = malloc(sizeof(sht3x_sensor_t))) == NULL)
        return NULL;

    // inititalize sensor data structure
    dev->bus = bus;
    dev->addr = addr;
    dev->mode = sht3x_periodic_05mps;
    dev->meas_start_time = 0;
    dev->meas_started = false;
    dev->meas_first = false;

    uint16_t status;

    // try to reset the sensor
    if (!sht3x_reset(dev))
    {
        // debug_dev("could not reset the sensor", __FUNCTION__, dev);
    }

    // check again the status after clear status command
    if (!sht3x_get_status(dev, &status))
    {
        // error_dev("could not get sensor status", __FUNCTION__, dev);
        free(dev);
        return NULL;
    }

    // debug_dev("sensor initialized", __FUNCTION__, dev);
    return dev;
}

static sht3x_sensor_t *sensor; // sensor device data structure

/* -- main program ------------------------------------------------- */

static void sht3x_vProcess_data_sensor(void *pvParameters)
{
    float temperature;
    float humidity;

    // Start periodic measurements with 1 measurement per second.
    sht3x_start_measurement(sensor, sht3x_periodic_05mps, sht3x_high);

    // Wait until first measurement is ready (constant time of at least 30 ms
    // or the duration returned from *sht3x_get_measurement_duration*).
    vTaskDelay(sht3x_get_measurement_duration(sht3x_high));

    TickType_t last_wakeup = xTaskGetTickCount();

    for (;;)
    {
        // Get the values and do something with them.
        if (sht3x_get_results(sensor, &temperature, &humidity))
        {
            f_Sht3x_temp = temperature;
            f_Sht3x_humi = humidity;
        }

        //  Wait until 2 seconds (cycle time) are over.
        vTaskDelayUntil(&last_wakeup, 2000 / portTICK_PERIOD_MS);
    }
}

void sht3x_start(void)
{
    // Init I2C bus interfaces at which SHT3x sensors are connected
    // (different busses are possible).
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    // Create the sensors, multiple sensors are possible.
    if ((sensor = sht3x_init_sensor(I2C_BUS, SHT3x_ADDR_1)))
    {
        // Create a user task that uses the sensors.
        xTaskCreate(sht3x_vProcess_data_sensor, "sht3x_vProcess_data_sensor", TASK_STACK_DEPTH, NULL, 3, NULL);
    }
    else
        printf("Could not initialize SHT3x sensor\n");
    // That's it.
}

static uint32_t sdk_system_get_time()
{
    struct timeval time;
    gettimeofday(&time, 0);
    return time.tv_sec * 1e6 + time.tv_usec;
}

static bool sht3x_send_command(sht3x_sensor_t *dev, uint16_t cmd)
{
    if (!dev)
        return false;

    uint8_t data[2] = {cmd >> 8, cmd & 0xff};

    // debug_dev("send command MSB=%02x LSB=%02x", __FUNCTION__, dev, data[0], data[1]);

    int err = i2c_slave_write(dev->bus, dev->addr, 0, data, 2);

    if (err)
    {
        dev->error_code |= (err == -EBUSY) ? SHT3x_I2C_BUSY : SHT3x_I2C_SEND_CMD_FAILED;
        // error_dev("i2c error %d on write command %02x", __FUNCTION__, dev, err, cmd);
        return false;
    }

    return true;
}

static bool sht3x_read_data(sht3x_sensor_t *dev, uint8_t *data, uint32_t len)
{
    if (!dev)
        return false;
    int err = i2c_slave_read(dev->bus, dev->addr, 0, data, len);

    if (err)
    {
        dev->error_code |= (err == -EBUSY) ? SHT3x_I2C_BUSY : SHT3x_I2C_READ_FAILED;
        // error_dev("error %d on read %d byte", __FUNCTION__, dev, err, len);
        return false;
    }

#ifdef SHT3x_DEBUG_LEVEL_2
    printf("SHT3x %s: bus %d, addr %02x - read following bytes: ",
           __FUNCTION__, dev->bus, dev->addr);
    for (int i = 0; i < len; i++)
        printf("%02x ", data[i]);
    printf("\n");
#endif // ifdef SHT3x_DEBUG_LEVEL_2

    return true;
}

static bool sht3x_is_measuring(sht3x_sensor_t *dev)
{
    if (!dev)
        return false;

    dev->error_code = SHT3x_OK;

    // not running if measurement is not started at all or
    // it is not the first measurement in periodic mode
    if (!dev->meas_started || !dev->meas_first)
        return false;

    // not running if time elapsed is greater than duration
    uint32_t elapsed = sdk_system_get_time() - dev->meas_start_time;

    return elapsed < SHT3x_MEAS_DURATION_US[dev->repeatability];
}

static bool sht3x_reset(sht3x_sensor_t *dev)
{
    if (!dev)
        return false;

    // debug_dev("soft-reset triggered", __FUNCTION__, dev);

    dev->error_code = SHT3x_OK;

    // send reset command
    if (!sht3x_send_command(dev, SHT3x_RESET_CMD))
    {
        dev->error_code |= SHT3x_SEND_RESET_CMD_FAILED;
        return false;
    }
    // wait for small amount of time needed (according to datasheet 0.5ms)
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint16_t status;

    // check the status after reset
    if (!sht3x_get_status(dev, &status))
        return false;

    return true;
}

static uint8_t crc8(uint8_t data[], int len)
{
    static const uint8_t g_polynom = 0x31;
    // initialization value
    uint8_t crc = 0xff;

    // iterate over all bytes
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int i = 0; i < 8; i++)
        {
            bool xor = crc & 0x80;
            crc = crc << 1;
            crc = xor? crc ^ g_polynom : crc;
        }
    }

    return crc;
}

static bool sht3x_get_raw_data(sht3x_sensor_t *dev, sht3x_raw_data_t raw_data)
{
    if (!dev || !raw_data)
        return false;

    dev->error_code = SHT3x_OK;

    if (!dev->meas_started)
    {
        // debug_dev("measurement is not started", __FUNCTION__, dev);
        dev->error_code = SHT3x_MEAS_NOT_STARTED;
        return sht3x_is_measuring(dev);
    }

    if (sht3x_is_measuring(dev))
    {
        // error_dev("measurement is still running", __FUNCTION__, dev);
        dev->error_code = SHT3x_MEAS_STILL_RUNNING;
        return false;
    }

    // send fetch command in any periodic mode (mode > 0) before read raw data
    if (dev->mode && !sht3x_send_command(dev, SHT3x_FETCH_DATA_CMD))
    {
        // debug_dev("send fetch command failed", __FUNCTION__, dev);
        dev->error_code |= SHT3x_SEND_FETCH_CMD_FAILED;
        return false;
    }

    // read raw data
    if (!sht3x_read_data(dev, raw_data, sizeof(sht3x_raw_data_t)))
    {
        // error_dev("read raw data failed", __FUNCTION__, dev);
        dev->error_code |= SHT3x_READ_RAW_DATA_FAILED;
        return false;
    }

    // reset first measurement flag
    dev->meas_first = false;

    // reset measurement started flag in single shot mode
    if (dev->mode == sht3x_single_shot)
        dev->meas_started = false;

    // check temperature crc
    if (crc8(raw_data, 2) != raw_data[2])
    {
        // error_dev("CRC check for temperature data failed", __FUNCTION__, dev);
        dev->error_code |= SHT3x_WRONG_CRC_TEMPERATURE;
        return false;
    }

    // check humidity crc
    if (crc8(raw_data + 3, 2) != raw_data[5])
    {
        // error_dev("CRC check for humidity data failed", __FUNCTION__, dev);
        dev->error_code |= SHT3x_WRONG_CRC_HUMIDITY;
        return false;
    }

    return true;
}

static bool sht3x_compute_values(sht3x_raw_data_t raw_data, float *temperature, float *humidity)
{
    if (!raw_data)
        return false;

    if (temperature)
        *temperature = ((((raw_data[0] * 256.0) + raw_data[1]) * 175) / 65535.0) - 45;

    if (humidity)
        *humidity = ((((raw_data[3] * 256.0) + raw_data[4]) * 100) / 65535.0);

    return true;
}

static bool sht3x_get_status(sht3x_sensor_t *dev, uint16_t *status)
{
    if (!dev || !status)
        return false;

    dev->error_code = SHT3x_OK;

    uint8_t data[3];

    if (!sht3x_send_command(dev, SHT3x_STATUS_CMD) || !sht3x_read_data(dev, data, 3))
    {
        dev->error_code |= SHT3x_SEND_STATUS_CMD_FAILED;
        return false;
    }

    *status = data[0] << 8 | data[1];
    // debug_dev("status=%02x", __FUNCTION__, dev, *status);
    return true;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/