#include <esp_types.h>
#include <memory.h>
#include "include/ds_temp_sensor.h"
#include "BME280_driver/bme280.h"
#include "driver/i2c.h"
#include "ds_message.h"
#include "ds_log.h"

static const char *TAG = "Temp sensor";

static gpio_num_t i2c_sda = GPIO_NUM_21;
static gpio_num_t i2c_scl = GPIO_NUM_22;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

static uint8_t i2c_init() {
    esp_err_t err;
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = i2c_sda,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = i2c_scl,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = i2c_frequency
    };

    err = i2c_param_config(i2c_port, &conf);
    if (err != ESP_OK) {
        return 1;
    }

    err = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        return 1;
    }

    DS_LOG_INFO(TAG, "I2C protocol initialized correctly ...");
    return 0;
}

static void i2c_print_command_error(esp_err_t i2c_err) {
    switch (i2c_err) {
        case ESP_ERR_INVALID_STATE:
            DS_LOG_ERROR(TAG, "I2C driver not installed or not in master mode.");
            break;
        case ESP_ERR_TIMEOUT:
            DS_LOG_ERROR(TAG, "Operation timeout because the bus is busy.");
            break;
        case ESP_FAIL:
            DS_LOG_ERROR(TAG, "Sending command error, slave doesn’t ACK the transfer.");
            break;
        case ESP_ERR_INVALID_ARG:
            DS_LOG_ERROR(TAG, "Parameter error.");
            break;
        default:
            break;
    }
}

static void print_sensor_data(struct bme280_data *comp_data) {
    float temp, press, hum;

    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;
    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
//    printf("%0.2lf deg C\n", temp);
}

static void i2c_delay_uc(uint32_t period, void *dev) {
    vTaskDelay(period / portTICK_PERIOD_MS);
}

static int8_t i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *dev_addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    uint8_t device_address = *((uint8_t *) dev_addr);

    // Set the register to write
    i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_WRITE, 0x01);
    i2c_master_write_byte(cmd, reg_addr, 0x01);

    // Write the data
    i2c_master_write(cmd, (uint8_t *) reg_data, length, 0x01);
    i2c_master_stop(cmd);

    esp_err_t i2c_err = i2c_master_cmd_begin(i2c_port, cmd, (1000 / portTICK_PERIOD_MS));
    i2c_print_command_error(i2c_err);
    i2c_cmd_link_delete(cmd);
    return i2c_err != ESP_OK;
}

static int8_t i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *dev_addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    uint8_t device_address = *((uint8_t *) dev_addr);

    // Select the register to read
    i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_WRITE, 0x01);
    i2c_master_write_byte(cmd, reg_addr, 0x01);

    // Read the data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_READ, 0x01);
    i2c_master_read(cmd, reg_data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t i2c_err = i2c_master_cmd_begin(i2c_port, cmd, (1000 / portTICK_PERIOD_MS));
    i2c_print_command_error(i2c_err);
    i2c_cmd_link_delete(cmd);
    return i2c_err != ESP_OK;
}

static uint8_t sensor_config(struct bme280_dev *dev) {
    int8_t rslt;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    /* Variable to define the selecting sensors */
    uint8_t settings_sel = 0 | BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    /* Set the sensor settings */
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_OK) {
        DS_LOG_ERROR(TAG, "Failed to set sensor settings (code %+d)", rslt);
        return 1;
    }

    printf("Temperature, Pressure, Humidity\n");

    /* Set the sensor to forced mode */
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
    if (rslt != BME280_OK) {
        DS_LOG_ERROR(TAG, "Failed to set sensor mode (code 0x%x)", rslt);
        return 1;
    }

    DS_LOG_INFO(TAG, "Sensor configured correctly ...");
    return 0;
}

static uint8_t sensor_init(struct bme280_dev *dev) {

    uint8_t dev_addr = BME280_I2C_ADDR_PRIM;
    int8_t rslt = BME280_OK;

    dev->intf_ptr = malloc(sizeof(dev_addr));
    memcpy(dev->intf_ptr, &dev_addr, sizeof(dev_addr));
    dev->intf = BME280_I2C_INTF;
    dev->read = i2c_reg_read;
    dev->write = i2c_reg_write;
    dev->delay_us = i2c_delay_uc;

    rslt = bme280_init(dev);
    if (rslt != BME280_OK) {
        DS_LOG_ERROR(TAG, "Failed to initialize the device (code 0x%x)", rslt);
        return 1;
    }

    DS_LOG_INFO(TAG, "Sensor initialized correctly ...");
    return 0;
}

_Noreturn void sensor_task(void *pvParameters) {
    if (i2c_init() != 0) {
        DS_LOG_ERROR(TAG, "Cannot initialize the i2c protocol");
        abort();
    }

    struct bme280_dev dev;
    if (sensor_init(&dev) != 0) {
        DS_LOG_ERROR(TAG, "Cannot initialize the sensor");
        abort();
    };

    if (sensor_config(&dev) != 0) {
        DS_LOG_ERROR(TAG, "Failed to configure the sensor");
        abort();
    }

    xQueueHandle *sensor_queue = (xQueueHandle *) pvParameters;

    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
    for (;;) {
        /* Structure to get the pressure, temperature and humidity values */
        struct bme280_data comp_data;
        int8_t rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        if (rslt != BME280_OK) {
            DS_LOG_WARNING(TAG, "Failed to get sensor data (code 0x%x)", rslt);
        }
//        print_sensor_data(&comp_data);

        // Creante the sensor_message
        message_t message = {
                .message_type = MESSAGE_TYPE_SENSOR_DATA_TEMP
        };
        message.message_data = malloc(sizeof(ds_temp_sensor_data_t));
        ds_temp_sensor_data_t *sensor_data = message.message_data;
        sensor_data->temperature = comp_data.temperature;

        if(xQueueSend(*sensor_queue, &message, 0) != pdTRUE){
            DS_LOG_WARNING(TAG, "Unable to send the data");
        };

        vTaskDelay(xDelay);

    }
}
