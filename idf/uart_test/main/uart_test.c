#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
static const char *TAG = "ESP_INTERFACE";

// Shared sensor data
static int16_t ax, ay, az;
static SemaphoreHandle_t data_mutex;

// UART configuration
#define UART_PORT_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024

#define I2C_MASTER_SCL_IO 22      // SCL pin
#define I2C_MASTER_SDA_IO 21      // SDA pin
#define I2C_MASTER_FREQ_HZ 400000 // I²C speed
#define I2C_MASTER_PORT I2C_NUM_0
#define MPU9250_ADDR 0x68 // AD0 = GND
#define MPU9250_WHO_AM_I 0x75
//--------------------------------------------------
// Mock sensor reading functions
//--------------------------------------------------

//--------------------------------------------------
// Sensor reading task (4kHz)
//--------------------------------------------------

typedef struct
{
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu9250_raw_data_t;

void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

esp_err_t mpu9250_read_reg(uint8_t reg_addr, uint8_t *data)
{
    return i2c_master_write_read_device(
        I2C_MASTER_PORT, MPU9250_ADDR, &reg_addr, 1, data, 1, pdMS_TO_TICKS(1000));
}

esp_err_t mpu9250_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_PORT, MPU9250_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(1000));
}

esp_err_t mpu9250_init(void)
{
    uint8_t who_am_i = 0;
    esp_err_t ret = mpu9250_read_reg(MPU9250_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    if (who_am_i != 0x70)
    { // Check if the device is MPU9250
        ESP_LOGE(TAG, "Unexpected WHO_AM_I value: 0x%02X", who_am_i);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MPU9250 initialized successfully");
    ESP_ERROR_CHECK(mpu9250_write_reg(0x1B, 0x00));

    // Accel config: ±2 g
    ESP_ERROR_CHECK(mpu9250_write_reg(0x1C, 0x00));

    // Low-pass filter config (optional)
    ESP_ERROR_CHECK(mpu9250_write_reg(0x1A, 0x03));
    return ESP_OK;
}
static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_PORT, MPU9250_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}
int16_t getAccelX()
{
    uint8_t buf[2];
    ESP_ERROR_CHECK(mpu_read(0x3B, buf, sizeof(buf)));
    int16_t ax = (buf[0] << 8) | buf[1];
    return ax;
}
int16_t getAccelY()
{
    uint8_t buf[2];
    ESP_ERROR_CHECK(mpu_read(0x3D, buf, sizeof(buf)));
    int16_t ay = (buf[0] << 8) | buf[1];
    return ay;
}
int16_t getAccelZ()
{
    uint8_t buf[2];
    ESP_ERROR_CHECK(mpu_read(0x3F, buf, sizeof(buf)));
    int16_t az = (buf[0] << 8) | buf[1];
    return az;
}

static void sensor_timer_cb(void *pv)
{
    // Initialize sensor data
    int16_t new_ax = 0;
    int16_t new_ay = 0;
    int16_t new_az = 0;
    new_ax = getAccelX();
    new_ay = getAccelY();
    new_az = getAccelZ();

    if (xSemaphoreTake(data_mutex, portMAX_DELAY))
    {
        ax = new_ax;
        ay = new_ay;
        az = new_az;
        xSemaphoreGive(data_mutex);
    }
}

//--------------------------------------------------
// Main entry point
//--------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP interface");

    // Create mutex
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // UART configuration
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);

    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");

    if (mpu9250_init() == ESP_OK)
    {
        ESP_LOGI(TAG, "MPU9250 initialized successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize MPU9250");
        return;
    }

    // sensor read callback
    esp_timer_create_args_t sensor_timer_args = {
        .callback = &sensor_timer_cb,
        .name = "sensor_read_timer",
    };
    esp_timer_handle_t sensor_timer;
    ESP_ERROR_CHECK(esp_timer_create(&sensor_timer_args, &sensor_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sensor_timer, 1000)); // 1khz

    // // control loop callback
    // const esp_timer_create_args_t control_timer_args = {
    //     .callback = &control_timer_cb,
    //     .name = "control_timer",
    // };
    // esp_timer_handle_t control_timer;
    // ESP_ERROR_CHECK(esp_timer_create(&control_timer_args, &control_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(control_timer, 2000)); // 500hz
    int8_t uart_buf[50];
    int8_t buf_idx = 0;
    // UART command handling loop
        ESP_LOGI(TAG, "starting UART command handling loop");

    while (1)
    {

        int len = uart_read_bytes(UART_PORT_NUM, uart_buf + buf_idx, 10, pdMS_TO_TICKS(10));

        buf_idx += len;
        switch (uart_buf[0])
        {
            case 'R': // read item
                int16_t value = 0;
                if (buf_idx < 2)
                {
                    uart_buf[buf_idx] = 0; // Null-terminate the buffer
                    continue;              // Not enough data yet
                }
                else
                {
                    uint8_t addr = uart_buf[1];
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY))
                    {
                        switch (addr)
                        {
                        case 0x01:
                            value = ax;
                            break;
                        case 0x02:
                            value = ay;
                            break;
                        case 0x03:
                            value = az;
                            break;
                        default:
                            // uart_write_bytes(UART_PORT_NUM, "ERR\n", 4);
                            xSemaphoreGive(data_mutex);
                            continue;
                        }
                        xSemaphoreGive(data_mutex);
                    }
                    uint8_t write_start = 0x77;
                    uint8_t val_low = value & 0xFF;
                    uint8_t val_high = (value >> 8) & 0xFF;
                    uint8_t checksum = addr ^ val_low ^ val_high;
                    uint8_t terminator = 0xAA; // Fixed terminator
                    uint8_t packet[6] = {write_start, addr, val_low, val_high, checksum, terminator};
                    for (int i = 0; i < 3; i++)
                    {
                        uart_write_bytes(UART_PORT_NUM, (const char *)packet, 6);
                    }
                    // remove two byts from start - 'R' and address
                    memmove(uart_buf, uart_buf + 2, buf_idx - 2);
                    buf_idx -= 2; // Adjust buffer index
                }
                break;

            case 'Q': // read batch
                if (buf_idx < 2)
                {
                    uart_buf[buf_idx] = 0; // Null-terminate the buffer
                    continue;              // Not enough data yet
                }
                if (uart_buf[1] == 0x10)
                {
                    uint8_t packet[10];
                    uint8_t idx = 0;
                    packet[idx++] = 0x77; // Start byte
                    packet[idx++] = 0x10; // Address for batch read
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY))
                    {
                        packet[idx++] = ax & 0xFF;        // ax low byte
                        packet[idx++] = (ax >> 8) & 0xFF; // ax high byte
                        packet[idx++] = ay & 0xFF;        // ay low byte
                        packet[idx++] = (ay >> 8) & 0xFF; // ay high byte
                        packet[idx++] = az & 0xFF;        // az low byte
                        packet[idx++] = (az >> 8) & 0xFF; // az high byte
                        xSemaphoreGive(data_mutex);
                    }
                    uint8_t checksum = 0;
                    for (int i = 1; i < idx; i++)
                    {
                        checksum ^= packet[i];
                    }
                    packet[idx++] = checksum; // Checksum
                    packet[idx++] = 0xAA;     // Terminator
                    uart_write_bytes(UART_PORT_NUM, (const char *)packet, idx);
                    // remove two byts from start - 'Q' and address
                    memmove(uart_buf, uart_buf + 2, buf_idx - 2);
                    buf_idx -= 2; // Adjust buffer index
                }
                break;
            case 'W': // write item
                if (buf_idx < 4)
                {
                    uart_buf[buf_idx] = 0; // Null-terminate the buffer
                    continue;              // Not enough data yet
                }
                if (buf_idx >= 4) // Ensure we have enough bytes to read
                {
                    uint8_t addr = uart_buf[1];
                    uint8_t val_low = uart_buf[2];
                    uint8_t val_high = uart_buf[3];
                    int16_t value = (val_high << 8) | val_low;
                    mpu9250_write_reg(addr, value >> 8);       // high byte
                    mpu9250_write_reg(addr + 1, value & 0xFF); // low byte
                    // remove four byts from start - 'W', address, low byte, high byte
                    memmove(uart_buf, uart_buf + 4, buf_idx - 4);
                    buf_idx -= 4; // Adjust buffer index
                }
                break;
            default:
                // remove first byte and shift buffer
                if(buf_idx > 0){

                    memmove(uart_buf, uart_buf + 1, buf_idx - 1);
                    buf_idx -= 1;
                }
                break;
        }
        //
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz loop
    }
}
