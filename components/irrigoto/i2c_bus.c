#include "i2c_bus.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "i2c_bus";

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    GPIO_NUM_17
#define I2C_SCL_GPIO    GPIO_NUM_23
#define I2C_FREQ_HZ     400000
#define I2C_TIMEOUT_MS  100

static SemaphoreHandle_t s_mutex = NULL;
static bool s_init = false;

esp_err_t i2c_bus_init(void)
{
    if (s_init) return ESP_OK;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_GPIO,
        .scl_io_num       = I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    esp_err_t r = i2c_param_config(I2C_PORT, &conf);
    if (r != ESP_OK) { vSemaphoreDelete(s_mutex); s_mutex = NULL; return r; }

    r = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (r == ESP_ERR_INVALID_STATE) {
        // Driver already installed -- treat as success
        r = ESP_OK;
    }
    if (r != ESP_OK) { vSemaphoreDelete(s_mutex); s_mutex = NULL; return r; }

    // Set clock stretch timeout -- some sensors hold SCL low while preparing data
    // Default is too short; 40ms covers slow sensors like MPRLS variants
    i2c_set_timeout(I2C_PORT, 0xFFFFF);    // max hardware value ~13ms @ 80MHz APB

    s_init = true;
    ESP_LOGI(TAG, "Init SDA=GPIO%d SCL=GPIO%d @ %dkHz",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ / 1000);
    return ESP_OK;
}

esp_err_t i2c_bus_deinit(void)
{
    if (!s_init) return ESP_OK;
    i2c_driver_delete(I2C_PORT);
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    s_init  = false;
    return ESP_OK;
}

static bool take(void)
{
    return s_init && s_mutex &&
           xSemaphoreTake(s_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE;
}

esp_err_t i2c_bus_write(uint8_t addr, const uint8_t *data, size_t len)
{
    if (!take()) return ESP_ERR_TIMEOUT;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    if (data && len > 0) i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(s_mutex);
    if (r != ESP_OK && len > 0)
        ESP_LOGW(TAG, "write 0x%02x failed: %s", addr, esp_err_to_name(r));
    return r;
}

esp_err_t i2c_bus_write_reg(uint8_t addr, uint8_t reg,
                             const uint8_t *data, size_t len)
{
    if (!take()) return ESP_ERR_TIMEOUT;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (data && len > 0) i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(s_mutex);
    if (r != ESP_OK)
        ESP_LOGW(TAG, "write_reg 0x%02x:0x%02x failed: %s",
                 addr, reg, esp_err_to_name(r));
    return r;
}

esp_err_t i2c_bus_read_reg(uint8_t addr, uint8_t reg,
                            uint8_t *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    if (!take()) return ESP_ERR_TIMEOUT;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(s_mutex);
    if (r != ESP_OK)
        ESP_LOGW(TAG, "read_reg 0x%02x:0x%02x failed: %s",
                 addr, reg, esp_err_to_name(r));
    return r;
}

esp_err_t i2c_bus_read(uint8_t addr, uint8_t *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    if (!take()) return ESP_ERR_TIMEOUT;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(s_mutex);
    if (r != ESP_OK)
        ESP_LOGW(TAG, "read 0x%02x failed: %s", addr, esp_err_to_name(r));
    return r;
}
