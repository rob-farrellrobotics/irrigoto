#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize I2C master bus
 *        SDA=GPIO21, SCL=GPIO22, 400kHz
 *        Must be called after 3V3Sen rail is asserted and stable.
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Deinitialize I2C bus (call before deep sleep)
 */
esp_err_t i2c_bus_deinit(void);

/**
 * @brief Write bytes to device (no register prefix)
 */
esp_err_t i2c_bus_write(uint8_t addr, const uint8_t *data, size_t len);

/**
 * @brief Write to a specific register address
 */
esp_err_t i2c_bus_write_reg(uint8_t addr, uint8_t reg,
                             const uint8_t *data, size_t len);

/**
 * @brief Read from a specific register address (write-then-read)
 */
esp_err_t i2c_bus_read_reg(uint8_t addr, uint8_t reg,
                            uint8_t *data, size_t len);

/**
 * @brief Read bytes from device (no register prefix)
 */
esp_err_t i2c_bus_read(uint8_t addr, uint8_t *data, size_t len);

/**
 * @brief Scan bus and log all responding addresses (debug only)
 */
esp_err_t i2c_bus_scan(void);
