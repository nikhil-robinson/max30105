/***************************************************
  This is a library written for the Maxim MAX30105 Optical Smoke Detector.
  It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

  These sensors use I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  Originally written by Peter Jansen and Nathan Seidle (SparkFun)
  Copyright (c) SparkFun Electronics
  BSD License - all text above must be included in any redistribution.

  --------------------------------------------------
  ESP-IDF C Port by Nikhil Robinson, 2025
  Modifications and ESP-IDF adaptation licensed under the MIT License.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above attribution and original BSD license from SparkFun must be retained
  in any distribution.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *****************************************************/


#include "max30105.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

static const char *TAG = "MAX30105";

// Low-level I2C Communication
static esp_err_t max30105_read_register8(max30105_t *sensor, uint8_t reg, uint8_t *value) {
    // return i2c_master_write_read_device(sensor->i2c_port, sensor->i2c_addr, &reg, 1, value, 1, pdMS_TO_TICKS(100));
    return i2c_bus_read_bytes(sensor->i2c_device, reg, 1, value);
}

static esp_err_t max30105_write_register8(max30105_t *sensor, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    // return i2c_master_write_to_device(sensor->i2c_port, sensor->i2c_addr, data, 2, pdMS_TO_TICKS(100));
    return i2c_bus_write_bytes(sensor->i2c_device , reg, 1, data);
}

static esp_err_t max30105_bit_mask(max30105_t *sensor, uint8_t reg, uint8_t mask, uint8_t thing) {
    uint8_t original_contents;
    esp_err_t ret = max30105_read_register8(sensor, reg, &original_contents);
    if (ret != ESP_OK) return ret;
    
    original_contents = original_contents & mask;
    return max30105_write_register8(sensor, reg, original_contents | thing);
}

// Initialization
esp_err_t max30105_init(max30105_t *sensor, i2c_bus_handle_t i2c_bus, uint8_t i2c_addr, uint32_t i2c_speed) {
    sensor->i2c_device = i2c_bus_device_create(i2c_bus, i2c_addr, 0);
    sensor->i2c_addr = i2c_addr;
    sensor->active_leds = 0;
    sensor->revision_id = 0;
    memset(&sensor->sense, 0, sizeof(max30105_sense_t));

    // Verify part ID
    uint8_t part_id;
    esp_err_t ret = max30105_read_part_id(sensor, &part_id);
    if (ret != ESP_OK || part_id != MAX30105_EXPECTEDPARTID) {
        ESP_LOGE(TAG, "Failed to verify part ID: 0x%02x", part_id);
        return ESP_ERR_NOT_FOUND;
    }

    // Read revision ID
    ret = max30105_read_revision_id(sensor, &sensor->revision_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read revision ID");
        return ret;
    }

    return ESP_OK;
}

// Configuration
esp_err_t max30105_setup(max30105_t *sensor, uint8_t power_level, uint8_t sample_average, uint8_t led_mode,
                         int sample_rate, int pulse_width, int adc_range) {
    esp_err_t ret;

    ret = max30105_soft_reset(sensor);
    if (ret != ESP_OK) return ret;

    // FIFO Configuration
    if (sample_average == 1) ret = max30105_set_fifo_average(sensor, MAX30105_SAMPLEAVG_1);
    else if (sample_average == 2) ret = max30105_set_fifo_average(sensor, MAX30105_SAMPLEAVG_2);
    else if (sample_average == 4) ret = max30105_set_fifo_average(sensor, MAX30105_SAMPLEAVG_4);
    else if (sample_average == 8) ret = max30105_set_fifo_average(sensor, MAX30105_SAMPLEAVG_8);
    else if (sample_average == 16) ret = max30105_set_fifo_average(sensor, MAX30105_SAMPLEAVG_16);
    else if (sample_average == 32) ret = max30105_set_fifo_average(sensor, MAX30105_SAMPLEAVG_32);
    else ret = max30105_set_fifo_average(sensor, MAX30105_SAMPLEAVG_4);
    if (ret != ESP_OK) return ret;

    ret = max30105_enable_fifo_rollover(sensor);
    if (ret != ESP_OK) return ret;

    // Mode Configuration
    if (led_mode == 3) ret = max30105_set_led_mode(sensor, MAX30105_MODE_MULTILED);
    else if (led_mode == 2) ret = max30105_set_led_mode(sensor, MAX30105_MODE_REDIRONLY);
    else ret = max30105_set_led_mode(sensor, MAX30105_MODE_REDONLY);
    if (ret != ESP_OK) return ret;
    sensor->active_leds = led_mode;

    // Particle Sensing Configuration
    if (adc_range < 4096) ret = max30105_set_adc_range(sensor, MAX30105_ADCRANGE_2048);
    else if (adc_range < 8192) ret = max30105_set_adc_range(sensor, MAX30105_ADCRANGE_4096);
    else if (adc_range < 16384) ret = max30105_set_adc_range(sensor, MAX30105_ADCRANGE_8192);
    else if (adc_range == 16384) ret = max30105_set_adc_range(sensor, MAX30105_ADCRANGE_16384);
    else ret = max30105_set_adc_range(sensor, MAX30105_ADCRANGE_2048);
    if (ret != ESP_OK) return ret;

    if (sample_rate < 100) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_50);
    else if (sample_rate < 200) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_100);
    else if (sample_rate < 400) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_200);
    else if (sample_rate < 800) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_400);
    else if (sample_rate < 1000) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_800);
    else if (sample_rate < 1600) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_1000);
    else if (sample_rate < 3200) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_1600);
    else if (sample_rate == 3200) ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_3200);
    else ret = max30105_set_sample_rate(sensor, MAX30105_SAMPLERATE_50);
    if (ret != ESP_OK) return ret;

    if (pulse_width < 118) ret = max30105_set_pulse_width(sensor, MAX30105_PULSEWIDTH_69);
    else if (pulse_width < 215) ret = max30105_set_pulse_width(sensor, MAX30105_PULSEWIDTH_118);
    else if (pulse_width < 411) ret = max30105_set_pulse_width(sensor, MAX30105_PULSEWIDTH_215);
    else if (pulse_width == 411) ret = max30105_set_pulse_width(sensor, MAX30105_PULSEWIDTH_411);
    else ret = max30105_set_pulse_width(sensor, MAX30105_PULSEWIDTH_69);
    if (ret != ESP_OK) return ret;

    // LED Pulse Amplitude Configuration
    ret = max30105_set_pulse_amplitude_red(sensor, power_level);
    if (ret != ESP_OK) return ret;
    ret = max30105_set_pulse_amplitude_ir(sensor, power_level);
    if (ret != ESP_OK) return ret;
    ret = max30105_set_pulse_amplitude_green(sensor, power_level);
    if (ret != ESP_OK) return ret;
    ret = max30105_set_pulse_amplitude_proximity(sensor, power_level);
    if (ret != ESP_OK) return ret;

    // Multi-LED Mode Configuration
    ret = max30105_enable_slot(sensor, 1, SLOT_RED_LED);
    if (ret != ESP_OK) return ret;
    if (led_mode > 1) {
        ret = max30105_enable_slot(sensor, 2, SLOT_IR_LED);
        if (ret != ESP_OK) return ret;
    }
    if (led_mode > 2) {
        ret = max30105_enable_slot(sensor, 3, SLOT_GREEN_LED);
        if (ret != ESP_OK) return ret;
    }

    ret = max30105_clear_fifo(sensor);
    return ret;
}

// Configuration Functions
esp_err_t max30105_soft_reset(max30105_t *sensor) {
    esp_err_t ret = max30105_bit_mask(sensor, MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);
    if (ret != ESP_OK) return ret;

    uint64_t start_time = esp_timer_get_time() / 1000; // Convert to ms
    while (esp_timer_get_time() / 1000 - start_time < 100) {
        uint8_t response;
        ret = max30105_read_register8(sensor, MAX30105_MODECONFIG, &response);
        if (ret != ESP_OK) return ret;
        if ((response & MAX30105_RESET) == 0) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_OK;
}

esp_err_t max30105_shutdown(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

esp_err_t max30105_wakeup(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

esp_err_t max30105_set_led_mode(max30105_t *sensor, uint8_t mode) {
    return max30105_bit_mask(sensor, MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

esp_err_t max30105_set_adc_range(max30105_t *sensor, uint8_t adc_range) {
    return max30105_bit_mask(sensor, MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adc_range);
}

esp_err_t max30105_set_sample_rate(max30105_t *sensor, uint8_t sample_rate) {
    return max30105_bit_mask(sensor, MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sample_rate);
}

esp_err_t max30105_set_pulse_width(max30105_t *sensor, uint8_t pulse_width) {
    return max30105_bit_mask(sensor, MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulse_width);
}

esp_err_t max30105_set_pulse_amplitude_red(max30105_t *sensor, uint8_t amplitude) {
    return max30105_write_register8(sensor, MAX30105_LED1_PULSEAMP, amplitude);
}

esp_err_t max30105_set_pulse_amplitude_ir(max30105_t *sensor, uint8_t amplitude) {
    return max30105_write_register8(sensor, MAX30105_LED2_PULSEAMP, amplitude);
}

esp_err_t max30105_set_pulse_amplitude_green(max30105_t *sensor, uint8_t amplitude) {
    return max30105_write_register8(sensor, MAX30105_LED3_PULSEAMP, amplitude);
}

esp_err_t max30105_set_pulse_amplitude_proximity(max30105_t *sensor, uint8_t amplitude) {
    return max30105_write_register8(sensor, MAX30105_LED_PROX_AMP, amplitude);
}

esp_err_t max30105_set_proximity_threshold(max30105_t *sensor, uint8_t thresh_msb) {
    return max30105_write_register8(sensor, MAX30105_PROXINTTHRESH, thresh_msb);
}

esp_err_t max30105_enable_slot(max30105_t *sensor, uint8_t slot_number, uint8_t device) {
    switch (slot_number) {
        case 1:
            return max30105_bit_mask(sensor, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
        case 2:
            return max30105_bit_mask(sensor, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
        case 3:
            return max30105_bit_mask(sensor, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
        case 4:
            return max30105_bit_mask(sensor, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t max30105_disable_slots(max30105_t *sensor) {
    esp_err_t ret = max30105_write_register8(sensor, MAX30105_MULTILEDCONFIG1, 0);
    if (ret != ESP_OK) return ret;
    return max30105_write_register8(sensor, MAX30105_MULTILEDCONFIG2, 0);
}

// Interrupt Configuration
esp_err_t max30105_get_int1(max30105_t *sensor, uint8_t *int1) {
    return max30105_read_register8(sensor, MAX30105_INTSTAT1, int1);
}

esp_err_t max30105_get_int2(max30105_t *sensor, uint8_t *int2) {
    return max30105_read_register8(sensor, MAX30105_INTSTAT2, int2);
}

esp_err_t max30105_enable_afull(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_ENABLE);
}

esp_err_t max30105_disable_afull(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_DISABLE);
}

esp_err_t max30105_enable_datardy(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_ENABLE);
}

esp_err_t max30105_disable_datardy(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_DISABLE);
}

esp_err_t max30105_enable_alcovf(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_ENABLE);
}

esp_err_t max30105_disable_alcovf(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_DISABLE);
}

esp_err_t max30105_enable_proxint(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_ENABLE);
}

esp_err_t max30105_disable_proxint(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_DISABLE);
}

esp_err_t max30105_enable_dietemprdy(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_ENABLE);
}

esp_err_t max30105_disable_dietemprdy(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_DISABLE);
}

// FIFO Configuration
esp_err_t max30105_set_fifo_average(max30105_t *sensor, uint8_t number_of_samples) {
    return max30105_bit_mask(sensor, MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, number_of_samples);
}

esp_err_t max30105_enable_fifo_rollover(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

esp_err_t max30105_disable_fifo_rollover(max30105_t *sensor) {
    return max30105_bit_mask(sensor, MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

esp_err_t max30105_set_fifo_almost_full(max30105_t *sensor, uint8_t number_of_samples) {
    return max30105_bit_mask(sensor, MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, number_of_samples);
}

esp_err_t max30105_clear_fifo(max30105_t *sensor) {
    esp_err_t ret = max30105_write_register8(sensor, MAX30105_FIFOWRITEPTR, 0);
    if (ret != ESP_OK) return ret;
    ret = max30105_write_register8(sensor, MAX30105_FIFOOVERFLOW, 0);
    if (ret != ESP_OK) return ret;
    return max30105_write_register8(sensor, MAX30105_FIFOREADPTR, 0);
}

esp_err_t max30105_get_write_pointer(max30105_t *sensor, uint8_t *write_ptr) {
    return max30105_read_register8(sensor, MAX30105_FIFOWRITEPTR, write_ptr);
}

esp_err_t max30105_get_read_pointer(max30105_t *sensor, uint8_t *read_ptr) {
    return max30105_read_register8(sensor, MAX30105_FIFOREADPTR, read_ptr);
}

// Data Collection
uint8_t max30105_available(max30105_t *sensor) {
    int8_t number_of_samples = sensor->sense.head - sensor->sense.tail;
    if (number_of_samples < 0) number_of_samples += STORAGE_SIZE;
    return (uint8_t)number_of_samples;
}

esp_err_t max30105_check(max30105_t *sensor, uint16_t *num_samples) {
    uint8_t read_ptr, write_ptr;
    esp_err_t ret = max30105_get_read_pointer(sensor, &read_ptr);
    if (ret != ESP_OK) return ret;
    ret = max30105_get_write_pointer(sensor, &write_ptr);
    if (ret != ESP_OK) return ret;

    *num_samples = 0;
    if (read_ptr != write_ptr) {
        int number_of_samples = write_ptr - read_ptr;
        if (number_of_samples < 0) number_of_samples += 32;

        int bytes_to_read = number_of_samples * sensor->active_leds * 3;

        uint8_t reg = MAX30105_FIFODATA;
#if 0
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (sensor->i2c_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(sensor->i2c_port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) return ret;
#endif

        while (bytes_to_read > 0) {
            int to_get = bytes_to_read;
            if (to_get > I2C_BUFFER_LENGTH) {
                to_get = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (sensor->active_leds * 3));
            }
            bytes_to_read -= to_get;

            uint8_t buffer[I2C_BUFFER_LENGTH];
            // ret = i2c_master_read_from_device(sensor->i2c_port, sensor->i2c_addr, buffer, to_get, pdMS_TO_TICKS(100));
            ret = i2c_bus_read_bytes(sensor->i2c_device, reg, to_get, buffer);
            if (ret != ESP_OK) return ret;

            int offset = 0;
            while (to_get > 0) {
                sensor->sense.head++;
                sensor->sense.head %= STORAGE_SIZE;

                uint8_t temp[4];
                uint32_t temp_long;

                // Read Red
                temp[3] = 0;
                temp[2] = buffer[offset++];
                temp[1] = buffer[offset++];
                temp[0] = buffer[offset++];
                memcpy(&temp_long, temp, sizeof(temp_long));
                temp_long &= 0x3FFFF;
                sensor->sense.red[sensor->sense.head] = temp_long;

                if (sensor->active_leds > 1) {
                    // Read IR
                    temp[3] = 0;
                    temp[2] = buffer[offset++];
                    temp[1] = buffer[offset++];
                    temp[0] = buffer[offset++];
                    memcpy(&temp_long, temp, sizeof(temp_long));
                    temp_long &= 0x3FFFF;
                    sensor->sense.IR[sensor->sense.head] = temp_long;
                }

                if (sensor->active_leds > 2) {
                    // Read Green
                    temp[3] = 0;
                    temp[2] = buffer[offset++];
                    temp[1] = buffer[offset++];
                    temp[0] = buffer[offset++];
                    memcpy(&temp_long, temp, sizeof(temp_long));
                    temp_long &= 0x3FFFF;
                    sensor->sense.green[sensor->sense.head] = temp_long;
                }

                to_get -= sensor->active_leds * 3;
            }
        }

        *num_samples = number_of_samples;
    }

    return ESP_OK;
}

bool max30105_safe_check(max30105_t *sensor, uint8_t max_time_to_check) {
    uint64_t mark_time = esp_timer_get_time() / 1000;
    while (1) {
        if (esp_timer_get_time() / 1000 - mark_time > max_time_to_check) return false;
        uint16_t num_samples;
        if (max30105_check(sensor, &num_samples) == ESP_OK && num_samples > 0) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

uint32_t max30105_get_red(max30105_t *sensor) {
    if (max30105_safe_check(sensor, 250)) return sensor->sense.red[sensor->sense.head];
    return 0;
}

uint32_t max30105_get_ir(max30105_t *sensor) {
    if (max30105_safe_check(sensor, 250)) return sensor->sense.IR[sensor->sense.head];
    return 0;
}

uint32_t max30105_get_green(max30105_t *sensor) {
    if (max30105_safe_check(sensor, 250)) return sensor->sense.green[sensor->sense.head];
    return 0;
}

uint32_t max30105_get_fifo_red(max30105_t *sensor) {
    return sensor->sense.red[sensor->sense.tail];
}

uint32_t max30105_get_fifo_ir(max30105_t *sensor) {
    return sensor->sense.IR[sensor->sense.tail];
}

uint32_t max30105_get_fifo_green(max30105_t *sensor) {
    return sensor->sense.green[sensor->sense.tail];
}

void max30105_next_sample(max30105_t *sensor) {
    if (max30105_available(sensor)) {
        sensor->sense.tail++;
        sensor->sense.tail %= STORAGE_SIZE;
    }
}

// Temperature Reading
esp_err_t max30105_read_temperature(max30105_t *sensor, float *temperature) {
    esp_err_t ret = max30105_write_register8(sensor, MAX30105_DIETEMPCONFIG, 0x01);
    if (ret != ESP_OK) return ret;

    uint64_t start_time = esp_timer_get_time() / 1000;
    while (esp_timer_get_time() / 1000 - start_time < 100) {
        uint8_t response;
        ret = max30105_read_register8(sensor, MAX30105_INTSTAT2, &response);
        if (ret != ESP_OK) return ret;
        if (response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    int8_t temp_int;
    uint8_t temp_frac;
    ret = max30105_read_register8(sensor, MAX30105_DIETEMPINT, (uint8_t *)&temp_int);
    if (ret != ESP_OK) return ret;
    ret = max30105_read_register8(sensor, MAX30105_DIETEMPFRAC, &temp_frac);
    if (ret != ESP_OK) return ret;

    *temperature = (float)temp_int + ((float)temp_frac * 0.0625);
    return ESP_OK;
}

esp_err_t max30105_read_temperature_f(max30105_t *sensor, float *temperature) {
    esp_err_t ret = max30105_read_temperature(sensor, temperature);
    if (ret == ESP_OK && *temperature != -999.0) {
        *temperature = *temperature * 1.8 + 32.0;
    }
    return ret;
}

// Device ID and Revision
esp_err_t max30105_read_part_id(max30105_t *sensor, uint8_t *part_id) {
    return max30105_read_register8(sensor, MAX30105_PARTID, part_id);
}

esp_err_t max30105_read_revision_id(max30105_t *sensor, uint8_t *revision_id) {
    return max30105_read_register8(sensor, MAX30105_REVISIONID, revision_id);
}