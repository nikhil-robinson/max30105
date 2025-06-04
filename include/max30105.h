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

#ifndef MAX30105_H
#define MAX30105_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "i2c_bus.h"
#include "helper.h"

#define MAX30105_ADDRESS          0x57 // 7-bit I2C Address
#define I2C_BUFFER_LENGTH         32   // Default buffer size for ESP32
#define STORAGE_SIZE              4     // Circular buffer size for sensor readings

// Status Registers
#define MAX30105_INTSTAT1         0x00
#define MAX30105_INTSTAT2         0x01
#define MAX30105_INTENABLE1       0x02
#define MAX30105_INTENABLE2       0x03

// FIFO Registers
#define MAX30105_FIFOWRITEPTR     0x04
#define MAX30105_FIFOOVERFLOW     0x05
#define MAX30105_FIFOREADPTR      0x06
#define MAX30105_FIFODATA         0x07

// Configuration Registers
#define MAX30105_FIFOCONFIG       0x08
#define MAX30105_MODECONFIG       0x09
#define MAX30105_PARTICLECONFIG   0x0A
#define MAX30105_LED1_PULSEAMP    0x0C
#define MAX30105_LED2_PULSEAMP    0x0D
#define MAX30105_LED3_PULSEAMP    0x0E
#define MAX30105_LED_PROX_AMP     0x10
#define MAX30105_MULTILEDCONFIG1  0x11
#define MAX30105_MULTILEDCONFIG2  0x12

// Die Temperature Registers
#define MAX30105_DIETEMPINT       0x1F
#define MAX30105_DIETEMPFRAC      0x20
#define MAX30105_DIETEMPCONFIG    0x21

// Proximity Function Registers
#define MAX30105_PROXINTTHRESH    0x30

// Part ID Registers
#define MAX30105_REVISIONID       0xFE
#define MAX30105_PARTID           0xFF

// MAX30105 Commands
#define MAX30105_INT_A_FULL_MASK      ((uint8_t)~0x80)
#define MAX30105_INT_A_FULL_ENABLE    0x80
#define MAX30105_INT_A_FULL_DISABLE   0x00

#define MAX30105_INT_DATA_RDY_MASK    ((uint8_t)~0x40)
#define MAX30105_INT_DATA_RDY_ENABLE  0x40
#define MAX30105_INT_DATA_RDY_DISABLE 0x00

#define MAX30105_INT_ALC_OVF_MASK     ((uint8_t)~0x20)
#define MAX30105_INT_ALC_OVF_ENABLE   0x20
#define MAX30105_INT_ALC_OVF_DISABLE  0x00

#define MAX30105_INT_PROX_INT_MASK    ((uint8_t)~0x10)
#define MAX30105_INT_PROX_INT_ENABLE  0x10
#define MAX30105_INT_PROX_INT_DISABLE 0x00

#define MAX30105_INT_DIE_TEMP_RDY_MASK    ((uint8_t)~0x02)
#define MAX30105_INT_DIE_TEMP_RDY_ENABLE  0x02
#define MAX30105_INT_DIE_TEMP_RDY_DISABLE 0x00

#define MAX30105_SAMPLEAVG_MASK       ((uint8_t)~0xE0)
#define MAX30105_SAMPLEAVG_1          0x00
#define MAX30105_SAMPLEAVG_2          0x20
#define MAX30105_SAMPLEAVG_4          0x40
#define MAX30105_SAMPLEAVG_8          0x60
#define MAX30105_SAMPLEAVG_16         0x80
#define MAX30105_SAMPLEAVG_32         0xA0

#define MAX30105_ROLLOVER_MASK        0xEF
#define MAX30105_ROLLOVER_ENABLE      0x10
#define MAX30105_ROLLOVER_DISABLE     0x00

#define MAX30105_A_FULL_MASK          0xF0

#define MAX30105_SHUTDOWN_MASK        0x7F
#define MAX30105_SHUTDOWN             0x80
#define MAX30105_WAKEUP               0x00

#define MAX30105_RESET_MASK           0xBF
#define MAX30105_RESET                0x40

#define MAX30105_MODE_MASK            0xF8
#define MAX30105_MODE_REDONLY         0x02
#define MAX30105_MODE_REDIRONLY       0x03
#define MAX30105_MODE_MULTILED        0x07

#define MAX30105_ADCRANGE_MASK        0x9F
#define MAX30105_ADCRANGE_2048        0x00
#define MAX30105_ADCRANGE_4096        0x20
#define MAX30105_ADCRANGE_8192        0x40
#define MAX30105_ADCRANGE_16384       0x60

#define MAX30105_SAMPLERATE_MASK      0xE3
#define MAX30105_SAMPLERATE_50        0x00
#define MAX30105_SAMPLERATE_100       0x04
#define MAX30105_SAMPLERATE_200       0x08
#define MAX30105_SAMPLERATE_400       0x0C
#define MAX30105_SAMPLERATE_800       0x10
#define MAX30105_SAMPLERATE_1000      0x14
#define MAX30105_SAMPLERATE_1600      0x18
#define MAX30105_SAMPLERATE_3200      0x1C

#define MAX30105_PULSEWIDTH_MASK      0xFC
#define MAX30105_PULSEWIDTH_69        0x00
#define MAX30105_PULSEWIDTH_118       0x01
#define MAX30105_PULSEWIDTH_215       0x02
#define MAX30105_PULSEWIDTH_411       0x03

#define MAX30105_SLOT1_MASK           0xF8
#define MAX30105_SLOT2_MASK           0x8F
#define MAX30105_SLOT3_MASK           0xF8
#define MAX30105_SLOT4_MASK           0x8F

#define SLOT_NONE                     0x00
#define SLOT_RED_LED                  0x01
#define SLOT_IR_LED                   0x02
#define SLOT_GREEN_LED                0x03
#define SLOT_NONE_PILOT               0x04
#define SLOT_RED_PILOT                0x05
#define SLOT_IR_PILOT                 0x06
#define SLOT_GREEN_PILOT              0x07

#define MAX30105_EXPECTEDPARTID       0x15

// Sensor data structure
typedef struct {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
} max30105_sense_t;

// MAX30105 device structure
typedef struct {
    i2c_bus_device_handle_t i2c_device; // I2C device handle
    uint8_t i2c_addr;    // I2C address
    uint8_t active_leds; // Number of active LEDs
    uint8_t revision_id; // Revision ID
    max30105_sense_t sense; // Sensor data
} max30105_t;

// Initialization and Configuration
esp_err_t max30105_init(max30105_t *sensor, i2c_bus_handle_t i2c_bus, uint8_t i2c_addr, uint32_t i2c_speed);
esp_err_t max30105_setup(max30105_t *sensor, uint8_t power_level, uint8_t sample_average, uint8_t led_mode,
                         int sample_rate, int pulse_width, int adc_range);

// Configuration Functions
esp_err_t max30105_soft_reset(max30105_t *sensor);
esp_err_t max30105_shutdown(max30105_t *sensor);
esp_err_t max30105_wakeup(max30105_t *sensor);
esp_err_t max30105_set_led_mode(max30105_t *sensor, uint8_t mode);
esp_err_t max30105_set_adc_range(max30105_t *sensor, uint8_t adc_range);
esp_err_t max30105_set_sample_rate(max30105_t *sensor, uint8_t sample_rate);
esp_err_t max30105_set_pulse_width(max30105_t *sensor, uint8_t pulse_width);
esp_err_t max30105_set_pulse_amplitude_red(max30105_t *sensor, uint8_t amplitude);
esp_err_t max30105_set_pulse_amplitude_ir(max30105_t *sensor, uint8_t amplitude);
esp_err_t max30105_set_pulse_amplitude_green(max30105_t *sensor, uint8_t amplitude);
esp_err_t max30105_set_pulse_amplitude_proximity(max30105_t *sensor, uint8_t amplitude);
esp_err_t max30105_set_proximity_threshold(max30105_t *sensor, uint8_t thresh_msb);
esp_err_t max30105_enable_slot(max30105_t *sensor, uint8_t slot_number, uint8_t device);
esp_err_t max30105_disable_slots(max30105_t *sensor);

// Interrupt Configuration
esp_err_t max30105_get_int1(max30105_t *sensor, uint8_t *int1);
esp_err_t max30105_get_int2(max30105_t *sensor, uint8_t *int2);
esp_err_t max30105_enable_afull(max30105_t *sensor);
esp_err_t max30105_disable_afull(max30105_t *sensor);
esp_err_t max30105_enable_datardy(max30105_t *sensor);
esp_err_t max30105_disable_datardy(max30105_t *sensor);
esp_err_t max30105_enable_alcovf(max30105_t *sensor);
esp_err_t max30105_disable_alcovf(max30105_t *sensor);
esp_err_t max30105_enable_proxint(max30105_t *sensor);
esp_err_t max30105_disable_proxint(max30105_t *sensor);
esp_err_t max30105_enable_dietemprdy(max30105_t *sensor);
esp_err_t max30105_disable_dietemprdy(max30105_t *sensor);

// FIFO Configuration
esp_err_t max30105_set_fifo_average(max30105_t *sensor, uint8_t number_of_samples);
esp_err_t max30105_enable_fifo_rollover(max30105_t *sensor);
esp_err_t max30105_disable_fifo_rollover(max30105_t *sensor);
esp_err_t max30105_set_fifo_almost_full(max30105_t *sensor, uint8_t number_of_samples);
esp_err_t max30105_clear_fifo(max30105_t *sensor);
esp_err_t max30105_get_write_pointer(max30105_t *sensor, uint8_t *write_ptr);
esp_err_t max30105_get_read_pointer(max30105_t *sensor, uint8_t *read_ptr);

// Data Collection
uint8_t max30105_available(max30105_t *sensor);
esp_err_t max30105_check(max30105_t *sensor, uint16_t *num_samples);
bool max30105_safe_check(max30105_t *sensor, uint8_t max_time_to_check);
uint32_t max30105_get_red(max30105_t *sensor);
uint32_t max30105_get_ir(max30105_t *sensor);
uint32_t max30105_get_green(max30105_t *sensor);
uint32_t max30105_get_fifo_red(max30105_t *sensor);
uint32_t max30105_get_fifo_ir(max30105_t *sensor);
uint32_t max30105_get_fifo_green(max30105_t *sensor);
void max30105_next_sample(max30105_t *sensor);

// Temperature Reading
esp_err_t max30105_read_temperature(max30105_t *sensor, float *temperature);
esp_err_t max30105_read_temperature_f(max30105_t *sensor, float *temperature);

// Device ID and Revision
esp_err_t max30105_read_part_id(max30105_t *sensor, uint8_t *part_id);
esp_err_t max30105_read_revision_id(max30105_t *sensor, uint8_t *revision_id);

#endif // MAX30105_H