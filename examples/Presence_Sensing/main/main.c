#include "max30105.h"
#include <driver/i2c.h>

void app_main(void)
{
    // Configure I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21, // Adjust GPIO pins
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0));

    // Initialize MAX30105
    max30105_t sensor;
    ESP_ERROR_CHECK(max30105_init(&sensor, I2C_NUM_0, MAX30105_ADDRESS, 100000));

    // Setup to sense up to 18 inches, max LED brightness
    uint8_t ledBrightness = 0xFF; // Options: 0=Off to 255=50mA
    uint8_t sampleAverage = 4;    // Options: 1, 2, 4, 8, 16, 32
    uint8_t ledMode = 2;          // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    int sampleRate = 400;         // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411;         // Options: 69, 118, 215, 411
    int adcRange = 2048;          // Options: 2048, 4096, 8192, 16384
    ESP_ERROR_CHECK(max30105_setup(&sensor, ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange));
    ESP_ERROR_CHECK(max30105_set_pulse_amplitude_red(&sensor, 0));   // Turn off Red LED
    ESP_ERROR_CHECK(max30105_set_pulse_amplitude_green(&sensor, 0)); // Turn off Green LED

    uint32_t unblockedValue = 0;
    for (uint8_t x = 0; x < 32; x++)
    {
        unblockedValue += max30105_get_ir(); // Read the IR value
    }
    unblockedValue /= 32;

    uint32_t start_time = esp_timer_get_time() / 1000; // Get current time in milliseconds
    uint32_t samplesTaken = 0;
    while (1)
    {
        samplesTaken++;
        uint32_t ir_value = max30105_get_ir(&sensor);
        printf("IR[%u],HZ[%.2X] \n", ir_value, (float)samplesTaken / (((esp_timer_get_time() / 1000) - start_time) / 1000.0));

        uint32_t current_delta = ir_value - unblockedValue; // Calculate the difference from the unblocked value

        printf("Current Delta: %u\n", current_delta);
        if (current_delta > 100) // If the difference is greater than 1000, we assume the sensor is blocked
        {
            printf("Something is there\n");
        }
        vTaskDelay(1); // Delay for 1 second
    }
}