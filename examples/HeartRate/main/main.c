#include "max30105.h"
#include <driver/i2c.h>
#include "heartRate.h"

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
    ESP_ERROR_CHECK(max30105_setup(&sensor, 0x1F, 4, 3, 400, 411, 4096));
    ESP_ERROR_CHECK(max30105_set_pulse_amplitude_red(&sensor, 0x0A)); // Turn off Red LED
    ESP_ERROR_CHECK(max30105_set_pulse_amplitude_green(&sensor, 0));  // Turn off Green LED

    const uint8_t RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
    uint8_t rates[RATE_SIZE];    // Array of heart rates
    uint8_t rateSpot = 0;
    long lastBeat = 0; // Time at which the last beat occurred

    float beatsPerMinute;
    int beatAvg;

    while (1)
    {
        uint32_t ir_value = max30105_get_ir(&sensor);
        if (checkForBeat(ir_value) == true)
        {
            // We sensed a beat!
            long delta = esp_timer_get_time() / 1000 - lastBeat;
            lastBeat = esp_timer_get_time() / 1000;

            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
                rates[rateSpot++] = (uint8_t)beatsPerMinute; // Store this reading in the array
                rateSpot %= RATE_SIZE;                       // Wrap variable

                // Take average of readings
                beatAvg = 0;
                for (uint8_t x = 0; x < RATE_SIZE; x++)
                    beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
            }
        }

        printf("Heart Rate: %f bpm\n", beatsPerMinute);
        printf("IR Value: %ld\n", ir_value);
        printf("Avg Heart Rate: %d bpm\n", beatAvg);

        if (ir_value < 5000)
        {
            printf("No finger detected.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}