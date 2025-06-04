#include "max30105.h"
#include "heartRate.h"
#include "i2c_bus.h"

#define I2C_MASTER_SCL_IO   (gpio_num_t)15       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   (gpio_num_t)16       /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ  100000               /*!< I2C master clock frequency */
#define DATA_LENGTH         64                   /*!<Data buffer length for test buffer*/


void app_main(void) {
    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_bus_handle_t i2c0_bus = i2c_bus_create(I2C_NUM_0, &conf);

    // Initialize MAX30105
    max30105_t sensor;
    ESP_ERROR_CHECK(max30105_init(&sensor, i2c0_bus, MAX30105_ADDRESS, 100000));
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