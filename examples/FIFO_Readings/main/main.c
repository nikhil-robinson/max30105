#include "max30105.h"
#include "max30105.h"
#include "i2c_bus.h"

#define I2C_MASTER_SCL_IO   (gpio_num_t)15       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   (gpio_num_t)16       /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ  100000               /*!< I2C master clock frequency */



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

    //Setup to sense up to 18 inches, max LED brightness
    uint8_t ledBrightness = 0xFF; //Options: 0=Off to 255=50mA
    uint8_t sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
    uint8_t ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange = 2048; //Options: 2048, 4096, 8192, 16384

    ESP_ERROR_CHECK(max30105_setup(&sensor, ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange));
    
    uint32_t startTime = esp_timer_get_time() / 1000; // Get start time in milliseconds
    while (1)
    {
        uint16_t num_of_samples =0;
        if (max30105_check(&sensor, &num_of_samples) == ESP_OK)
        {
            while (max30105_available(&sensor))
            { // Do we have new data?
                uint32_t red = max30105_get_red(&sensor);
                uint32_t ir = max30105_get_ir(&sensor);
                uint32_t green = max30105_get_green(&sensor);

                // Print the readings
                printf("R[%u] IR[%u] G[%u] Hz[%.2f]\n", red, ir, green, (float)num_of_samples / (((esp_timer_get_time() / 1000) - startTime) / 1000.0), 2);
                max30105_next_sample(&sensor); // Move to the next sample in the FIFO
                vTaskDealy(1);
            }
        }
        vTaskDelay(1); // Delay to avoid busy-waiting
    }
}