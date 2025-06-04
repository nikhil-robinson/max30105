#include "max30105.h"
#include "spo2_algorithm.h"
#include "i2c_bus.h"

#define I2C_MASTER_SCL_IO   (gpio_num_t)15       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   (gpio_num_t)16       /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ  100000               /*!< I2C master clock frequency */


uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data

int32_t bufferLength;  // data length
int32_t spo2;          // SPO2 value
int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
int32_t heartRate;     // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

uint8_t pulseLED = 11; // Must be on PWM pin
uint8_t readLED = 13;  // Blinks with each data read



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

    uint8_t ledBrightness = 60; // Options: 0=Off to 255=50mA
    uint8_t sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
    uint8_t ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    uint8_t sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411;       // Options: 69, 118, 215, 411
    int adcRange = 4096;        // Options: 2048, 4096, 8192, 16384

    ESP_ERROR_CHECK(max30105_setup(&sensor, ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange));

    uint32_t staart_time = esp_timer_get_time() / 1000; // Get start time in milliseconds

    while (1)
    {
        // Check the sensor, read up to 3 samples
        uint16_t num_of_samples = 0;

        for (uint8_t i = 0; i < bufferLength; i++)
        {
            while (max30105_available(&sensor) == false)                   // do we have new data?
                max30105_check(&sensor, &num_of_samples));                 // Check the sensor for new data

            redBuffer[i] = max30105_get_red(&sensor);       // Read the red LED value
            irBuffer[i] = max30105_get_ir(&sensor);         // Read the IR LED value
            ESP_ERROR_CHECK(max30105_next_sample(&sensor)); // Check the sensor again for new data
        }
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

        while (1)
        {
            // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
            for (uint8_t i = 25; i < 100; i++)
            {
                redBuffer[i - 25] = redBuffer[i];
                irBuffer[i - 25] = irBuffer[i];
            }

            // take 25 sets of samples before calculating the heart rate.
            for (uint8_t i = 75; i < 100; i++)
            {
                while (max30105_available() == false) // do we have new data?
                    max30105_check();                 // Check the sensor for new data

                redBuffer[i] = max30105_get_red();
                irBuffer[i] = max30105_get_ir();
                max30105_next_sample(); // We're finished with this sample so move to next sample

                // send samples and calculation result to terminal program through UART
                printf("red=%u, ir=%u, HR=%d, HRvalid=%d, SPO2=%d, SPO2Valid=%d\n",
                       redBuffer[i],
                       irBuffer[i],
                       heartRate,
                       validHeartRate,
                       spo2,
                       validSPO2);
            }

            // After gathering 25 new samples recalculate HR and SP02
            maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
        }
        vTaskDelay(1); // Delay to avoid busy-waiting
    }
}