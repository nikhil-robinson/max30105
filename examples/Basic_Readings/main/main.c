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
    ESP_ERROR_CHECK(max30105_setup(&sensor, 0x1F, 4, 3, 400, 411, 4096));


   while (1)
   {
        uint32_t red_value = max30105_get_red(&sensor);
        uint32_t ir_value = max30105_get_ir(&sensor);
        uint32_t green_value = max30105_get_green(&sensor);
        printf("R[%lu], IR[%lu], G[%lu]\n", red_value, ir_value, green_value);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
   }

}