#include "max30105.h"
#include <driver/i2c.h>

void app_main(void) {
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


   while (1)
   {
        uint32_t red_value = max30105_get_red(&sensor);
        uint32_t ir_value = max30105_get_ir(&sensor);
        uint32_t green_value = max30105_get_green(&sensor);
        printf("R[%u], IR[%u], G[%u]\n", red_value, ir_value, green_value);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
   }

}