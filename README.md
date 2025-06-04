# MAX3010x Sensor Driver for ESP-IDF

This is a **C-based ESP-IDF driver** for the **Maxim Integrated MAX30105/MAX30102** optical sensors, adapted from the original [SparkFun MAX3010x Arduino Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library).

It supports reading photoplethysmographic data (red, IR, and green channels) via I2C, and is suitable for embedded applications like heart-rate monitoring, SpO2 tracking, and smoke detection—optimized for use on ESP32 with the ESP-IDF framework.

---

## 📦 Features

- Written in pure C for ESP-IDF  
- I2C communication  
- Works with MAX30102 and MAX30105 sensors  
- Initialization and configuration  
- FIFO data reading  
- LED power control and multi-LED mode configuration  
- Ported from the SparkFun Arduino Library  

---

## ⚙️ Hardware Compatibility

| Sensor    | Red LED | IR LED | Green LED |
|-----------|---------|--------|-----------|
| MAX30102  | ✅       | ✅      | ❌         |
| MAX30105  | ✅       | ✅      | ✅         |

This library uses **I2C**. Ensure you connect the sensor to the correct SDA and SCL pins and enable pull-ups (typically 4.7kΩ).

---

## 🧰 Requirements

- ESP32 or ESP32x series board  
- ESP-IDF v4.0 or later   
- MAX30102 or MAX30105 sensor module  

---

## 📥 Installation

Clone this repository inside your ESP-IDF project under the `components` folder:

```bash
cd your_project/components
git clone https://github.com/nikhil-robinson/max30105.git
````

Then include the component in your `CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS components/max30105)
```

Or add to `idf_component_register` if inside a monolithic `CMakeLists.txt`.

---

## 🧪 Example Usage

```c
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
```

A full example project is available in the `example/` directory.

---

## 📄 License

### Original Arduino Library:

* Written by Peter Jansen and Nathan Seidle (SparkFun Electronics)
* Licensed under the **BSD 3-Clause License**
* [SparkFun MAX3010x Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)

### ESP-IDF Port:

* Ported by Nikhil Robinson, 2025
* Licensed under the **MIT License**
* Derivative work retains BSD License as required by original authors

Please see the `LICENSE` file for full terms.

---

## 🤝 Acknowledgements

* Special thanks to **SparkFun Electronics** for the original MAX3010x Arduino Library.
* This ESP-IDF driver was created for better support in real-time, resource-constrained environments.

---

## 📬 Contact

For questions, bug reports, or contributions, open an issue or reach out via:

* Website: [techprogeny.com](https://techprogeny.com)
* GitHub: [@nikhil-robinson](https://github.com/nikhil-robinson)

```