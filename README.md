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

- ESP32 or ESP32-S series board  
- ESP-IDF v4.x or later  
- I2C driver enabled in `menuconfig`  
- MAX30102 or MAX30105 sensor module  

---

## 📥 Installation

Clone this repository inside your ESP-IDF project under the `components` folder:

```bash
cd your_project/components
git clone https://github.com/yourusername/esp-idf-max3010x.git
````

Then include the component in your `CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS components/esp-idf-max3010x)
```

Or add to `idf_component_register` if inside a monolithic `CMakeLists.txt`.

---

## 🧪 Example Usage

```c
#include "max3010x.h"

max3010x_config_t config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .i2c_frequency = 400000
};

if (max3010x_init(&config) == ESP_OK) {
    printf("MAX3010x initialized.\n");

    max3010x_sample_t sample;
    if (max3010x_read_fifo(&sample) == ESP_OK) {
        printf("Red: %lu IR: %lu Green: %lu\n",
               sample.red, sample.ir, sample.green);
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