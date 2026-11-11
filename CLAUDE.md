# CLAUDE.md

Use `esphome/core/helpers.h` when necessary



When working with esphome components you can reference https://developers.esphome.io/architecture/components/ for best practices.



ESPHome is a system to configure microcontrollers (like ESP32, ESP8266, RP2040, and LibreTiny-based chips) using simple yet powerful YAML configuration files. It generates C++ firmware that can be compiled and flashed to these devices, allowing users to control them remotely through home automation systems.

## 4. Coding Conventions & Style Guide

*   **Formatting:**
    *   **Python:** Uses `ruff` and `flake8` for linting and formatting. Configuration is in `pyproject.toml`.
    *   **C++:** Uses `clang-format` for formatting. Configuration is in `.clang-format`.

*   **Naming Conventions:**
    *   **Python:** Follows PEP 8. Use clear, descriptive names following snake_case.
    *   **C++:** Follows the Google C++ Style Guide.

*   **Component Structure:**
    *   **Standard Files:**
        ```
        components/[component_name]/
        ├── __init__.py          # Component configuration schema and code generation
        ├── [component].h        # C++ header file (if needed)
        ├── [component].cpp      # C++ implementation (if needed)
        └── [platform]/          # Platform-specific implementations
            ├── __init__.py      # Platform-specific configuration
            ├── [platform].h     # Platform C++ header
            └── [platform].cpp   # Platform C++ implementation
        ```

    *   **Component Metadata:**
        - `DEPENDENCIES`: List of required components
        - `AUTO_LOAD`: Components to automatically load
        - `CONFLICTS_WITH`: Incompatible components
        - `CODEOWNERS`: GitHub usernames responsible for maintenance
        - `MULTI_CONF`: Whether multiple instances are allowed

*   **Code Generation & Common Patterns:**
    *   **Configuration Schema Pattern:**
        ```python
        import esphome.codegen as cg
        import esphome.config_validation as cv
        from esphome.const import CONF_KEY, CONF_ID

        CONF_PARAM = "param"  # A constant that does not yet exist in esphome/const.py

        my_component_ns = cg.esphome_ns.namespace("my_component")
        MyComponent = my_component_ns.class_("MyComponent", cg.Component)

        CONFIG_SCHEMA = cv.Schema({
            cv.GenerateID(): cv.declare_id(MyComponent),
            cv.Required(CONF_KEY): cv.string,
            cv.Optional(CONF_PARAM, default=42): cv.int_,
        }).extend(cv.COMPONENT_SCHEMA)

        async def to_code(config):
            var = cg.new_Pvariable(config[CONF_ID])
            await cg.register_component(var, config)
            cg.add(var.set_key(config[CONF_KEY]))
            cg.add(var.set_param(config[CONF_PARAM]))
        ```

    *   **C++ Class Pattern:**
        ```cpp
        namespace esphome {
        namespace my_component {

        class MyComponent : public Component {
         public:
          void setup() override;
          void loop() override;
          void dump_config() override;

          void set_key(const std::string &key) { this->key_ = key; }
          void set_param(int param) { this->param_ = param; }

         protected:
          std::string key_;
          int param_{0};
        };

        }  // namespace my_component
        }  // namespace esphome
        ```

    *   **Common Component Examples:**
        - **Sensor:**
          ```python
          from esphome.components import sensor
          CONFIG_SCHEMA = sensor.sensor_schema(MySensor).extend(cv.polling_component_schema("60s"))
          async def to_code(config):
              var = await sensor.new_sensor(config)
              await cg.register_component(var, config)
          ```

        - **Binary Sensor:**
          ```python
          from esphome.components import binary_sensor
          CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend({ ... })
          async def to_code(config):
              var = await binary_sensor.new_binary_sensor(config)
          ```

        - **Switch:**
          ```python
          from esphome.components import switch
          CONFIG_SCHEMA = switch.switch_schema().extend({ ... })
          async def to_code(config):
              var = await switch.new_switch(config)
          ```

*   **Configuration Validation:**
    *   **Common Validators:** `cv.int_`, `cv.float_`, `cv.string`, `cv.boolean`, `cv.int_range(min=0, max=100)`, `cv.positive_int`, `cv.percentage`.
    *   **Complex Validation:** `cv.All(cv.string, cv.Length(min=1, max=50))`, `cv.Any(cv.int_, cv.string)`.
    *   **Platform-Specific:** `cv.only_on(["esp32", "esp8266"])`, `esp32.only_on_variant(...)`, `cv.only_on_esp32`, `cv.only_on_esp8266`, `cv.only_on_rp2040`.
    *   **Framework-Specific:** `cv.only_with_framework(...)`, `cv.only_with_arduino`, `cv.only_with_esp_idf`.
    *   **Schema Extensions:**
        ```python
        CONFIG_SCHEMA = cv.Schema({ ... })
         .extend(cv.COMPONENT_SCHEMA)
         .extend(uart.UART_DEVICE_SCHEMA)
         .extend(i2c.i2c_device_schema(0x48))
         .extend(spi.spi_device_schema(cs_pin_required=True))
