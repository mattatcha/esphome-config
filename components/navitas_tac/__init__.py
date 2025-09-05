import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_AMPERE,
)

DEPENDENCIES = ["ble_client"]

navitas_tac_ns = cg.esphome_ns.namespace("navitas_tac")
NavitasTAC = navitas_tac_ns.class_(
    "NavitasTAC", cg.PollingComponent, ble_client.BLEClientNode
)

CONF_NAVITAS_TAC_ID = "navitas_tac_id"
CONF_BLE_CLIENT_ID = "ble_client_id"
CONF_TEMPERATURE = "temperature"
CONF_VOLTAGE = "voltage"
CONF_CURRENT = "current"
CONF_STATE = "state"
CONF_CONNECTED = "connected"
CONF_SPEED = "speed"
CONF_SOC = "soc"
CONF_TIRE_DIAMETER = "tire_diameter"
CONF_REAR_AXLE_RATIO = "rear_axle_ratio"
CONF_USE_KILOMETERS = "use_kilometers"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NavitasTAC),
        cv.Required(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SPEED): sensor.sensor_schema(
            unit_of_measurement="mph",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SOC): sensor.sensor_schema(
            unit_of_measurement="%",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_STATE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_TIRE_DIAMETER, default=22.0): cv.float_range(
            min=10.0, max=50.0
        ),
        cv.Optional(CONF_REAR_AXLE_RATIO, default=12.5): cv.float_range(
            min=1.0, max=50.0
        ),
        cv.Optional(CONF_USE_KILOMETERS, default=False): cv.boolean,
    }
).extend(cv.polling_component_schema("2s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    ble_client_var = await cg.get_variable(config[CONF_BLE_CLIENT_ID])
    cg.add(ble_client_var.register_ble_node(var))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VOLTAGE])
        cg.add(var.set_voltage_sensor(sens))

    if CONF_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT])
        cg.add(var.set_current_sensor(sens))

    if CONF_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_SPEED])
        cg.add(var.set_speed_sensor(sens))

    if CONF_SOC in config:
        sens = await sensor.new_sensor(config[CONF_SOC])
        cg.add(var.set_soc_sensor(sens))

    if CONF_STATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATE])
        cg.add(var.set_state_sensor(sens))

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_sensor(sens))

    # Vehicle configuration for speed calculation
    cg.add(var.set_tire_diameter(config[CONF_TIRE_DIAMETER]))
    cg.add(var.set_rear_axle_ratio(config[CONF_REAR_AXLE_RATIO]))
    cg.add(var.set_use_kilometers(config[CONF_USE_KILOMETERS]))
