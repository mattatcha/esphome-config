import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_SPEED,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_PERCENT,
)

from . import NavitasTAC, CONF_NAVITAS_TAC_ID

DEPENDENCIES = ["navitas_tac"]

# Parameter names matching TACDictionary.xml <Name> values
CONF_CONTROLLER_TEMPERATURE = (
    "controller_temperature"  # "Controller Temperature (C)" - PBTEMPC
)
CONF_MOTOR_TEMPERATURE = "motor_temperature"  # "Motor Temperature (C)" - MTTEMPC
CONF_DC_BUS_VOLTAGE = "dc_bus_voltage"  # "DC Bus Voltage (V)" - VBATVDC
CONF_BATTERY_CURRENT = "battery_current"  # "Battery Current" - IBATADC
CONF_MOTOR_RPM = "motor_rpm"  # "Motor RPM" - ROTORRPM
CONF_SPEED = "speed"  # Calculated from Motor RPM
CONF_SOC = "soc"  # State of Charge (if available)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_NAVITAS_TAC_ID): cv.use_id(NavitasTAC),
        cv.Optional(CONF_CONTROLLER_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MOTOR_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DC_BUS_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MOTOR_RPM): sensor.sensor_schema(
            unit_of_measurement="rpm",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SPEED): sensor.sensor_schema(
            unit_of_measurement="mph",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_SPEED,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SOC): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NAVITAS_TAC_ID])

    if CONF_CONTROLLER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_CONTROLLER_TEMPERATURE])
        cg.add(parent.set_temperature_sensor(sens))

    if CONF_MOTOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_MOTOR_TEMPERATURE])
        cg.add(parent.set_motor_temperature_sensor(sens))

    if CONF_DC_BUS_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_DC_BUS_VOLTAGE])
        cg.add(parent.set_voltage_sensor(sens))

    if CONF_BATTERY_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_CURRENT])
        cg.add(parent.set_current_sensor(sens))

    if CONF_MOTOR_RPM in config:
        sens = await sensor.new_sensor(config[CONF_MOTOR_RPM])
        cg.add(parent.set_motor_rpm_sensor(sens))

    if CONF_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_SPEED])
        cg.add(parent.set_speed_sensor(sens))

    if CONF_SOC in config:
        sens = await sensor.new_sensor(config[CONF_SOC])
        cg.add(parent.set_soc_sensor(sens))
