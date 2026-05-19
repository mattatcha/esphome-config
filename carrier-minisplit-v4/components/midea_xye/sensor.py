import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from . import CONF_MIDEA_XYE_ID, MideaXYE

DEPENDENCIES = ["midea_xye"]

CONF_T1_INDOOR = "t1_indoor"
CONF_T2_INDOOR_COIL = "t2_indoor_coil"
CONF_T3_OUTDOOR_COIL = "t3_outdoor_coil"

_TEMP_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MIDEA_XYE_ID): cv.use_id(MideaXYE),
        cv.Optional(CONF_T1_INDOOR): _TEMP_SCHEMA,
        cv.Optional(CONF_T2_INDOOR_COIL): _TEMP_SCHEMA,
        cv.Optional(CONF_T3_OUTDOOR_COIL): _TEMP_SCHEMA,
    }
)

# Maps YAML key -> setter method on MideaXYE.
_SENSOR_SETTERS = {
    CONF_T1_INDOOR: "set_t1_sensor",
    CONF_T2_INDOOR_COIL: "set_t2_sensor",
    CONF_T3_OUTDOOR_COIL: "set_t3_sensor",
}


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MIDEA_XYE_ID])
    for key, setter in _SENSOR_SETTERS.items():
        if conf := config.get(key):
            s = await sensor.new_sensor(conf)
            cg.add(getattr(parent, setter)(s))
