import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_MIDEA_XYE_ID, MideaXYE

DEPENDENCIES = ["midea_xye"]

CONF_FAN_SPEED = "fan_speed"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MIDEA_XYE_ID): cv.use_id(MideaXYE),
        cv.Optional(CONF_FAN_SPEED): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MIDEA_XYE_ID])
    if conf := config.get(CONF_FAN_SPEED):
        ts = await text_sensor.new_text_sensor(conf)
        cg.add(parent.set_fan_speed_text_sensor(ts))
