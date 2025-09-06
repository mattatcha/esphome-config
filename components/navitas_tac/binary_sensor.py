import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_CONNECTIVITY

from . import CONF_NAVITAS_TAC_ID, NavitasTAC

DEPENDENCIES = ["navitas_tac"]

CONF_CONNECTED = "connected"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_NAVITAS_TAC_ID): cv.use_id(NavitasTAC),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NAVITAS_TAC_ID])

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(parent.set_connected_sensor(sens))
