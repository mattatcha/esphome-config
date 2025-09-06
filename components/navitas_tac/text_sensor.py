import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv

from . import CONF_NAVITAS_TAC_ID, NavitasTAC

DEPENDENCIES = ["navitas_tac"]

CONF_STATE = "state"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_NAVITAS_TAC_ID): cv.use_id(NavitasTAC),
        cv.Optional(CONF_STATE): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NAVITAS_TAC_ID])

    if CONF_STATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATE])
        cg.add(parent.set_state_sensor(sens))
