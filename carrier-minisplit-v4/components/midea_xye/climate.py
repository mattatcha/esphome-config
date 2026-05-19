import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate

from . import CONF_MIDEA_XYE_ID, MideaXYE, midea_xye_ns

DEPENDENCIES = ["midea_xye"]

MideaXYEClimate = midea_xye_ns.class_(
    "MideaXYEClimate", climate.Climate, cg.Component
)

CONFIG_SCHEMA = (
    climate.climate_schema(MideaXYEClimate)
    .extend(
        {
            cv.GenerateID(CONF_MIDEA_XYE_ID): cv.use_id(MideaXYE),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_MIDEA_XYE_ID])
    cg.add(parent.set_climate(var))
    cg.add(var.set_parent(parent))
