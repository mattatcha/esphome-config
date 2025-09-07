import esphome.codegen as cg
from esphome.components import ble_client
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["ble_client"]

AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

navitas_tac_ns = cg.esphome_ns.namespace("navitas_tac")
NavitasTAC = navitas_tac_ns.class_(
    "NavitasTAC", cg.PollingComponent, ble_client.BLEClientNode
)

CONF_NAVITAS_TAC_ID = "navitas_tac_id"
CONF_BLE_CLIENT_ID = "ble_client_id"
CONF_TIRE_DIAMETER = "tire_diameter"
CONF_REAR_AXLE_RATIO = "rear_axle_ratio"
CONF_USE_KILOMETERS = "use_kilometers"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NavitasTAC),
        cv.Required(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),
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

    # Vehicle configuration for speed calculation
    cg.add(var.set_tire_diameter(config[CONF_TIRE_DIAMETER]))
    cg.add(var.set_rear_axle_ratio(config[CONF_REAR_AXLE_RATIO]))
    cg.add(var.set_use_kilometers(config[CONF_USE_KILOMETERS]))
