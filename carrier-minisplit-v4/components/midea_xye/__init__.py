import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@matt"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = []

CONF_MIDEA_XYE_ID = "midea_xye_id"
CONF_TX_ENABLED = "tx_enabled"

midea_xye_ns = cg.esphome_ns.namespace("midea_xye")
MideaXYE = midea_xye_ns.class_("MideaXYE", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MideaXYE),
            # SAFETY: defaults to False. Only enable after physically disconnecting
            # the original wired controller — otherwise the two masters will collide.
            cv.Optional(CONF_TX_ENABLED, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    # set_tx_enabled MUST be added before register_component so the C++ setter
    # statement appears in main.cpp before the component's setup() is invoked.
    # Otherwise setup() reads the default value (false) and the boot path is
    # mis-initialised.
    cg.add(var.set_tx_enabled(config[CONF_TX_ENABLED]))
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
