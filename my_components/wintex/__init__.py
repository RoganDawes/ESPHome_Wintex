from esphome.components import time
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PASSWORD

DEPENDENCIES = ["uart"]

CONF_UDL = "udl"

wintex_ns = cg.esphome_ns.namespace("wintex")
Wintex = wintex_ns.class_("Wintex", cg.Component, uart.UARTDevice)

CONF_WINTEX_ID = "wintex_id"
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Wintex),
            cv.Required(CONF_UDL): cv.string,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if CONF_UDL in config:
      cg.add(var.set_udl(config[CONF_UDL]))
