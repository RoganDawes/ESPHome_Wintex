from esphome.components import time, binary_sensor, switch, sensor
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_DEVICE_CLASS, CONF_NAME, CONF_PASSWORD

CODEOWNERS = ["@RoganDawes"]

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "sensor", "switch", "text_sensor"]

CONF_UDL = "udl"
CONF_ZONE = "zone"
CONF_ZONES = "zones"

CONF_WINTEX_ADDRESS = "read_address"
CONF_WINTEX_LENGTH = "read_length"
CONF_WINTEX_OFFSET = "read_offset"
CONF_WINTEX_MASK = "read_mask"

wintex_ns = cg.esphome_ns.namespace("wintex")
Wintex = wintex_ns.class_("Wintex", cg.Component, uart.UARTDevice)
WintexZone = wintex_ns.class_("WintexZone", binary_sensor.BinarySensor, cg.Component)
WintexBinarySensor = wintex_ns.class_("WintexBinarySensor", binary_sensor.BinarySensor, cg.Component)
WintexSensor = wintex_ns.class_("WintexSensor", sensor.Sensor, cg.Component)
WintexSwitch = wintex_ns.class_("WintexSwitch", switch.Switch, cg.Component)

CONFIG_ZONE_SCHEMA = (
    binary_sensor.BINARY_SENSOR_SCHEMA.extend(
        cv.Schema(
            {
                cv.GenerateID(): cv.declare_id(WintexZone),
                cv.Required(CONF_ZONE): cv.positive_int,
            }
        )
    )
)

CONF_WINTEX_ID = "wintex_id"
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Wintex),
            cv.Required(CONF_UDL): cv.string,
            cv.Optional(CONF_ZONES, default=[]): cv.ensure_list(CONFIG_ZONE_SCHEMA),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_udl(config[CONF_UDL]))
    if CONF_ZONES in config:
        for zone in config[CONF_ZONES]:
            z = cg.new_Pvariable(zone[CONF_ID], zone[CONF_ZONE])
            if CONF_NAME in zone:
                cg.add(z.set_name(zone[CONF_NAME]))
            if CONF_DEVICE_CLASS in zone:
                cg.add(z.set_device_class(zone[CONF_DEVICE_CLASS]))
            cg.add(var.register_zone(z))