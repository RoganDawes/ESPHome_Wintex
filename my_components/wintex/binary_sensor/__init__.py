from esphome.components import binary_sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from .. import wintex_ns, CONF_WINTEX_ID, Wintex

DEPENDENCIES = ["wintex"]
CODEOWNERS = ["@jesserockz"]

CONF_SENSOR_DATAPOINT = "sensor_datapoint"

WintexBinarySensor = wintex_ns.class_(
    "WintexBinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(WintexBinarySensor),
        cv.GenerateID(CONF_WINTEX_ID): cv.use_id(Wintex),
        cv.Required(CONF_SENSOR_DATAPOINT): cv.uint8_t,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await binary_sensor.register_binary_sensor(var, config)

    paren = await cg.get_variable(config[CONF_WINTEX_ID])
    cg.add(var.set_wintex_parent(paren))

    cg.add(var.set_sensor_id(config[CONF_SENSOR_DATAPOINT]))
