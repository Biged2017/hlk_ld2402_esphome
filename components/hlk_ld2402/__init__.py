import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor
from esphome.const import CONF_ID, CONF_TIMEOUT, ENTITY_CATEGORY_DIAGNOSTIC

# Make sure text_sensor is listed as a direct dependency
DEPENDENCIES = ["uart", "text_sensor"]
AUTO_LOAD = ["sensor", "binary_sensor"]  # Remove text_sensor from AUTO_LOAD

# Define our own constants
CONF_MAX_DISTANCE = "max_distance"
CONF_HLK_LD2402_ID = "hlk_ld2402_id" 

hlk_ld2402_ns = cg.esphome_ns.namespace("hlk_ld2402")
HLKLD2402Component = hlk_ld2402_ns.class_(
    "HLKLD2402Component", cg.Component, uart.UARTDevice
)

# This makes the component properly visible and available for other platforms
MULTI_CONF = True

# Main component schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HLKLD2402Component),
    cv.Optional(CONF_MAX_DISTANCE, default=5.0): cv.float_range(min=0.7, max=10.0),
    cv.Optional(CONF_TIMEOUT, default=5): cv.int_range(min=0, max=65535),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    if CONF_MAX_DISTANCE in config:
        cg.add(var.set_max_distance(config[CONF_MAX_DISTANCE]))
    if CONF_TIMEOUT in config:
        cg.add(var.set_timeout(config[CONF_TIMEOUT]))

# Services are defined in services.yaml file and automatically loaded by ESPHome