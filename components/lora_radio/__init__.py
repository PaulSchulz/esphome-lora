import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_ID, CONF_PIN

DEPENDENCIES = ["spi"]

lora_radio_ns = cg.esphome_ns.namespace("lora_radio")
LoRaRadio = lora_radio_ns.class_(
    "LoRaRadio", cg.Component, spi.SPIDevice
)

CONF_RESET_PIN = "reset_pin"
CONF_DIO1_PIN  = "dio1_pin"
CONF_BUSY_PIN  = "busy_pin"

#CONFIG_SCHEMA = (
#    cv.Schema({cv.GenerateID(): cv.declare_id(LoRaRadio)})
#    .extend(cv.COMPONENT_SCHEMA)
#    .extend(spi.spi_device_schema(0x01))
#    .extend(cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
#            cv.Required(CONF_DIO1_PIN): pins.gpio_input_pin_schema,
#            cv.Required(CONF_BUSY_PIN): pins.gpio_input_pin_schema,
#))

CONFIG_SCHEMA = spi.spi_device_schema().extend({
    cv.GenerateID(): cv.declare_id(LoRaRadio),
    cv.Required(CONF_RESET_PIN): pins.internal_gpio_output_pin_number,
    cv.Required(CONF_DIO1_PIN):  pins.internal_gpio_input_pin_number,
    cv.Required(CONF_BUSY_PIN):  pins.internal_gpio_input_pin_number,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # cs_pin = config[spi.CONF_CS_PIN]
    #cs_pin = await cg.gpio_pin_expression(config[spi.CONF_CS_PIN])
    #cs_number = cg.RawExpression(str(cs_pin))
    cg.add(var.set_control_pins(
        # cs_number,
        config[CONF_RESET_PIN],
        config[CONF_DIO1_PIN],
        config[CONF_BUSY_PIN],
    ))
    #reset = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    #dio1 = await cg.gpio_pin_expression(config[CONF_DIO1_PIN])
    #busy = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    #cg.add(var.set_control_pins(reset, dio1, busy))

    await spi.register_spi_device(var, config)

# THe following does NOT work.
# Add RadioLib as an external library
#BUILD_FLAGS = "-D RADIOLIB_BUILD_FLAG"

#cg.add_library(
#    name="SPI",
#    repository="https://github.com/espressif/arduino-esp32.git",
#    version="latest"  # Specify the version you want; omit for latest
#)

#cg.add_library(
#    name="RadioLib",
#    repository="https://github.com/jgromes/RadioLib.git",
#    version="5.3.0"  # Specify the version you want; omit for latest
#)

#cg.add_library(
#    name="RadioLib",
#    repository="https://github.com/PaulSchulz/RadioLib",
#    version=""  # Specify the version you want; omit for latest
#)
