import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]

lora_radio_ns = cg.esphome_ns.namespace("lora_radio")
LoRaRadio = lora_radio_ns.class_(
    "LoRaRadio", cg.Component, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(LoRaRadio)})
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(0x01))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
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
