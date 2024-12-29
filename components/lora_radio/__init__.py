import esphome.codegen as cg
import esphome.config_validation as cv
# from esphome.components import output
from esphome.const import CONF_ID
# from esphome.core import CORE
3#from esphome.yaml_util import is_template

# DEPENDENCIES = ["spi"]

# CONF_LORA_RADIO = "lora_radio"
lora_radio_ns = cg.esphome_ns.namespace("lora_radio")
LoraRadio = lora_radio_ns.class_('LoraRadio', cg.Component)

CONFIG_SCHEMA = cv.Schema({
        cv.GenerateID(): cv.declare_id(LoraRadio),
        cv.Optional("debug", default=False): cv.boolean,
    }).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var,config)

# Add RadioLib as an external library
#BUILD_FLAGS = "-D RADIOLIB_BUILD_FLAG"

#cg.add_library(
#    name="RadioLib",
#    repository="https://github.com/jgromes/RadioLib.git",
#    version="5.3.0"  # Specify the version you want; omit for latest
#)
