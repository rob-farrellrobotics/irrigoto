"""
ESPHome external component — irrigoto
Wraps the irrigoto firmware (irrigoto.c) as an ESPHome component.

YAML key: irrigoto
  id: irrigoto   # optional, needed if referenced in lambdas / services
"""

import os

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components.esp32 import add_idf_component, include_builtin_idf_component
from esphome.components.deep_sleep import DeepSleepComponent

CONF_DEEP_SLEEP_ID = "deep_sleep_id"

# HTML fragment "headers" live in ./html/ — they are not real C headers
# (raw R"...(...)..." string bodies, only valid when #included inside a
# string literal definition). Two reasons for the subdir:
#   1. Keeps them out of ESPHome's component-root auto-copy (otherwise
#      they'd land alongside irrigoto.h in the build tree and risk
#      being pulled into esphome.h).
#   2. ESPHome doesn't actually copy subdirs into its build tree at
#      all, so the compiler reads these straight from the original
#      source location via the -I flag added below.
# irrigoto.c references them as bare #include "cal_html.h".
_HTML_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "html")
).replace("\\", "/")

CODEOWNERS = ["@rob-farrellrobotics"]
DEPENDENCIES = []
AUTO_LOAD = []

# Declare the C++ namespace and class that ESPHome will instantiate
irrigoto_ns = cg.esphome_ns.namespace("irrigoto")
IrrigotoComponent = irrigoto_ns.class_(
    "IrrigotoComponent", cg.Component
)

# Top-level YAML schema: just an optional id
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(IrrigotoComponent),
        # Optional link to ESPHome's deep_sleep component. If provided,
        # all sleep requests from irrigoto.c are routed through it so HA
        # gets a graceful "going to sleep" notification.
        cv.Optional(CONF_DEEP_SLEEP_ID): cv.use_id(DeepSleepComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # ESPHome excludes these IDF components by default (see DEFAULT_EXCLUDED_IDF_COMPONENTS
    # in esp32/__init__.py) — re-include them since irrigoto.c uses them.
    include_builtin_idf_component("driver")    # legacy I2C driver (driver/i2c.h)
    include_builtin_idf_component("esp_adc")   # esp_adc/adc_oneshot.h

    # Managed component for LittleFS persistence.
    add_idf_component(name="joltwallet/littlefs", ref="^1.21.1")

    # Tell irrigoto.c it is being compiled as an ESPHome component.
    # Activates irrigoto_init() and deactivates app_main() / wifi_init().
    cg.add_build_flag("-DESPHOME_COMPONENT=1")
    # Silence legacy I2C deprecation warnings (we use it intentionally).
    cg.add_build_flag("-Wno-deprecated-declarations")
    # Let irrigoto.c find the HTML fragment headers in ./html/ (which
    # ESPHome doesn't copy into the build tree — the compiler reads
    # them straight from the original source location).
    cg.add_build_flag(f"-I{_HTML_DIR}")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if (ds_id := config.get(CONF_DEEP_SLEEP_ID)) is not None:
        ds = await cg.get_variable(ds_id)
        cg.add(var.set_deep_sleep(ds))
