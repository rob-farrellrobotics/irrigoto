"""select platform — zone selector, water mode selector"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_OPTIONS
from . import IrrigotoComponent, irrigoto_ns

DEPENDENCIES = ["irrigoto"]

CONF_ZONE = "zone"
CONF_MODE = "mode"

IrrigotoZoneSelect = irrigoto_ns.class_(
    "IrrigotoZoneSelect", select.Select, cg.Component
)
IrrigotoModeSelect = irrigoto_ns.class_(
    "IrrigotoModeSelect", select.Select, cg.Component
)

_SUB_SCHEMA = lambda cls: select.select_schema(cls).extend(
    {cv.Required(CONF_OPTIONS): cv.ensure_list(cv.string_strict)}
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(IrrigotoComponent),
        cv.Optional(CONF_ZONE): _SUB_SCHEMA(IrrigotoZoneSelect),
        cv.Optional(CONF_MODE): _SUB_SCHEMA(IrrigotoModeSelect),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[cv.GenerateID()])

    if zone_conf := config.get(CONF_ZONE):
        zone_sel = cg.new_Pvariable(zone_conf[cv.CONF_ID])
        await cg.register_component(zone_sel, zone_conf)
        await select.register_select(zone_sel, zone_conf, options=zone_conf.get("options", []))
        cg.add(hub.set_zone_select(zone_sel))

    if mode_conf := config.get(CONF_MODE):
        mode_sel = cg.new_Pvariable(mode_conf[cv.CONF_ID])
        await cg.register_component(mode_sel, mode_conf)
        await select.register_select(mode_sel, mode_conf, options=mode_conf.get("options", []))
        cg.add(hub.set_mode_select(mode_sel))
