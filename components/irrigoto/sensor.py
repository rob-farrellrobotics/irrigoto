"""
sensor platform for irrigoto — exposes pressure, battery_mv, throw_mm as
ESPHome sensor entities that Home Assistant can read.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID
from . import IrrigotoComponent, irrigoto_ns

DEPENDENCIES = ["irrigoto"]

CONF_PRESSURE_PSI = "pressure_psi"
CONF_BATTERY_MV   = "battery_mv"
CONF_THROW_MM     = "throw_mm"
CONF_WATER_MINUTES_REMAINING = "water_minutes_remaining"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(IrrigotoComponent),
        cv.Optional(CONF_PRESSURE_PSI): sensor.sensor_schema(),
        cv.Optional(CONF_BATTERY_MV):   sensor.sensor_schema(),
        cv.Optional(CONF_THROW_MM):     sensor.sensor_schema(),
        cv.Optional(CONF_WATER_MINUTES_REMAINING): sensor.sensor_schema(),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ID])

    if pres_conf := config.get(CONF_PRESSURE_PSI):
        sens = await sensor.new_sensor(pres_conf)
        cg.add(hub.set_pressure_sensor(sens))

    if bat_conf := config.get(CONF_BATTERY_MV):
        sens = await sensor.new_sensor(bat_conf)
        cg.add(hub.set_battery_sensor(sens))

    if throw_conf := config.get(CONF_THROW_MM):
        sens = await sensor.new_sensor(throw_conf)
        cg.add(hub.set_throw_sensor(sens))

    if wmr_conf := config.get(CONF_WATER_MINUTES_REMAINING):
        sens = await sensor.new_sensor(wmr_conf)
        cg.add(hub.set_water_minutes_remaining_sensor(sens))
