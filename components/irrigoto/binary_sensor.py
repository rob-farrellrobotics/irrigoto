"""binary_sensor platform — watering running state"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from . import IrrigotoComponent

DEPENDENCIES = ["irrigoto"]

CONF_WATERING = "watering"
CONF_VALVE_OPEN = "valve_open"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(IrrigotoComponent),
        cv.Optional(CONF_WATERING): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_VALVE_OPEN): binary_sensor.binary_sensor_schema(),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[cv.GenerateID()])

    if wat_conf := config.get(CONF_WATERING):
        sens = await binary_sensor.new_binary_sensor(wat_conf)
        cg.add(hub.set_watering_sensor(sens))

    if vo_conf := config.get(CONF_VALVE_OPEN):
        sens = await binary_sensor.new_binary_sensor(vo_conf)
        cg.add(hub.set_valve_open_sensor(sens))
