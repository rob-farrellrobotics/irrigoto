"""text_sensor platform — status string, active zone name"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import IrrigotoComponent

DEPENDENCIES = ["irrigoto"]

CONF_STATUS            = "status"
CONF_ZONE_NAME         = "zone_name"
CONF_CAL_STATE         = "cal_state"
CONF_LAST_SLEEP_REASON = "last_sleep_reason"
CONF_NEXT_RUN          = "next_run"
CONF_SCHEDULE_STATUS   = "schedule_status"
CONF_SCHEDULE_TEXT     = "schedule_text"
CONF_SCHEDULE_DELAY    = "schedule_delay"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(IrrigotoComponent),
        cv.Optional(CONF_STATUS):            text_sensor.text_sensor_schema(),
        cv.Optional(CONF_ZONE_NAME):         text_sensor.text_sensor_schema(),
        cv.Optional(CONF_CAL_STATE):         text_sensor.text_sensor_schema(),
        cv.Optional(CONF_LAST_SLEEP_REASON): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_NEXT_RUN):          text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SCHEDULE_STATUS):   text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SCHEDULE_TEXT):     text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SCHEDULE_DELAY):    text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[cv.GenerateID()])

    if st_conf := config.get(CONF_STATUS):
        sens = await text_sensor.new_text_sensor(st_conf)
        cg.add(hub.set_status_sensor(sens))

    if zn_conf := config.get(CONF_ZONE_NAME):
        sens = await text_sensor.new_text_sensor(zn_conf)
        cg.add(hub.set_zone_name_sensor(sens))

    if cal_conf := config.get(CONF_CAL_STATE):
        sens = await text_sensor.new_text_sensor(cal_conf)
        cg.add(hub.set_cal_state_sensor(sens))

    if lsr_conf := config.get(CONF_LAST_SLEEP_REASON):
        sens = await text_sensor.new_text_sensor(lsr_conf)
        cg.add(hub.set_last_sleep_reason_sensor(sens))

    if nr_conf := config.get(CONF_NEXT_RUN):
        sens = await text_sensor.new_text_sensor(nr_conf)
        cg.add(hub.set_next_run_sensor(sens))

    if ss_conf := config.get(CONF_SCHEDULE_STATUS):
        sens = await text_sensor.new_text_sensor(ss_conf)
        cg.add(hub.set_schedule_status_sensor(sens))

    if st_conf := config.get(CONF_SCHEDULE_TEXT):
        sens = await text_sensor.new_text_sensor(st_conf)
        cg.add(hub.set_schedule_text_sensor(sens))

    if sd_conf := config.get(CONF_SCHEDULE_DELAY):
        sens = await text_sensor.new_text_sensor(sd_conf)
        cg.add(hub.set_schedule_delay_sensor(sens))
