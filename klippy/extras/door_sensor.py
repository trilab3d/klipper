# Generic Filament Sensor Module
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class OpenHelper:
    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # Read config
        self.open_pause = config.getboolean('pause_on_open', True)
        if self.open_pause:
            self.printer.load_object(config, 'pause_resume')
        self.open_gcode = self.close_gcode = None
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        if self.open_pause or config.get('runout_gcode', None) is not None:
            self.open_gcode = gcode_macro.load_template(
                config, 'open_gcode', '')
        if config.get('close_gcode', None) is not None:
            self.close_gcode = gcode_macro.load_template(
                config, 'close_gcode')
        self.pause_delay = config.getfloat('pause_delay', .5, above=.0)
        self.event_delay = config.getfloat('event_delay', 3., above=0.)
        # Internal state
        self.min_event_systime = self.reactor.NEVER
        self.door_closed = False
        self.sensor_enabled = True
        # Register commands and event handlers
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_mux_command(
            "QUERY_DOOR_SENSOR", "SENSOR", self.name,
            self.cmd_QUERY_DOOR_SENSOR,
            desc=self.cmd_QUERY_DOOR_SENSOR_help)
        self.gcode.register_mux_command(
            "SET_DOOR_SENSOR", "SENSOR", self.name,
            self.cmd_SET_DOOR_SENSOR,
            desc=self.cmd_SET_DOOR_SENSOR_help)
    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 2.
    def _open_event_handler(self, eventtime):
        # Pausing from inside an event requires that the pause portion
        # of pause_resume execute immediately.
        pause_prefix = ""
        if self.open_pause:
            pause_resume = self.printer.lookup_object('pause_resume')
            pause_resume.send_pause_command()
            pause_prefix = "PAUSE\n"
            self.printer.get_reactor().pause(eventtime + self.pause_delay)
        self._exec_gcode(pause_prefix, self.open_gcode)
    def _close_event_handler(self, eventtime):
        self._exec_gcode("", self.close_gcode)
    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script(prefix + template.render() + "\nM400")
        except Exception:
            logging.exception("Script running error")
        self.min_event_systime = self.reactor.monotonic() + self.event_delay
    def note_door_closed(self, is_door_closed):
        if is_door_closed == self.door_closed:
            return
        self.door_closed = is_door_closed
        eventtime = self.reactor.monotonic()
        if eventtime < self.min_event_systime or not self.sensor_enabled:
            # do not process during the initialization time, duplicates,
            # during the event delay time, while an event is running, or
            # when the sensor is disabled
            return
        # Determine "printing" status
        idle_timeout = self.printer.lookup_object("idle_timeout")
        is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # Perform filament action associated with status change (if any)
        if is_door_closed:
            if not is_printing and self.close_gcode is not None:
                # Close detected
                self.min_event_systime = self.reactor.NEVER
                logging.info(
                    "Door Sensor %s: close event detected, Time %.2f" %
                    (self.name, eventtime))
                self.reactor.register_callback(self._close_event_handler)
        elif is_printing and self.open_gcode is not None:
            # Open detected
            self.min_event_systime = self.reactor.NEVER
            logging.info(
                "Door Sensor %s: open event detected, Time %.2f" %
                (self.name, eventtime))
            self.reactor.register_callback(self._open_event_handler)
    def get_status(self, eventtime):
        return {
            "door_closed": bool(self.door_closed),
            "enabled": bool(self.sensor_enabled)}
    cmd_QUERY_DOOR_SENSOR_help = "Query the status of the Door Sensor"
    def cmd_QUERY_DOOR_SENSOR(self, gcmd):
        if self.door_closed:
            msg = "Door Sensor %s: door closed" % (self.name)
        else:
            msg = "Door Sensor %s: door opened" % (self.name)
        gcmd.respond_info(msg)
    cmd_SET_DOOR_SENSOR_help = "Sets the door sensor on/off"
    def cmd_SET_DOOR_SENSOR(self, gcmd):
        self.sensor_enabled = gcmd.get_int("ENABLE", 1)

class DoorSensor:
    def __init__(self, config):
        printer = config.get_printer()
        buttons = printer.load_object(config, 'buttons')
        switch_pin = config.get('switch_pin')
        buttons.register_buttons([switch_pin], self._button_handler)
        self.open_helper = OpenHelper(config)
        self.get_status = self.open_helper.get_status
    def _button_handler(self, eventtime, state):
        self.open_helper.note_door_closed(state)

def load_config_prefix(config):
    return DoorSensor(config)
