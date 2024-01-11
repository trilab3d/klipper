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
        self.printer.load_object(config, 'respond')
        self.printer.load_object(config, 'print_interlock')
        self.print_interlock = self.printer.lookup_object('print_interlock',None)
        self.save_variables = None
        self.interlock = None
        if self.print_interlock is not None:
            self.interlock = self.print_interlock.create_interlock("Doors are open")
            self.interlock.set_lock(True)
        self.open_gcode = self.close_gcode = None
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        if self.open_pause or config.get('open_gcode', None) is not None:
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
        # Register commands and event handlers
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        logging.info(f"Door Sensor registering connect")
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.gcode.register_command(
            "QUERY_DOOR_SENSOR",
            self.cmd_QUERY_DOOR_SENSOR,
            desc=self.cmd_QUERY_DOOR_SENSOR_help)
        self.gcode.register_command(
            "SET_DOOR_SENSOR_DISABLED",
            self.cmd_SET_DOOR_SENSOR_DISABLED,
            desc=self.cmd_SET_DOOR_SENSOR_help)
    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 2.
    def _handle_connect(self):
        logging.info(f"Door Sensor handle connect")
        try:
            self.save_variables = self.printer.lookup_object('save_variables')
            logging.info(f"Save Variables Object is {self.save_variables}")
            dds = self.save_variables.get_variable("disable-door-sensor")
            if dds is None:
                self.save_variables.save_variable("disable-door-sensor", False)
            elif dds is True:
                self.interlock.set_lock(False)
        except Exception as e:
            logging.error(f"Door Sensor Error {e}")
            self.printer.invoke_shutdown(e)
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
        sensor_disabled = self.save_variables.get_variable("disable-door-sensor")
        if self.interlock is not None:
            if sensor_disabled:
                self.interlock.set_lock(False)
            else:
                self.interlock.set_lock(not is_door_closed)
        if is_door_closed == self.door_closed:
            return
        self.door_closed = is_door_closed
        eventtime = self.reactor.monotonic()
        if eventtime < self.min_event_systime:
            # do not process during the initialization time, duplicates,
            # during the event delay time, while an event is running, or
            # when the sensor is disabled
            return
        # Determine "printing" status
        print_stats = self.printer.lookup_object("print_stats")
        is_printing = print_stats.get_status(eventtime)["state"] == "printing"
        # Perform printer action associated with status change (if any)
        if is_door_closed:
            if not is_printing and self.close_gcode is not None:
                # Close detected
                self.min_event_systime = self.reactor.NEVER
                logging.info(
                    "Door Sensor %s: close event detected, Time %.2f" %
                    (self.name, eventtime))
                self.reactor.register_callback(self._close_event_handler)
        elif is_printing and self.open_gcode is not None and not sensor_disabled:
            # Open detected
            self.min_event_systime = self.reactor.NEVER
            logging.info(
                "Door Sensor %s: open event detected, Time %.2f" %
                (self.name, eventtime))
            self.reactor.register_callback(self._open_event_handler)
    def get_status(self, eventtime=None):
        disabled = self.save_variables.get_variable("disable-door-sensor") \
            if self.save_variables is not None else False
        return {
            "door_closed": self.door_closed or disabled,
            "enabled": not disabled}
    cmd_QUERY_DOOR_SENSOR_help = "Query the status of the Door Sensor"
    def cmd_QUERY_DOOR_SENSOR(self, gcmd):
        if self.door_closed:
            msg = "Door Sensor %s: door closed" % (self.name)
        else:
            msg = "Door Sensor %s: door opened" % (self.name)
        gcmd.respond_info(msg)
    cmd_SET_DOOR_SENSOR_help = "Sets the door sensor on/off"
    def cmd_SET_DOOR_SENSOR_DISABLED(self, gcmd):
        disabled = bool(gcmd.get_int("DISABLED"))
        self.save_variables.save_variable("disable-door-sensor", disabled)
        if disabled and self.interlock is not None:
            self.interlock.set_lock(False)
        if not disabled and self.interlock is not None:
            self.interlock.set_lock(not self.door_closed)

class DoorSensor:
    def __init__(self, config):
        printer = config.get_printer()
        buttons = printer.load_object(config, 'buttons')
        switch_pin = config.get('switch_pin')
        buttons.register_buttons([switch_pin], self._button_handler)
        self.open_helper = OpenHelper(config)
        self.get_status = self.open_helper.get_status
    def _button_handler(self, eventtime, state):
        logging.info(f"=======================Door sensor button handler. State {state}=========================================")
        self.open_helper.note_door_closed(state)

def load_config(config):
    return DoorSensor(config)
