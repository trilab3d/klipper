import logging

from . import neopixel
import configparser
from configfile import ConfigWrapper

class StatusLed:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.mcu.register_config_callback(self.build_config)
        self.toolhead = None
        self.idle_timeout = None
        self.set_status_led_cmd = None
        self.brightness = config.getfloat('brightness', 0.5, minval=0, maxval=1)
        self.printer.register_event_handler('klippy:connect', self._handle_connect)
        self.reactor = self.printer.get_reactor()
        self.update_timer = self.reactor.register_timer(self.do_update_value)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.idle_timeout = self.printer.lookup_object('idle_timeout')
        self.update_timer.waketime = self.reactor.monotonic() + 1

    def build_config(self):
        cmd_queue = self.mcu.alloc_command_queue()
        self.set_status_led_cmd = self.mcu.lookup_command(
            "set_status_led data=%*s", cq=cmd_queue)

    def do_update_value(self, eventtime):
        color_data = bytearray(3)
        color_data[0] = 0   # G
        color_data[1] = 255 # R
        color_data[2] = 0   # B
        status = self.idle_timeout.get_status(eventtime)["state"]

        if status == "Idle":
            color_data[0] = 255  # G
            color_data[1] = 255  # R
            color_data[2] = 255  # B
        elif status == "Printing":
            color_data[0] = 0    # G
            color_data[1] = 0    # R
            color_data[2] = 255  # B
        elif status == "Ready":
            color_data[0] = 255  # G
            color_data[1] = 255  # R
            color_data[2] = 0  # B

        self.set_status_led_cmd.send([[int(x * self.brightness) for x in color_data]*3])
        return eventtime + 1


def load_config(config):
    return StatusLed(config)