# Printer states compatible with Prusa Connect
#
# Copyright (C) 2024  Michal Tomek
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from enum import Enum

class ConnectState(Enum):
    IDLE = "idle"
    READY = "ready"
    BUSY = "busy"
    PRINTING = "printing"
    PAUSED = "paused"
    STOPPED = "stopped"
    FINISHED = "finished"
    ATTENTION = "attention"
    ERROR = "error"

class PrusaState:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.state = ConnectState.IDLE
        self.is_ready = False
        self.is_paused = False
        self.is_printing = False
        self.printer.register_event_handler("idle_timeout:idle", self.handle_idle)
        self.printer.register_event_handler("idle_timeout:ready", self.handle_ready)
        self.printer.register_event_handler("idle_timeout:printing", self.handle_printing)
        self.printer.register_event_handler("pause_resume:pause", self.handle_pause)
        self.printer.register_event_handler("pause_resume:resume", self.handle_resume)
        self.printer.register_event_handler("virtual_sdcard:resume", self.handle_sd_resume)
        self.printer.register_event_handler("virtual_sdcard:cancel", self.handle_sd_cancel)
        self.printer.register_event_handler("virtual_sdcard:finished", self.handle_sd_finished)
        self.printer.register_event_handler("klippy:shutdown", self.handle_shutdown)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SET_READY", self.cmd_SET_READY, desc="Sets printer ready to new job")

    def handle_idle(self, *args, **kvargs):
       self.handle_ready()

    def handle_ready(self, *args, **kvargs):
        if self.state in [ConnectState.STOPPED, ConnectState.FINISHED]:
            return
        if self.is_paused:
            self.state = ConnectState.PAUSED
        elif self.is_ready:
            self.state = ConnectState.READY
        else:
            self.state = ConnectState.IDLE

    # Not actually printing, just busy
    def handle_printing(self, *args, **kvargs):
        if not self.is_printing:
            self.state = ConnectState.BUSY

    def handle_pause(self, reason, *args, **kvargs):
        self.is_printing = False
        self.is_paused = True
        if reason is None:
            self.state = ConnectState.PAUSED
        else:
            self.state = ConnectState.ATTENTION

    def handle_resume(self, *args, **kvargs):
        self.handle_sd_resume()

    def handle_sd_resume(self, *args, **kvargs):
        self.is_printing = True
        self.is_paused = False
        self.state = ConnectState.PRINTING

    def handle_sd_cancel(self, *args, **kvargs):
        self.is_printing = False
        self.is_ready = False
        self.is_paused = False
        self.state = ConnectState.STOPPED

    def handle_sd_finished(self, *args, **kvargs):
        self.is_printing = False
        self.is_ready = False
        self.is_paused = False
        self.state = ConnectState.FINISHED

    def handle_shutdown(self, *args, **kvargs):
        self.is_printing = False
        self.is_ready = False
        self.is_paused = False
        self.state = ConnectState.ERROR

    def cmd_SET_READY(self, gcmd):
        if self.state in [ConnectState.IDLE, ConnectState.FINISHED, ConnectState.STOPPED]:
            self.is_printing = False
            self.is_paused = False
            self.is_ready = True
            self.state = ConnectState.READY
        elif self.state == ConnectState.BUSY and not self.is_paused:
            self.is_ready = True
        else:
            gcmd.respond_info(f"Printer cant be set to 'ready', because it's in state '{self.state.value}'")

    def get_status(self, eventtime):

        return {
            'state': self.state.value
        }

def load_config(config):
    return PrusaState(config)
