# Module to handle M73 and M117 display status commands
#
# Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

M73_TIMEOUT = 5.

class DisplayStatus:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("virtual_sdcard:reset_file",
                                            self._reset_file)
        self.progress = self.message = self.remaining = self.remaining_to_pause = None
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('M73', self.cmd_M73)
        gcode.register_command('M117', self.cmd_M117)
        gcode.register_command(
            'SET_DISPLAY_TEXT', self.cmd_SET_DISPLAY_TEXT,
            desc=self.cmd_SET_DISPLAY_TEXT_help)
    def get_status(self, eventtime):
        progress = self.progress
        if progress is None:
            progress = 0.
            sdcard = self.printer.lookup_object('virtual_sdcard', None)
            if sdcard is not None:
                progress = sdcard.get_status(eventtime)['progress']
        return {
            'progress': progress,
            'message': self.message,
            'remaining': self.remaining,
            'remaining_to_pause': self.remaining_to_pause
        }
    def cmd_M73(self, gcmd):
        progress = gcmd.get_float('P', None)
        remaining = gcmd.get_float('R', None)
        remaining_to_pause = gcmd.get_float('C', None)
        if progress is not None:
            progress = progress / 100.
            self.progress = min(1., max(0., progress))
        if remaining is not None:
            self.remaining = remaining
        if remaining_to_pause is not None:
            self.remaining_to_pause = remaining_to_pause
    def cmd_M117(self, gcmd):
        msg = gcmd.get_raw_command_parameters() or None
        self.message = msg
    cmd_SET_DISPLAY_TEXT_help = "Set or clear the display message"
    def cmd_SET_DISPLAY_TEXT(self, gcmd):
        self.message = gcmd.get("MSG", None)

    def _reset_file(self):
        self.progress = self.message = self.remaining = self.remaining_to_pause = None

def load_config(config):
    return DisplayStatus(config)
