# Analog Probe support
#
# Copyright (C) 2023-2023  Michal O'Tomek
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import probe
import statistics

# Analog "endstop" wrapper
class AnalogEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self.handle_mcu_identify)
        self.position_endstop = config.getfloat('z_offset')
        self.stow_on_each_sample = config.getboolean('stow_on_each_sample',
                                                     True)
        self.probe_touch_mode = config.getboolean('probe_with_touch_mode',
                                                  False)
        # Command timing
        self.next_cmd_time = self.action_end_time = 0.
        self.finish_home_complete = self.wait_trigger_complete = None
        # Create an "endstop" object to handle the sensor pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('sensor_pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        self.treshold = config.getint("treshold", 0)
        self.base_adc = 0
        self.mcu_endstop = mcu.setup_pin('analog_endstop', pin_params)
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # Register BLTOUCH_DEBUG command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("ANALOG_PROBE_DEBUG",
                                    self.cmd_ANALOG_PROBE_DEBUG,
                                    desc=self.cmd_ANALOG_PROBE_DEBUG_help)
        # multi probes state
        self.multi = 'OFF'
    def handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
    def handle_connect(self):
        self.sync_mcu_print_time()
        self.next_cmd_time += 0.200
    def sync_mcu_print_time(self):
        # Not sure what this is for
        pass
    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time
    def multi_probe_begin(self):
        filter = []
        for i in range(1024):
            filter.append(self.mcu_endstop.query_endstop())

        quantiles = statistics.quantiles(filter,n=4)
        self.base_adc = int(quantiles[1])
        min_v = min(filter)
        max_v = max(filter)
        q1 = quantiles[0]
        q2 = quantiles[1]
        q3 = quantiles[2]
        logging.info(f"ADC base value set to {self.base_adc}. min:{min_v}, "
                     f"q1:{q1}, q2:{q2}, q3:{q3}, max:{max_v}")

        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'
    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self.sync_print_time()
        self.sync_print_time()
        self.multi = 'OFF'
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            # FIXME - get base ADC level hír results incrementing ADC
            # self.base_adc = self.mcu_endstop.query_endstop()
            if self.multi == 'FIRST':
                self.multi = 'ON'
        self.sync_print_time()
    def home_start(self, print_time, sample_time, oversample_count, rest_time,
                   triggered=None):
        rest_time = rest_time
        self.finish_home_complete = self.mcu_endstop.home_start(
            print_time, sample_time, oversample_count, rest_time,
            self.base_adc + self.treshold)
        # Schedule wait_for_trigger callback
        r = self.printer.get_reactor()
        self.wait_trigger_complete = r.register_callback(self.wait_for_trigger)
        return self.finish_home_complete
    def wait_for_trigger(self, eventtime):
        self.finish_home_complete.wait()
    def probe_finish(self, hmove):
        self.wait_trigger_complete.wait()
        self.sync_print_time()
    def get_position_endstop(self):
        return self.position_endstop

    cmd_ANALOG_PROBE_DEBUG_help = "Returns analog probe debug report"
    def cmd_ANALOG_PROBE_DEBUG(self, gcmd):
        # cmd = gcmd.get('COMMAND', None)
        gcmd.respond_info(f"Analog Probe debug. ADC Value="
                          f"{self.mcu_endstop.query_endstop()}, ADC base value"
                          f" = {self.base_adc}, treshold={self.treshold}")
        self.sync_print_time()

def load_config(config):
    endstop = AnalogEndstopWrapper(config)
    config.get_printer().add_object('probe',
                                    probe.PrinterProbe(config,endstop))
    return endstop
