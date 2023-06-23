import logging
import math
from . import servo
from enum import Enum

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 0.100
RANGE_CHECK_COUNT = 4

class TUNING_PHASE(Enum):
    WAITING = 1
    PHASE_1 = 2
    PHASE_2 = 3
    TUNED = 4
    FAILED = 5


class ServoFanFlap:
    cmd_FLAP_DEBUG_help = "Returns debug informations."
    cmd_FLAP_SET_help = "Sets the flap position. Usage: FLAP_SET flap_name " \
                        "VALUE=<0. - 1. | 0 - 255>"
    def __init__(self, config):
        self.printer = config.get_printer()
        self.servo = servo.PrinterServo(config)
        self.flap_name = config.get_name().split()[-1]
        self.last_adc = 0
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.is_print_fan = config.getboolean("is_print_fan", False)
        self.open_at_sp = config.getboolean("open_at_sp", False)
        self.min_pulse_width = config.getfloat("min_pulse_width", 0.001)
        self.max_pulse_width = config.getfloat("max_pulse_width", 0.002)
        self.start_value = config.getfloat(
            "start_value", 0, minval=0, maxval=1)
        self.last_value = -1
        self.power_off_time = config.getfloat("power_off_time", 0)
        self.power_off_timeout = 0

        self.actual_tuning_width = self.min_pulse_width + (
                self.max_pulse_width - self.min_pulse_width) * 0.5
        self.tuning_timeout = 0
        self.tuning_step = config.getfloat("tuning_step", 0.000005)
        self.tuning_treshold = config.getfloat("tuning_treshold", 0)
        self.tuning_start_time = config.getfloat("tuning_start_time", 0.1)
        self.tuning_step_time = config.getfloat("tuning_step_time", 3)
        if config.getboolean("perform_range_tune", False):
            self.tuning_state = TUNING_PHASE.WAITING
        else:
            self.tuning_state = TUNING_PHASE.TUNED
            self.set_value(self.start_value)

        # register ADC pin
        ppins = self.printer.lookup_object('pins')
        feedback_pin_name = config.get("feedback_pin")
        self.feedback_pin = ppins.setup_pin('adc', feedback_pin_name)
        self.feedback_pin.get_last_value()
        self.feedback_pin.setup_adc_callback(REPORT_TIME,
                                             self._analog_feedback_callback)
        self.feedback_pin.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT, minval=0.,
                                       maxval=1.,
                                       range_check_count=RANGE_CHECK_COUNT)
        query_adc = config.get_printer().load_object(config, 'query_adc')
        query_adc.register_adc(self.flap_name + ":feedback",
                               self.feedback_pin)
        self.mcu = self.feedback_pin.get_mcu()

        # register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("FLAP_DEBUG", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_DEBUG,
                                   desc=self.cmd_FLAP_DEBUG_help)
        gcode.register_mux_command("FLAP_SET", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_SET,
                                   desc=self.cmd_FLAP_SET_help)
        if self.is_print_fan:
            gcode.register_command("M106", self.cmd_M106)
            gcode.register_command("M107", self.cmd_M107)

    def cmd_FLAP_DEBUG(self, gcmd):
        gcmd.respond_info(f"Last ADC reading: {self.last_adc}, min pulse "
                          f"width: {self.min_pulse_width}, max pulse width: "
                          f"{self.max_pulse_width}")
    def cmd_FLAP_SET(self, gcmd):
        val = gcmd.get_float('VALUE', minval=0., maxval= 255.)
        if val > 1:
            val = val / 255
        self.set_value(val)

    def cmd_M106(self, gcmd):
        val = gcmd.get_float('S', 255., minval=0.) / 255.
        self.set_value(val)
    def cmd_M107(self, gcmd):
        self.set_value(0)

    def set_value(self, value):
        if math.isclose(self.last_value,value):
            return
        self.last_value = value
        if self.open_at_sp:
            value = 1 - value
        w = self.min_pulse_width + (self.max_pulse_width -
                                    self.min_pulse_width) * value
        eventtime = self.reactor.monotonic()
        self.power_off_timeout = eventtime + self.power_off_time
        self.servo.set_width(w)

    def _analog_feedback_callback(self, last_read_time, last_value):
        # This callback is called periodically. I need it for servo range
        # callibration, but once created, I can't stop it. So I check servo
        # timeout here. If it can't be stopped, it will suffer for rest of
        # his poor life as general program loop.
        if self.tuning_state != TUNING_PHASE.TUNED:
            self._handle_range_tuning(last_read_time, last_value)
        self._handle_timeout()
        self.last_adc = last_value

    def _handle_timeout(self):
        if self.power_off_time > 0:
            eventtime = self.reactor.monotonic()
            if eventtime > self.power_off_timeout:
                self.servo.power_off()

    def _handle_range_tuning(self, last_read_time, last_value):
        eventtime = self.reactor.monotonic()
        print_time = self.mcu.estimated_print_time(eventtime)
        if self.tuning_state == TUNING_PHASE.WAITING:
            self.power_off_timeout = eventtime + self.power_off_time
            self.servo.set_width(self.actual_tuning_width)
            self.tuning_timeout = print_time + self.tuning_start_time
            self.tuning_state = TUNING_PHASE.PHASE_1
            return

        if self.tuning_state == TUNING_PHASE.PHASE_1:
            if last_read_time < self.tuning_timeout:
                return
            if last_value > self.tuning_treshold:
                max_w = self.actual_tuning_width
                self.actual_tuning_width = self.min_pulse_width + (
                        self.max_pulse_width - self.min_pulse_width) * 0.5
                self.max_pulse_width = max_w
                self.tuning_state = TUNING_PHASE.PHASE_2
                self.tuning_timeout = print_time + self.tuning_start_time
            else:
                self.actual_tuning_width += self.tuning_step
                if self.actual_tuning_width > self.max_pulse_width:
                    self.actual_tuning_width = self.min_pulse_width + (
                            self.max_pulse_width - self.min_pulse_width) * 0.5
                    self.tuning_timeout = print_time + self.tuning_start_time
                    self.tuning_state = TUNING_PHASE.PHASE_2
                else:
                    self.tuning_timeout = print_time + self.tuning_step_time
            self.power_off_timeout = eventtime + self.power_off_time
            self.servo.set_width(self.actual_tuning_width)
            return

        if self.tuning_state == TUNING_PHASE.PHASE_2:
            if last_read_time < self.tuning_timeout:
                return
            if last_value > self.tuning_treshold:
                self.min_pulse_width = self.actual_tuning_width
                self.tuning_state = TUNING_PHASE.TUNED
                self.set_value(self.start_value)
            else:
                self.actual_tuning_width -= self.tuning_step
                if self.actual_tuning_width < self.min_pulse_width:
                    self.tuning_state = TUNING_PHASE.TUNED
                else:
                    self.tuning_timeout = print_time + self.tuning_step_time
                    self.power_off_timeout = eventtime + self.power_off_time
                    self.servo.set_width(self.actual_tuning_width)
            return
        if self.tuning_state == TUNING_PHASE.FAILED:
            return
def load_config_prefix(config):
    return ServoFanFlap(config)