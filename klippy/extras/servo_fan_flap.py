import logging
import math
from . import servo
from enum import Enum
import threading

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 0.100
RANGE_CHECK_COUNT = 4

class SERVO_STATE_MACHINE(Enum):
    TUNING_START = 1
    TUNING_PHASE_1 = 2
    TUNING_PHASE_2 = 3
    TUNED = 4
    ALL_DONE = 5

HINT_SERVO_FLAP = """
This may indicate servo mechanism malfunction. Check servo wiring and flap mechanical parts.
"""


class ServoFanFlap:
    cmd_FLAP_DEBUG_help = "Returns debug informations."
    cmd_FLAP_SET_help = "Sets the flap position. Usage: FLAP_SET FLAP=flap_name " \
                        "[ VALUE=<0. - 1. | 0 - 255> | WIDTH=pulse_width ]"
    cmd_FLAP_AUTOTUNE_help = "Finds flap range. Usage: FLAP_AUTOTUNE FLAP=flap_name " \
                             "MIN_PW=min_pulse_width MAX_PW=max_pulse_width START_PW=start_pulse_width " \
                             "SPRINGBACK=spring_back"
    def __init__(self, config):
        self.printer = config.get_printer()
        self.servo = servo.PrinterServo(config)
        self.flap_name = config.get_name().split()[-1]
        self.last_adc = 0
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.lock = threading.Lock()

        self.is_print_fan = config.getboolean("is_print_fan", False)
        self.open_at_sp = config.getboolean("open_at_sp", False)
        self.min_pulse_width = self.tuning_min_pulse_width = config.getfloat("minimum_pulse_width", 0.001)
        self.max_pulse_width = self.tuning_max_pulse_width = config.getfloat("maximum_pulse_width", 0.002)
        self.tuning_start_width = config.getfloat("tuning_start_width", self.min_pulse_width + (
                self.max_pulse_width - self.min_pulse_width) * 0.5)
        self.tuning_spring_back = config.getfloat("tuning_spring_back", 0.0001)
        self.start_value = config.getfloat(
            "start_value", 0, minval=0, maxval=1)
        self.last_value = -1
        self.power_off_time = config.getfloat("power_off_time", 0)
        self.power_off_timeout = 0
        self.is_on = True

        self.actual_tuning_width = self.tuning_start_width
        self.tuning_timeout = 0
        self.tuning_step = config.getfloat("tuning_step", 0.000005)
        self.tuning_treshold = config.getfloat("tuning_treshold", 0)
        self.tuning_start_time = config.getfloat("tuning_start_time", 0.1)
        self.tuning_step_time = config.getfloat("tuning_step_time", 3)
        if config.getboolean("perform_range_tune", False):
            self.tuning_state = SERVO_STATE_MACHINE.TUNING_START
        else:
            self.tuning_state = SERVO_STATE_MACHINE.TUNED

        self.validate_upper_max = config.getfloat("validate_upper_max", None)
        self.validate_upper_min = config.getfloat("validate_upper_min", None)
        self.validate_lower_max = config.getfloat("validate_lower_max", None)
        self.validate_lower_min = config.getfloat("validate_lower_min", None)
        self.validate_range_max = config.getfloat("validate_range_max", None)
        self.validate_range_min = config.getfloat("validate_range_min", None)

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
        gcode.register_mux_command("FLAP_AUTOTUNE", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_AUTOTUNE,
                                   desc=self.cmd_FLAP_AUTOTUNE_help)
        if self.is_print_fan:
            gcode.register_command("M106", self.cmd_M106)
            gcode.register_command("M107", self.cmd_M107)

    def cmd_FLAP_DEBUG(self, gcmd):
        gcmd.respond_info(f"Last ADC reading: {self.last_adc}, min pulse "
                          f"width: {self.min_pulse_width}, max pulse width: "
                          f"{self.max_pulse_width}, servo state machine: {self.tuning_state}")
    def cmd_FLAP_SET(self, gcmd):
        width = gcmd.get_float('WIDTH', None)
        if width is not None:
            self.servo.set_width(width)
        else:
            val = gcmd.get_float('VALUE', minval=0., maxval= 255.)
            if val > 1:
                val = val / 255
            self.set_value(val)

    def cmd_FLAP_AUTOTUNE(self, gcmd):
        width_changed = False
        tuning_min_pulse_width = gcmd.get_float("MIN_PW", None)
        if tuning_min_pulse_width is not None:
            width_changed = True
            self.min_pulse_width = self.tuning_min_pulse_width = tuning_min_pulse_width
        else:
            self.min_pulse_width = self.tuning_min_pulse_width

        tuning_max_pulse_width = gcmd.get_float("MAX_PW", None)
        if tuning_max_pulse_width is not None:
            width_changed = True
            self.max_pulse_width = self.tuning_max_pulse_width = tuning_max_pulse_width
        else:
            self.max_pulse_width = self.tuning_max_pulse_width

        tuning_start_width = gcmd.get_float("START_PW", None)
        if width_changed and tuning_start_width is None:
            tuning_start_width = self.min_pulse_width + (self.max_pulse_width - self.min_pulse_width) * 0.5

        if tuning_start_width is not None:
            self.tuning_start_width = tuning_start_width

        spring_back = gcmd.get_float("SPRINGBACK", None)
        if spring_back is not None:
            self.tuning_spring_back = spring_back

        self.actual_tuning_width = self.tuning_start_width
        self.tuning_state = SERVO_STATE_MACHINE.TUNING_START

    def cmd_M106(self, gcmd):
        val = gcmd.get_float('S', 255., minval=0.) / 255.
        self.set_value(val)
    def cmd_M107(self, gcmd):
        self.set_value(0)

    def set_value(self, value, print_time=None):
        if math.isclose(self.last_value,value):
            return
        self.last_value = value
        if self.open_at_sp:
            value = 1 - value
        w = self.min_pulse_width + (self.max_pulse_width -
                                    self.min_pulse_width) * value
        eventtime = self.reactor.monotonic()
        self.power_off_timeout = eventtime + self.power_off_time
        self.is_on = True
        self.servo.set_width(w, print_time)

    def _analog_feedback_callback(self, last_read_time, last_value):
        # This callback is called periodically. I need it for servo range
        # callibration, but once created, I can't stop it. So I check servo
        # timeout here. If it can't be stopped, it will suffer for rest of
        # his poor life as general program loop.
        with self.lock:
            if self.tuning_state != SERVO_STATE_MACHINE.ALL_DONE:
                self._handle_range_tuning(last_read_time, last_value)
            else:
                self._handle_timeout(last_read_time)

            self.last_adc = last_value

    def _handle_timeout(self, print_time):
        if self.power_off_time > 0:
            eventtime = self.reactor.monotonic()
            if eventtime > self.power_off_timeout and self.is_on:
                self.is_on = False
                self.servo.power_off(print_time + 0.005)

    def _handle_range_tuning(self, last_read_time, last_value):
        eventtime = self.reactor.monotonic()
        print_time = self.mcu.estimated_print_time(eventtime)
        if self.tuning_state == SERVO_STATE_MACHINE.TUNING_START:
            self.power_off_timeout = eventtime + self.power_off_time
            self.servo.set_width(self.actual_tuning_width, print_time + 0.005)
            self.tuning_timeout = print_time + self.tuning_start_time
            self.tuning_state = SERVO_STATE_MACHINE.TUNING_PHASE_1
            return

        if self.tuning_state == SERVO_STATE_MACHINE.TUNING_PHASE_1:
            if last_read_time < self.tuning_timeout:
                return
            if last_value > self.tuning_treshold:
                self.max_pulse_width = self.actual_tuning_width - self.tuning_spring_back
                self.actual_tuning_width = self.tuning_start_width
                self.tuning_state = SERVO_STATE_MACHINE.TUNING_PHASE_2
                self.tuning_timeout = print_time + self.tuning_start_time
            else:
                self.actual_tuning_width += self.tuning_step
                if self.actual_tuning_width > self.max_pulse_width:
                    self.actual_tuning_width = self.tuning_start_width
                    self.tuning_timeout = print_time + self.tuning_start_time
                    self.tuning_state = SERVO_STATE_MACHINE.TUNING_PHASE_2
                else:
                    self.tuning_timeout = print_time + self.tuning_step_time
            self.power_off_timeout = eventtime + self.power_off_time
            self.servo.set_width(self.actual_tuning_width, print_time + 0.005)
            return

        if self.tuning_state == SERVO_STATE_MACHINE.TUNING_PHASE_2:
            if last_read_time < self.tuning_timeout:
                return
            if last_value > self.tuning_treshold:
                self.min_pulse_width = self.actual_tuning_width + self.tuning_spring_back
                self.tuning_state = SERVO_STATE_MACHINE.TUNED
                self.set_value(self.start_value, print_time + 0.005)
            else:
                self.actual_tuning_width -= self.tuning_step
                if self.actual_tuning_width < self.min_pulse_width:
                    self.tuning_state = SERVO_STATE_MACHINE.TUNED
                else:
                    self.tuning_timeout = print_time + self.tuning_step_time
                    self.power_off_timeout = eventtime + self.power_off_time
                    self.servo.set_width(self.actual_tuning_width, print_time + 0.005)
            return
        if self.tuning_state == SERVO_STATE_MACHINE.TUNED:

            is_invalid = False
            msg = f"Servo flap {self.flap_name} seems to have invalid range. Following error occured:\n"

            if self.validate_upper_max is not None:
                if self.max_pulse_width > self.validate_upper_max:
                    is_invalid = True
                    msg += f"Max_pulse_width should be <= {self.validate_upper_max}, but is {self.max_pulse_width}.\n"

            if self.validate_upper_min is not None:
                if self.max_pulse_width < self.validate_upper_min:
                    is_invalid = True
                    msg += f"Max_pulse_width should be >= {self.validate_upper_min}, but is {self.max_pulse_width}.\n"

            if self.validate_lower_max is not None:
                if self.min_pulse_width > self.validate_lower_max:
                    is_invalid = True
                    msg += f"Min_pulse_width should be <= {self.validate_lower_max}, but is {self.min_pulse_width}.\n"

            if self.validate_lower_min is not None:
                if self.min_pulse_width < self.validate_lower_min:
                    is_invalid = True
                    msg += f"Min_pulse_width should be >= {self.validate_lower_min}, but is {self.min_pulse_width}.\n"

            range = self.max_pulse_width - self.min_pulse_width

            if self.validate_range_max is not None:
                if range > self.validate_range_min:
                    is_invalid = True
                    msg += f"Range should be <= {self.validate_range_max}, but is {range}. "

            if self.validate_range_min is not None:
                if range < self.validate_lower_min:
                    is_invalid = True
                    msg += f"Range should be >= {self.validate_range_min}, but is {range}. "

            if is_invalid:
                self.printer.invoke_shutdown(msg + HINT_SERVO_FLAP)
                return

            self.set_value(self.start_value, print_time)
            self.tuning_state = SERVO_STATE_MACHINE.ALL_DONE
            return
def load_config_prefix(config):
    return ServoFanFlap(config)
