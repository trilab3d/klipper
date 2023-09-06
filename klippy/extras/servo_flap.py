import logging
import math
from . import servo
from enum import Enum
import threading

SAMPLE_TIME = 0.05
SAMPLE_COUNT = 3
REPORT_TIME = 0.200
RANGE_CHECK_COUNT = 4
SERVO_MIN_TIME = 0.100
MIN_PULSE_WIDTH = .00005
MAX_PULSE_WIDTH = .0025

class SERVO_STATE_MACHINE(Enum):
    NOT_TUNED = 0
    TUNING_START = 1
    TUNING_UPPER = 2
    TUNING_LOWER = 3
    TUNING_VALIDATE = 4
    TUNING_ERROR = 5
    TUNING_DONE = 6

HINT_SERVO_FLAP = """
This may indicate servo mechanism malfunction. Check servo wiring and flap mechanical parts.
"""

class ServoFanFlap:
    cmd_FLAP_SET_help = "Sets the flap position. Usage: FLAP_SET FLAP=flap_name " \
                        "[ VALUE=<0. - 1.> | WIDTH=pulse_width ]"
    cmd_FLAP_AUTOTUNE_help = "Finds flap range. Usage: FLAP_AUTOTUNE FLAP=flap_name " \
                             "MIN_PW=min_pulse_width MAX_PW=max_pulse_width START_PW=start_pulse_width " \
                             "SPRINGBACK=spring_back"
    cmd_FLAP_DEBUG_help = "Returns debug informations."
    def __init__(self, config):
        self.printer = config.get_printer()
        self.servo = servo.PrinterServo(config)
        self.flap_name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.min_pulse_width = config.getfloat("minimum_pulse_width", 0.001)
        self.max_pulse_width = config.getfloat("maximum_pulse_width", 0.002)
        self.start_value = config.getfloat("start_value", 0, minval=0, maxval=1)
        self.tuning_start_width = config.getfloat("tuning_start_width", self.min_pulse_width + (
                self.max_pulse_width - self.min_pulse_width) * 0.5)
        self.tuning_step = config.getfloat("tuning_step", 0.000005)
        self.tuning_treshold = config.getfloat("tuning_treshold", 0.1)
        self.tuning_start_time = config.getfloat("tuning_start_time", 1.5)
        self.tuning_step_time = config.getfloat("tuning_step_time", 0.5)
        self.tuning_spring_back = config.getfloat("tuning_spring_back", 0.0001)
        self.validate_upper_max = config.getfloat("validate_upper_max", None)
        self.validate_upper_min = config.getfloat("validate_upper_min", None)
        self.validate_lower_max = config.getfloat("validate_lower_max", None)
        self.validate_lower_min = config.getfloat("validate_lower_min", None)
        self.validate_range_max = config.getfloat("validate_range_max", None)
        self.validate_range_min = config.getfloat("validate_range_min", None)
        self.is_print_fan = config.getboolean("is_print_fan", False)
        self.open_at_sp = config.getboolean("open_at_sp", False)

        self.tuning_state = SERVO_STATE_MACHINE.NOT_TUNED
        if config.getboolean("perform_range_tune", False):
            self.tuning_state = SERVO_STATE_MACHINE.TUNING_START

        self.current_adc = 0.
        self.current_value = -1
        self.current_width = -1
        self.last_time = 0.
        self.move_time = self.tuning_step_time
        self.move_done = threading.Event()
        self.move_timer = self.reactor.register_timer(self.do_move_done)
        self.power_off_time = config.getfloat("power_off_time", 0)
        self.poweroff_timer = self.reactor.register_timer(self.do_poweroff)
        self.tuning_running = False

        # register ADC pin
        #ppins = self.printer.lookup_object('pins')
        feedback_pin_name = config.get("feedback_pin")
        #self.feedback_pin = ppins.setup_pin('adc', feedback_pin_name)
        #self.feedback_pin.get_last_value()
        #self.feedback_pin.setup_adc_callback(REPORT_TIME,
        #                                     self._analog_feedback_callback)
        #self.feedback_pin.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT, minval=0.,
        #                               maxval=1.,
        #                               range_check_count=RANGE_CHECK_COUNT)
        #query_adc = config.get_printer().load_object(config, 'query_adc')
        #query_adc.register_adc(self.flap_name + ":feedback",
        #                       self.feedback_pin)
        #self.mcu = self.feedback_pin.get_mcu()

        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        # register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("FLAP_SET", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_SET,
                                   desc=self.cmd_FLAP_SET_help)
        gcode.register_mux_command("FLAP_AUTOTUNE", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_AUTOTUNE,
                                   desc=self.cmd_FLAP_AUTOTUNE_help)
        gcode.register_mux_command("SET_FAN_SPEED", "FAN",
                                   self.flap_name,
                                   self.cmd_SET_FAN_SPEED,
                                   desc="")
        gcode.register_mux_command("FLAP_DEBUG", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_DEBUG,
                                   desc=self.cmd_FLAP_DEBUG_help)
        
        if self.is_print_fan:
            gcode.register_command("M106", self.cmd_M106)
            gcode.register_command("M107", self.cmd_M107)

    def _handle_connect(self):
        self.move_done.set()
        self.tuning_state = SERVO_STATE_MACHINE.TUNING_DONE

    def cmd_FLAP_SET(self, gcmd):
        width = gcmd.get_float('WIDTH', None)
        if width is not None:
            self.set_width_from_command(width)
            return
        value = gcmd.get_float('VALUE')
        if value is not None:
            self.set_value_from_command(value)
            return  


    def cmd_FLAP_AUTOTUNE(self, gcmd):
        width_changed = False
        tuning_min_pulse_width = gcmd.get_float("MIN_PW", None)
        if tuning_min_pulse_width is not None:
            width_changed = True
            self.min_pulse_width = tuning_min_pulse_width

        tuning_max_pulse_width = gcmd.get_float("MAX_PW", None)
        if tuning_max_pulse_width is not None:
            width_changed = True
            self.max_pulse_width = tuning_max_pulse_width

        tuning_start_width = gcmd.get_float("START_PW", None)
        if width_changed and tuning_start_width is None:
            tuning_start_width = self.min_pulse_width + (self.max_pulse_width - self.min_pulse_width) * 0.5

        if tuning_start_width is not None:
            self.tuning_start_width = tuning_start_width

        spring_back = gcmd.get_float("SPRINGBACK", None)
        if spring_back is not None:
            self.tuning_spring_back = spring_back

        self.tuning_state = SERVO_STATE_MACHINE.TUNING_START

    def cmd_SET_FAN_SPEED(self, gcmd):
        speed = gcmd.get_float('SPEED', minval=0., maxval= 255.)
        if speed is not None:
            if speed > 1:
                speed = speed / 255
            self.set_value_from_command(speed)

    def cmd_FLAP_DEBUG(self, gcmd):
        gcmd.respond_info(f"Current ADC reading: {self.current_adc}, min pulse "
                          f"width: {self.min_pulse_width}, max pulse width: "
                          f"{self.max_pulse_width}, servo state machine: {self.tuning_state}")

    def cmd_M106(self, gcmd):
        val = gcmd.get_float('S', 255., minval=0.) / 255.
        self.set_value_from_command(val)
    def cmd_M107(self, gcmd):
        self.set_value_from_command(0)

    def set_value_from_command(self, value):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt:
                                              self.set_value(pt, value)))
    def set_width_from_command(self, width):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt:
                                              self.set_width(pt, width)))
    def set_value(self, print_time, value):
        if value == self.current_value:
            return
        
        self.current_value = value
        if self.open_at_sp:
            value = 1. - value
        width = self.min_pulse_width + (self.max_pulse_width -
                                    self.min_pulse_width) * value
        self.set_width(print_time, width)

    def set_width(self, print_time, width):
        if width == self.current_width:
            return
        print_time = max(self.last_time + SERVO_MIN_TIME, print_time)
        self.servo.set_width(print_time, width)
        self.current_value = -1 #Reset current value
        self.current_width = width
        self.last_time = print_time
        self.move_done.clear()
        self.move_timer.waketime = self.reactor.monotonic() + self.move_time
        if not self.tuning_running and width > 0 and self.power_off_time > 0:
            self.poweroff_timer.waketime = self.reactor.monotonic() + self.power_off_time
        else:
            self.poweroff_timer.waketime = self.reactor.NEVER
            
    def do_poweroff(self, arg):
        self.set_width_from_command(0.)
        return self.reactor.NEVER
    
    def do_move_done(self, arg):
        self.move_done.set()
        return self.reactor.NEVER

    def get_status(self, eventtime):
        return {
            'speed': self.current_value
        }

    def _analog_feedback_callback(self, last_read_time, last_value):
        # This callback is called periodically. I need it for servo range
        # callibration, but once created, I can't stop it. So I check servo
        # timeout here. If it can't be stopped, it will suffer for rest of
        # his poor life as general program loop.
        self.current_adc = last_value

        if self.tuning_state in [SERVO_STATE_MACHINE.TUNING_START, 
                                 SERVO_STATE_MACHINE.TUNING_UPPER,
                                 SERVO_STATE_MACHINE.TUNING_LOWER, 
                                 SERVO_STATE_MACHINE.TUNING_VALIDATE]:
            if not self.move_done.is_set(): # Wait for moving done
                return

            err_msg = ""
            if self.tuning_state == SERVO_STATE_MACHINE.TUNING_START:
                self.tuning_running = True
                self.move_time = self.tuning_start_time
                self.set_width_from_command(self.tuning_start_width)
                self.tuning_state = SERVO_STATE_MACHINE.TUNING_UPPER

            elif self.tuning_state == SERVO_STATE_MACHINE.TUNING_UPPER:
                if last_value > self.tuning_treshold:
                    self.max_pulse_width = self.current_width - self.tuning_spring_back

                    self.move_time = self.tuning_start_time
                    self.set_width_from_command(self.tuning_start_width)
                    self.tuning_state = SERVO_STATE_MACHINE.TUNING_LOWER
                else:
                    if self.current_width < MAX_PULSE_WIDTH:
                        self.move_time = self.tuning_step_time
                        self.set_width_from_command(self.current_width + self.tuning_step)
                    else:
                        err_msg = f"Servo flap {self.flap_name} tuning cannot found upper limit\n"
                        self.tuning_state = SERVO_STATE_MACHINE.TUNING_ERROR

            elif self.tuning_state == SERVO_STATE_MACHINE.TUNING_LOWER:
                if last_value > self.tuning_treshold:
                    self.min_pulse_width = self.current_width + self.tuning_spring_back
                    self.move_time = self.tuning_start_time
                    self.set_width_from_command(self.tuning_start_width)
                    self.tuning_state = SERVO_STATE_MACHINE.TUNING_VALIDATE
                else:
                    if self.current_width > MIN_PULSE_WIDTH:
                        self.move_time = self.tuning_step_time
                        self.set_width_from_command(self.current_width - self.tuning_step)
                    else:
                        err_msg = f"Servo flap {self.flap_name} tuning cannot found lower limit\n"
                        self.tuning_state = SERVO_STATE_MACHINE.TUNING_ERROR

            elif self.tuning_state == SERVO_STATE_MACHINE.TUNING_VALIDATE:
                is_invalid = False
                err_msg = f"Servo flap {self.flap_name} seems to have invalid range. Following error occured:\n"

                if self.validate_upper_max is not None:
                    if self.max_pulse_width > self.validate_upper_max:
                        is_invalid = True
                        err_msg += f"Max_pulse_width should be <= {self.validate_upper_max}, but is {self.max_pulse_width}.\n"
                if self.validate_upper_min is not None:
                    if self.max_pulse_width < self.validate_upper_min:
                        is_invalid = True
                        err_msg += f"Max_pulse_width should be >= {self.validate_upper_min}, but is {self.max_pulse_width}.\n"
                if self.validate_lower_max is not None:
                    if self.min_pulse_width > self.validate_lower_max:
                        is_invalid = True
                        err_msg += f"Min_pulse_width should be <= {self.validate_lower_max}, but is {self.min_pulse_width}.\n"
                if self.validate_lower_min is not None:
                    if self.min_pulse_width < self.validate_lower_min:
                        is_invalid = True
                        err_msg += f"Min_pulse_width should be >= {self.validate_lower_min}, but is {self.min_pulse_width}.\n"

                pulse_range = self.max_pulse_width - self.min_pulse_width
                if self.validate_range_max is not None:
                    if pulse_range > self.validate_range_min:
                        is_invalid = True
                        err_msg += f"Range should be <= {self.validate_range_max}, but is {pulse_range}. "
                if self.validate_range_min is not None:
                    if pulse_range < self.validate_lower_min:
                        is_invalid = True
                        err_msg += f"Range should be >= {self.validate_range_min}, but is {pulse_range}. "

                if is_invalid:
                    self.tuning_state = SERVO_STATE_MACHINE.TUNING_ERROR
                else:
                    self.tuning_state = SERVO_STATE_MACHINE.TUNING_DONE

            if self.tuning_state == SERVO_STATE_MACHINE.TUNING_DONE:
                self.tuning_running = False
                self.set_value_from_command(self.start_value)
                return
            elif self.tuning_state == SERVO_STATE_MACHINE.TUNING_ERROR:
                self.tuning_running = False
                self.printer.invoke_shutdown(err_msg + HINT_SERVO_FLAP)
                return
            
def load_config_prefix(config):
    return ServoFanFlap(config)