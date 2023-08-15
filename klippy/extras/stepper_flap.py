import logging
import math
import stepper, chelper
from enum import Enum
from . import force_move

class SERVO_STATE_MACHINE(Enum):
    MOVING = 1
    DISABLING = 2
    IDLE = 3

class StepperFlap:
    cmd_FLAP_SET_help = "Sets the flap position. Usage: FLAP_SET FLAP=flap_name " \
                        "[ VALUE=<0. - 1. | 0 - 255> | WIDTH=pulse_width ]"
    def __init__(self, config):
        self.printer = config.get_printer()
        self.flap_name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.stepper = stepper.PrinterStepper(config)
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        
        self.requested_value = 0
        self.current_value = 0

        self.update_timer = self.reactor.register_timer(self.do_update_value)
        self.disable_timer = self.reactor.register_timer(self.do_disable)

        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)
        self.next_cmd_time = 0.

        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves

        self.stepper.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.stepper.set_trapq(self.trapq)

        self.is_print_fan = config.getboolean("is_print_fan", False)
        self.wanted_value = config.getfloat("start_value", 0, minval=0, maxval=1)

        # register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("FLAP_SET", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_SET,
                                   desc=self.cmd_FLAP_SET_help)
        if self.is_print_fan:
            gcode.register_command("M106", self.cmd_M106)
            gcode.register_command("M107", self.cmd_M107)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        self.update_timer.waketime = self.reactor.monotonic() + 2

    def cmd_FLAP_SET(self, gcmd):
        width = gcmd.get_float('WIDTH', None)
        if width is not None:
            return
        else:
            val = gcmd.get_float('VALUE', minval=0., maxval= 255.)
            if val > 1:
                val = val / 255
            self.requested_value = val

    def cmd_M106(self, gcmd):
        self.requested_value = gcmd.get_float('S', 255., minval=0.) / 255.
        
    def cmd_M107(self, gcmd):
        self.requested_value = 0

    def do_update_value(self, time):
        move_time = 0.05
        if self.current_value != self.requested_value:
            move_time = self.set_value(self.requested_value)

        self.current_value = self.requested_value

        return self.reactor.monotonic() + move_time
    
    def do_disable(self, arg):
        stepper_enable = self.printer.lookup_object('stepper_enable')
        se = stepper_enable.lookup_enable(self.stepper.get_name())
        self.toolhead.register_lookahead_callback((lambda pt: se.motor_disable(pt)))
        return self.reactor.NEVER

    def set_value(self, value):
        self.disable_timer.waketime = self.reactor.NEVER
        move_time = self.do_move(value, self.velocity, self.accel)
        self.disable_timer.waketime = self.reactor.monotonic() + 1

        return move_time

    def sync_print_time(self):
        curtime = self.reactor.monotonic()
        est_print_time = self.toolhead.mcu.estimated_print_time(curtime)
        print_time = self.toolhead.get_last_move_time()
        print_time = max(print_time, est_print_time)
        if self.next_cmd_time > print_time:
            self.toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def do_move(self, movepos, speed, accel):
        self.sync_print_time()
        cp = self.stepper.get_commanded_position()
        dist = movepos - cp
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            dist, speed, accel)
        self.trapq_append(self.trapq, self.next_cmd_time,
                          accel_t, cruise_t, accel_t,
                          cp, 0., 0., axis_r, 0., 0.,
                          0., cruise_v, accel)
        self.next_cmd_time = self.next_cmd_time + accel_t + cruise_t + accel_t
        self.stepper.generate_steps(self.next_cmd_time)
        self.trapq_finalize_moves(self.trapq, self.next_cmd_time + 99999.9)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.note_kinematic_activity(self.next_cmd_time)
        return accel_t + cruise_t + accel_t

def load_config_prefix(config):
    return StepperFlap(config)
