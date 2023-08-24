import logging
import math
import stepper, chelper
from enum import Enum
from . import force_move

class SERVO_STATE_MACHINE(Enum):
    MOVING = 1
    DISABLING = 2
    IDLE = 3

FLUSH_DELAY = 0.001

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

        self.invert = config.getboolean('invert', False)

        self.wanted_value = config.getfloat("start_value", 0, minval=0, maxval=1)

        # register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("FLAP_SET", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_SET,
                                   desc=self.cmd_FLAP_SET_help)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        self.update_timer.waketime = self.reactor.monotonic() + 1
        self.disable_timer.waketime = self.reactor.monotonic() + 2            

    def cmd_FLAP_SET(self, gcmd):
        val = gcmd.get_float('VALUE', minval=0., maxval= 255.)
        if val > 1:
            val = val / 255
        self.set_value(val)

    def set_value(self, value):
        if self.invert:
            self.requested_value = -1.0 * value
        else:
            self.requested_value = value

        eventtime = self.reactor.monotonic()
        toolhead = self.printer.lookup_object('toolhead')
        _, _, lookahead_empty = toolhead.check_busy(
            eventtime)

        if lookahead_empty:
            self.update_timer.waketime = self.reactor.monotonic() + 0.5
            self.do_move(self.requested_value, self.velocity, self.accel, True)

    def do_update_value(self, time):
        move_time = 0.05
        if self.current_value != self.requested_value:
            move_time = self.do_move(self.requested_value, self.velocity, self.accel, False)

            print_time = self.printer.lookup_object('toolhead').print_time
            if self.next_cmd_time > print_time:
                move_time += (self.next_cmd_time - print_time)

            self.current_value = self.requested_value

        return self.reactor.monotonic() + move_time
    
    def do_disable(self, arg):
        stepper_enable = self.printer.lookup_object('stepper_enable')
        se = stepper_enable.lookup_enable(self.stepper.get_name())
        self.toolhead.register_lookahead_callback((lambda pt: se.motor_disable(pt)))
        return self.reactor.NEVER

    def sync_print_time(self):
        print_time = self.toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            self.toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def do_move(self, movepos, speed, accel, sync=True):
        self.disable_timer.waketime = self.reactor.NEVER
        if sync:
            self.sync_print_time()
        else:
            self.next_cmd_time = self.printer.lookup_object('toolhead').print_time

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
        self.trapq_finalize_moves(self.trapq, self.next_cmd_time + 2.)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.note_kinematic_activity(self.next_cmd_time)
        if sync:
            self.sync_print_time()
        self.disable_timer.waketime = self.reactor.monotonic() + 2
        return accel_t + cruise_t + accel_t

def load_config_prefix(config):
    return StepperFlap(config)
