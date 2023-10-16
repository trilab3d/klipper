import logging
import stepper, chelper
from queue import Queue
from . import force_move

DISABLE_STALL_TIME = 0.100


class IndependentStepper:
    class Move:
        def __init__(self, start_pos, end_pos, speed, accel):
            self.start_pos = start_pos
            self.end_pos = end_pos
            self.speed = speed
            self.accel = accel
            distance = self.end_pos - self.start_pos
            self.axis_r_x, self.accel_t, self.cruise_t, self.cruise_v = force_move.calc_move_time(distance, self.speed,
                                                                                                  self.accel)
            self.decel_t = self.accel_t
            self.total_t = self.accel_t + self.cruise_t + self.decel_t

    def __init__(self, config):
        self.printer = config.get_printer()
        self.stepper = stepper.PrinterStepper(config)
        self.commanded_pos = 0.
        self.next_cmd_time = 0.

        # Create separate move queue and setup iterative solver.
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves

        # Setup the stepper with cartesian kinematics and assign the separate move queue.
        self.stepper.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.stepper.set_trapq(self.trapq)

        # Queue of moves before they are inserted into trapq, because we want to synchronize begging of the movement
        # of the independent stepper with the movement of the toolhead or the extruder if it's possible.
        self.move_queue = Queue()
        self.registered_callback_for_toolhead_last_move = False

        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = config.getfloat('accel', 0., minval=0.)
        self.disable_when_inactive = config.getboolean('disable_when_inactive', True)
        self.disable_delay = config.getboolean('disable_delay', 2.)
        self.last_active_time = 0.

        if self.disable_when_inactive:
            self.stepper.add_persistent_active_callback(self._handle_active)
            self.stepper.add_persistent_inactive_callback(self._handle_inactive)

        # We need to wait until toolhead is fully initialized.
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_active(self, print_time):
        self.last_active_time = print_time

    def _handle_inactive(self, print_time, stepqueue):
        if print_time < self.last_active_time + self.disable_delay:
            # We have to wait until the disable delay elapses.
            return

        _, ffi_lib = chelper.get_ffi()
        has_untransmitted_steps = ffi_lib.stepcompress_has_untransmitted_steps(stepqueue)
        if has_untransmitted_steps:
            # Stepper motor has untransmitted messages or pending steps in stepcompress.
            # That means that the stepper will be active, so we have to wait before we
            # disable the stepper motor.
            return

        stepper_enable = self.printer.lookup_object('stepper_enable')
        enable_line = stepper_enable.lookup_enable(self.stepper.get_name())
        enable_line.motor_disable(print_time)

    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_step_generator(self.stepper.generate_steps)
        toolhead.register_independent_stepper(self)
        toolhead.register_move_queue_add_move_callback(self._handle_move_queue_added_move)

    def _handle_move_queue_added_move(self):
        self.registered_callback_for_toolhead_last_move = False

    def _handle_lookahead_processed_move(self, print_time):
        if self.move_queue.empty():
            logging.error("Move queue of independent stepper: %s is empty.", self.stepper.get_name())
        else:
            moves = self.move_queue.get()
            while not moves.empty():
                move = moves.get()
                self.process_move(move, print_time)

        # When the move_queue is empty, we have to reset the callback indicator because this move is already
        # processed. So we don't want to assign another move for them.
        if self.move_queue.empty():
            self.registered_callback_for_toolhead_last_move = False

    def update_move_time(self, flush_time):
        self.trapq_finalize_moves(self.trapq, flush_time)

    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def enable(self, enable):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()

        stepper_enable = self.printer.lookup_object('stepper_enable')
        enable_line = stepper_enable.lookup_enable(self.stepper.get_name())
        if enable:
            enable_line.motor_enable(print_time)
            logging.debug("%s has been manually enabled.", self.stepper.get_name())
        else:
            enable_line.motor_disable(print_time)
            logging.debug("%s has been manually disabled.", self.stepper.get_name())

        toolhead.dwell(DISABLE_STALL_TIME)

    def set_position(self, new_pos):
        # Fow now setting position needs to flush all toolhead.move_queue, all trapq, and generated steps.
        # So this led to the printer could stop printing for a short amount of time.
        # Probably can be implemented without stopping if it is needed, but all attempts lead to crashing
        # Klipper for unknown reasons.
        self.sync_print_time()
        if not self.move_queue.empty():
            logging.error("Move queue of independent stepper: %s wasn't emptied after flushing toolhead.",
                          self.stepper.get_name())

        _, ffi_lib = chelper.get_ffi()
        ffi_lib.trapq_set_position(self.trapq, self.next_cmd_time, new_pos, 0., 0.)
        self.commanded_pos = new_pos
        self.stepper.set_position([new_pos, 0., 0.])

    def move(self, newpos, speed, accel):
        toolhead = self.printer.lookup_object('toolhead')
        move = self.Move(self.commanded_pos, newpos, speed, accel)
        if move.total_t <= 0.:
            return

        if len(toolhead.move_queue.queue) == 0:
            # When the toolhead.move_queue is empty, put move into trapq imminently.
            if not self.move_queue.empty():
                logging.error("Move queue of independent stepper: %s isn't empty.", self.stepper.get_name())

            # There are no moves in the toolhead queue, so we synchronize time and process this move immediately.
            self.sync_print_time()
            self.process_move(move, self.next_cmd_time)
        elif self.registered_callback_for_toolhead_last_move:
            # The callback on the toolhead last move is already registered, so we just add the move to the end of the associated queue.
            if self.move_queue.empty():
                logging.error("Move queue of independent stepper: %s is empty.", self.stepper.get_name())

            self.move_queue.queue[-1].put(move)
        else:
            self.move_queue.put(Queue())
            self.move_queue.queue[-1].put(move)

            # Move could be immediately processed.
            if toolhead.register_lookahead_callback(self._handle_lookahead_processed_move):
                self.registered_callback_for_toolhead_last_move = True

        self.commanded_pos = move.end_pos

    def process_move(self, move, print_time):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = max(print_time, self.next_cmd_time)
        self.trapq_append(self.trapq, print_time, move.accel_t, move.cruise_t, move.accel_t, move.start_pos, 0., 0.,
                          move.axis_r_x, 0., 0., 0., move.cruise_v, move.accel)
        self.next_cmd_time = print_time + move.total_t

        if len(toolhead.move_queue.queue) == 0:
            toolhead.note_kinematic_activity(self.next_cmd_time)
            self.sync_print_time()
            if self.disable_delay > 0:
                toolhead.dwell(self.disable_delay + toolhead.kin_flush_delay)
                self.enable(False)

    def flush(self):
        self.sync_print_time()


class IndependentStepperImplementation:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.independent_stepper = IndependentStepper(config)

        # Register commands
        self.stepper_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('INDEPENDENT_STEPPER', "STEPPER", self.stepper_name, self.cmd_INDEPENDENT_STEPPER,
                                   desc=self.cmd_INDEPENDENT_STEPPER_help)

    cmd_INDEPENDENT_STEPPER_help = "Command an independently configured stepper."

    def cmd_INDEPENDENT_STEPPER(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.independent_stepper.enable(enable)

        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.independent_stepper.set_position(setpos)
        speed = gcmd.get_float('SPEED', self.independent_stepper.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.independent_stepper.accel, minval=0.)

        if gcmd.get_float('MOVE', None) is not None:
            movepos = gcmd.get_float('MOVE')
            self.independent_stepper.move(movepos, speed, accel)


def load_config_prefix(config):
    return IndependentStepperImplementation(config)
