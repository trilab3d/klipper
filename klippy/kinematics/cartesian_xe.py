# Code for handling cartesian XE kinematics - rotující extruder
# X osa má dva motory:
#   X_left = X
#   X_right = X - rotation_ratio * E
# Rozdíl pozic motorů otáčí extruderem
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper, chelper

class CartesianXEKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead

        # Setup normální Y a Z osy
        self.rails = [
            None,  # X bude speciální
            stepper.LookupMultiRail(config.getsection('stepper_y')),
            stepper.LookupMultiRail(config.getsection('stepper_z'))
        ]

        # Setup Y a Z s normální cartesian kinematikou
        self.rails[1].setup_itersolve('cartesian_stepper_alloc', b'y')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')

        # Setup dvou X motorů s cartesian_xe kinematikou
        # stepper_x - standardní X motor
        x_config = config.getsection('stepper_x')
        self.stepper_x = stepper.PrinterStepper(x_config)

        # stepper_xe - X motor který reaguje na E (X - rotation_ratio * E)
        xe_config = config.getsection('stepper_xe')
        self.stepper_xe = stepper.PrinterStepper(xe_config)

        # Načti rotation_ratio ze stepper_xe configu
        self.rotation_ratio = xe_config.getfloat('rotation_ratio', above=0.)

        # Validace rozumných hodnot
        if self.rotation_ratio > 10.0:
            raise config.error(
                "rotation_ratio %.2f is too high. "
                "Maximum recommended value is 10.0. "
                "High values can cause slow step generation and motor instability."
                % self.rotation_ratio)
        if self.rotation_ratio < 0.1:
            raise config.error(
                "rotation_ratio %.2f is too low. "
                "Minimum recommended value is 0.1. "
                "Low values may not provide sufficient rotation."
                % self.rotation_ratio)

        # Alokuj cartesian_xe kinematics pro oba X steppers
        ffi_main, ffi_lib = chelper.get_ffi()
        self.sk_x = ffi_main.gc(
            ffi_lib.cartesian_xe_stepper_alloc(ord('x')), ffi_lib.free)
        self.sk_xe = ffi_main.gc(
            ffi_lib.cartesian_xe_stepper_alloc(ord('x')), ffi_lib.free)

        self.stepper_x.set_stepper_kinematics(self.sk_x)
        self.stepper_xe.set_stepper_kinematics(self.sk_xe)

        # Nastav trapq pro všechny steppers
        xyz_trapq = toolhead.get_trapq()

        # stepper_x sleduje XYZ trapq (standardní)
        self.stepper_x.set_trapq(xyz_trapq)

        # stepper_xe také XYZ trapq, ale bude používat custom generate_steps
        self.stepper_xe.set_trapq(xyz_trapq)


        for rail in self.rails[1:]:
            for s in rail.get_steppers():
                s.set_trapq(xyz_trapq)

        # Extruder stepper kinematics bude nastavena později v _handle_connect
        self.cartesian_xe_set_sk = ffi_lib.cartesian_xe_set_extruder_sk

        # Setup boundary checks
        ranges = [
            x_config.getfloat('position_min', 0.),
            x_config.getfloat('position_max', above=0.)
        ] + [r.get_range() for r in self.rails[1:]]

        self.axes_min = toolhead.Coord(ranges[0][0], ranges[1][0],
                                       ranges[2][0], e=0.)
        self.axes_max = toolhead.Coord(ranges[0][1], ranges[1][1],
                                       ranges[2][1], e=0.)

        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3

        # Register event handler pro propojení extruder trapq
        self.printer.register_event_handler("klippy:connect",
                                           self._handle_connect)

    def _handle_connect(self):
        # Získej extruder stepper kinematics (obsahuje PA logiku!)
        extruder = self.toolhead.get_extruder()

        # Extruder MUSÍ mít stepper pro cartesian_xe
        if not hasattr(extruder, 'extruder_stepper') or not extruder.extruder_stepper:
            raise self.printer.config_error(
                "Cartesian XE kinematics requires extruder with stepper!\n"
                "Please define step_pin, dir_pin, and rotation_distance in [extruder] section.\n"
                "Example:\n"
                "  step_pin: PA1\n"
                "  dir_pin: PA2\n"
                "  enable_pin: PA3\n"
                "  rotation_distance: 33.5\n"
                "  microsteps: 16")

        # Extruder má stepper - použij jeho kinematics (s PA)
        extruder_sk = extruder.extruder_stepper.stepper.get_stepper_kinematics()
        extruder_trapq = extruder.get_trapq()

        # ŘEŠENÍ: stepper_xe sleduje E trapq + gen_steps_post_active=INF
        # Tím itersolve generuje kroky i když E move skončil ale XYZ probíhá
        # - Kombinované pohyby (X+E) - funguje ✅
        # - Čisté E pohyby (retrakce) - funguje ✅
        # - Čisté X pohyby - funguje ✅ (díky post_active)
        # - Pressure advance - funguje ✅

        logging.info("Cartesian XE: stepper_xe using dual-trapq with post_active")

        # Nastav extruder SK pro X steppers
        ffi_main, ffi_lib = chelper.get_ffi()
        # stepper_x = X (bez E komponenty)
        self.cartesian_xe_set_sk(self.sk_x, extruder_sk, 0.0)
        # stepper_xe = X - rotation_ratio * E
        self.cartesian_xe_set_sk(self.sk_xe, extruder_sk,
                                 -self.rotation_ratio)

    def get_steppers(self):
        return ([self.stepper_x, self.stepper_xe] +
                [s for rail in self.rails[1:] for s in rail.get_steppers()])

    def calc_position(self, stepper_positions):
        x_pos = stepper_positions[self.stepper_x.get_name()]
        y_pos = stepper_positions[self.rails[1].get_name()]
        z_pos = stepper_positions[self.rails[2].get_name()]

        return [x_pos, y_pos, z_pos]

    def set_position(self, newpos, homing_axes):
        for s in [self.stepper_x, self.stepper_xe]:
            s.set_position(newpos)
        for i, rail in enumerate(self.rails[1:], 1):
            rail.set_position(newpos)
            if "xyz"[i] in homing_axes:
                self.limits[i] = rail.get_range()

        if 'x' in homing_axes:
            self.limits[0] = (self.axes_min.x, self.axes_max.x)

    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate("xyz"):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)

    def home(self, homing_state):
        for axis in homing_state.get_axes():
            if axis == 0:  # X axis
                # AUTOMATICKY nastav E na 0 před homingem
                # Tím zajistíme že oba X motory jsou na stejné pozici
                extruder = self.toolhead.get_extruder()
                print_time = self.toolhead.get_last_move_time()

                # Nastav E pozici na 0 v extruder trapq
                extruder_trapq = extruder.get_trapq()
                ffi_main, ffi_lib = chelper.get_ffi()
                ffi_lib.trapq_set_position(extruder_trapq, print_time, 0., 0., 0.)

                # Nastav extruder stepper pozici na 0
                if hasattr(extruder, 'extruder_stepper') and extruder.extruder_stepper:
                    extruder.extruder_stepper.stepper.set_position([0., 0., 0., 0.])

                logging.info("Cartesian XE: Reset E position to 0 before X homing")

                # Použij stepper_x config pro homing
                x_config = self.printer.lookup_object('configfile').getsection('stepper_x')
                position_min = x_config.getfloat('position_min', 0.)
                position_max = x_config.getfloat('position_max', above=0.)
                position_endstop = x_config.getfloat('position_endstop')

                # Homing parametry
                hi_dict = {
                    'position_endstop': position_endstop,
                    'position_min': position_min,
                    'position_max': position_max,
                    'homing_speed': x_config.getfloat('homing_speed', 5.0),
                    'homing_retract_dist': x_config.getfloat('homing_retract_dist', 5.0),
                    'homing_positive_dir': x_config.getboolean('homing_positive_dir',
                                                               position_endstop > (position_min + position_max) / 2),
                    'second_homing_speed': x_config.getfloat('second_homing_speed', None)
                }

                homepos = [None, None, None, None]
                homepos[0] = position_endstop
                forcepos = list(homepos)
                if hi_dict['homing_positive_dir']:
                    forcepos[0] -= 1.5 * (position_endstop - position_min)
                else:
                    forcepos[0] += 1.5 * (position_max - position_endstop)

                # Home oba steppers současně
                # Teď je bezpečné - E=0, takže oba motory jsou na stejné pozici
                homing_state.home_rails([[self.stepper_x, self.stepper_xe]],
                                       forcepos, homepos)

                logging.info("X axis homed successfully (both motors synchronized)")
            else:
                # Normální homing pro Y a Z
                rail = self.rails[axis]
                self.home_axis(homing_state, axis, rail)

    def home_axis(self, homing_state, axis, rail):
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CartesianXEKinematics(toolhead, config)
