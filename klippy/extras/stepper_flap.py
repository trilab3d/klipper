import stepper, chelper
from extras.independent_stepper import IndependentStepper


class StepperFlap:
    cmd_FLAP_SET_help = "Sets the flap position. Usage: FLAP_SET FLAP=flap_name " \
                        "[ VALUE=<0. - 1. | 0 - 255> | WIDTH=pulse_width ]"

    def __init__(self, config):
        self.printer = config.get_printer()
        self.flap_name = config.get_name().split()[-1]
        self.independent_stepper = IndependentStepper(config)

        self.requested_value = 0
        self.current_value = 0

        self.invert = config.getboolean('invert', False)
        # self.wanted_value = config.getfloat("start_value", 0, minval=0, maxval=1)
        self.is_print_fan = config.getboolean("is_print_fan", False)

        # register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("FLAP_SET", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_SET,
                                   desc=self.cmd_FLAP_SET_help)
        gcode.register_mux_command("SET_FAN_SPEED", "FAN",
                                   self.flap_name,
                                   self.cmd_FLAP_SET,
                                   desc="")
        gcode.register_mux_command("FLAP_HOME", "FLAP",
                                   self.flap_name,
                                   self.cmd_FLAP_HOME,
                                   desc="")
        if self.is_print_fan:
            gcode.register_command("M106", self.cmd_M106)
            gcode.register_command("M107", self.cmd_M107)

    def cmd_FLAP_SET(self, gcmd):
        val = gcmd.get_float('SPEED', minval=0., maxval=255.)
        if val > 1:
            val = val / 255
        self.set_value(val)

    def cmd_FLAP_HOME(self, gcmd):
        self.set_value(1.5)
        self.set_value(0)

    def cmd_M106(self, gcmd):
        val = gcmd.get_float('S', 255., minval=0.) / 255.
        self.set_value(val)

    def cmd_M107(self, gcmd):
        self.set_value(0.)

    def set_value(self, value):
        if self.invert:
            self.requested_value = -1.0 * value
        else:
            self.requested_value = value

        if self.current_value != self.requested_value:
            self.current_value = self.requested_value
            self.independent_stepper.move(self.requested_value, self.independent_stepper.velocity,
                                          self.independent_stepper.accel)

    def get_status(self, eventtime):
        return {
            'speed': self.requested_value
        }


def load_config_prefix(config):
    return StepperFlap(config)
