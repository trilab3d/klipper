class PrinterHeaterChamber:
    def __init__(self, config):
        self.printer = config.get_printer()
        pheaters = self.printer.load_object(config, 'heaters')
        self.heater = pheaters.setup_heater(config, master=True)
        self.get_status = self.heater.get_status
        self.stats = self.heater.stats
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M141", self.cmd_M141)
        gcode.register_command("M191", self.cmd_M191)
    def cmd_M141(self, gcmd, wait=False):
        # Set Chamber Temperature
        temp = gcmd.get_float('S', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self.heater, temp, wait)
    def cmd_M191(self, gcmd):
        # Set Bed Temperature and Wait
        self.cmd_M141(gcmd, wait=True)

def load_config(config):
    return PrinterHeaterChamber(config)
