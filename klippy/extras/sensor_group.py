import math, logging
import configparser
from configfile import ConfigWrapper

KELVIN_TO_CELSIUS = -273.15
class SensorGroup:
    cmd_SENSOR_GROUP_DEBUG_help = "Return sensor group temperatures"
    def __init__(self, config, sensors, name, max_absolute_deviation):
        self.name = name
        self.num_sensors = len(sensors)
        self.temps = [0]*self.num_sensors
        self.temps_valid = [False]*self.num_sensors
        self.last_valid_temps = [0]*self.num_sensors
        self.last_temp = 0
        self.sensors = sensors
        self.max_absolute_deviation = max_absolute_deviation
        for i, s in enumerate(self.sensors):
            cb = self._callback_factory(i)
            s.setup_callback(cb)
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SENSOR_GROUP_DEBUG", "GROUP",
                                   self.name,
                                   self.cmd_SENSOR_GROUP_DEBUG,
                                   desc=self.cmd_SENSOR_GROUP_DEBUG_help)
    def cmd_SENSOR_GROUP_DEBUG(self, gcmd):
        gcmd.respond_info(f"Last average temperature: {self.last_temp}. Last sensors temperatures: {self.last_valid_temps}")
    def _callback_factory(self, i):
        def cb(read_time, read_value):
            nonlocal self
            nonlocal i
            self.temps[i] = read_value
            self.temps_valid[i] = True
            all_valid = True
            for v in self.temps_valid:
                all_valid = all_valid and v
            if all_valid:
                # invalidate all values
                for i, s in enumerate(self.temps_valid):
                    self.temps_valid[i] = False
                awg = 0
                for i, t in enumerate(self.temps):
                    awg += t
                    self.last_valid_temps[i] = t
                awg = awg / self.num_sensors
                self.last_temp = awg
                if self.max_absolute_deviation is not None:
                    for i, t in enumerate(self.temps):
                        if abs(t-awg) > self.max_absolute_deviation:
                            self.printer.invoke_shutdown(f"Sensor group {self.name} sensor {i} deviated so much "
                                                         f"from average. Avg temp: {awg}, Sensor temps: {self.temps}")
                if self.temperature_callback is not None:
                    self.temperature_callback(read_time, awg)
        return cb
    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback
    def get_report_time_delta(self):
        return 0
    def setup_minmax(self, min_temp, max_temp):
        for s in self.sensors:
            s.setup_minmax(min_temp, max_temp)

class SensorGroupFactory:
    def __init__(self, config):
        self.name = " ".join(config.get_name().split()[1:])
        pheaters = config.get_printer().load_object(config, "heaters")
        self.num_sensors = config.getint("num_sensors")
        self.max_absolute_deviation = config.getfloat("max_absolute_deviation", 20)
        self.sensors = []
        for i in range(self.num_sensors):
            prefix = f"sensor_{i+1}_"
            rc = configparser.RawConfigParser()
            rc.add_section(prefix)
            for o in config.get_prefix_options(prefix):
                rc.set(prefix, o[len(prefix):], config.get(o))
            sensor_config = ConfigWrapper(config.printer, rc, config.access_tracking, prefix)
            self.sensors.append(pheaters.setup_sensor(sensor_config))

    def create(self, config):
        return SensorGroup(config, self.sensors, self.name, self.max_absolute_deviation)

def load_config_prefix(config):
    sensor_group_factory = SensorGroupFactory(config)
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory(sensor_group_factory.name, sensor_group_factory.create)