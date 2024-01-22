
import logging
from . import bus

MLX90614_I2C_ADDR= 0x5A

class MLX90614:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=MLX90614_I2C_ADDR, default_speed=100000)
        self.report_time = config.getfloat('mlx90614_report_time',0.2,minval=0.01)
        self.iir = config.getint('mlx90614_IIR', 4, minval=0, maxval=7)
        self.ta_tobj = config.getint('mlx90614_Ta_Tobj',0, minval=0, maxval=3)
        self.dual_ir = config.getboolean('mlx90614_dual_ir', False)
        self.fir = config.getint('mlx90614_FIR', 4, minval=0, maxval=7)
        self.sensor_test = config.getboolean('mlx90614_sensor_test', False)
        self.temp = self.min_temp = self.max_temp = self.humidity = 0.
        self.sample_timer = self.reactor.register_timer(self._sample_mlx90614)
        self.printer.add_object("mlx90614 " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.is_calibrated  = False
        self.init_sent = False

    def handle_connect(self):
        self._init_mlx90614()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _reset_device(self):
        pass

    def _calc_PEC(self, bytes):
        crc = 0
        for byte in bytes:
            for i in range(8, 0, -1):
                carry = ((crc ^ byte) & 0x80)
                crc <<= 1
                if carry:
                    crc ^= 0x7
                byte <<= 1
        return crc & 0xFF

    def _init_mlx90614(self):
        read = self.i2c.i2c_read([0x25], 3)["response"]
        read_data = read[0] | (read[1] << 8)
        iir_modes = [0b011,0b010,0b001,0b000,0b111,0b110,0b101,0b100]
        config_data = (iir_modes[self.iir] | (self.ta_tobj << 4) | (self.dual_ir << 6) | (self.fir << 8) | (self.sensor_test << 15))
        new_data = 0x25 | ((config_data | (read_data & 0b01111000_10001000)) << 8)
        new_bytes = new_data.to_bytes(3,"little")
        crc = self._calc_PEC(new_bytes)
        self.i2c.i2c_write(new_bytes+bytes([crc]))

    def _sample_mlx90614(self, eventtime):
        n = 0
        while True:
            try:
                if n > 20:
                    self.printer.invoke_shutdown("MLX90614 is unresponsive")
                read = self.i2c.i2c_read([0x07], 3)
                data = read['response']
                temp = data[0] | (data[1] << 8)
                temp *= .02
                temp -= 273.15
                self.temp = temp
                break
            except Exception as e:
                n += 1
                logging.error(f"Failed to sample mlx90614 ({e})")


        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "MLX90614 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        self._callback(print_time, self.temp)
        return measured_time + self.report_time

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
        }


def load_config(config):
    # Register sensor
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory("MLX90614", MLX90614)
