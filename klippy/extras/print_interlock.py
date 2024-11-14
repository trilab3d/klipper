
class Interlock:
    def __init__(self, lock_reason,callback=None):
        self.locked = False
        self.lock_reason = lock_reason
        self.callback = callback

    def set_lock(self, locked):
        self.locked = locked

class PrintInterlock:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.gcode = self.printer.lookup_object('gcode')
        self.respond = None
        self.locks = []
        self.debug_lock = Interlock("Debug lock is Locked")
        self.locks.append(self.debug_lock)
        self.gcode.register_command('SET_DEBUG_LOCK', self.cmd_SET_DEBUG_LOCK, True,
                               desc="SET_DEBUG_LOCK LOCKED=0|1")
        self.gcode.register_command('QUERY_LOCK', self.cmd_QUERY_LOCK, True,
                                    desc="QUERY_LOCK")
    def cmd_SET_DEBUG_LOCK(self, gcmd):
        if gcmd.get_int("LOCKED", 0) != 0:
            self.debug_lock.set_lock(True)
        else:
            self.debug_lock.set_lock(False)

    def cmd_QUERY_LOCK(self, gcmd):
        locked = False
        resp = "Print interlocks status\n"
        for interlock in self.locks:
            resp += f"{interlock.lock_reason}: {interlock.locked}\n"
            if interlock.locked:
                locked = True
        resp += f"SUM STATE: {'LOCKED' if locked else 'UNLOCKED'}"
        gcmd.respond_info(resp)

    def handle_connect(self):
        self.respond = self.printer.lookup_object('respond', None)

    def create_interlock(self, lock_reason, callback=None):
        interlock = Interlock(lock_reason,callback)
        self.locks.append(interlock)
        return interlock

    def check_locked(self, print_reason=False, do_callback=True):
        for interlock in self.locks:
            if interlock.locked:
                if print_reason:
                    self.gcode.respond_raw(f"!! {interlock.lock_reason}")
                if do_callback and interlock.callback:
                    interlock.callback()
                return True
        return False

    def get_status(self, eventtime):
        lock_statuses = {}
        for lock in self.locks:
            lock_statuses[lock.lock_reason] = lock.locked
        return {
            'lock_statuses': lock_statuses,
        }

def load_config(config):
    return PrintInterlock(config)
