# Save arbitrary variables so that values can be kept across restarts.
#
# Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast, configparser

class SaveVariables:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('filename'))
        self.allVariables = {}
        try:
            if not os.path.exists(self.filename):
                open(self.filename, "w").close()
            self.loadVariables()
        except self.printer.command_error as e:
            raise config.error(str(e))
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SAVE_VARIABLE', self.cmd_SAVE_VARIABLE,
                               desc=self.cmd_SAVE_VARIABLE_help)
    def loadVariables(self):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            if varfile.has_section('Variables'):
                for name, val in varfile.items('Variables'):
                    allvars[name] = ast.literal_eval(val)
        except Exception as e:
            msg = "Unable to parse existing variable file"
            logging.exception(msg)
            import klippy
            klippy.log_exception(type(e), e, e.__traceback__)
            raise self.printer.command_error(msg)
        self.allVariables = allvars
        # fix for older HT-A nozzle notation
        if "nozzle" in allvars and allvars["nozzle"].endswith("HT-A"):
            parts = allvars["nozzle"].split(" ")
            self.save_variable("nozzle", f"{parts[0]} HT")
    cmd_SAVE_VARIABLE_help = "Save arbitrary variables to disk"
    def cmd_SAVE_VARIABLE(self, gcmd):
        varname = gcmd.get('VARIABLE')
        value = gcmd.get('VALUE')
        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            raise gcmd.error("Unable to parse '%s' as a literal" % (value,))
        try:
           self.save_variable(varname, value)
        except Exception as e:
            msg = "Unable to save variable"
            logging.exception(msg)
            import klippy
            klippy.log_exception(type(e), e, e.__traceback__)
            self.loadVariables()
            raise gcmd.error(msg)
    def save_variable(self, varname, value):
        newvars = dict(self.allVariables)
        newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section('Variables')
        for name, val in sorted(newvars.items()):
            varfile.set('Variables', name, repr(val))
        f = open(self.filename, "w")
        varfile.write(f)
        f.close()
        self.loadVariables()
    def get_variable(self, varname):
        if varname in self.allVariables:
            return self.allVariables[varname]
        return None
    def get_status(self, eventtime):
        return {'variables': self.allVariables}

def load_config(config):
    return SaveVariables(config)
