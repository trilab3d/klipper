import logging

import configparser


class ConfigConstant:
    def __init__(self, config):
        self.value = config.get("value")

    def get_status(self, eventtime):
        return {
            "value": self.value
        }



def load_config_prefix(config):
    return ConfigConstant(config)