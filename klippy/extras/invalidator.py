import logging

import configparser

error = configparser.Error

class ConfigConstant:
    def __init__(self, config):
        raise error(config.get("message"))



def load_config(config):
    return ConfigConstant(config)