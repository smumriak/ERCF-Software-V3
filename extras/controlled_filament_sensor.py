from . import filament_switch_sensor

class ControlledFilamentSensor(filament_switch_sensor.SwitchSensor):
    def __init__(self, config):
        super(ControlledFilamentSensor, self).__init__(config)

    def get_runout_pause(self):
        return self.runout_helper.runout_pause

    def set_runout_pause(self, newValue):
        self.runout_helper.runout_pause = newValue

def load_config_prefix(config):
    return ControlledFilamentSensor(config)
