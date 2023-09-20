from . import filament_switch_sensor

class ControlledFilamentSensor(filament_switch_sensor.RunoutHelper):
    def get_runout_pause(self):
        return self.runout_pause

    def set_runout_pause(self, newValue):
        self.runout_pause = newValue