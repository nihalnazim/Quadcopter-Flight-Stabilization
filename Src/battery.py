import numpy as np


class Battery:
    def __init__(self, initial_voltage=12.6, decay_rate=0.01):
        self.voltage = initial_voltage
        self.decay_rate = decay_rate

    def update(self, dt):
        # simple linear decay model
        self.voltage -= self.decay_rate * dt
        self.voltage = max(self.voltage, 9.0)  # minimum safe voltage

    def read_voltage(self):
        return self.voltage