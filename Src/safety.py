class SafetySystem:
    def __init__(self, max_angle=45.0):
        self.max_angle = max_angle
        self.armed = False

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False

    def check(self, roll, pitch):
        if abs(roll) > self.max_angle or abs(pitch) > self.max_angle:
            self.armed = False
            return False
        return self.armed

    def throttle_scale(self):
        return 1.0 if self.armed else 0.0