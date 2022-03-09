class Parameters():

    def __init__(self):
        self.Kp = 1
        self.Ki = 0
        self.Kd = 0
        self.error_threshold = 0.15
        self.zero_throttle = 0.0
        self.max_throttle = 0.2
        self.min_throttle = 0.1
        self.max_right_steering = 1.0
        self.max_left_steering = -1.0

    def upd_Kp(self, val): self.Kp = val
    def upd_Ki(self, val): self.Ki = val
    def upd_Kd(self, val): self.Kd = val
    def upd_error_threshold(self, val): self.error_threshold = val
    def upd_zero_throttle(self, val): self.zero_throttle = val
    def upd_max_throttle(self, val): self.max_throttle = val
    def upd_min_throttle(self, val): self.min_throttle = val
    def upd_max_right_steering(self, val): self.max_right_steering = val
    def upd_max_left_steering(self, val): self.max_left_steering = val