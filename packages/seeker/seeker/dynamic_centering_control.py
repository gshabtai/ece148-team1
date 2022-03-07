#Caculate needed throttle and steering to center robocar on ball

class DynamicCenteringControl():
    def __init__(self, param):
        # initializing PID control
        self.Ts = float(1/20)
        self.ek = 0 # current error
        self.ek_1 = 0 # previous error
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.integral_max = 1E-8

        self.Kp = param.Kp
        self.Ki = param.Ki
        self.Kd = param.Kd
        self.error_threshold = param.error_threshold
        self.zero_throttle = param.zero_throttle
        self.max_throttle = param.max_throttle
        self.min_throttle = param.min_throttle
        self.max_right_steering = param.max_right_steering
        self.max_left_steering = param.max_left_steering

    def update_ek_1(self, ek_1):
        self.ek_1 = ek_1

    def cal_throttle(self, ek):
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = (self.min_throttle - self.max_throttle) * abs(ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

    def cal_steering(self, ek):
        self.proportional_error = self.Kp * ek
        self.derivative_error = self.Kd * (ek - ek_1) / self.Ts
        self.integral_error += self.Ki * ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 