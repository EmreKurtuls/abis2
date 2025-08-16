import time


class PID:

    def __init__(self, Kp, Ki, Kd, setpoint=0, windup_limit=None, ramp_rate=None, filter_coefficient=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.windup_limit = windup_limit
        self.ramp_rate = ramp_rate
        self.filter_coefficient = filter_coefficient

        self.last_error = 0
        self.integral = 0
        self.last_output = 0
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return self.last_output

        error = self.setpoint - current_value

        if self.filter_coefficient is not None:
            self.integral = self.filter_coefficient * self.integral + (1 - self.filter_coefficient) * error * dt
        else:
            self.integral += error * dt

        if self.windup_limit is not None:
            self.integral = max(-self.windup_limit, min(self.integral, self.windup_limit))

        derivative = (error - self.last_error) / dt

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        if self.ramp_rate is not None:
            max_change = self.ramp_rate * dt
            change = output - self.last_output
            if change > max_change:
                output = self.last_output + max_change
            elif change < -max_change:
                output = self.last_output - max_change

        self.last_error = error
        self.last_time = current_time
        self.last_output = output

        return output

    def set_target(self, new_setpoint):
        self.setpoint = new_setpoint
        self.integral = 0
        self.last_error = 0


class StabilizationController:

    def __init__(self, config):
        self.pitch_pid = PID(config.PITCH_KP, config.PITCH_KI, config.PITCH_KD,
                             windup_limit=config.PITCH_WINDUP, ramp_rate=config.PITCH_RAMP_RATE,
                             filter_coefficient=config.PITCH_FILTER_COEFF)
        self.roll_pid = PID(config.ROLL_KP, config.ROLL_KI, config.ROLL_KD,
                            windup_limit=config.ROLL_WINDUP, ramp_rate=config.ROLL_RAMP_RATE,
                            filter_coefficient=config.ROLL_FILTER_COEFF)
        self.yaw_pid = PID(config.YAW_KP, config.YAW_KI, config.YAW_KD,
                           windup_limit=config.YAW_WINDUP, ramp_rate=config.YAW_RAMP_RATE,
                           filter_coefficient=config.YAW_FILTER_COEFF)
        self.depth_pid = PID(config.DEPTH_KP, config.DEPTH_KI, config.DEPTH_KD,
                             windup_limit=config.DEPTH_WINDUP, ramp_rate=config.DEPTH_RAMP_RATE,
                             filter_coefficient=config.DEPTH_FILTER_COEFF)

    def calculate_efforts(self, current_state, desired_state):
        self.pitch_pid.set_target(desired_state['pitch'])
        self.roll_pid.set_target(desired_state['roll'])
        self.yaw_pid.set_target(desired_state['yaw'])
        self.depth_pid.set_target(desired_state['depth'])

        efforts = {'pitch': self.pitch_pid.update(current_state['pitch']),
                   'roll': self.roll_pid.update(current_state['roll']),
                   'yaw': self.yaw_pid.update(current_state['yaw']),
                   'depth': self.depth_pid.update(current_state['depth']), 'surge': desired_state.get('surge', 0.0),
                   'sway': desired_state.get('sway', 0.0)}

        return efforts