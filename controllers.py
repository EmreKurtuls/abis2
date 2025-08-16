# controllers.py
import time


class PID:
    """
    Standart, sağlam ve test edilmiş PID sınıfı.
    Zamanlama sorunlarına karşı daha dirençlidir.
    """

    def __init__(self, Kp=0, Ki=0, Kd=0, setpoint=0, windup_limit=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.windup_limit = windup_limit

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def update(self, current_value, dt=None):
        # Eğer dışarıdan bir zaman farkı (dt) verilmezse, kendisi hesaplar.
        # Bu, test ve simülasyon için esneklik sağlar.
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

        # Çok küçük zaman adımlarını atla
        if dt <= 0:
            return 0.0

        error = self.setpoint - current_value

        # P (Oransal) Terimi
        p_term = self.Kp * error

        # I (İntegral) Terimi
        self.integral += error * dt
        if self.windup_limit is not None:
            self.integral = max(-self.windup_limit, min(self.integral, self.windup_limit))
        i_term = self.Ki * self.integral

        # D (Türev) Terimi
        derivative = (error - self.last_error) / dt
        d_term = self.Kd * derivative

        # Nihai PID Çıktısı
        output = p_term + i_term + d_term

        self.last_error = error

        return output

    def set_target(self, new_setpoint):
        """PID için yeni bir hedef belirler ve hafızayı sıfırlar."""
        self.setpoint = new_setpoint
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()


class StabilizationController:
    """Tüm eksenlerin PID kontrolcülerini yönetir."""

    def __init__(self, config):
        self.pitch_pid = PID(config.PITCH_KP, config.PITCH_KI, config.PITCH_KD, windup_limit=config.PITCH_WINDUP)
        self.roll_pid = PID(config.ROLL_KP, config.ROLL_KI, config.ROLL_KD, windup_limit=config.ROLL_WINDUP)
        self.yaw_pid = PID(config.YAW_KP, config.YAW_KI, config.YAW_KD, windup_limit=config.YAW_WINDUP)
        self.pressure_pid = PID(config.PRESSURE_KP, config.PRESSURE_KI, config.PRESSURE_KD,
                                windup_limit=config.PRESSURE_WINDUP)

    def calculate_efforts(self, current_state, desired_state):
        self.pitch_pid.set_target(desired_state['pitch'])
        self.roll_pid.set_target(desired_state['roll'])
        self.yaw_pid.set_target(desired_state['yaw'])
        self.pressure_pid.set_target(desired_state['pressure'])

        efforts = {
            'pitch': self.pitch_pid.update(current_state['pitch']),
            'roll': self.roll_pid.update(current_state['roll']),
            'yaw': self.yaw_pid.update(current_state['yaw']),
            'pressure_effort': self.pressure_pid.update(current_state['pressure']),
            'surge': desired_state.get('surge', 0.0),
            'sway': desired_state.get('sway', 0.0)
        }
        return efforts