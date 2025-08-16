import serial
import time
import threading


class ArduinoLink:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.latest_state = None
        self.is_running = False
        self._lock = threading.Lock()

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)
        except serial.SerialException as e:
            raise e

    def _read_loop(self):
        while self.is_running:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line_str = self.ser.readline().decode('utf-8').strip()
                    if line_str.startswith('S:') and ',C:' in line_str:
                        main_data_part, cal_data_part = line_str.split(',C:')
                        sensor_values_str = main_data_part[2:]
                        parts = sensor_values_str.split(',')
                        cal_values_str = cal_data_part

                        if len(parts) == 4 and len(cal_values_str) == 4:
                            data = {
                                'yaw': float(parts[0]),
                                'roll': float(parts[1]),
                                'pitch': float(parts[2]),
                                'pressure': float(parts[3]),  # 'pressure' anahtarı burada oluşturuluyor
                                'cal_sys': int(cal_values_str[0]),
                                'cal_gyro': int(cal_values_str[1]),
                                'cal_accel': int(cal_values_str[2]),
                                'cal_mag': int(cal_values_str[3])
                            }
                            with self._lock:
                                self.latest_state = data
                except Exception:
                    pass
            time.sleep(0.001)

    def start_reading(self):
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("Arduino okuma thread'i başlatıldı.")

    def get_latest_state(self):
        with self._lock:
            return self.latest_state

    def send_motor_commands(self, motor_pwms):
        if self.ser and self.ser.is_open:
            try:
                command_string = "M:" + ",".join(map(str, motor_pwms)) + "\n"
                self.ser.write(command_string.encode('utf-8'))
            except Exception as e:
                print(f"Arduino'ya komut gönderirken hata: {e}")

    def close(self):
        self.is_running = False
        if hasattr(self, 'thread') and self.thread:
            self.thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Seri port kapatıldı.")