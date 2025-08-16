import time
import config
from hardware_interface import ArduinoLink
from controllers import StabilizationController


# --- YARDIMCI FONKSİYONLAR ---
def map_value(x, in_min, in_max, out_min, out_max):
    """ Bir değeri bir aralıktan diğerine oranlar. """
    x = max(in_min, min(x, in_max))
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def mix_motors(efforts):
    """
    Kullanıcı tarafından doğrulanmış ve ayarlanmış mikser mantığı.
    """
    pressure_effort = efforts.get('pressure_effort', 0.0)
    pitch_effort = efforts.get('pitch', 0.0)
    roll_effort = efforts.get('roll', 0.0)
    yaw_effort = efforts.get('yaw', 0.0)
    surge_effort = efforts.get('surge', 0.0)
    sway_effort = efforts.get('sway', 0.0)

    motor_powers = {}

    # Dikey Motorlar (3, 4, 5, 6)
    motor_powers['motor_3'] = pressure_effort - pitch_effort + roll_effort
    motor_powers['motor_4'] = -pressure_effort + pitch_effort + roll_effort
    motor_powers['motor_5'] = pressure_effort + pitch_effort + roll_effort
    motor_powers['motor_6'] = pressure_effort + pitch_effort - roll_effort

    # Yatay Motorlar (1, 2, 7, 8)
    motor_powers['motor_1'] = surge_effort - sway_effort - yaw_effort
    motor_powers['motor_2'] = surge_effort + sway_effort + yaw_effort
    motor_powers['motor_7'] = -surge_effort - sway_effort - yaw_effort
    motor_powers['motor_8'] = surge_effort - sway_effort + yaw_effort

    return motor_powers


# --- ANA PROGRAM ---
def main_loop():
    print("Araç yazılımı başlatılıyor...")

    try:
        arduino = ArduinoLink(port=config.ARDUINO_PORT)
        arduino.start_reading()
    except Exception as e:
        print(f"Arduino'ya bağlanılamadığı için program sonlandırılıyor. Hata: {e}")
        return

    stabilizer = StabilizationController(config)
    initial_roll_offset, initial_pitch_offset, initial_pressure, initial_yaw = 0.0, 0.0, 1013.25, 0.0

    # --- KALİBRASYON BEKLEME DÖNGÜSÜ ---
    print("Sensör kalibrasyonu bekleniyor... (TÜM değerler 3 olmalı)")
    while True:
        current_state = arduino.get_latest_state()
        if current_state and all(
                current_state.get(key, 0) >= 3 for key in ['cal_sys', 'cal_gyro', 'cal_accel', 'cal_mag']):
            print("\n\nTüm sistemler tam kalibreli!")
            time.sleep(1)
            break
        elif current_state:
            cal_status = current_state
            print(
                f"\rKalibrasyon Durumu: Sys:{cal_status['cal_sys']} G:{cal_status['cal_gyro']} A:{cal_status['cal_accel']} M:{cal_status['cal_mag']}",
                end="")
        time.sleep(0.5)

    # --- SIFIRLAMA (TARİNG) VE BAŞLANGIÇ HEDEFLERİ ---
    print("\n--- Sıfır Noktası Ayarlanıyor ---")
    print("Lütfen aracın TAMAMEN DÜZ bir zeminde hareketsiz olduğundan emin olun.")
    print("Ofset değerleri 3 saniye içinde hesaplanacak...")
    time.sleep(2)
    roll_samples, pitch_samples, pressure_samples = [], [], []
    start_time = time.time()
    while time.time() - start_time < 3:
        state = arduino.get_latest_state()
        if state:
            roll_samples.append(state['roll'])
            pitch_samples.append(state['pitch'])
            pressure_samples.append(state['pressure'])
        time.sleep(0.05)

    initial_roll_offset = sum(roll_samples) / len(roll_samples) if roll_samples else 0.0
    initial_pitch_offset = sum(pitch_samples) / len(pitch_samples) if pitch_samples else 0.0
    initial_pressure = sum(pressure_samples) / len(pressure_samples) if pressure_samples else 1013.25
    initial_yaw = arduino.get_latest_state()['yaw'] if arduino.get_latest_state() else 0.0

    print(f"Sıfırlama tamamlandı. Roll Ofseti: {initial_roll_offset:.2f}, Pitch Ofseti: {initial_pitch_offset:.2f}")
    print(f"Başlangıç Hedefleri -> Yaw: {initial_yaw:.1f}, Basınç: {initial_pressure:.2f} mbar")
    print("Ana kontrol döngüsü ve HATA AYIKLAMA başlıyor...")

    # Ana kontrol döngüsü
    try:
        while True:
            current_state = arduino.get_latest_state()
            if current_state:
                desired_state = {
                    'pitch': initial_pitch_offset,
                    'roll': initial_roll_offset,
                    'yaw': initial_yaw,
                    'pressure': initial_pressure,
                    'surge': 0.0,
                    'sway': 0.0
                }

                print("\n" + "=" * 50)

                # 1. Adım: Sensör verisi doğru okunuyor mu?
                print(
                    f"ADIM 1: SENSÖR OKUMASI -> Pitch: {current_state['pitch']:.2f}, Roll: {current_state['roll']:.2f}, Pressure: {current_state['pressure']:.2f}")

                pid_efforts = stabilizer.calculate_efforts(current_state, desired_state)

                # 2. Adım: PID'ler bu sensör verisine göre bir çıktı üretiyor mu?
                print(
                    f"ADIM 2: PID EFORLARI -> Pitch: {pid_efforts['pitch']:.2f}, Roll: {pid_efforts['roll']:.2f}, Pressure: {pid_efforts.get('pressure_effort', 0.0):.2f}")

                motor_powers_normalized = mix_motors(pid_efforts)

                # 3. Adım: Motor mikseri bu eforlara göre bir çıktı üretiyor mu?
                print(
                    f"ADIM 3: MİKSER ÇIKTISI -> Motor 3: {motor_powers_normalized.get('motor_3', 0.0):.2f}, Motor 4: {motor_powers_normalized.get('motor_4', 0.0):.2f}")

                motor_pwms_dict = {}
                for motor, power in motor_powers_normalized.items():
                    power = max(-1.0, min(1.0, power))
                    motor_pwms_dict[motor] = int(map_value(power, -1.0, 1.0, 1100, 1900))

                pwm_list_to_send = [motor_pwms_dict.get(f'motor_{i}', 1500) for i in range(1, 9)]

                # 4. Adım: Arduino'ya gönderilen nihai PWM komutu nedir?
                print(f"ADIM 4: NİHAİ PWM -> {pwm_list_to_send}")

                arduino.send_motor_commands(pwm_list_to_send)

            # Çıktıyı rahat okuyabilmek için döngüyü yavaşlatalım
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nProgram kullanıcı tarafından durduruldu.")
    finally:
        print("\nProgram sonlanıyor. Motorlar durduruluyor...")
        if 'arduino' in locals() and arduino:
            arduino.send_motor_commands([1500] * 8)
            arduino.close()
        print("Program sonlandı.")


if __name__ == "__main__":
    main_loop()