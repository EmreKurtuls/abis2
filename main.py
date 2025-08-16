import time
import config
from hardware_interface import ArduinoLink
from controllers import StabilizationController

# == ÇALIŞMA MODU AYARI
# Normal stabilizasyon için: TEST_MODE = None
# Sadece ileri gitme testi için: TEST_MODE = "FORWARD_ONLY"
TEST_MODE = None



def map_value(x, in_min, in_max, out_min, out_max):
    """ Bir değeri bir aralıktan diğerine oranlar. PWM sinyali üretmek için. """
    # Değerin giriş aralığı içinde olduğundan emin ol (saturasyon)
    x = max(in_min, min(x, in_max))
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def mix_motors(efforts):
    """
    PID'lerden gelen eforları alır ve şemadaki doğru motor rollerine
    göre her motor için ayrı güç hesaplar.
    """
    pitch_effort = efforts.get('pitch', 0.0)
    roll_effort = efforts.get('roll', 0.0)
    yaw_effort = efforts.get('yaw', 0.0)
    depth_effort = efforts.get('depth', 0.0)  # Dikey hareket (Heave)
    surge_effort = efforts.get('surge', 0.0)  # İleri/Geri
    sway_effort = efforts.get('sway', 0.0)  # Sağ/Sol

    motor_powers = {}

    # --- DİKEY MOTORLAR (3, 4, 5, 6) ---
    motor_powers['motor_3'] = depth_effort - pitch_effort + roll_effort
    motor_powers['motor_4'] = -depth_effort + pitch_effort + roll_effort
    motor_powers['motor_5'] = depth_effort + pitch_effort + roll_effort
    motor_powers['motor_6'] = depth_effort + pitch_effort - roll_effort

    # --- YATAY/VEKTÖREL MOTORLAR (1, 2, 7, 8) ---
    motor_powers['motor_1'] = surge_effort - sway_effort + yaw_effort
    motor_powers['motor_2'] = surge_effort + sway_effort - yaw_effort
    motor_powers['motor_7'] = -surge_effort - sway_effort - yaw_effort
    motor_powers['motor_8'] = -surge_effort + sway_effort + yaw_effort

    return motor_powers


# --- ANA PROGRAM ---

def main_loop():
    print("Araç yazılımı başlatılıyor...")

    try:
        arduino = ArduinoLink(port=config.ARDUINO_PORT)
        arduino.start_reading()  # Arka plan okuma thread'ini başlat
    except Exception as e:
        print(f"Arduino'ya bağlanılamadığı için program sonlandırılıyor. Hata: {e}")
        return

    stabilizer = StabilizationController(config)
    initial_yaw = None

    # --- KALİBRASYON BEKLEME DÖNGÜSÜ ---
    print("Sensör kalibrasyonu bekleniyor... (TÜM değerler 3 olmalı)")
    while True:
        current_state = arduino.get_latest_state()

        if current_state:
            cal_sys = current_state.get('cal_sys', 0)
            cal_gyro = current_state.get('cal_gyro', 0)
            cal_accel = current_state.get('cal_accel', 0)
            cal_mag = current_state.get('cal_mag', 0)

            print(f"\rKalibrasyon Durumu: Sys:{cal_sys} G:{cal_gyro} A:{cal_accel} M:{cal_mag}", end="")

            if cal_sys >= 3 and cal_gyro >= 3 and cal_accel >= 3 and cal_mag >= 3:
                print("\n\nTüm sistemler tam kalibreli! Otonom sistemler başlatılıyor.")
                time.sleep(1)
                initial_yaw = current_state['yaw']
                break

        time.sleep(0.5)
    # --- DÖNGÜ BİTTİ ---

    # Ana kontrol döngüsü
    try:
        # === TEST MODU KONTROLÜ ===
        if TEST_MODE == "FORWARD_ONLY":
            print("\n!!! TEST MODU AKTİF: SADECE İLERİ GİTME !!!")
            TEST_DURATION = 80.0
            FORWARD_POWER = 1
            start_time = time.time()

            while time.time() - start_time < TEST_DURATION:
                efforts = {'sway': FORWARD_POWER}

                motor_powers_normalized = mix_motors(efforts)
                motor_pwms_dict = {}
                for motor, power in motor_powers_normalized.items():
                    power = max(-1.0, min(1.0, power))
                    motor_pwms_dict[motor] = int(map_value(power, -1.0, 1.0, 1100, 1900))

                pwm_list_to_send = [motor_pwms_dict.get(f'motor_{i}', 1500) for i in range(1, 9)]

                arduino.send_motor_commands(pwm_list_to_send)
                print(f"\rİleri gitme komutu gönderiliyor... Güç: %{FORWARD_POWER * 100}", end="")
                time.sleep(0.05)

        else:  # TEST_MODE = None ise normal stabilizasyon çalışır
            print("\n--- Normal Stabilizasyon Modu ---")
            while True:
                current_state = arduino.get_latest_state()

                if current_state:
                    desired_state = {
                        'pitch': 0.0, 'roll': 0.0, 'yaw': initial_yaw,
                        'depth': 0.0, 'surge': 0.0, 'sway': 0.0
                    }

                    pid_efforts = stabilizer.calculate_efforts(current_state, desired_state)
                    motor_powers_normalized = mix_motors(pid_efforts)
                    motor_pwms_dict = {}
                    for motor, power in motor_powers_normalized.items():
                        power = max(-1.0, min(1.0, power))
                        motor_pwms_dict[motor] = int(map_value(power, -1.0, 1.0, 1100, 1900))

                    pwm_list_to_send = [motor_pwms_dict.get(f'motor_{i}', 1500) for i in range(1, 9)]

                    arduino.send_motor_commands(pwm_list_to_send)

                    print(
                        f"\rYaw: {current_state['yaw']:.1f}/{desired_state['yaw']:.1f} | Roll: {current_state['roll']:.1f}/{desired_state['roll']:.1f} | Pitch: {current_state['pitch']:.1f}/{desired_state['pitch']:.1f}",
                        end="")

                time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nProgram kullanıcı tarafından durduruldu.")
    finally:
        print("\nProgram sonlanıyor. Motorlar durduruluyor...")
        if 'arduino' in locals() and arduino:
            arduino.send_motor_commands([1500] * 8)
            arduino.close()
        print("Program sonlandı.")


# Ana programı çalıştır
if __name__ == "__main__":
    main_loop()