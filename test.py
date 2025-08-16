import time
import config
from hardware_interface import ArduinoLink


def channel_test():
    print("--- Motor Kanal Test Programı ---")
    print("UYARI: BU TEST SIRASINDA TÜM PERVANELER SÖKÜLÜ OLMALIDIR!")

    try:
        arduino = ArduinoLink(port=config.ARDUINO_PORT)
        # Arka plan okuma thread'ini başlatıyoruz ama verisini kullanmayacağız
        arduino.start_reading()
    except Exception as e:
        print(f"Arduino'ya bağlanılamadı: {e}")
        return

    try:
        # --- ESC ARMING SEKASI ---
        print("\nESC'lerin 'arm' olması için 3 saniye boyunca nötr (1500) sinyal gönderiliyor...")
        arming_start_time = time.time()
        while time.time() - arming_start_time < 3.0:
            arduino.send_motor_commands([1500] * 8)
            time.sleep(0.05)
        print("Arming tamamlandı. Teste hazır.")
        print("---------------------------------------------------------")
        print("Komut formatı: <motor_numarası>:<pwm_değeri>")
        print("Örnekler: 1:1550, 3:1450, 8:1500")
        print("Tüm motorları durdurmak için 'stop' yazın.")
        print("Programdan çıkmak için 'exit' yazın.")
        print("---------------------------------------------------------")

        while True:
            command = input("Komut girin: ")
            command = command.strip().lower()

            if command == 'exit':
                break

            if command == 'stop':
                print("Tüm motorlar durduruluyor...")
                arduino.send_motor_commands([1500] * 8)
                continue

            try:
                motor_str, pwm_str = command.split(':')
                motor_num = int(motor_str)
                pwm_val = int(pwm_str)

                # Girdileri kontrol et
                if not (1 <= motor_num <= 8):
                    print("HATA: Motor numarası 1 ile 8 arasında olmalıdır.")
                    continue
                if not (1100 <= pwm_val <= 1900):
                    print("HATA: PWM değeri 1100 ile 1900 arasında olmalıdır.")
                    continue

                # Tüm motorları durdurup sadece istenen motoru çalıştıracak listeyi hazırla
                pwm_list = [1500] * 8
                pwm_list[motor_num - 1] = pwm_val  # Liste 0'dan başladığı için -1

                print(f"{motor_num}. motora {pwm_val}us komutu gönderiliyor...")
                arduino.send_motor_commands(pwm_list)

            except ValueError:
                print("Hatalı komut formatı! Örnek: 5:1600")

    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu.")
    finally:
        print("\nProgram sonlanıyor. Motorlar durduruluyor...")
        if 'arduino' in locals() and arduino:
            arduino.send_motor_commands([1500] * 8)
            arduino.close()
        print("Program sonlandı.")


if __name__ == "__main__":
    channel_test()