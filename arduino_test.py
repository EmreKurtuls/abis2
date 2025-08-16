import serial
import time


ARDUINO_PORT = '/dev/cu.usbserial-11110'
BAUDRATE = 115200

try:
    ser = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    print("Bağlantı başarılı. Tam veri paketi ayrıştırma başlıyor...")
except serial.SerialException as e:
    print(f"HATA: Porta bağlanılamadı! - {e}")
    exit()

try:
    while True:
        if ser.in_waiting > 0:
            line_str = ser.readline().decode('utf-8').strip()


            if line_str.startswith('S:') and ',C:' in line_str:
                try:
                    main_data_part, cal_data_part = line_str.split(',C:')

                    sensor_values_str = main_data_part[2:]
                    parts = sensor_values_str.split(',')

                    cal_values_str = cal_data_part

                    if len(parts) == 4 and len(cal_values_str) == 4:
                        yaw = float(parts[0])
                        roll = float(parts[1])
                        pitch = float(parts[2])
                        depth = float(parts[3])

                        cal_sys = int(cal_values_str[0])
                        cal_gyro = int(cal_values_str[1])
                        cal_accel = int(cal_values_str[2])
                        cal_mag = int(cal_values_str[3])

                        print(
                            f"Başarılı: Yaw={yaw:.2f}, Roll={roll:.2f}, Pitch={pitch:.2f}, Depth={depth:.2f} --- CAL: [S:{cal_sys}, G:{cal_gyro}, A:{cal_accel}, M:{cal_mag}]")

                except (ValueError, IndexError) as e:
                    print(f"Ayrıştırma Hatası: Veri bozuk olabilir. Gelen Veri: '{line_str}', Hata: {e}")


except KeyboardInterrupt:
    print("\nProgram durduruluyor...")
finally:
    ser.close()
    print("Seri port kapatıldı.")