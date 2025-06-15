from pymavlink import mavutil
import time
import threading

connection_string = '/dev/ttyACM0' 
RC_OVERRIDE_RATE = 10

rc_override_active = False
sub = None
channels = [0] * 8
MOTOR_TEST_PWM = 1900 
MOTOR_RUN_DURATION = 5  
MOTOR_WAIT_DURATION = 5 


def connect_to_pixhawk():
    global sub
    print(f"Pixhawk'a bağlanılıyor: {connection_string}")
    sub = mavutil.mavlink_connection(connection_string, wait_ready=True)
    sub.wait_heartbeat()
    print("Heartbeat alındı (sistem %u, bileşen %u)" % (sub.target_system, sub.target_component))
    return sub

def rc_override_worker():
    global rc_override_active, channels
    while rc_override_active:
        sub.mav.rc_channels_override_send(
            sub.target_system,
            sub.target_component,
            *channels
        )
        time.sleep(1 / RC_OVERRIDE_RATE)
    print("RC Override thread'i durduruldu.")

def arm_vehicle():
    print("Arm komutu deneniyor...")
    sub.mav.command_long_send(
        sub.target_system, sub.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    ack = sub.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Araç ARM edildi.")
        sub.motors_armed_wait()
        print("Motorlar aktif.")
        return True
    else:
        print(f"ARM BAŞARISIZ! Alınan ACK: {ack}")
        return False

def disarm_vehicle():
    print("Disarm komutu gönderiliyor...")
    sub.mav.command_long_send(
        sub.target_system, sub.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    ack = sub.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Araç DISARM edildi.")
    else:
        print("DISARM BAŞARISIZ.")

def set_channel_pwm(channel_index, pwm):
    global channels
    channels[channel_index] = int(min(max(pwm, 1100), 1900))


def reset_all_motors_to_idle():
    print("Tüm motorlar durduruluyor (rölanti/nötr pozisyonuna alınıyor)...")
    global channels
    for i in range(6): # Motorların 1-6 arası kanallarda olduğunu varsayıyoruz
        channels[i] = 1500


def relinquish_control():
    global channels
    print("Tüm kanallarda kontrol Pixhawk'a devrediliyor...")
    channels = [0] * 8
    # Değişikliğin gittiğinden emin olalım
    for _ in range(int(RC_OVERRIDE_RATE)):
        sub.mav.rc_channels_override_send(
            sub.target_system, sub.target_component, *channels
        )
        time.sleep(1 / RC_OVERRIDE_RATE)

rc_override_thread = None
try:
    time.sleep(1)
    sub = connect_to_pixhawk()

    channels = [1500] * 8
    rc_override_active = True
    rc_override_thread = threading.Thread(target=rc_override_worker, daemon=True)
    rc_override_thread.start()
    print("RC Override arka plan thread'i başlatıldı.")
    time.sleep(1) 

    if arm_vehicle():
        time.sleep(1)
        
        set_channel_pwm(5, 1900)
        time.sleep(90)
        reset_all_motors_to_idle()

        #for i in range(6):
        #    print(f"\n====== TEST ADIM {i+1}: KANAL {i+1} ======")

        #    print(f"Kanal {i+1}, {MOTOR_RUN_DURATION} saniye boyunca {MOTOR_TEST_PWM} PWM ile çalıştırılıyor...")
        #    set_channel_pwm(i, 1900)
        #    time.sleep(15)

        #    reset_all_motors_to_idle()
            
        #    print(f"Tüm motorlar durduruldu. {MOTOR_WAIT_DURATION} saniye bekleniyor...")
        #    time.sleep(MOTOR_WAIT_DURATION)
        
        print("\n====== TÜM MOTOR TESTLERİ TAMAMLANDI ======")

except KeyboardInterrupt:
    print("\nKullanıcı tarafından durduruldu.")
except Exception as e:
    print(f"\nBir hata oluştu: {e}")
finally:
    if rc_override_thread and rc_override_thread.is_alive():
        print("\nTemizlik yapılıyor...")
        rc_override_active = False
        rc_override_thread.join(timeout=2) 

    if sub:
        reset_all_motors_to_idle()
        time.sleep(0.5)
        relinquish_control()
        time.sleep(0.5)
        disarm_vehicle()
        sub.close()
        print("Bağlantı kapatıldı.")
