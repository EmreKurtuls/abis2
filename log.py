import time
import csv
from datetime import datetime
from pymavlink import mavutil

# --- YAPILANDIRMA BÖLÜMÜ ---

# Pixhawk'a nasıl bağlanacağınızı burada belirtin.
# Örnekler:
# Windows için: 'COM3', 'COM4' vb.
# Linux için: '/dev/ttyACM0', '/dev/ttyUSB0' vb.
# SITL (Simülatör) için: 'udp:127.0.0.1:14550'
CONNECTION_STRING = '/dev/ttyACM0'  # <-- BU SATIRI KENDİ BAĞLANTINIZA GÖRE DEĞİŞTİRİN
BAUDRATE = 115200                   # Genellikle seri bağlantılar için bu değer kullanılır

# Verilerin ne sıklıkla dosyaya yazılacağını belirtin (saniye cinsinden).
LOG_INTERVAL = 1.0  # Her 1 saniyede bir logla

# --- KOD BAŞLANGICI ---

def main():
    """
    Ana fonksiyon, Pixhawk'a bağlanır ve basınç verilerini loglar.
    """
    # Benzersiz bir log dosyası adı oluştur
    log_dosya_adi = f'basin_logu_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

    # Pixhawk'a bağlanmayı dene
    try:
        print(f"Pixhawk'a bağlanılıyor: {CONNECTION_STRING}")
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUDRATE)
        
        # Bağlantının kurulduğunu doğrulamak için 'heartbeat' bekle
        master.wait_heartbeat()
        print("Heartbeat alındı! Sistem ID: %u, Komponent ID: %u" %
              (master.target_system, master.target_component))
    except Exception as e:
        print(f"HATA: Pixhawk'a bağlanılamadı. {e}")
        print("Lütfen CONNECTION_STRING değerinin doğru olduğundan emin olun.")
        return

    # En son okunan basınç değerlerini saklamak için bir sözlük
    en_son_degerler = {
        'press_abs': None,
        'press_abs2': None
    }

    try:
        # CSV dosyasını yazma modunda aç
        with open(log_dosya_adi, 'w', newline='', encoding='utf-8') as csvfile:
            # CSV yazıcısını oluştur
            csv_writer = csv.writer(csvfile)
            
            # Başlık satırını yaz
            csv_writer.writerow(['zaman_damgasi_utc', 'press_abs_hPa', 'press_abs2_hPa'])
            
            print(f"Loglama başladı. Veriler '{log_dosya_adi}' dosyasına kaydediliyor.")
            print("Durdurmak için Ctrl+C tuşlarına basın.")

            last_log_time = time.time()

            while True:
                # Gelen MAVLink mesajlarını dinle
                msg = master.recv_match(type=['SCALED_PRESSURE', 'SCALED_PRESSURE2'], blocking=True, timeout=2)

                if not msg:
                    # print("2 saniyedir veri alınamadı, kontrol ediliyor...") # Hata ayıklama için kullanılabilir
                    continue

                # Gelen mesajın tipine göre değeri güncelle
                msg_type = msg.get_type()
                if msg_type == 'SCALED_PRESSURE':
                    # press_abs değeri SCALED_PRESSURE mesajından gelir
                    en_son_degerler['press_abs'] = msg.press_abs
                elif msg_type == 'SCALED_PRESSURE2':
                    # ###################################### #
                    #                DÜZELTME                #
                    # ###################################### #
                    # İkinci barometrenin mutlak basınç değeri de 'press_abs' alanından gelir.
                    en_son_degerler['press_abs2'] = msg.press_abs

                # Belirlenen loglama aralığı geçti mi diye kontrol et
                current_time = time.time()
                if current_time - last_log_time >= LOG_INTERVAL:
                    utc_zaman_damgasi = datetime.utcnow().isoformat()
                    
                    # Değerler henüz gelmediyse 'N/A' yaz
                    press1 = en_son_degerler['press_abs'] if en_son_degerler['press_abs'] is not None else 'N/A'
                    press2 = en_son_degerler['press_abs2'] if en_son_degerler['press_abs2'] is not None else 'N/A'

                    # Verileri CSV dosyasına yaz
                    csv_writer.writerow([utc_zaman_damgasi, press1, press2])
                    csvfile.flush()
                    
                    print(f"Kaydedildi: Zaman: {utc_zaman_damgasi}, press_abs: {press1}, press_abs2: {press2}")
                    
                    last_log_time = current_time

    except KeyboardInterrupt:
        print("\nLoglama kullanıcı tarafından durduruldu.")
    except Exception as e:
        print(f"Bir hata oluştu: {e}")
    finally:
        print("Program sonlandırıldı.")

if __name__ == '__main__':
    main()
