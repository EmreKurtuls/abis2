import time
import datetime
import csv
from VehicleMav import Submarine  # VehicleMav.py dosyasından Submarine sınıfını içe aktar

# Pixhawk'ınızın bağlı olduğu port
PIXHAWK_PORT = '/dev/ttyACM0'  # Kendi portunuza göre düzenleyin

# Kayıtların tutulacağı CSV dosyasının adı
LOG_FILENAME = "pressure_log.csv"

# Ana program
if __name__ == "__main__":
    rov = None
    time.sleep(20)
    try:
        # CSV dosyasını oluştur ve başlık satırını yaz
        with open(LOG_FILENAME, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Sütun başlıkları
            writer.writerow(["Timestamp", "Absolute_Pressure (hPa)"])
        print(f"'{LOG_FILENAME}' dosyası oluşturuldu. Kayıt başlıyor...")

        # Pixhawk'a bağlan
        print(f"Pixhawk'a bağlanılıyor: {PIXHAWK_PORT}...")
        rov = Submarine(Usb=PIXHAWK_PORT)

        if rov.vehicle is None:
            print("Bağlantı BAŞARISIZ! Port adını ve bağlantıyı kontrol edin.")
        else:
            print("Bağlantı başarılı. Basınç verisi okunuyor...")
            print("Kaydı durdurmak için Ctrl+C tuşlarına basın.")
            
            # Listener'ların ilk verileri alması için kısa bir bekleme süresi
            time.sleep(2)

            # Veri kaydı için ana döngü
            while True:
                # VehicleMav nesnesi içindeki güncel basınç verisini al
                # Bu veri, arka plandaki message_listener_worker tarafından sürekli güncellenir.
                pressure_hpa = rov.scalePressure2.press_abs

                # Mevcut zamanı al
                current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] # Milisaniyeli format

                # Veriyi ekrana yazdır
                print(f"Zaman: {current_time}, Basınç: {pressure_hpa:.2f} hPa")
                
                # Veriyi CSV dosyasına ekle
                with open(LOG_FILENAME, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([current_time, f"{pressure_hpa:.2f}"])

                # Belirtilen aralıkla kayıt yap (örneğin saniyede 5 kez)
                time.sleep(0.2)

    except KeyboardInterrupt:
        # Kullanıcı Ctrl+C ile çıkmak istediğinde
        print("\n\nKullanıcı tarafından kayıt durduruldu.")
    except Exception as e:
        # Beklenmedik bir hata oluşursa
        print(f"\nBir hata oluştu: {e}")
    finally:
        # Program sonlandığında veya hata oluştuğunda bağlantıyı güvenle kapat
        if rov:
            print("Bağlantı kapatılıyor...")
            rov.close()
        print(f"Kayıt tamamlandı. Veriler '{LOG_FILENAME}' dosyasına kaydedildi.")
