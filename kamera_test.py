import cv2

def live_camera_test(camera_index=0):
    """
    Belirtilen indeksteki kameradan canlı video akışı gösterir.
    """
    print(f"--- Canlı video testi için kamera indeksi {camera_index} deneniyor... ---")
    
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"HATA: Kamera {camera_index} açılamadı. Lütfen bağlantıyı kontrol edin.")
        return

    print("BAŞARILI: Kamera açıldı. Canlı video akışı gösteriliyor...")
    print("Pencereyi kapatmak için klavyeden 'q' tuşuna basın.")

    # Canlı video için sonsuz döngü
    while True:
        # Kameradan yeni bir kare oku
        ret, frame = cap.read()
        
        # Eğer kare okunamadıysa (video bitti veya kamera koptu), döngüden çık
        if not ret:
            print("Hata: Görüntü akışı kesildi.")
            break
            
        # Okunan kareyi ekranda göster
        cv2.imshow('Canli Kamera Akisi - Cikmak icin "q" ya basin', frame)

        # 1 milisaniye bekle ve 'q' tuşuna basılıp basılmadığını kontrol et
        # Bu satır, hem videonun akıcı görünmesini sağlar hem de çıkış komutunu dinler
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Döngü bittiğinde kaynakları serbest bırak ve pencereleri kapat
    cap.release()
    cv2.destroyAllWindows()
    print("Canlı video testi sonlandırıldı.")


if __name__ == "__main__":
    # Testi çalışan kamera indeksimiz olan 0 ile yapıyoruz.
    live_camera_test(camera_index=0)
