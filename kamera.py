import cv2
import datetime
import time
def record_video(camera_index=0):
    """
    Belirtilen kamerayı açar, canlı görüntüyü gösterir ve aynı anda
    görüntüyü zaman damgalı bir AVI dosyasına kaydeder.
    """
    # Kamerayı aç
    cap = cv2.VideoCapture(camera_index)

    # Kamera başarıyla açıldı mı diye kontrol et
    if not cap.isOpened():
        print(f"HATA: Kamera indeksi {camera_index} açılamadı.")
        print("Kameranın bağlı olduğundan ve başka bir program tarafından kullanılmadığından emin olun.")
        return

    # Görüntü boyutlarını kameradan al
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Videonun oynatma hızını (FPS) belirle
    fps = 20.0

    # Benzersiz bir dosya adı oluştur (örn: kayit_20250708_194023.avi)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = f"kayit_{timestamp}.avi"

    # Video kaydı için gerekli olan VideoWriter nesnesini oluştur
    # Codec olarak XVID, .avi formatı için en uyumlu olanlardan biridir.
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

    print("Kamera açıldı. Canlı görüntü gösteriliyor...")
    print(f"Kayıt '{output_filename}' dosyasına yapılıyor.")
    print("Durdurmak için 'q' tuşuna basın.")

    # Ana döngü
    while True:
        # Kameradan yeni bir kare (frame) oku
        ret, frame = cap.read()

        # Eğer kare başarıyla okunamazsa (örneğin kamera bağlantısı koptu) döngüyü sonlandır
        if not ret:
            print("Hata: Kameradan görüntü alınamadı. Kayıt durduruluyor.")
            break

        # Okunan kareyi dosyaya yaz
        out.write(frame)

        # Okunan kareyi ekranda göster
        cv2.imshow('Canlı Kamera Kaydı - Çıkmak için "q" tuşuna basın', frame)

        # 'q' tuşuna basıldığında döngüden çık
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Tüm işlemler bittiğinde kaynakları serbest bırak ve pencereleri kapat
    print("Kayıt sonlandırılıyor ve dosya kaydediliyor...")
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"Video başarıyla '{output_filename}' olarak kaydedildi.")


if __name__ == "__main__":
	
    # 0, genellikle bilgisayarın ilk tanıdığı kameradır (dahili veya USB).
    # Eğer bu çalışmazsa 1, 2 gibi değerleri deneyebilirsiniz.
    time.sleep(20)
    record_video(camera_index=0)
