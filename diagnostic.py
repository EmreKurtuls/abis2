import sys
import os
from pprint import pprint

print("--- Python Yorumlayıcı Bilgileri ---")
print(f"Sürüm: {sys.version}")
print(f"Çalıştırılabilir Dosya: {sys.executable}")
print("-" * 35)

print("\n--- Python Arama Yolları (sys.path) ---")
pprint(sys.path)
print("-" * 35)

try:
    print("\n--- 'torch' import ediliyor... ---")
    import torch
    print("\n--- Bulunan torch Kütüphanesi Bilgileri ---")
    print(f"torch.__version__: {torch.__version__}")
    # torch modülünün bulunduğu dosyanın tam yolunu yazdır
    print(f"torch.__file__: {torch.__file__}")
    
    # Bu dosyanın bulunduğu klasörün içeriğini listeleyelim
    torch_path = os.path.dirname(torch.__file__)
    print(f"İçerdiği Klasör: {torch_path}")
    print(f"Bu klasördeki dosyalar: {os.listdir(torch_path)}")

except ImportError as e:
    print("\n--- HATA ---")
    print("`import torch` komutu teşhis sırasında yine başarısız oldu.")
    print("Hata Mesajı:", e)
except Exception as e:
    print("\n--- BEKLENMEDİK HATA ---")
    print(e)
