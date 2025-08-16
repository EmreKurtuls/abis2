ARDUINO_PORT = '/dev/cu.usbserial-130'
BAUDRATE = 115200

PITCH_KP = 1.0
PITCH_KI = 0.1  # Çok küçük bir I değeri
PITCH_KD = 0.0  # <-- D TERİMİNİ KAPATTIK

PITCH_WINDUP = 10

# Roll (Yuvarlanma)
ROLL_KP = 1.0
ROLL_KI = 0.1   # Çok küçük bir I değeri
ROLL_KD = 0.0   # <-- D TERİMİNİ KAPATTIK

ROLL_WINDUP = 10

YAW_KP = 2.0   # Yaw için daha yumuşak bir başlangıç
YAW_KI = 0.1   # Küçük hataları düzeltmesi için
YAW_KD = 0.0   # Titremeyi önlemek için şimdilik kapalı

YAW_WINDUP = 15

PRESSURE_KP = 0.1  # Basınç değerleri büyük olduğu için Kp küçük olmalı
PRESSURE_KI = 0.02 # Küçük sapmaları düzeltmek için
PRESSURE_KD = 0.0  # Titremeyi önlemek için şimdilik kapalı

PRESSURE_WINDUP = 50