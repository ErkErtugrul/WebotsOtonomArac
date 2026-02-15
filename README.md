# Webots GPS Otonom Araç & Telemetri Sistemi

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Webots](https://img.shields.io/badge/Simülasyon-Webots-darkgreen)
![Flask](https://img.shields.io/badge/Backend-Flask-red)

Webots simülasyon ortamında çalışan, GPS tabanlı otonom araç projesi. Web arayüzü üzerinden aracın konumunu takip edebilir, kamera görüntüsünü işleyebilir ve gerçek zamanlı telemetri verilerine ulaşabilirsiniz.

---

## Ekran Görüntüleri

| **Sunucu Arayüzü (Web)** | **Webots Simülasyonu** |
|:---:|:---:|
| <img src="https://github.com/user-attachments/assets/97a86a75-3efe-484b-a0eb-29c31e415d44" width="100%" alt="Server Arayüzü"> | <img src="https://github.com/user-attachments/assets/a64413f5-1905-4872-9ca6-352d6aced9f4" width="100%" alt="Webots Görüntüsü"> |

---

## Kurulum ve Hazırlık

Projeyi çalıştırmadan önce gerekli kütüphaneleri yükleyin:

```bash
pip install flask flask-socketio requests opencv-python numpy
```
## 1. Navigasyon Ayarları 

controllers/FinalController/navigation.py:

REF_LAT = 0.0   # ← Kendi başlangıç enleminiz

REF_LNG = 0.0   # ← Kendi başlangıç boylamınız

## 2. Sunucu Bağlantısı
controllers/FinalController/server.py:

SERVER_URL = "[http://127.0.0.1:5000](http://127.0.0.1:5000)" <- Flask sunucunun IP:Port adresi

## Çalıştırma
1. Web Sunucusunu Başlatın:
   python app.py

2. Web Arayüzünü Açın:
   http://127.0.0.1:5000

3. Simülasyonu Başlatın:
    * Webots'u açın.
    * Araç nesnesini seçin.
    * controller alanına FinalController girin.
    * Simülasyonu başlatın.
