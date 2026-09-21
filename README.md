# ZED Sağlık ve Stres Monitörü (ESP32-C3)

ESP32-C3 SuperMini tabanlı, mobilite odaklı, uygulama kurulumu gerektirmeyen giyilebilir sağlık ve stres takip sistemi.

## 📌 Özellikler
- **Sıfır Kurulum / Web Arayüzü:** Uygulama indirmeden doğrudan Wi-Fi AP (`192.168.4.1`) üzerinden mobil veya masaüstü tarayıcıdan izleme.
- **Canlı Biyometrik Metrikler:**
  - **GSR (Galvanik Deri Tepkisi):** ADS1115 üzerinden deri iletkenliği.
  - **Kalp Atışı (BPM):** MAX30102 PPG sensörü ile dinamik nabız algılama.
  - **Cilt Sıcaklığı:** NTC termistör üzerinden anlık yüzey ısısı (°C).
  - **Hareket Analizi:** MPU6050 ivmeölçer ile $m/s^2$ cinsinden hareket ve durağanlık tespiti.
  - **Pil Yönetimi:** Gerilim bölücü hat üzerinden canlı voltaj ve yüzde (%) takibi.
- **Stres & Efor Füzyonu:** Biyometrik verileri birleştirerek *Sakin/Dinlenik*, *Fiziksel Efor*, *Hafif Stres* ve *Yüksek Stres* durumlarının anlık tespiti.
- **CSV Veri Kaydı:** Tarayıcı belleği üzerinden zaman damgalı verileri tek tuşla Excel uyumlu `.csv` formatında dışa aktarma.

## 🛠️ Donanım Mimarisi & Pin Yapısı
- **Mikrodenetleyici:** ESP32-C3 SuperMini
- **I2C Veri Yolu:**
  - `SDA` -> GPIO 8
  - `SCL` -> GPIO 9
- **Sensörler:**
  - ADS1115 (ADC: NTC, GSR, Pil) -> `0x48`
  - MPU6050 (İvmeölçer) -> `0x68`
  - MAX30102 (PPG / Kalp Atışı) -> `0x57`

## 🚀 Kurulum & Yükleme
Proje PlatformIO ortamında geliştirilmiştir.
```bash
git clone [https://github.com/devrimq/Stress.git](https://github.com/devrimq/Stress.git)
cd Stress
pio run --target upload