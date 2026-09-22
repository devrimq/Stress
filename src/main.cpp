#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"

const char *ssid = "Ok-Sens-Stress";
const char *password = "ok123456"; // 8 karakterli şifre

WebServer server(80);
Adafruit_ADS1115 ads;
Adafruit_MPU6050 mpu;
MAX30105 maxSensor;

// NTC Parametreleri
const float SERIES_RESISTOR = 100000.0;
const float NOMINAL_RESISTANCE = 100000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float B_COEFFICIENT = 3950.0;
const float SUPPLY_VOLTAGE = 3.30;

// PPG / Nabız Parametreleri
const uint32_t FINGER_ON_THRESHOLD = 50000;
float dcFilter = 0.0;
float dynamicThreshold = 25.0;
bool peakDetected = false;
unsigned long lastBeatTime = 0;
unsigned long ibi = 0;
float currentBPM = 0.0;

// Sensör Değerleri
float g_tempC = 0.0;
float g_voltGSR = 0.0;
float g_voltBat = 0.0;
int g_batPct = 0;
float g_accX = 0.0, g_accY = 0.0, g_accZ = 0.0, g_accelMag = 1.0;
String g_durumGenel = "CIHAZ BOSTA";

bool adsActive = false;
bool mpuActive = false;
bool maxActive = false;

// Fonksiyon Prototipleri
void processPPG(uint32_t irValue);

// --- OK-SENS GÖRSEL ARAYÜZÜ ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Ok-Sens | Biyometrik Takip</title>
  <style>
    :root {
      --bg: #0d1117;
      --card-bg: #161b22;
      --border: #30363d;
      --text: #c9d1d9;
      --text-bright: #ffffff;
      --accent: #58a6ff;
      --green: #2ea043;
      --red: #f85149;
      --orange: #d29922;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background-color: var(--bg); color: var(--text); padding: 16px; display: flex; flex-direction: column; align-items: center; }
    .header { width: 100%; max-width: 760px; display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; padding-bottom: 12px; border-bottom: 1px solid var(--border); }
    .brand { font-size: 1.4rem; font-weight: 700; color: var(--text-bright); display: flex; align-items: center; gap: 8px; }
    .brand span { color: var(--accent); }
    .battery-box { display: flex; align-items: center; gap: 8px; font-weight: 600; font-size: 0.95rem; }
    .grid { width: 100%; max-width: 760px; display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; margin-bottom: 20px; }
    .card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 8px; padding: 14px; display: flex; flex-direction: column; justify-content: space-between; }
    .card-title { font-size: 0.8rem; text-transform: uppercase; color: #8b949e; letter-spacing: 0.5px; }
    .card-value { font-size: 1.6rem; font-weight: 700; color: var(--text-bright); margin: 6px 0; }
    .card-sub { font-size: 0.75rem; color: #8b949e; }
    .status-card { grid-column: 1 / -1; background: var(--card-bg); border: 1px solid var(--border); border-radius: 8px; padding: 16px; text-align: center; }
    .status-text { font-size: 1.3rem; font-weight: 700; color: var(--accent); margin-top: 4px; }
    .actions { width: 100%; max-width: 760px; display: flex; gap: 10px; }
    button { flex: 1; padding: 12px; border-radius: 6px; border: 1px solid var(--border); font-weight: 600; cursor: pointer; transition: 0.2s; }
    .btn-log { background: var(--green); color: #fff; border: none; }
    .btn-export { background: var(--card-bg); color: var(--text-bright); }
    .btn-export:hover { background: #21262d; }
  </style>
</head>
<body>
  <div class="header">
    <div class="brand">Ok-Sens <span>Stress Monitor</span></div>
    <div class="battery-box" id="batBox">
      <span id="batVolt">-- V</span>
      <span id="batPct" style="color: var(--green);">--%</span>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <div class="card-title">Galvanik Direnç (GSR)</div>
      <div class="card-value" id="gsrVal">--</div>
      <div class="card-sub">Deri İletkenlik Voltajı</div>
    </div>
    <div class="card">
      <div class="card-title">Kalp Nabzı (BPM)</div>
      <div class="card-value" id="bpmVal">--</div>
      <div class="card-sub">MAX30102 PPG</div>
    </div>
    <div class="card">
      <div class="card-title">Yüzey Isısı</div>
      <div class="card-value" id="tempVal">-- °C</div>
      <div class="card-sub">NTC Termistör</div>
    </div>
    <div class="card">
      <div class="card-title">Dinamik İvme</div>
      <div class="card-value" id="accVal">--</div>
      <div class="card-sub" id="accAxes">X: -- | Y: -- | Z: --</div>
    </div>
    <div class="status-card">
      <div class="card-title">Füzyon Analiz Durumu</div>
      <div class="status-text" id="stressVal">Bağlanıyor...</div>
    </div>
  </div>

  <div class="actions">
    <button class="btn-log" id="logBtn" onclick="toggleLog()">Kaydı Başlat</button>
    <button class="btn-export" onclick="exportCSV()">CSV Dışa Aktar</button>
  </div>

  <script>
    let isLogging = false;
    let logData = [];
    let wakeLock = null;

    async function requestWakeLock() {
      try {
        if ('wakeLock' in navigator) {
          wakeLock = await navigator.wakeLock.request('screen');
        }
      } catch (err) {}
    }
    requestWakeLock();

    // Veri güncelleme
    setInterval(() => {
      fetch('/veri')
        .then(r => r.json())
        .then(d => {
          document.getElementById('batVolt').innerText = d.batV.toFixed(2) + " V";
          document.getElementById('batPct').innerText = "%" + d.batPct;
          document.getElementById('gsrVal').innerText = d.gsr.toFixed(2) + " V";
          document.getElementById('bpmVal').innerText = d.bpm > 0 ? d.bpm : "--";
          document.getElementById('tempVal').innerText = d.temp.toFixed(1) + " °C";
          document.getElementById('accVal').innerText = d.accM.toFixed(2) + " g";
          document.getElementById('accAxes').innerText = `X:${d.accX.toFixed(1)} Y:${d.accY.toFixed(1)} Z:${d.accZ.toFixed(1)}`;
          document.getElementById('stressVal').innerText = d.status;

          const st = document.getElementById('stressVal');
          if (d.status.includes("YUKSEK STRES")) st.style.color = "var(--red)";
          else if (d.status.includes("EFOR")) st.style.color = "var(--orange)";
          else if (d.status.includes("SAKIN")) st.style.color = "var(--green)";
          else st.style.color = "var(--accent)";

          if (isLogging) {
            logData.push({
              time: new Date().toLocaleTimeString(),
              gsr: d.gsr.toFixed(3),
              bpm: d.bpm,
              temp: d.temp.toFixed(2),
              accMag: d.accM.toFixed(2),
              stress: d.status,
              batV: d.batV.toFixed(2)
            });
          }
        }).catch(() => {});
    }, 500);

    function toggleLog() {
      isLogging = !isLogging;
      const b = document.getElementById('logBtn');
      b.innerText = isLogging ? "Kaydı Durdur" : "Kaydı Başlat";
      b.style.background = isLogging ? "var(--red)" : "var(--green)";
    }

    function exportCSV() {
      if (logData.length === 0) {
        alert("Henüz kaydedilmiş veri bulunmuyor!");
        return;
      }
      let csv = "Zaman;GSR_V;BPM;Sicaklik_C;Ivme_g;Durum;Pil_V\n";
      logData.forEach(r => {
        csv += `${r.time};${r.gsr};${r.bpm};${r.temp};${r.accMag};${r.stress};${r.batV}\n`;
      });
      const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
      const link = document.createElement("a");
      link.href = URL.createObjectURL(blob);
      link.setAttribute("download", `Ok-Sens_Veri_${Date.now()}.csv`);
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleVeri() {
  char buf[256];
  snprintf(buf, sizeof(buf), 
           "{\"status\":\"%s\",\"batPct\":%d,\"batV\":%.2f,\"bpm\":%.0f,\"temp\":%.1f,\"gsr\":%.3f,\"accM\":%.2f,\"accX\":%.2f,\"accY\":%.2f,\"accZ\":%.2f}", 
           g_durumGenel.c_str(), g_batPct, g_voltBat, currentBPM, g_tempC, g_voltGSR, g_accelMag, g_accX, g_accY, g_accZ);
  server.send(200, "application/json", buf);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n--- Ok-Sens Baslatiliyor ---");

  // 1) Wi-Fi SoftAP - 2 Baglanti siniri eklendi
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(50);
  WiFi.mode(WIFI_AP);

  WiFi.softAP(ssid, password, 1, 0, 2); 
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  server.on("/", handleRoot);
  server.on("/veri", handleVeri);
  server.begin();
  Serial.println("[Wi-Fi & Web] Aktif -> 192.168.4.1");

  // 2) I2C Hattı ve Kilitlenme Koruması
  Wire.begin(8, 9);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  // ADS1115
  if (!ads.begin(0x48)) {
    Serial.println("[HATA] ADS1115 bulunamadi!");
  } else {
    ads.setGain(GAIN_ONE);
    adsActive = true;
  }

  // MPU6050
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("[HATA] MPU6050 bulunamadi!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuActive = true;
  }

  // MAX30102
  if (!maxSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("[HATA] MAX30102 bulunamadi!");
  } else {
    maxSensor.setup();
    maxSensor.setPulseAmplitudeRed(0x1F);
    maxSensor.setPulseAmplitudeIR(0x1F);
    maxActive = true;
  }
}

void loop() {
  server.handleClient();

  static unsigned long lastSample = 0;
  if (millis() - lastSample >= 35) { 
    lastSample = millis();

    // 1. ADS1115 Okumaları
    if (adsActive) {
      int16_t rawNTC = ads.readADC_SingleEnded(0);
      float voltNTC = ads.computeVolts(rawNTC);
      if (voltNTC > 0.05 && voltNTC < (SUPPLY_VOLTAGE - 0.05)) {
        float resistance = ((SUPPLY_VOLTAGE - voltNTC) * SERIES_RESISTOR) / voltNTC;
        float steinhart = resistance / NOMINAL_RESISTANCE;
        steinhart = log(steinhart) / B_COEFFICIENT;
        steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
        g_tempC = (1.0 / steinhart) - 273.15;
      }

      int16_t rawGSR = ads.readADC_SingleEnded(1);
      g_voltGSR = ads.computeVolts(rawGSR);

      int16_t rawBat = ads.readADC_SingleEnded(2);
      g_voltBat = ads.computeVolts(rawBat) * 2.0; // Y-Koprusu 2.0x Carpani (Guncel)
      g_batPct = constrain((int)((g_voltBat - 3.30) / (4.20 - 3.30) * 100.0), 0, 100);
    }

    // 2. MPU6050 Okuması (G degerine cevrildi)
    bool isMoving = false;
    if (mpuActive) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      g_accX = a.acceleration.x / 9.81;
      g_accY = a.acceleration.y / 9.81;
      g_accZ = a.acceleration.z / 9.81;
      g_accelMag = sqrt(sq(g_accX) + sq(g_accY) + sq(g_accZ));
      isMoving = abs(g_accelMag - 1.0) > 0.15; // 1G referans sapmasi
    }

    // 3. MAX30102 Okuması & Nabız
    bool fingerOn = false;
    if (maxActive) {
      uint32_t irValue = maxSensor.getIR();
      fingerOn = (irValue > FINGER_ON_THRESHOLD);

      if (fingerOn) {
        if (!isMoving) processPPG(irValue);
      } else {
        currentBPM = 0.0;
        lastBeatTime = 0;
        peakDetected = false;
      }
    }

    // 4. Durum Füzyonu
    if (!fingerOn) {
      g_durumGenel = "CIHAZ BOSTA";
    } else if (isMoving) {
      g_durumGenel = "FIZIKSEL EFOR";
    } else {
      if (currentBPM > 95.0 && g_voltGSR < 1.70) {
        g_durumGenel = "YUKSEK STRES";
      } else if (currentBPM > 90.0 || g_voltGSR < 1.75) {
        g_durumGenel = "HAFIF STRES";
      } else {
        g_durumGenel = "SAKIN / DINLENIK";
      }
    }
  }

  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 2000) {
    lastLog = millis();
    Serial.printf("[%s] Pil: %%%d | BPM: %.0f | Isi: %.1f C | GSR: %.3f V | Ivme: %.1f g\n", 
                  g_durumGenel.c_str(), g_batPct, currentBPM, g_tempC, g_voltGSR, g_accelMag);
  }

  delay(2);
}

void processPPG(uint32_t irValue) {
  if (dcFilter == 0.0) dcFilter = irValue;
  dcFilter = (dcFilter * 0.95) + (irValue * 0.05);
  float acSignal = (float)irValue - dcFilter;

  dynamicThreshold = (dynamicThreshold * 0.97) + (max(acSignal, 0.0f) * 0.03);
  if (dynamicThreshold < 20.0) dynamicThreshold = 20.0;

  unsigned long currentTime = millis();

  if (acSignal > dynamicThreshold && !peakDetected) {
    if (lastBeatTime == 0) {
      lastBeatTime = currentTime;
      peakDetected = true;
    } else {
      unsigned long delta = currentTime - lastBeatTime;
      if (delta >= 350 && delta <= 1500) {
        ibi = delta;
        float instantBPM = 60000.0 / (float)ibi;
        if (currentBPM == 0.0) {
          currentBPM = instantBPM;
        } else {
          currentBPM = (currentBPM * 0.60) + (instantBPM * 0.40);
        }
        lastBeatTime = currentTime;
        peakDetected = true;
      } else if (delta > 1500) {
        lastBeatTime = currentTime;
      }
    }
  }

  if (acSignal < (dynamicThreshold * 0.30)) {
    peakDetected = false;
  }

  if (lastBeatTime != 0 && (currentTime - lastBeatTime > 3000)) {
    currentBPM = 0.0;
    lastBeatTime = 0;
    peakDetected = false;
  }
}