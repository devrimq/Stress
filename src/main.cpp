#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"

const char *ssid = "ZED_Saglik_Monitor";
const char *password = "12345678";

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
float g_accelMag = 9.8;
String g_durumGenel = "CIHAZ BOSTA";

bool adsActive = false;
bool mpuActive = false;
bool maxActive = false;

// Fonksiyon Prototipleri
void processPPG(uint32_t irValue);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ZED Sağlık Monitörü</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; }
    .container { max-width: 600px; margin: 0 auto; }
    h1 { font-size: 1.4rem; text-align: center; margin-bottom: 16px; color: #38bdf8; }
    .status-badge { text-align: center; padding: 12px; font-weight: bold; border-radius: 8px; margin-bottom: 16px; font-size: 1.05rem; background: #334155; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 20px; }
    .card { background: #1e293b; padding: 14px; border-radius: 8px; border: 1px solid #334155; }
    .card-title { font-size: 0.8rem; color: #94a3b8; text-transform: uppercase; }
    .card-value { font-size: 1.6rem; font-weight: bold; margin-top: 6px; }
    .btn { display: block; width: 100%; padding: 14px; background: #0284c7; color: white; border: none; border-radius: 8px; font-size: 1rem; font-weight: bold; cursor: pointer; }
    .btn:active { background: #0369a1; }
  </style>
</head>
<body>
  <div class="container">
    <h1>ZED Sağlık Monitörü</h1>
    <div id="status" class="status-badge">Bağlantı kuruluyor...</div>
    <div class="grid">
      <div class="card">
        <div class="card-title">Pil Durumu</div>
        <div id="bat" class="card-value">-- %</div>
      </div>
      <div class="card">
        <div class="card-title">Kalp Atışı</div>
        <div id="bpm" class="card-value">-- BPM</div>
      </div>
      <div class="card">
        <div class="card-title">Cilt Sıcaklığı</div>
        <div id="temp" class="card-value">-- °C</div>
      </div>
      <div class="card">
        <div class="card-title">GSR (İletkenlik)</div>
        <div id="gsr" class="card-value">-- V</div>
      </div>
      <div class="card" style="grid-column: span 2;">
        <div class="card-title">İvme Büyüklüğü</div>
        <div id="accel" class="card-value">-- m/s²</div>
      </div>
    </div>
    <button class="btn" onclick="downloadCSV()">Verileri CSV İndir</button>
  </div>

  <script>
    let logData = [["Zaman", "Durum", "Pil_Yuzde", "Pil_Volt", "BPM", "Sicaklik", "GSR_V", "Ivme"]];

    setInterval(() => {
      fetch('/veri')
        .then(r => r.json())
        .then(d => {
          document.getElementById('status').innerText = d.durum;
          document.getElementById('bat').innerText = "%" + d.bat_pct + " (" + d.bat_v.toFixed(2) + "V)";
          document.getElementById('bpm').innerText = d.bpm > 0 ? Math.round(d.bpm) + " BPM" : "--";
          document.getElementById('temp').innerText = d.temp.toFixed(1) + " °C";
          document.getElementById('gsr').innerText = d.gsr.toFixed(3) + " V";
          document.getElementById('accel').innerText = d.accel.toFixed(1) + " m/s²";

          const st = document.getElementById('status');
          if (d.durum.includes("STRES")) st.style.background = "#b91c1c";
          else if (d.durum.includes("EFOR")) st.style.background = "#d97706";
          else if (d.durum.includes("SAKIN")) st.style.background = "#15803d";
          else st.style.background = "#334155";

          const timeStr = new Date().toLocaleTimeString();
          logData.push([timeStr, d.durum, d.bat_pct, d.bat_v.toFixed(2), d.bpm, d.temp, d.gsr, d.accel]);
        })
        .catch(() => {});
    }, 500);

    function downloadCSV() {
      let csvContent = "data:text/csv;charset=utf-8," + logData.map(e => e.join(",")).join("\n");
      let encodedUri = encodeURI(csvContent);
      let link = document.createElement("a");
      link.setAttribute("href", encodedUri);
      link.setAttribute("download", "zed_saglik_log_" + Date.now() + ".csv");
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
  char buf[192];
  snprintf(buf, sizeof(buf), 
           "{\"durum\":\"%s\",\"bat_pct\":%d,\"bat_v\":%.2f,\"bpm\":%.0f,\"temp\":%.1f,\"gsr\":%.3f,\"accel\":%.1f}", 
           g_durumGenel.c_str(), g_batPct, g_voltBat, currentBPM, g_tempC, g_voltGSR, g_accelMag);
  server.send(200, "application/json", buf);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n--- ADIM 5: Tum Sensorler + Wi-Fi Entegrasyonu ---");

  // 1) Wi-Fi SoftAP
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(50);
  WiFi.mode(WIFI_AP);

  WiFi.softAP(ssid, password, 1, 0, 4);
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
    Serial.println("[BASARILI] ADS1115 Hazir.");
  }

  // MPU6050
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("[HATA] MPU6050 bulunamadi!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuActive = true;
    Serial.println("[BASARILI] MPU6050 Hazir.");
  }

  // MAX30102
  if (!maxSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("[HATA] MAX30102 bulunamadi!");
  } else {
    maxSensor.setup();
    maxSensor.setPulseAmplitudeRed(0x1F);
    maxSensor.setPulseAmplitudeIR(0x1F);
    maxActive = true;
    Serial.println("[BASARILI] MAX30102 Hazir.");
  }
}

void loop() {
  server.handleClient();

  static unsigned long lastSample = 0;
  if (millis() - lastSample >= 35) { // ~28 Hz ritim
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
      g_voltBat = ads.computeVolts(rawBat) * 2.0;
      g_batPct = constrain((int)((g_voltBat - 3.30) / (4.20 - 3.30) * 100.0), 0, 100);
    }

    // 2. MPU6050 Okuması
    bool isMoving = false;
    if (mpuActive) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      g_accelMag = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
      isMoving = abs(g_accelMag - 9.81) > 2.0;
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
        g_durumGenel = "YUKSEK STRES / UYARILMA";
      } else if (currentBPM > 90.0 || g_voltGSR < 1.75) {
        g_durumGenel = "HAFIF STRES / ODAKLANMA";
      } else {
        g_durumGenel = "SAKIN / DINLENIK";
      }
    }
  }

  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 2000) {
    lastLog = millis();
    Serial.printf("[%s] Pil: %%%d | BPM: %.0f | Isi: %.1f C | GSR: %.3f V | Ivme: %.1f\n", 
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