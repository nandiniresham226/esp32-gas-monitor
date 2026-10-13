/*
  ESP32 Multi-Gas Monitor v5 - Supabase (Postgres) cloud upload

  Everything from v3 is kept (sensors, ppm, LEDs, buzzer, OLED).
  Cloud (v5):
    - Wi-Fi
    - Inserts one row every 2 s into a Supabase table called "readings"
      (dashboard shows the newest row as live and the last rows as history;
       Supabase stamps the time itself, so no NTP is needed)
    - Telegram alert when status rises to WARNING / DANGER, and when back to SAFE
    - Network work runs in its own task on core 0, so the buzzer / LEDs / OLED
      never freeze while Wi-Fi is slow.

  PINS: OLED SDA 21, SCL 22 | MQ-6 D34 | MQ-135 D35 | NO2 D32
        LEDs: Green 25, Yellow 26, Red 27 | Buzzer 33

  Libraries: Adafruit GFX Library, Adafruit SSD1306  (Wi-Fi/HTTPClient come with the ESP32 core)
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>

// =====================================================
// CLOUD SETTINGS  (EDIT THESE)
// =====================================================
const char *WIFI_SSID = "ARM_WIFI_ROS";
const char *WIFI_PASS = "REPLACE_WITH_WIFI_PASSWORD";

// Supabase project URL, NO trailing slash
const char *SUPABASE_URL        = "https://nmcdlzwicwuvwutjchov.supabase.co";
const char *SUPABASE_SECRET_KEY = "REPLACE_WITH_NEW_SECRET_KEY";

// Telegram (optional). Leave TG_TOKEN as "" to disable notifications.
const char *TG_TOKEN   = "";
const char *TG_CHAT_ID = "";

const unsigned long INSERT_MS = 2000;    // one row uploaded every 2 s
const unsigned long NOTIFY_COOLDOWN_MS = 60000;  // minimum gap between Telegram messages

// ---------------- OLED ----------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- PINS ----------------
const int MQ6_PIN = 34, MQ135_PIN = 35, NO2_PIN = 32;
const int GREEN_LED = 25, YELLOW_LED = 26, RED_LED = 27;
const int BUZZER_PIN = 33;

// =====================================================
// HARDWARE SETTINGS
// =====================================================
const float SENSOR_VCC    = 5.0;
const float DIVIDER_RATIO = 1.0;    // (R1+R2)/R2. 10k+20k -> 1.5 | no divider -> 1.0
const float RL_MQ6        = 10.0;
const float RL_MQ135      = 10.0;

const float MQ6_CLEAN_AIR_RATIO   = 10.0;
const float MQ135_CLEAN_AIR_RATIO = 3.6;

const float MQ6_A   = 1009.2, MQ6_B   = -2.35;
const float MQ135_A = 110.47, MQ135_B = -2.862;
const float CO2_BASELINE_PPM = 400.0;

// Your calibrated values (leave 0 to auto-calibrate at boot)
float R0_MQ6   = 7.490;    // kOhm
float R0_MQ135 = 27.239;   // kOhm

// =====================================================
// ALARM THRESHOLDS
// =====================================================
const float LPG_WARNING_PPM  = 1000, LPG_DANGER_PPM  = 3000;
const float CO2_WARNING_PPM  = 1000, CO2_DANGER_PPM  = 2000;
const float NO2_WARNING_MV   = 1000, NO2_DANGER_MV   = 2000;

// ---------------- TIMING ----------------
const unsigned long WARMUP_MS = 20000;  // 5 min warm-up(300000)
const unsigned long CAL_MS    = 30000;
const unsigned long READ_MS   = 1000;
const int SAMPLES = 10;

// ---------------- STATE ----------------
enum Status { SAFE, WARNING, DANGER };
const char *STATUS_NAME[] = { "SAFE", "WARNING", "DANGER" };

float bufMQ6[SAMPLES], bufMQ135[SAMPLES], bufNO2[SAMPLES];
int bufIndex = 0;
bool bufFilled = false;

Status stLPG = SAFE, stCO2 = SAFE, stNO2 = SAFE, status = SAFE;
unsigned long lastRead = 0, lastBeep = 0;
bool buzzerState = false;

// Data shared with the cloud task
struct Snapshot { float lpg, co2, no2, rs6, rs135; int status; };
Snapshot snap;
volatile bool snapReady = false;
portMUX_TYPE snapMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- HELPERS ----------------
float readSensorMV(int pin) {
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(pin);
  return (sum / 16.0) * DIVIDER_RATIO;
}

float average(float *buf) {
  int n = bufFilled ? SAMPLES : bufIndex;
  if (n == 0) return 0;
  float s = 0;
  for (int i = 0; i < n; i++) s += buf[i];
  return s / n;
}

float calcRs(float vmv, float rl) {
  float v = vmv / 1000.0;
  if (v < 0.05) v = 0.05;
  if (v > SENSOR_VCC - 0.05) v = SENSOR_VCC - 0.05;
  return rl * (SENSOR_VCC - v) / v;
}

float calcPPM(float rs, float r0, float a, float b) {
  if (r0 <= 0) return 0;
  return a * pow(rs / r0, b);
}

Status levelFor(float v, float warn, float dang, Status prev) {
  if (v >= dang) return DANGER;
  if (prev == DANGER && v >= dang * 0.9) return DANGER;
  if (v >= warn) return WARNING;
  if (prev >= WARNING && v >= warn * 0.9) return WARNING;
  return SAFE;
}

void setLeds(Status s) {
  digitalWrite(GREEN_LED,  s == SAFE);
  digitalWrite(YELLOW_LED, s == WARNING);
  digitalWrite(RED_LED,    s == DANGER);
}

void updateBuzzer(Status s) {
  unsigned long now = millis();
  if (s == SAFE) { digitalWrite(BUZZER_PIN, LOW); buzzerState = false; }
  else if (s == DANGER) digitalWrite(BUZZER_PIN, HIGH);
  else if (now - lastBeep >= (buzzerState ? 150UL : 850UL)) {
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);
    lastBeep = now;
  }
}

void showMessage(const char *title, unsigned long sec) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.println(title);
  display.setTextSize(3);
  display.setCursor(40, 35);
  display.print(sec);
  display.display();
}

void showReadings(float lpg, float co2, float no2mv, Status s) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("LPG : "); display.print(lpg, 0);   display.print(" ppm");
  display.setCursor(0, 12); display.print("CO2e: "); display.print(co2, 0);   display.print(" ppm");
  display.setCursor(0, 24); display.print("NO2 : "); display.print(no2mv, 0); display.print(" mV");
  // small Wi-Fi marker, top-right
  display.setCursor(110, 0); display.print(WiFi.status() == WL_CONNECTED ? "WiFi" : "----");
  display.drawLine(0, 38, 127, 38, SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 46);
  display.print(s == SAFE ? "SAFE" : s == WARNING ? "WARNING" : "DANGER!");
  display.display();
}

// ---------------- CLOUD HELPERS ----------------
// One long-lived connection: the slow TLS handshake happens once, not on every upload
WiFiClientSecure sbClient;
HTTPClient sbHttp;
bool sbReady = false;

bool supabaseInsert(const String &body) {
  if (!sbReady) {
    sbClient.setInsecure();             // prototype: skips certificate check
    sbHttp.setReuse(true);              // keep the connection open between uploads
    sbReady = true;
  }
  String url = String(SUPABASE_URL) + "/rest/v1/readings";
  if (!sbHttp.begin(sbClient, url)) return false;
  sbHttp.setTimeout(8000);
  sbHttp.addHeader("Content-Type", "application/json");
  sbHttp.addHeader("apikey", SUPABASE_SECRET_KEY);   // new-style keys go in apikey only
  sbHttp.addHeader("Prefer", "return=minimal");
  int code = sbHttp.POST(body);
  if (code >= 400) Serial.println(sbHttp.getString());   // shows Supabase's error message
  sbHttp.end();
  if (code < 0) sbClient.stop();        // broken connection: force a fresh one next time
  Serial.printf("[cloud] POST /readings -> %d\n", code);   // 201 = success
  return code >= 200 && code < 300;
}

String urlEncode(const String &s) {
  String out;
  const char *hex = "0123456789ABCDEF";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.') out += c;
    else { out += '%'; out += hex[(c >> 4) & 15]; out += hex[c & 15]; }
  }
  return out;
}

bool telegramSend(const String &text) {
  if (TG_TOKEN[0] == 0) return false;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = String("https://api.telegram.org/bot") + TG_TOKEN +
               "/sendMessage?chat_id=" + TG_CHAT_ID + "&text=" + urlEncode(text);
  if (!http.begin(client, url)) return false;
  http.setTimeout(5000);
  int code = http.GET();
  http.end();
  Serial.printf("[telegram] -> %d\n", code);
  return code == 200;
}

// ---------------- CLOUD TASK (runs on core 0) ----------------
void cloudTask(void *) {
  unsigned long lastInsert = 0, lastNotify = 0;
  int lastNotified = SAFE;
  bool firstPass = true;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }
    if (!snapReady) {      // waiting for the first reading
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    Snapshot s;
    taskENTER_CRITICAL(&snapMux);
    s = snap;
    taskEXIT_CRITICAL(&snapMux);

    // skip a row if any value is NaN/inf (would make invalid JSON)
    bool valid = isfinite(s.lpg) && isfinite(s.co2) && isfinite(s.no2) && isfinite(s.rs6) && isfinite(s.rs135);

    if (valid && (firstPass || millis() - lastInsert >= INSERT_MS)) {
      char body[200];
      snprintf(body, sizeof(body),
               "{\"lpg\":%.1f,\"co2\":%.1f,\"no2\":%.0f,\"rs6\":%.2f,\"rs135\":%.2f,\"status\":\"%s\"}",
               s.lpg, s.co2, s.no2, s.rs6, s.rs135, STATUS_NAME[s.status]);
      supabaseInsert(body);
      lastInsert = millis();
      firstPass = false;
    }

    // ---- Telegram alerts ----
    bool cooled = (millis() - lastNotify >= NOTIFY_COOLDOWN_MS) || lastNotify == 0;
    if (s.status > lastNotified && cooled) {
      String msg = String(s.status == DANGER ? "DANGER" : "WARNING") +
                   " - gas monitor. LPG " + String(s.lpg, 0) + " ppm, CO2e " +
                   String(s.co2, 0) + " ppm, NO2 " + String(s.no2, 0) + " mV";
      if (telegramSend(msg) || TG_TOKEN[0] == 0) { lastNotified = s.status; lastNotify = millis(); }
    } else if (s.status == SAFE && lastNotified != SAFE && cooled) {
      if (telegramSend("Gas monitor is back to SAFE.") || TG_TOKEN[0] == 0) {
        lastNotified = SAFE; lastNotify = millis();
      }
    }

    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(GREEN_LED, OUTPUT); pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);   pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(GREEN_LED, LOW); digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);   digitalWrite(BUZZER_PIN, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found! Check wiring / try 0x3D.");
    while (true) delay(100);
  }

  analogReadResolution(12);
  analogSetPinAttenuation(MQ6_PIN, ADC_11db);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  analogSetPinAttenuation(NO2_PIN, ADC_11db);

  // Start Wi-Fi now; it connects in the background during warm-up
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Warm-up
  digitalWrite(YELLOW_LED, HIGH);
  unsigned long start = millis();
  while (millis() - start < WARMUP_MS) {
    showMessage("WARMING UP", (WARMUP_MS - (millis() - start)) / 1000);
    delay(250);
  }

  // Calibration (skipped when R0 values are set)
  if (R0_MQ6 <= 0 || R0_MQ135 <= 0) {
    Serial.println("Calibrating R0 in clean air...");
    double sum6 = 0, sum135 = 0;
    int n = 0;
    start = millis();
    while (millis() - start < CAL_MS) {
      showMessage("CALIBRATE", (CAL_MS - (millis() - start)) / 1000);
      sum6   += calcRs(readSensorMV(MQ6_PIN),   RL_MQ6);
      sum135 += calcRs(readSensorMV(MQ135_PIN), RL_MQ135);
      n++;
      delay(500);
    }
    if (R0_MQ6 <= 0)   R0_MQ6   = (sum6 / n)   / MQ6_CLEAN_AIR_RATIO;
    if (R0_MQ135 <= 0) R0_MQ135 = (sum135 / n) / MQ135_CLEAN_AIR_RATIO;
  }
  digitalWrite(YELLOW_LED, LOW);

  Serial.print("R0_MQ6 (kOhm)   = "); Serial.println(R0_MQ6, 3);
  Serial.print("R0_MQ135 (kOhm) = "); Serial.println(R0_MQ135, 3);
  Serial.print("Wi-Fi: "); Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "not connected yet");
  Serial.println("ms,LPG_ppm,CO2eq_ppm,NO2_mV,Rs_MQ6_k,Rs_MQ135_k,STATUS");

  xTaskCreatePinnedToCore(cloudTask, "cloud", 10240, NULL, 1, NULL, 0);
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = millis();

  if (now - lastRead >= READ_MS) {
    lastRead = now;

    bufMQ6[bufIndex]   = readSensorMV(MQ6_PIN);
    bufMQ135[bufIndex] = readSensorMV(MQ135_PIN);
    bufNO2[bufIndex]   = readSensorMV(NO2_PIN);
    bufIndex++;
    if (bufIndex >= SAMPLES) { bufIndex = 0; bufFilled = true; }

    float rs6   = calcRs(average(bufMQ6),   RL_MQ6);
    float rs135 = calcRs(average(bufMQ135), RL_MQ135);
    float no2mv = average(bufNO2);

    float lpg = calcPPM(rs6, R0_MQ6, MQ6_A, MQ6_B);
    float co2 = calcPPM(rs135, R0_MQ135, MQ135_A, MQ135_B) + CO2_BASELINE_PPM;

    stLPG = levelFor(lpg,   LPG_WARNING_PPM, LPG_DANGER_PPM, stLPG);
    stCO2 = levelFor(co2,   CO2_WARNING_PPM, CO2_DANGER_PPM, stCO2);
    stNO2 = levelFor(no2mv, NO2_WARNING_MV,  NO2_DANGER_MV,  stNO2);
    status = max(stLPG, max(stCO2, stNO2));

    setLeds(status);
    showReadings(lpg, co2, no2mv, status);

    // hand the newest values to the cloud task
    taskENTER_CRITICAL(&snapMux);
    snap = { lpg, co2, no2mv, rs6, rs135, (int)status };
    snapReady = true;
    taskEXIT_CRITICAL(&snapMux);

    Serial.print(now);        Serial.print(",");
    Serial.print(lpg, 1);     Serial.print(",");
    Serial.print(co2, 1);     Serial.print(",");
    Serial.print(no2mv, 0);   Serial.print(",");
    Serial.print(rs6, 2);     Serial.print(",");
    Serial.print(rs135, 2);   Serial.print(",");
    Serial.println(STATUS_NAME[status]);
  }

  updateBuzzer(status);
}
