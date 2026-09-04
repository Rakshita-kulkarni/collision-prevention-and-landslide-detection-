#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <math.h>
#include "mbedtls/base64.h"

// -------- SH1106 128x64 OLED --------
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(
  U8G2_R0,
  U8X8_PIN_NONE
);

// -------- WiFi Setup --------
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
WebServer server(80);

// -------- Twilio WhatsApp Setup --------
const char* twilioAccountSID = "YOUR_TWILIO_ACCOUNT_SID";
const char* twilioAuthToken  = "YOUR_TWILIO_AUTH_TOKEN";
const char* twilioFromNumber = "whatsapp:YOUR_TWILIO_WHATSAPP_NUMBER";
const char* alertToNumber    = "whatsapp:YOUR_RECEIVER_WHATSAPP_NUMBER";

// Base64 helper for HTTP Auth
String base64Encode(String input) {
  size_t outputLength = 0;
  size_t inputLength = input.length();

  // Calculate maximum output buffer size
  unsigned char outputBuffer[128];
  mbedtls_base64_encode(outputBuffer, sizeof(outputBuffer), &outputLength,
                        (const unsigned char*)input.c_str(), inputLength);

  return String((char*)outputBuffer);
}


// Send WhatsApp message using Twilio
void sendWhatsAppMessage(String message) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Cannot send WhatsApp message.");
    return;
  }

  HTTPClient http;
  String url = "https://api.twilio.com/2010-04-01/Accounts/" + String(twilioAccountSID) + "/Messages.json";
  String auth = String(twilioAccountSID) + ":" + String(twilioAuthToken);

  String postData = "To=" + String(alertToNumber) +
                    "&From=" + String(twilioFromNumber) +
                    "&Body=" + message;

  http.begin(url);
  http.addHeader("Authorization", "Basic " + base64Encode(auth));
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpResponseCode = http.POST(postData);
  if (httpResponseCode > 0) {
    Serial.printf("Twilio WhatsApp message sent, code: %d\n", httpResponseCode);
  } else {
    Serial.printf("Twilio WhatsApp message failed, code: %d\n", httpResponseCode);
  }
  http.end();
}


// -------- Hardware Pins --------
#define TRIG_A 18
#define ECHO_A 19
#define TRIG_B 2
#define ECHO_B 4
#define IR_A 34
#define IR_B 35
#define SERVO_A_PIN 32
#define SERVO_B_PIN 33
#define BUZZER_A 23
#define BUZZER_B 27
#define I2C_SDA 21
#define I2C_SCL 22

// -------- MPU6050 --------
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
const float ACCEL_SCALE = 16384.0;

Servo servoA, servoB;

bool laneA_IR = false, laneB_IR = false;
int distA = 0, distB = 0;
String lastOpened = "A";
unsigned long lastLCDUpdate = 0, lastDecisionTime = 0;
bool barricadeA_Open = false, barricadeB_Open = false;

bool eventActive = false;
String eventType = "";
float eventMagnitude = 0.0;
unsigned long eventStart = 0, eventLastAbove = 0;

float ax_offset = 0.0, ay_offset = 0.0, az_offset = 0.0;

// Detection thresholds
const float DETECT_G_THRESHOLD = 5.0;
const float STOP_G_THRESHOLD   = 0.12;
const float TILT_CHANGE_DEG    = 12.0;
const unsigned long EVENT_MIN_MS  = 1500;
const unsigned long EVENT_HOLD_MS = 2000;

void webKeepAlive() {
  server.handleClient();
  delay(1);
}

long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}

void openGateA()  {
  servoA.write(0);
  barricadeA_Open = true;
}

void closeGateA() {
  servoA.write(70);
  barricadeA_Open = false;
}

void openGateB()  {
  servoB.write(0);
  barricadeB_Open = true;
}

void closeGateB() {
  servoB.write(90);
  barricadeB_Open = false;
}

void buzzAlert(int buzzerPin, int distance) {
  if (distance > 60) noTone(buzzerPin);
  else if (distance > 40) {
    tone(buzzerPin, 1000, 150);
    delay(500);
  }
  else if (distance > 20) {
    tone(buzzerPin, 2000, 150);
    delay(300);
  }
  else if (distance > 10) {
    tone(buzzerPin, 3000, 150);
    delay(150);
  }
  else tone(buzzerPin, 4000);
}

bool mpuBegin() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  return (Wire.endTransmission() == 0);
}

bool readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) return false;

  uint8_t cnt = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);

  if (cnt < 6) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  return true;
}

bool readAccelG(float &ax_g, float &ay_g, float &az_g, float &mag_g) {
  int16_t ax_raw, ay_raw, az_raw;

  if (!readAccelRaw(ax_raw, ay_raw, az_raw)) return false;

  ax_g = ((float)ax_raw / ACCEL_SCALE) - ax_offset;
  ay_g = ((float)ay_raw / ACCEL_SCALE) - ay_offset;
  az_g = ((float)az_raw / ACCEL_SCALE) - az_offset;

  mag_g = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  return true;
}

void accelToAngles(float ax, float ay, float az,
                   float &roll_deg, float &pitch_deg) {
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  roll_deg = roll * 180.0 / M_PI;
  pitch_deg = pitch * 180.0 / M_PI;
}


// -------- OLED Display Helper --------
void oledShowMessage(String line1, String line2 = "", String line3 = "", String line4 = "") {
  display.clearBuffer();

  display.setFont(u8g2_font_6x10_tf);

  display.drawStr(0, 12, line1.c_str());

  if (line2.length() > 0)
    display.drawStr(0, 28, line2.c_str());

  if (line3.length() > 0)
    display.drawStr(0, 44, line3.c_str());

  if (line4.length() > 0)
    display.drawStr(0, 60, line4.c_str());

  display.sendBuffer();
}


void calibrateMPU(int samples = 200, int delayMs = 8) {

  oledShowMessage("Calibrating MPU...");

  double sum_ax = 0, sum_ay = 0, sum_az = 0;
  int valid = 0;

  for (int i = 0; i < samples; ++i) {
    int16_t ax_r, ay_r, az_r;

    if (readAccelRaw(ax_r, ay_r, az_r)) {
      sum_ax += (double)ax_r / 16384.0;
      sum_ay += (double)ay_r / 16384.0;
      sum_az += (double)az_r / 16384.0;
      valid++;
    }

    delay(delayMs);
  }

  float ax_mean = sum_ax / valid;
  float ay_mean = sum_ay / valid;
  float az_mean = sum_az / valid;

  ax_offset = ax_mean - 0.0;
  ay_offset = ay_mean - 0.0;
  az_offset = az_mean - 1.0;

  Serial.printf("Cal done ax=%.3f ay=%.3f az=%.3f\n",
                ax_offset, ay_offset, az_offset);
}


// ================= DASHBOARD PAGE HTML ==================
String dashboardPage() {
  String page = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Smart Barricade Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: 'Segoe UI', Tahoma, sans-serif; background:#2c3e50; color:white; text-align:center; margin:0; padding:0;}
      header { background:#1abc9c; padding:12px; font-size:20px; font-weight:600; }
      .container { padding:18px; }
      .road { margin: 20px auto; width: 700px; height:500px; background: linear-gradient(#2f3b46,#232a30); border-radius:18px; position:relative; overflow:hidden; box-shadow:0 8px 30px rgba(0,0,0,0.6);}
      .divider { position:absolute; top:0; bottom:0; left:50%; width:12px; background: repeating-linear-gradient( to bottom, #f1c40f, #f1c40f 24px, transparent 24px, transparent 48px); transform:translateX(-50%); border-radius:6px; box-shadow: 0 0 12px rgba(0,0,0,0.6); }
      .laneBox { position:absolute; top:30px; bottom:30px; width:45%; display:flex; flex-direction:column; align-items:center; justify-content:center; color:#eee; font-size:18px; }
      #laneA { left:5%; }
      #laneB { right:5%; }
      .stat { background:rgba(255,255,255,0.06); padding:12px 16px; border-radius:12px; margin:8px; min-width:180px; }
      .overlay { position:absolute; left:0; top:0; right:0; bottom:0; background:rgba(0,0,0,0.65); display:flex; align-items:center; justify-content:center; flex-direction:column; z-index:50; color:#fff; visibility:hidden; opacity:0; transition:opacity .25s; }
      .overlay.show { visibility:visible; opacity:1; }
      .overlay .title { font-size:28px; font-weight:700; color:#ffcc00; margin-bottom:8px; }
      .overlay .mag { font-size:22px; margin-bottom:10px; }
      .shakeAnim { width:300px; height:420px; background:linear-gradient(#666,#444); border-radius:12px; position:relative; }
      .shakeBar { position:absolute; left:50%; width:10px; background:#ffd; transform:translateX(-50%); top:-10%; height:120%; border-radius:6px; }
      footer { padding:10px; font-size:13px; color:#ddd; }
      @keyframes shakeX {
        0% { transform: translateX(-50%) translateY(0) rotate(0); }
        25% { transform: translateX(-52%) translateY(-3px) rotate(-1deg); }
        50% { transform: translateX(-48%) translateY(4px) rotate(1.2deg); }
        75% { transform: translateX(-51%) translateY(-2px) rotate(-0.6deg); }
        100% { transform: translateX(-50%) translateY(0) rotate(0); }
      }
    </style>

    <script>
      async function updateStatus() {
        try {
          let res = await fetch('/status');
          let data = await res.json();

          if (data.event.active) {
            document.getElementById('overlay').classList.add('show');
            document.getElementById('etype').innerText = data.event.type;
            document.getElementById('emag').innerText = data.event.magnitude.toFixed(3) + ' g';

            let speed = Math.max(60, 500 - Math.round(data.event.magnitude * 200));
            document.getElementById('shakeBar').style.animation = 'shakeX ' + speed + 'ms infinite';
          } else {
            document.getElementById('overlay').classList.remove('show');
          }

          document.getElementById('va').innerText = data.laneA.vehicle;
          document.getElementById('da').innerText = data.laneA.distance + ' cm';
          document.getElementById('ba').innerText = data.laneA.barricade;

          document.getElementById('vb').innerText = data.laneB.vehicle;
          document.getElementById('db').innerText = data.laneB.distance + ' cm';
          document.getElementById('bb').innerText = data.laneB.barricade;

        } catch (e) {
          console.log(e);
        }
      }

      setInterval(updateStatus, 800);
      window.onload = updateStatus;
    </script>
  </head>

  <body>
    <header> Smart Barricade Control</header>

    <div class="container">
      <div class="road">
        <div class="divider"></div>

        <div id="laneA" class="laneBox">
          <div class="stat">Lane A Vehicle: <span id="va">--</span></div>
          <div class="stat">Distance: <span id="da">--</span></div>
          <div class="stat">Barricade: <span id="ba">--</span></div>
        </div>

        <div id="laneB" class="laneBox">
          <div class="stat">Lane B Vehicle: <span id="vb">--</span></div>
          <div class="stat">Distance: <span id="db">--</span></div>
          <div class="stat">Barricade: <span id="bb">--</span></div>
        </div>

        <div id="overlay" class="overlay">
          <div class="title"> SEISMIC EVENT DETECTED</div>
          <div class="mag">
            <span id="etype">EARTHQUAKE</span> —
            Magnitude: <span id="emag">0.000 g</span>
          </div>

          <div class="shakeAnim">
            <div id="shakeBar" class="shakeBar"></div>
          </div>

          <div style="margin-top:12px;color:#ddd">
            All barricades are CLOSED for safety
          </div>
        </div>

      </div>

      <footer>ESP32 System • Barricade & Seismic Monitor</footer>
    </div>
  </body>
  </html>
  )rawliteral";

  return page;
}


// ================= STATUS JSON ENDPOINT ==================
void handleRoot() {
  server.send(200, "text/html", dashboardPage());
}

void handleStatus() {
  String json = "{";

  json += "\"laneA\":{\"vehicle\":\"" + String(laneA_IR ? "YES" : "NO") + "\",";
  json += "\"distance\":" + String(distA) + ",";
  json += "\"barricade\":\"" + String(barricadeA_Open ? "OPEN" : "CLOSED") + "\"},";

  json += "\"laneB\":{\"vehicle\":\"" + String(laneB_IR ? "YES" : "NO") + "\",";
  json += "\"distance\":" + String(distB) + ",";
  json += "\"barricade\":\"" + String(barricadeB_Open ? "OPEN" : "CLOSED") + "\"},";

  json += "\"event\":{\"active\":" + String(eventActive ? "true" : "false") + ",";
  json += "\"type\":\"" + (eventType.length() ? eventType : "NONE") + "\",";

  char buf[16];
  dtostrf(eventMagnitude, 6, 3, buf);

  json += "\"magnitude\":" + String(buf);
  json += "}}";

  server.send(200, "application/json", json);
}


// -------- Setup --------
void setup() {
  Serial.begin(115200);
  Serial.println("\n===== Smart Barricade System Booting =====");

  pinMode(TRIG_A, OUTPUT);
  pinMode(ECHO_A, INPUT);

  pinMode(TRIG_B, OUTPUT);
  pinMode(ECHO_B, INPUT);

  pinMode(IR_A, INPUT);
  pinMode(IR_B, INPUT);

  pinMode(BUZZER_A, OUTPUT);
  pinMode(BUZZER_B, OUTPUT);

  servoA.attach(SERVO_A_PIN);
  servoB.attach(SERVO_B_PIN);

  closeGateA();
  closeGateB();

  // -------- OLED / I2C Initialization --------
  Wire.begin(21, 22);
  display.begin();

  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "System Starting...");
  display.sendBuffer();

  delay(800);

  Serial.println("Initializing MPU6050...");

  if (!mpuBegin()) {
    Serial.println("MPU6050 init failed!");
  }
  else {
    Serial.println("MPU6050 ok, calibrating...");
    calibrateMPU();
  }

  Serial.print("Connecting to WiFi ");

  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "Connecting to WiFi...");
  display.sendBuffer();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 12, "WIFI connected...");
  display.sendBuffer();

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.begin();
}


// -------- Loop --------
void loop() {
  server.handleClient();

  float ax_g = 0, ay_g = 0, az_g = 0, mag_g = 0;

  static float lastRoll = 0, lastPitch = 0, filteredMag = 1.0;

  unsigned long now = millis();

  bool ok = readAccelG(ax_g, ay_g, az_g, mag_g);

  if (ok) {
    filteredMag = 0.9 * filteredMag + 0.1 * mag_g;

    float roll_deg, pitch_deg;

    accelToAngles(
      ax_g,
      ay_g,
      az_g,
      roll_deg,
      pitch_deg
    );

    float mag_dev = fabs(filteredMag - 1.0f);
    float dRoll = fabs(roll_deg - lastRoll);
    float dPitch = fabs(pitch_deg - lastPitch);

    lastRoll = roll_deg;
    lastPitch = pitch_deg;

    bool shakeDetected = mag_dev >= DETECT_G_THRESHOLD;
    bool tiltDetected =
      (dRoll >= TILT_CHANGE_DEG || dPitch >= TILT_CHANGE_DEG);

    if (!eventActive && (shakeDetected || tiltDetected)) {
      eventActive = true;
      eventStart = now;
      eventLastAbove = now;

      eventMagnitude = mag_dev;
      eventType = tiltDetected ? "LANDSLIDE" : "EARTHQUAKE";

      Serial.printf(
        "Event start type=%s mag=%.3f g\n",
        eventType.c_str(),
        eventMagnitude
      );

      closeGateA();
      closeGateB();

      // 🔔 Buzzer Alert
      for (int i = 0; i < 3; i++) {
        tone(BUZZER_A, 1500);
        tone(BUZZER_B, 1500);
        delay(300);

        noTone(BUZZER_A);
        noTone(BUZZER_B);

        delay(200);
      }

      // 📲 WhatsApp Alert
      String msg =
        "⚠️ ALERT: " +
        eventType +
        " detected!\nMagnitude: " +
        String(eventMagnitude, 3) +
        "g\nBarricades closed for safety.";

      sendWhatsAppMessage(msg);
    }

    if (eventActive) {
      if (mag_dev >= DETECT_G_THRESHOLD || tiltDetected)
        eventLastAbove = now;

      eventMagnitude =
        eventMagnitude * 0.85 + mag_dev * 0.15;

      if (
        mag_dev < STOP_G_THRESHOLD &&
        (now - eventLastAbove) > EVENT_HOLD_MS &&
        (now - eventStart) > EVENT_MIN_MS
      ) {
        Serial.printf(
          "Event end mag=%.3f\n",
          eventMagnitude
        );

        eventActive = false;
        eventType = "";
        eventMagnitude = 0.0;
      }
    }
  }

  // --- Normal lane logic unchanged ---
  distA = readDistanceCM(TRIG_A, ECHO_A);
  distB = readDistanceCM(TRIG_B, ECHO_B);

  laneA_IR = digitalRead(IR_A) == LOW;
  laneB_IR = digitalRead(IR_B) == LOW;

  if (eventActive) {
    closeGateA();
    closeGateB();
    noTone(BUZZER_A);
    noTone(BUZZER_B);
  }
  else {
    if (!laneA_IR && !laneB_IR) {
      closeGateA();
      closeGateB();
      noTone(BUZZER_A);
      noTone(BUZZER_B);
    }
    else if (laneA_IR && !laneB_IR) {
      if (millis() - lastDecisionTime > 1000) {
        openGateA();
        closeGateB();
        lastOpened = "A";
        lastDecisionTime = millis();
      }

      buzzAlert(BUZZER_B, distA);
    }
    else if (!laneA_IR && laneB_IR) {
      if (millis() - lastDecisionTime > 1000) {
        openGateB();
        closeGateA();
        lastOpened = "B";
        lastDecisionTime = millis();
      }

      buzzAlert(BUZZER_A, distB);
    }
    else if (laneA_IR && laneB_IR) {
      closeGateA();
      closeGateB();
      delay(2000);

      if (lastOpened == "A") {
        openGateB();
        closeGateA();

        while (digitalRead(IR_B) == LOW) {
          server.handleClient();
          delay(10);
        }

        closeGateB();
        lastOpened = "B";
      }
      else {
        openGateA();
        closeGateB();

        while (digitalRead(IR_A) == LOW) {
          server.handleClient();
          delay(10);
        }

        closeGateA();
        lastOpened = "A";
      }
    }
  }


  // -------- OLED Status Update --------
  // Same 900 ms timing as the original LCD
  if (millis() - lastLCDUpdate > 900) {

    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);

    if (eventActive) {

      // Original:
      // Line 1: !SEISMIC ALERT!
      // Line 2: EVENTTYPE:MAGg

      display.drawStr(0, 12, "!SEISMIC ALERT!");

      char b[12];
      dtostrf(eventMagnitude, 4, 3, b);

      String eventLine =
        eventType.substring(0, 8) +
        ":" +
        String(b) +
        "g";

      display.drawStr(0, 30, eventLine.c_str());

      // Extra clear safety status on OLED
      display.drawStr(0, 48, "Barricades: CLOSED");

    }
    else {

      // Same information as the original LCD:
      // A:distance cm OPN/CLS
      // B:distance cm OPN/CLS

      String lineA =
        "A:" +
        String(distA) +
        "cm " +
        (barricadeA_Open ? "OPN" : "CLS");

      String lineB =
        "B:" +
        String(distB) +
        "cm " +
        (barricadeB_Open ? "OPN" : "CLS");

      display.drawStr(0, 18, lineA.c_str());
      display.drawStr(0, 38, lineB.c_str());
    }

    display.sendBuffer();

    lastLCDUpdate = millis();
  }
}
