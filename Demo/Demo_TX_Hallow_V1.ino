/**
 * Demo_TX_Hallow_V1 - ESP32-C6 HT-HC01P WiFi HaLow TX Demo
 *
 * Raw SPI communication to HT-HC01P (Morse Micro MM6108) module.
 * Transmits "HELLO" payload every 2 seconds as proof-of-concept.
 *
 * Features:
 *   - SPI TX to HT-HC01P (MISO=GPIO0, MOSI=GPIO1, CS=GPIO2, CLK=GPIO20)
 *   - WiFi + MQTT web config portal (AP mode captive portal)
 *   - OTA firmware updates (ArduinoOTA)
 *   - Colorful web dashboard with status cards
 *   - RED built-in LED (GPIO8) flashes on each TX
 *   - MQTT publishing of TX status/count
 *   - Config saved to NVS (Preferences)
 *
 * Hardware: ESP32-C6 Super-mini + HT-HC01P WiFi HaLow module
 * FQBN: esp32:esp32:esp32c6
 *
 * Note: Full WiFi HaLow protocol requires Morse Micro SDK (ESP-IDF).
 *       This demo sends raw SPI frames as a framework/proof-of-concept.
 *
 * Date: 01 Feb 2026
 * Version: 1.0
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <SPI.h>

// ==================== PIN DEFINITIONS ====================
#define LED_PIN     8    // ESP32-C6 Super-mini built-in RED LED
#define SPI_MISO    0    // HT-HC01P MISO
#define SPI_MOSI    1    // HT-HC01P MOSI
#define SPI_CS      2    // HT-HC01P CS
#define SPI_CLK     20   // HT-HC01P CLK

// ==================== TIMING ====================
#define TX_INTERVAL         2000
#define LED_FLASH_MS        50
#define MQTT_RECONNECT_MS   5000
#define WIFI_CONNECT_MS     15000
#define AP_TIMEOUT_MS       300000  // 5 min AP portal timeout

// ==================== AP CONFIG ====================
static const char AP_SSID[] PROGMEM = "HaLow-Config";
static const char AP_PASS[] PROGMEM = "halow1234";

// ==================== OBJECTS ====================
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Preferences prefs;
SPIClass hspi(HSPI);

// ==================== CONFIG (from NVS) ====================
char cfgSSID[33]      = "";
char cfgPass[65]      = "";
char cfgMqttServer[65] = "";
uint16_t cfgMqttPort  = 1883;
char cfgMqttTopic[65] = "halow/tx";
char cfgMqttUser[33]  = "";
char cfgMqttPass[65]  = "";

// ==================== STATE ====================
bool apMode = false;
bool wifiConnected = false;
bool mqttConnected = false;
unsigned long txCount = 0;
unsigned long lastTx = 0;
unsigned long lastLedOff = 0;
unsigned long lastMqttReconnect = 0;
unsigned long apStartTime = 0;
bool ledOn = false;
int lastSpiResult = -1;  // -1=not yet, 0=ok, 1=fail

// ==================== SPI TX PAYLOAD ====================
static const uint8_t HELLO_PAYLOAD[] = {'H', 'E', 'L', 'L', 'O'};
#define HELLO_LEN 5

// ==================== HTML PAGES (PROGMEM) ====================
static const char HTML_DASHBOARD[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HaLow TX Dashboard</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',sans-serif;background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);color:#fff;min-height:100vh;padding:20px}
h1{text-align:center;font-size:1.8em;margin-bottom:8px;background:linear-gradient(90deg,#f093fb,#f5576c);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;color:#aaa;margin-bottom:20px;font-size:0.9em}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;max-width:900px;margin:0 auto}
.card{background:rgba(255,255,255,0.08);backdrop-filter:blur(10px);border-radius:16px;padding:20px;border:1px solid rgba(255,255,255,0.1);transition:transform 0.2s}
.card:hover{transform:translateY(-4px)}
.card h3{font-size:0.85em;color:#aaa;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}
.card .val{font-size:2em;font-weight:bold}
.ok{color:#00e676}.err{color:#ff5252}.warn{color:#ffab40}
.btn-row{text-align:center;margin-top:20px}
a.btn{display:inline-block;padding:10px 24px;background:linear-gradient(135deg,#667eea,#764ba2);color:#fff;text-decoration:none;border-radius:30px;font-weight:600;margin:4px}
a.btn:hover{opacity:0.85}
.footer{text-align:center;color:#666;margin-top:30px;font-size:0.8em}
</style></head><body>
<h1>HaLow TX Dashboard</h1>
<p class="sub">ESP32-C6 + HT-HC01P WiFi HaLow Demo</p>
<div class="grid">
<div class="card"><h3>TX Count</h3><div class="val" id="tx">--</div></div>
<div class="card"><h3>SPI Status</h3><div class="val" id="spi">--</div></div>
<div class="card"><h3>WiFi RSSI</h3><div class="val" id="rssi">--</div></div>
<div class="card"><h3>MQTT</h3><div class="val" id="mqtt">--</div></div>
<div class="card"><h3>Uptime</h3><div class="val" id="up">--</div></div>
<div class="card"><h3>Free Heap</h3><div class="val" id="heap">--</div></div>
</div>
<div class="btn-row">
<a class="btn" href="/config">Config</a>
<a class="btn" href="/reboot">Reboot</a>
</div>
<p class="footer">HaLow TX V1 | OTA Enabled</p>
<script>
function u(){fetch('/api/status').then(r=>r.json()).then(d=>{
document.getElementById('tx').textContent=d.tx;
var s=document.getElementById('spi');s.textContent=d.spi;s.className='val '+(d.spi==='OK'?'ok':'err');
document.getElementById('rssi').innerHTML=d.rssi+' <small>dBm</small>';
var m=document.getElementById('mqtt');m.textContent=d.mqtt?'Connected':'Disconnected';m.className='val '+(d.mqtt?'ok':'err');
document.getElementById('up').textContent=d.uptime;
document.getElementById('heap').innerHTML=d.heap+' <small>B</small>';
});}
u();setInterval(u,2000);
</script></body></html>
)rawliteral";

static const char HTML_CONFIG[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HaLow Config</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',sans-serif;background:linear-gradient(135deg,#1a1a2e,#16213e,#0f3460);color:#fff;min-height:100vh;padding:20px;display:flex;justify-content:center;align-items:center}
.box{background:rgba(255,255,255,0.06);backdrop-filter:blur(12px);border-radius:20px;padding:30px;max-width:420px;width:100%;border:1px solid rgba(255,255,255,0.1)}
h2{text-align:center;margin-bottom:20px;background:linear-gradient(90deg,#00d2ff,#3a7bd5);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
label{display:block;color:#aaa;font-size:0.85em;margin:12px 0 4px;text-transform:uppercase;letter-spacing:1px}
input{width:100%;padding:10px 14px;border:1px solid rgba(255,255,255,0.15);border-radius:10px;background:rgba(255,255,255,0.05);color:#fff;font-size:1em;outline:none}
input:focus{border-color:#3a7bd5}
button{width:100%;margin-top:20px;padding:12px;border:none;border-radius:30px;background:linear-gradient(135deg,#00d2ff,#3a7bd5);color:#fff;font-size:1em;font-weight:600;cursor:pointer}
button:hover{opacity:0.85}
.back{display:block;text-align:center;margin-top:14px;color:#aaa;font-size:0.9em}
</style></head><body>
<div class="box">
<h2>WiFi + MQTT Config</h2>
<form method="POST" action="/save">
<label>WiFi SSID</label><input name="ssid" value="%SSID%">
<label>WiFi Password</label><input name="pass" type="password" value="%PASS%">
<label>MQTT Server</label><input name="ms" value="%MS%">
<label>MQTT Port</label><input name="mp" type="number" value="%MP%">
<label>MQTT Topic</label><input name="mt" value="%MT%">
<label>MQTT User</label><input name="mu" value="%MU%">
<label>MQTT Password</label><input name="mw" type="password" value="%MW%">
<button type="submit">Save & Reboot</button>
</form>
<a class="back" href="/">Back to Dashboard</a>
</div></body></html>
)rawliteral";

static const char HTML_SAVED[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Saved</title>
<style>
body{font-family:'Segoe UI',sans-serif;background:#1a1a2e;color:#fff;display:flex;justify-content:center;align-items:center;height:100vh}
.box{text-align:center;background:rgba(255,255,255,0.06);padding:40px;border-radius:20px;border:1px solid rgba(255,255,255,0.1)}
h2{color:#00e676;margin-bottom:10px}
</style></head><body>
<div class="box"><h2>Config Saved!</h2><p>Rebooting in 3 seconds...</p></div>
</body></html>
)rawliteral";

// ==================== FORWARD DECLARATIONS ====================
void loadConfig();
void saveConfig();
void startAP();
void connectWiFi();
void setupOTA();
void setupWebServer();
void setupMQTT();
void mqttReconnect();
void spiTransmit();
void handleRoot();
void handleConfig();
void handleSave();
void handleStatus();
void handleReboot();
String formatUptime(unsigned long ms);

// ==================== LOAD CONFIG FROM NVS ====================
void loadConfig() {
  prefs.begin("halow", true);
  String s;
  s = prefs.getString("ssid", "");      s.toCharArray(cfgSSID, sizeof(cfgSSID));
  s = prefs.getString("pass", "");      s.toCharArray(cfgPass, sizeof(cfgPass));
  s = prefs.getString("mqttSrv", "");   s.toCharArray(cfgMqttServer, sizeof(cfgMqttServer));
  cfgMqttPort = prefs.getUShort("mqttPort", 1883);
  s = prefs.getString("mqttTopic", "halow/tx"); s.toCharArray(cfgMqttTopic, sizeof(cfgMqttTopic));
  s = prefs.getString("mqttUser", "");  s.toCharArray(cfgMqttUser, sizeof(cfgMqttUser));
  s = prefs.getString("mqttPass", "");  s.toCharArray(cfgMqttPass, sizeof(cfgMqttPass));
  prefs.end();
}

// ==================== SAVE CONFIG TO NVS ====================
void saveConfig() {
  prefs.begin("halow", false);
  prefs.putString("ssid", cfgSSID);
  prefs.putString("pass", cfgPass);
  prefs.putString("mqttSrv", cfgMqttServer);
  prefs.putUShort("mqttPort", cfgMqttPort);
  prefs.putString("mqttTopic", cfgMqttTopic);
  prefs.putString("mqttUser", cfgMqttUser);
  prefs.putString("mqttPass", cfgMqttPass);
  prefs.end();
}

// ==================== START AP MODE ====================
void startAP() {
  apMode = true;
  apStartTime = millis();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(FPSTR(AP_SSID), FPSTR(AP_PASS));
  Serial.print(F("AP started: "));
  Serial.println(FPSTR(AP_SSID));
  Serial.print(F("AP IP: "));
  Serial.println(WiFi.softAPIP());
}

// ==================== CONNECT WIFI (STA) ====================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("Hallow_1");
  WiFi.begin(cfgSSID, cfgPass);
  Serial.print(F("Connecting WiFi"));

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_MS) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print(F("WiFi connected: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi failed, starting AP mode"));
    startAP();
  }
}

// ==================== SETUP OTA ====================
void setupOTA() {
  ArduinoOTA.setHostname("Hallow_1");
  ArduinoOTA.onStart([]() {
    Serial.println(F("OTA Start"));
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("OTA Done"));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print(F("OTA Error: "));
    Serial.println(error);
  });
  ArduinoOTA.begin();
}

// ==================== SETUP WEB SERVER ====================
void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/api/status", handleStatus);
  server.on("/reboot", handleReboot);
  server.onNotFound(handleRoot);
  server.begin();
  Serial.println(F("Web server started"));
}

// ==================== WEB HANDLERS ====================
void handleRoot() {
  if (apMode) {
    handleConfig();
    return;
  }
  server.send_P(200, "text/html", HTML_DASHBOARD);
}

void handleConfig() {
  // Build config page with current values substituted
  String page = FPSTR(HTML_CONFIG);
  page.replace(F("%SSID%"), cfgSSID);
  page.replace(F("%PASS%"), cfgPass);
  page.replace(F("%MS%"), cfgMqttServer);
  page.replace(F("%MP%"), String(cfgMqttPort));
  page.replace(F("%MT%"), cfgMqttTopic);
  page.replace(F("%MU%"), cfgMqttUser);
  page.replace(F("%MW%"), cfgMqttPass);
  server.send(200, "text/html", page);
}

void handleSave() {
  if (server.hasArg("ssid")) server.arg("ssid").toCharArray(cfgSSID, sizeof(cfgSSID));
  if (server.hasArg("pass")) server.arg("pass").toCharArray(cfgPass, sizeof(cfgPass));
  if (server.hasArg("ms"))   server.arg("ms").toCharArray(cfgMqttServer, sizeof(cfgMqttServer));
  if (server.hasArg("mp"))   cfgMqttPort = server.arg("mp").toInt();
  if (server.hasArg("mt"))   server.arg("mt").toCharArray(cfgMqttTopic, sizeof(cfgMqttTopic));
  if (server.hasArg("mu"))   server.arg("mu").toCharArray(cfgMqttUser, sizeof(cfgMqttUser));
  if (server.hasArg("mw"))   server.arg("mw").toCharArray(cfgMqttPass, sizeof(cfgMqttPass));

  saveConfig();
  server.send_P(200, "text/html", HTML_SAVED);
  delay(3000);
  ESP.restart();
}

void handleStatus() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"tx\":%lu,\"spi\":\"%s\",\"rssi\":%d,\"mqtt\":%s,\"uptime\":\"%s\",\"heap\":%lu}",
    txCount,
    (lastSpiResult == 0) ? "OK" : (lastSpiResult < 0 ? "IDLE" : "FAIL"),
    (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0,
    mqtt.connected() ? "true" : "false",
    formatUptime(millis()).c_str(),
    (unsigned long)ESP.getFreeHeap()
  );
  server.send(200, "application/json", buf);
}

void handleReboot() {
  server.send(200, "text/html", "<html><body style='background:#1a1a2e;color:#fff;text-align:center;padding-top:40vh;font-family:sans-serif'><h2>Rebooting...</h2></body></html>");
  delay(1000);
  ESP.restart();
}

// ==================== FORMAT UPTIME ====================
String formatUptime(unsigned long ms) {
  unsigned long s = ms / 1000;
  unsigned long m = s / 60;
  unsigned long h = m / 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%luh %lum %lus", h, m % 60, s % 60);
  return String(buf);
}

// ==================== SETUP MQTT ====================
void setupMQTT() {
  if (cfgMqttServer[0] == '\0') return;
  mqtt.setServer(cfgMqttServer, cfgMqttPort);
  Serial.print(F("MQTT configured: "));
  Serial.print(cfgMqttServer);
  Serial.print(':');
  Serial.println(cfgMqttPort);
}

// ==================== MQTT RECONNECT ====================
void mqttReconnect() {
  if (cfgMqttServer[0] == '\0') return;
  if (mqtt.connected()) return;
  if (millis() - lastMqttReconnect < MQTT_RECONNECT_MS) return;
  lastMqttReconnect = millis();

  Serial.print(F("MQTT connecting..."));
  bool ok;
  if (cfgMqttUser[0] != '\0') {
    ok = mqtt.connect("Hallow_1", cfgMqttUser, cfgMqttPass);
  } else {
    ok = mqtt.connect("Hallow_1");
  }

  if (ok) {
    mqttConnected = true;
    Serial.println(F(" connected"));
  } else {
    mqttConnected = false;
    Serial.print(F(" failed, rc="));
    Serial.println(mqtt.state());
  }
}

// ==================== SPI TRANSMIT ====================
void spiTransmit() {
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);

  hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  for (uint8_t i = 0; i < HELLO_LEN; i++) {
    hspi.transfer(HELLO_PAYLOAD[i]);
  }
  hspi.endTransaction();

  digitalWrite(SPI_CS, HIGH);
  lastSpiResult = 0;
  txCount++;

  // Flash LED
  digitalWrite(LED_PIN, LOW);  // Active LOW on ESP32-C6 Super-mini
  ledOn = true;
  lastLedOff = millis() + LED_FLASH_MS;

  // Serial log
  Serial.print(F("SPI TX #"));
  Serial.print(txCount);
  Serial.println(F(": HELLO"));

  // Publish MQTT
  if (mqtt.connected()) {
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"tx\":%lu,\"msg\":\"HELLO\"}", txCount);
    mqtt.publish(cfgMqttTopic, buf);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F(" HaLow TX Demo V1 - ESP32-C6"));
  Serial.println(F("================================"));

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // OFF (active LOW)

  // SPI CS
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  // Init SPI with custom pins
  hspi.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);
  Serial.println(F("SPI initialized (CLK=20, MISO=0, MOSI=1, CS=2)"));

  // Load config from NVS
  loadConfig();
  Serial.print(F("Config SSID: "));
  Serial.println(cfgSSID);

  // Connect WiFi or start AP
  if (cfgSSID[0] != '\0') {
    connectWiFi();
  } else {
    Serial.println(F("No WiFi config, starting AP"));
    startAP();
  }

  // OTA
  if (wifiConnected) {
    setupOTA();
  }

  // Web server (works in both AP and STA mode)
  setupWebServer();

  // MQTT
  if (wifiConnected) {
    setupMQTT();
  }

  Serial.println(F("Setup complete"));
  Serial.println(F("================================"));
}

// ==================== LOOP ====================
void loop() {
  // OTA
  if (wifiConnected) {
    ArduinoOTA.handle();
  }

  // Web server
  server.handleClient();

  // MQTT
  if (wifiConnected) {
    if (!mqtt.connected()) {
      mqttReconnect();
    }
    mqtt.loop();
  }

  // SPI TX every 2 seconds
  if (millis() - lastTx >= TX_INTERVAL) {
    lastTx = millis();
    spiTransmit();
  }

  // LED off after flash
  if (ledOn && millis() >= lastLedOff) {
    digitalWrite(LED_PIN, HIGH);  // OFF
    ledOn = false;
  }

  // AP timeout: reboot after 5 minutes in AP mode
  if (apMode && millis() - apStartTime > AP_TIMEOUT_MS) {
    Serial.println(F("AP timeout, rebooting"));
    ESP.restart();
  }
}
