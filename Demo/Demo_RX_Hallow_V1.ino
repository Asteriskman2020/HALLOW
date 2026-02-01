/**
 * Demo_RX_Hallow_V1 - ESP32-C6 HT-HC01P WiFi HaLow RX Demo
 *
 * Raw SPI communication to HT-HC01P (Morse Micro MM6108) module.
 * Reads data from the module every 2 seconds as proof-of-concept.
 *
 * Features:
 *   - SPI RX from HT-HC01P (MISO=GPIO0, MOSI=GPIO1, CS=GPIO2, CLK=GPIO20)
 *   - WiFi + MQTT web config portal (AP mode captive portal)
 *   - OTA firmware updates (ArduinoOTA)
 *   - Colorful green-themed web dashboard with status cards
 *   - GREEN built-in LED (GPIO8) flashes on each RX
 *   - MQTT publishing of RX status/count/data
 *   - Config saved to NVS (Preferences)
 *   - RAM optimized: PROGMEM HTML, F() macros, stack buffers
 *
 * Hardware: ESP32-C6 Super-mini + HT-HC01P WiFi HaLow module
 * FQBN: esp32:esp32:esp32c6
 *
 * Note: Full WiFi HaLow protocol requires Morse Micro SDK (ESP-IDF).
 *       This demo reads raw SPI frames as a framework/proof-of-concept.
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
#define LED_PIN     8    // ESP32-C6 Super-mini built-in LED (GREEN)
#define SPI_MISO    0    // HT-HC01P MISO
#define SPI_MOSI    1    // HT-HC01P MOSI
#define SPI_CS      2    // HT-HC01P CS
#define SPI_CLK     20   // HT-HC01P CLK

// ==================== TIMING ====================
#define RX_INTERVAL         2000
#define LED_FLASH_MS        80
#define MQTT_RECONNECT_MS   5000
#define WIFI_CONNECT_MS     15000
#define AP_TIMEOUT_MS       300000  // 5 min AP portal timeout

// ==================== RX BUFFER ====================
#define RX_BUF_LEN  32
static uint8_t rxBuf[RX_BUF_LEN];
static uint8_t rxLen = 0;

// ==================== AP CONFIG ====================
static const char AP_SSID[] PROGMEM = "HaLow-RX-Config";
static const char AP_PASS[] PROGMEM = "halow1234";

// ==================== OBJECTS ====================
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Preferences prefs;
SPIClass hspi(HSPI);

// ==================== CONFIG (from NVS) ====================
char cfgSSID[33]       = "";
char cfgPass[65]       = "";
char cfgMqttServer[65] = "";
uint16_t cfgMqttPort   = 1883;
char cfgMqttTopic[65]  = "halow/rx";
char cfgMqttUser[33]   = "";
char cfgMqttPass[65]   = "";

// ==================== STATE ====================
bool apMode = false;
bool wifiConnected = false;
unsigned long rxCount = 0;
unsigned long lastRx = 0;
unsigned long lastLedOff = 0;
unsigned long lastMqttReconnect = 0;
unsigned long apStartTime = 0;
bool ledOn = false;
int lastSpiResult = -1;  // -1=not yet, 0=ok(data), 1=ok(empty), 2=fail
char lastRxData[RX_BUF_LEN * 3 + 1];  // hex display buffer
char lastRxAscii[RX_BUF_LEN + 1];     // ascii display buffer

// ==================== HTML PAGES (PROGMEM) ====================
static const char HTML_DASHBOARD[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HaLow RX Dashboard</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',sans-serif;background:linear-gradient(135deg,#0a1628,#132743,#1a3a5c);color:#fff;min-height:100vh;padding:20px}
h1{text-align:center;font-size:1.8em;margin-bottom:8px;background:linear-gradient(90deg,#00e676,#00bfa5);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;color:#aaa;margin-bottom:20px;font-size:0.9em}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;max-width:960px;margin:0 auto}
.card{background:rgba(255,255,255,0.06);backdrop-filter:blur(10px);border-radius:16px;padding:20px;border:1px solid rgba(0,230,118,0.15);transition:transform 0.2s}
.card:hover{transform:translateY(-4px);border-color:rgba(0,230,118,0.4)}
.card h3{font-size:0.8em;color:#78909c;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}
.card .val{font-size:1.8em;font-weight:bold}
.wide{grid-column:1/-1}
.wide .val{font-size:1.1em;font-family:'Courier New',monospace;word-break:break-all;line-height:1.6}
.ok{color:#00e676}.err{color:#ff5252}.warn{color:#ffab40}.idle{color:#78909c}
.btn-row{text-align:center;margin-top:20px}
a.btn{display:inline-block;padding:10px 24px;background:linear-gradient(135deg,#00e676,#00bfa5);color:#0a1628;text-decoration:none;border-radius:30px;font-weight:700;margin:4px;transition:opacity 0.2s}
a.btn:hover{opacity:0.8}
.footer{text-align:center;color:#546e7a;margin-top:30px;font-size:0.8em}
.pulse{animation:pulse 1.5s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
</style></head><body>
<h1>HaLow RX Dashboard</h1>
<p class="sub">ESP32-C6 + HT-HC01P WiFi HaLow Receiver</p>
<div class="grid">
<div class="card"><h3>RX Count</h3><div class="val ok" id="rx">--</div></div>
<div class="card"><h3>SPI Status</h3><div class="val" id="spi">--</div></div>
<div class="card"><h3>WiFi RSSI</h3><div class="val" id="rssi">--</div></div>
<div class="card"><h3>MQTT</h3><div class="val" id="mqtt">--</div></div>
<div class="card"><h3>Uptime</h3><div class="val" id="up">--</div></div>
<div class="card"><h3>Free Heap</h3><div class="val" id="heap">--</div></div>
<div class="card wide"><h3>Last RX HEX</h3><div class="val" id="hex">--</div></div>
<div class="card wide"><h3>Last RX ASCII</h3><div class="val ok" id="ascii">--</div></div>
</div>
<div class="btn-row">
<a class="btn" href="/config">Config</a>
<a class="btn" href="/reboot">Reboot</a>
</div>
<p class="footer">HaLow RX V1 | Hallow_2 | OTA Enabled</p>
<script>
function u(){fetch('/api/status').then(r=>r.json()).then(d=>{
document.getElementById('rx').textContent=d.rx;
var s=document.getElementById('spi');
if(d.spi==='DATA'){s.textContent='DATA';s.className='val ok';}
else if(d.spi==='EMPTY'){s.textContent='EMPTY';s.className='val warn';}
else if(d.spi==='IDLE'){s.textContent='IDLE';s.className='val idle';}
else{s.textContent='FAIL';s.className='val err';}
document.getElementById('rssi').innerHTML=d.rssi+' <small>dBm</small>';
var m=document.getElementById('mqtt');m.textContent=d.mqtt?'Connected':'Disconnected';m.className='val '+(d.mqtt?'ok':'err');
document.getElementById('up').textContent=d.uptime;
document.getElementById('heap').innerHTML=d.heap+' <small>B</small>';
document.getElementById('hex').textContent=d.hex||'--';
document.getElementById('ascii').textContent=d.ascii||'--';
});}
u();setInterval(u,2000);
</script></body></html>
)rawliteral";

static const char HTML_CONFIG[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HaLow RX Config</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',sans-serif;background:linear-gradient(135deg,#0a1628,#132743,#1a3a5c);color:#fff;min-height:100vh;padding:20px;display:flex;justify-content:center;align-items:center}
.box{background:rgba(255,255,255,0.06);backdrop-filter:blur(12px);border-radius:20px;padding:30px;max-width:420px;width:100%;border:1px solid rgba(0,230,118,0.15)}
h2{text-align:center;margin-bottom:20px;background:linear-gradient(90deg,#00e676,#00bfa5);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
label{display:block;color:#78909c;font-size:0.85em;margin:12px 0 4px;text-transform:uppercase;letter-spacing:1px}
input{width:100%;padding:10px 14px;border:1px solid rgba(0,230,118,0.15);border-radius:10px;background:rgba(255,255,255,0.05);color:#fff;font-size:1em;outline:none}
input:focus{border-color:#00e676}
button{width:100%;margin-top:20px;padding:12px;border:none;border-radius:30px;background:linear-gradient(135deg,#00e676,#00bfa5);color:#0a1628;font-size:1em;font-weight:700;cursor:pointer}
button:hover{opacity:0.85}
.back{display:block;text-align:center;margin-top:14px;color:#78909c;font-size:0.9em}
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
body{font-family:'Segoe UI',sans-serif;background:#0a1628;color:#fff;display:flex;justify-content:center;align-items:center;height:100vh}
.box{text-align:center;background:rgba(255,255,255,0.06);padding:40px;border-radius:20px;border:1px solid rgba(0,230,118,0.2)}
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
void spiReceive();
void handleRoot();
void handleConfig();
void handleSave();
void handleStatus();
void handleReboot();

// ==================== FORMAT UPTIME (stack buffer) ====================
void formatUptime(unsigned long ms, char* out, uint8_t len) {
  unsigned long s = ms / 1000;
  unsigned long m = s / 60;
  unsigned long h = m / 60;
  snprintf(out, len, "%luh %lum %lus", h, m % 60, s % 60);
}

// ==================== LOAD CONFIG FROM NVS ====================
void loadConfig() {
  prefs.begin("halowrx", true);
  size_t n;
  n = prefs.getString("ssid", cfgSSID, sizeof(cfgSSID));
  n = prefs.getString("pass", cfgPass, sizeof(cfgPass));
  n = prefs.getString("mqttSrv", cfgMqttServer, sizeof(cfgMqttServer));
  cfgMqttPort = prefs.getUShort("mqttPort", 1883);
  n = prefs.getString("mqttTopic", cfgMqttTopic, sizeof(cfgMqttTopic));
  if (cfgMqttTopic[0] == '\0') strncpy(cfgMqttTopic, "halow/rx", sizeof(cfgMqttTopic));
  n = prefs.getString("mqttUser", cfgMqttUser, sizeof(cfgMqttUser));
  n = prefs.getString("mqttPass", cfgMqttPass, sizeof(cfgMqttPass));
  (void)n;
  prefs.end();
}

// ==================== SAVE CONFIG TO NVS ====================
void saveConfig() {
  prefs.begin("halowrx", false);
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
  WiFi.setHostname("Hallow_2");
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
  ArduinoOTA.setHostname("Hallow_2");
  ArduinoOTA.onStart([]() { Serial.println(F("OTA Start")); });
  ArduinoOTA.onEnd([]()   { Serial.println(F("OTA Done")); });
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
  if (apMode) { handleConfig(); return; }
  server.send_P(200, "text/html", HTML_DASHBOARD);
}

void handleConfig() {
  char buf[16];
  String page = FPSTR(HTML_CONFIG);
  page.replace(F("%SSID%"), cfgSSID);
  page.replace(F("%PASS%"), cfgPass);
  page.replace(F("%MS%"), cfgMqttServer);
  snprintf(buf, sizeof(buf), "%u", cfgMqttPort);
  page.replace(F("%MP%"), buf);
  page.replace(F("%MT%"), cfgMqttTopic);
  page.replace(F("%MU%"), cfgMqttUser);
  page.replace(F("%MW%"), cfgMqttPass);
  server.send(200, "text/html", page);
}

void handleSave() {
  if (server.hasArg(F("ssid"))) server.arg(F("ssid")).toCharArray(cfgSSID, sizeof(cfgSSID));
  if (server.hasArg(F("pass"))) server.arg(F("pass")).toCharArray(cfgPass, sizeof(cfgPass));
  if (server.hasArg(F("ms")))   server.arg(F("ms")).toCharArray(cfgMqttServer, sizeof(cfgMqttServer));
  if (server.hasArg(F("mp")))   cfgMqttPort = server.arg(F("mp")).toInt();
  if (server.hasArg(F("mt")))   server.arg(F("mt")).toCharArray(cfgMqttTopic, sizeof(cfgMqttTopic));
  if (server.hasArg(F("mu")))   server.arg(F("mu")).toCharArray(cfgMqttUser, sizeof(cfgMqttUser));
  if (server.hasArg(F("mw")))   server.arg(F("mw")).toCharArray(cfgMqttPass, sizeof(cfgMqttPass));

  saveConfig();
  server.send_P(200, "text/html", HTML_SAVED);
  delay(3000);
  ESP.restart();
}

void handleStatus() {
  char buf[384];
  char uptBuf[20];
  formatUptime(millis(), uptBuf, sizeof(uptBuf));

  const char* spiStr;
  switch (lastSpiResult) {
    case 0:  spiStr = "DATA";  break;
    case 1:  spiStr = "EMPTY"; break;
    case 2:  spiStr = "FAIL";  break;
    default: spiStr = "IDLE";  break;
  }

  snprintf(buf, sizeof(buf),
    "{\"rx\":%lu,\"spi\":\"%s\",\"rssi\":%d,\"mqtt\":%s,"
    "\"uptime\":\"%s\",\"heap\":%lu,\"hex\":\"%s\",\"ascii\":\"%s\"}",
    rxCount, spiStr,
    (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0,
    mqtt.connected() ? "true" : "false",
    uptBuf, (unsigned long)ESP.getFreeHeap(),
    lastRxData, lastRxAscii
  );
  server.send(200, F("application/json"), buf);
}

void handleReboot() {
  server.send_P(200, "text/html",
    "<html><body style='background:#0a1628;color:#fff;text-align:center;"
    "padding-top:40vh;font-family:sans-serif'><h2>Rebooting...</h2></body></html>");
  delay(1000);
  ESP.restart();
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
    ok = mqtt.connect("Hallow_2", cfgMqttUser, cfgMqttPass);
  } else {
    ok = mqtt.connect("Hallow_2");
  }

  if (ok) {
    Serial.println(F(" connected"));
  } else {
    Serial.print(F(" failed, rc="));
    Serial.println(mqtt.state());
  }
}

// ==================== SPI RECEIVE ====================
void spiReceive() {
  memset(rxBuf, 0, RX_BUF_LEN);

  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);

  hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // Send dummy 0x00 bytes to clock data in from MISO
  for (uint8_t i = 0; i < RX_BUF_LEN; i++) {
    rxBuf[i] = hspi.transfer(0x00);
  }
  hspi.endTransaction();

  digitalWrite(SPI_CS, HIGH);

  // Check if we got any non-zero data
  bool hasData = false;
  rxLen = 0;
  for (uint8_t i = 0; i < RX_BUF_LEN; i++) {
    if (rxBuf[i] != 0x00 && rxBuf[i] != 0xFF) {
      hasData = true;
      rxLen = i + 1;
    }
  }

  // Build hex string
  lastRxData[0] = '\0';
  lastRxAscii[0] = '\0';
  if (hasData && rxLen > 0) {
    char* p = lastRxData;
    char* a = lastRxAscii;
    for (uint8_t i = 0; i < rxLen && i < RX_BUF_LEN; i++) {
      p += sprintf(p, "%02X ", rxBuf[i]);
      *a++ = (rxBuf[i] >= 0x20 && rxBuf[i] <= 0x7E) ? (char)rxBuf[i] : '.';
    }
    *a = '\0';
    if (p > lastRxData) *(p - 1) = '\0';  // trim trailing space
  }

  if (hasData) {
    lastSpiResult = 0;  // DATA
    rxCount++;

    // Flash GREEN LED
    digitalWrite(LED_PIN, LOW);  // Active LOW
    ledOn = true;
    lastLedOff = millis() + LED_FLASH_MS;

    // Serial log
    Serial.print(F("SPI RX #"));
    Serial.print(rxCount);
    Serial.print(F(": HEX="));
    Serial.print(lastRxData);
    Serial.print(F(" ASCII="));
    Serial.println(lastRxAscii);

    // Publish MQTT
    if (mqtt.connected()) {
      char buf[128];
      snprintf(buf, sizeof(buf),
        "{\"rx\":%lu,\"hex\":\"%s\",\"ascii\":\"%s\"}",
        rxCount, lastRxData, lastRxAscii);
      mqtt.publish(cfgMqttTopic, buf);
    }
  } else {
    lastSpiResult = 1;  // EMPTY
    Serial.print(F("SPI RX: empty ("));
    Serial.print(rxBuf[0], HEX);
    Serial.println(')');
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F(" HaLow RX Demo V1 - ESP32-C6"));
  Serial.println(F(" Hostname: Hallow_2"));
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

  // Init display buffers
  lastRxData[0] = '\0';
  lastRxAscii[0] = '\0';

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

  // SPI RX every 2 seconds
  if (millis() - lastRx >= RX_INTERVAL) {
    lastRx = millis();
    spiReceive();
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
