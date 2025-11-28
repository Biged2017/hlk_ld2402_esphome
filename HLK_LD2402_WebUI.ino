/*
  HLK-LD2402 Web UI + Graphs for ESP32
  ------------------------------------
  - Интерфейс на русском
  - Графики (Chart.js) через браузер
  - Сохранение настроек в Preferences (энергонезависимо)
  - HEX-лог входящих UART-пакетов
  - Отправка управляющих команд на модуль
  - Поддерживает базовые команды: sensitivity, hold time, powersave (можно править)
  - Если формат сообщений вашего HLK отличается — зайдите в HEX-log и пришлите примеры,
    я подправлю парсер.

  Подключение:
    HLK TX -> ESP32 RX1 (GPIO16)
    HLK RX -> ESP32 TX1 (GPIO17)
    GND общий
    VCC -> 5V (проверьте ревизию модуля!)
*/

#include <WiFi.h>
#include <WebServer.h>           // Встроенный web сервер
#include <Preferences.h>        // Для сохранения настроек в NVS
#include <ArduinoJson.h>        // Для формирования JSON (включено в Arduino core)
#include <vector>

/////////////////////
// Настройки WiFi  //
/////////////////////
const char* WIFI_SSID = "<ВАШ_SSID>";
const char* WIFI_PASS = "<ВАШ_PASSWORD>";

/////////////////////
// UART (RADAR)     //
/////////////////////
// Используем Serial1: RX1 = GPIO16, TX1 = GPIO17 (можно изменить)
HardwareSerial RadarSerial(1);
const int RADAR_RX_PIN = 16;
const int RADAR_TX_PIN = 17;
const uint32_t RADAR_BAUD = 115200;

/////////////////////
// Webserver & SSE //
/////////////////////
WebServer server(80);

// Для Server Sent Events (EventSource) будем хранить клиентов как простую очередь
// — встроенный WebServer не имеет нативного SSE, но мы будем отправлять chunked ответы.
// Для простоты реализуем endpoint /events, который держит одно соединение (обычно работает с 1 клиентом).
WiFiClient eventClient;
bool eventClientConnected = false;

/////////////////////
// Preferences (NVS)//
/////////////////////
Preferences prefs;

struct Settings {
  uint8_t sensitivity;      // 0=Low,1=Med,2=High
  uint8_t hold_seconds;     // время удержания
  bool power_save;          // энергосбережение включено
  float distance_threshold; // порог срабатывания по расстоянию (m)
} settings;

// История для графиков (круговой буфер)
const size_t HISTORY_LEN = 120; // 120 точек ~ 2 минуты при 1s/точка
std::vector<float> history_distance;   // метры
std::vector<float> history_energy;     // условная "энергия"/уровень сигнала
std::vector<unsigned long> history_time; // unix millis

/////////////////////
// Вспомогательные //
/////////////////////
unsigned long lastSampleMillis = 0;
const unsigned long SAMPLE_INTERVAL = 1000; // ms для обновления графиков и отправки SSE

// Буфер входящих байтов от радара (для логирования)
String hexLog = ""; // накапливаем последние N байт в hex виде (показываем в UI)

// Простая функция для перевода байтов в hex строку
String byteToHex(uint8_t b) {
  char buf[4];
  sprintf(buf, "%02X", b);
  return String(buf);
}

// Пример команд (шаблонные по предыдущему ответу) - можно править под ваш модуль
const uint8_t CMD_READ_STATUS[]      = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x01};
uint8_t CMD_SET_SENSITIVITY[7] = {0xFD,0xFC,0xFB,0xFA,0x05,0x01,0x00};  // last = 0..2
uint8_t CMD_SET_HOLD_TIME[7]   = {0xFD,0xFC,0xFB,0xFA,0x06,0x01,0x05};  // last = seconds
uint8_t CMD_SET_POWERSAVE[7]   = {0xFD,0xFC,0xFB,0xFA,0x07,0x01,0x00};  // last = 0/1

// Функции отправки команд
void sendCommand(const uint8_t* cmd, size_t len){
  RadarSerial.write(cmd, len);
  RadarSerial.flush();
}

// Отправка команд с последним байтом value
void sendCmdSetSensitivity(uint8_t level) {
  CMD_SET_SENSITIVITY[6] = level;
  sendCommand(CMD_SET_SENSITIVITY, sizeof(CMD_SET_SENSITIVITY));
}

void sendCmdSetHold(uint8_t seconds) {
  CMD_SET_HOLD_TIME[6] = seconds;
  sendCommand(CMD_SET_HOLD_TIME, sizeof(CMD_SET_HOLD_TIME));
}

void sendCmdSetPowerSave(bool en) {
  CMD_SET_POWERSAVE[6] = en ? 0x01 : 0x00;
  sendCommand(CMD_SET_POWERSAVE, sizeof(CMD_SET_POWERSAVE));
}

/////////////////////
// Парсер ответов  //
/////////////////////
// Поскольку разные ревизии HLK могут отправлять разный формат,
// здесь реализован гибкий подход:
// - сохраняем сырые байты (HEX) для лога
// - пытаемся найти ASCII-паттерны "D=", "DIST=" или числа, и вытащить расстояние
// - если данные бинарные — отображаем HEX и оставляем парсер открытым для правок

float lastDistance = 0.0;
float lastEnergy = 0.0;
bool lastPresence = false;

// Пробуем распарсить текстовую строку на наличие числа расстояния (метры/см)
bool tryParseAsciiDistance(const String &s, float &outDist) {
  // Примеры строк: "D:123" "DIST=1.23m" "distance=123"
  int idx;
  idx = s.indexOf("DIST=");
  if (idx >= 0) {
    int start = idx + 5;
    int end = start;
    while (end < s.length() && (isDigit(s[end]) || s[end]=='.')) end++;
    String num = s.substring(start, end);
    outDist = num.toFloat();
    return true;
  }
  idx = s.indexOf("D:");
  if (idx >= 0) {
    int start = idx + 2;
    int end = start;
    while (end < s.length() && (isDigit(s[end]) || s[end]=='.')) end++;
    String num = s.substring(start, end);
    outDist = num.toFloat() / 100.0; // если приходит в см, попытка деления
    return true;
  }
  // поиск одиночного числа (рисковато)
  // не делаем пока этому приоритета
  return false;
}

// Парсер прихода байтов (вызывается в loop)
void handleRadarInput() {
  static String line = "";
  while (RadarSerial.available()) {
    uint8_t b = RadarSerial.read();
    // Добавляем в HEX-лог (ограничение длины)
    if (hexLog.length() > 4000) hexLog = hexLog.substring(hexLog.length() - 2000);
    hexLog += byteToHex(b) + " ";
    // Если это печатный ASCII — накапливаем строку для парсинга
    if (b == '\n' || b == '\r') {
      if (line.length() > 0) {
        // Попытка парсинга ASCII distance
        float d;
        if (tryParseAsciiDistance(line, d)) {
          lastDistance = d;
          lastPresence = true;
        }
        // Попытка извлечь "energy" или "amp"
        // Здесь оставим простую попытку: искать "E:" или "ENERGY="
        int pos = line.indexOf("E:");
        if (pos >= 0) {
          int s = pos + 2;
          int e = s;
          while (e < line.length() && (isDigit(line[e]) || line[e]=='.')) e++;
          lastEnergy = line.substring(s,e).toFloat();
        }
        line = "";
      }
    } else {
      // Добавляем в строковый буфер, если это печатные символы
      if (b >= 32 && b <= 126) {
        line += (char)b;
        // Если строка станет слишком длинной, обрезаем
        if (line.length() > 200) line = line.substring(line.length()-200);
      } else {
        // бинарный фрагмент — можно реализовать парсер бинарных пакетов тут
      }
    }
  }
}

// Принудительно сохранить последние состояния в историю (вызывается каждую секунду)
void pushHistoryPoint() {
  unsigned long now = millis();
  if (history_distance.size() >= HISTORY_LEN) {
    history_distance.erase(history_distance.begin());
    history_energy.erase(history_energy.begin());
    history_time.erase(history_time.begin());
  }
  history_distance.push_back(lastDistance);
  history_energy.push_back(lastEnergy);
  history_time.push_back(now);
}

/////////////////////
// SSE / Events     //
/////////////////////
// Отправляем JSON пакет с состоянием и частью истории
void sendEventUpdate() {
  if (!eventClientConnected) return;

  // Соберём небольшой JSON вручную (экономим память)
  String payload = "data: {";
  payload += "\"time\":" + String(millis()) + ",";
  payload += "\"distance\":" + String(lastDistance,2) + ",";
  payload += "\"energy\":" + String(lastEnergy,2) + ",";
  payload += "\"presence\":" + String(lastPresence ? "true" : "false") + ",";
  payload += "\"hexlog\":\"" + String(hexLog).substring(max(0, (int)hexLog.length()-800)) + "\"";
  payload += "}\n\n";

  // Отправляем как Server-Sent Event
  eventClient.print(payload);
}

// Эндпоинт для событий /events
void handleEvents() {
  // Устанавливаем заголовки
  WiFiClient client = server.client();
  if (!client) return;
  // Пометим, что у нас есть подключение
  eventClient = client;
  eventClientConnected = true;

  server.sendHeader("Content-Type", "text/event-stream");
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Connection", "keep-alive");
  server.send(200, "text/event-stream", "");

  // После этого соединение остаётся открытым — мы будем писать в eventClient из loop()
  // Важно: если клиент закроет соединение, eventClient.connected() станет false
}

/////////////////////
// Web endpoints   //
/////////////////////

// Главная страница — HTML/JS интерфейс (использует EventSource и Chart.js)
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HLK-LD2402 — Панель управления</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/normalize/8.0.1/normalize.min.css">
<style>
  body{font-family:Inter,system-ui,-apple-system,Segoe UI,Roboto,"Helvetica Neue",Arial; background:#0f1724; color:#e6eef8; margin:0; padding:12px;}
  .container{max-width:1100px;margin:0 auto;}
  header{display:flex;align-items:center;gap:12px;}
  h1{margin:6px 0 0;font-size:20px;}
  .card{background:linear-gradient(180deg, rgba(255,255,255,0.02), rgba(0,0,0,0.05)); border-radius:12px; padding:14px; box-shadow: 0 6px 18px rgba(2,6,23,0.6); margin-top:12px;}
  .row{display:flex;gap:12px;flex-wrap:wrap;}
  .col{flex:1; min-width:260px;}
  .controls button, .controls input[type=range]{width:100%;}
  label{font-size:13px;color:#9fb0c8;}
  .status{font-weight:600; color:#8ef3a3;}
  .hexlog{font-family:monospace; font-size:12px; max-height:140px; overflow:auto; background:#031022; padding:8px; border-radius:8px;}
  footer{margin-top:16px;color:#9fb0c8;font-size:13px;}
</style>
</head>
<body>
<div class="container">
  <header>
    <div>
      <h1>HLK-LD2402 — Панель управления и калибровки</h1>
      <div style="color:#9fb0c8;font-size:13px;">Отображение данных, графики и сохранение настроек (русский интерфейс)</div>
    </div>
  </header>

  <div class="card">
    <div class="row">
      <div class="col">
        <label>Текущее расстояние (м)</label>
        <div id="distance" style="font-size:28px;margin:6px 0;">—</div>

        <label>Уровень энергии / амплитуда</label>
        <div id="energy" style="font-size:20px;margin:6px 0;">—</div>

        <label>Детектор присутствия</label>
        <div id="presence" class="status">—</div>

        <label>HEX-log (последние байты)</label>
        <div class="hexlog" id="hexlog">Ожидание данных...</div>
      </div>

      <div class="col controls">
        <label>Чувствительность (0=Low,1=Med,2=High)</label>
        <input id="sensitivity" type="range" min="0" max="2" step="1" value="1">
        <div style="display:flex;gap:8px;margin-top:8px;">
          <button id="btnSetSens">Установить чувствительность</button>
          <button id="btnReadStatus">Прочитать статус</button>
        </div>

        <label style="margin-top:10px;">Время удержания (сек)</label>
        <input id="holdtime" type="range" min="1" max="30" step="1" value="5">
        <div style="display:flex;gap:8px;margin-top:8px;">
          <button id="btnSetHold">Установить удержание</button>
          <button id="btnPowerToggle">Вкл/Выкл энергосбережение</button>
        </div>

        <label style="margin-top:10px;">Порог срабатывания по расстоянию (м)</label>
        <input id="dist_threshold" type="number" step="0.01" min="0" value="1.0">
        <div style="display:flex;gap:8px;margin-top:8px;">
          <button id="btnSaveThreshold">Сохранить порог</button>
          <button id="btnSaveAll">Сохранить все настройки</button>
        </div>
      </div>
    </div>
  </div>

  <div class="card" style="margin-top:12px;">
    <canvas id="chartDistance" height="120"></canvas>
  </div>

  <div class="card" style="margin-top:12px;">
    <label>Отправка произвольной команды (HEX - пробелы разделяют байты)</label>
    <input id="cmd_hex" style="width:100%;padding:8px;margin-top:6px;" placeholder="FD FC FB FA 05 01 02">
    <div style="display:flex;gap:8px;margin-top:8px;">
      <button id="btnSendHex">Отправить HEX</button>
      <button id="btnClearLog">Очистить лог</button>
    </div>
  </div>

  <footer>
    <div>Подключение UART: RX=GPIO16, TX=GPIO17. Если формат сообщений отличается — откройте HEX-log и пришлите пример, я адаптирую парсер.</div>
  </footer>
</div>

<!-- Chart.js CDN -->
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<script>
  // UI элементы
  const distanceEl = document.getElementById('distance');
  const energyEl = document.getElementById('energy');
  const presenceEl = document.getElementById('presence');
  const hexlogEl = document.getElementById('hexlog');

  const sensitivity = document.getElementById('sensitivity');
  const holdtime = document.getElementById('holdtime');
  const distThreshold = document.getElementById('dist_threshold');

  // Кнопки
  document.getElementById('btnSetSens').onclick = () => {
    fetch('/set_sensitivity?s=' + sensitivity.value);
  };
  document.getElementById('btnSetHold').onclick = () => {
    fetch('/set_hold?h=' + holdtime.value);
  };
  document.getElementById('btnPowerToggle').onclick = () => {
    fetch('/toggle_powersave');
  };
  document.getElementById('btnReadStatus').onclick = () => { fetch('/read_status'); };
  document.getElementById('btnSaveThreshold').onclick = () => {
    fetch('/save_threshold?d=' + distThreshold.value).then(()=>alert('Порог сохранён'));
  };
  document.getElementById('btnSaveAll').onclick = () => {
    const obj = {
      s: sensitivity.value,
      h: holdtime.value,
      p: false,
      d: distThreshold.value
    };
    fetch('/save_all?s='+obj.s+'&h='+obj.h+'&p=0&d='+obj.d).then(()=>alert('Настройки сохранены'));
  };

  document.getElementById('btnSendHex').onclick = () => {
    fetch('/send_hex?data=' + encodeURIComponent(document.getElementById('cmd_hex').value));
  };
  document.getElementById('btnClearLog').onclick = () => {
    fetch('/clear_log');
    hexlogEl.textContent = '';
  };

  // Chart.js setup
  const ctx = document.getElementById('chartDistance').getContext('2d');
  const chart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Расстояние (м)', data: [], fill:false, tension:0.2 },
        { label: 'Энергия', data: [], fill:false, tension:0.2 }
      ]
    },
    options: {
      responsive: true,
      interaction: {mode:'index', intersect:false},
      scales: {
        x: { display: false },
        y: { beginAtZero:true }
      }
    }
  });

  // SSE - получаем данные в реальном времени
  const evtSource = new EventSource('/events');
  evtSource.onmessage = function(e) {
    try {
      const o = JSON.parse(e.data);
      // Обновляем видимые поля
      distanceEl.textContent = o.distance + ' м';
      energyEl.textContent = o.energy;
      presenceEl.textContent = o.presence ? 'Да' : 'Нет';
      hexlogEl.textContent = o.hexlog;

      // Обновляем график (append)
      const labels = chart.data.labels;
      const t = new Date().toLocaleTimeString();
      labels.push(t);
      if (labels.length > 120) { labels.shift(); }

      chart.data.datasets[0].data.push(parseFloat(o.distance));
      chart.data.datasets[1].data.push(parseFloat(o.energy));
      if (chart.data.datasets[0].data.length > 120) {
        chart.data.datasets.forEach(ds => ds.data.shift());
      }
      chart.update();

      // Если расстояние меньше порога — мигаем статус
      const thr = parseFloat(distThreshold.value);
      if (!isNaN(thr) && o.distance>0 && o.distance <= thr) {
        presenceEl.style.color = '#ffcc00';
        setTimeout(()=>presenceEl.style.color='#8ef3a3',500);
      }
    } catch(err){
      console.error(err);
    }
  };

</script>
</body>
</html>
)rawliteral";

/////////////////////
// HTTP handlers   //
/////////////////////

// Главная страница
void handleRoot() {
  server.sendHeader("Content-Type", "text/html; charset=utf-8");
  server.send(200, "text/html", index_html);
}

// Установка чувствительности
void handleSetSensitivity() {
  if (server.hasArg("s")) {
    int s = server.arg("s").toInt();
    if (s < 0) s = 0; if (s > 2) s = 2;
    settings.sensitivity = s;
    sendCmdSetSensitivity((uint8_t)s);
    prefs.putUChar("sensitivity", settings.sensitivity);
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "Missing param s");
}

// Установка hold time
void handleSetHold() {
  if (server.hasArg("h")) {
    int h = server.arg("h").toInt();
    if (h < 1) h = 1;
    if (h > 30) h = 30;
    settings.hold_seconds = h;
    sendCmdSetHold((uint8_t)h);
    prefs.putUChar("hold", settings.hold_seconds);
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "Missing param h");
}

// Toggle power save
void handleTogglePower() {
  settings.power_save = !settings.power_save;
  sendCmdSetPowerSave(settings.power_save);
  prefs.putBool("powersave", settings.power_save);
  server.send(200, "text/plain", settings.power_save ? "ON" : "OFF");
}

// Read status
void handleReadStatus() {
  sendCommand(CMD_READ_STATUS, sizeof(CMD_READ_STATUS));
  server.send(200, "text/plain", "READ");
}

// Send arbitrary HEX command
void handleSendHex() {
  if (!server.hasArg("data")) { server.send(400, "text/plain", "Missing data"); return; }
  String s = server.arg("data");
  // Парсим байты разделённые пробелом
  std::vector<uint8_t> buf;
  int len = s.length();
  String token = "";
  for (int i=0;i<len;i++) {
    char c = s.charAt(i);
    if (c == ' ' || c=='\t') {
      if (token.length()>0) {
        uint8_t val = (uint8_t) strtol(token.c_str(), NULL, 16);
        buf.push_back(val);
        token = "";
      }
    } else token += c;
  }
  if (token.length()>0) {
    uint8_t val = (uint8_t) strtol(token.c_str(), NULL, 16);
    buf.push_back(val);
  }
  if (buf.size()>0) {
    sendCommand(buf.data(), buf.size());
    server.send(200, "text/plain", "SENT");
  } else {
    server.send(400, "text/plain", "PARSE_ERROR");
  }
}

// Save single threshold
void handleSaveThreshold() {
  if (!server.hasArg("d")) { server.send(400, "text/plain", "Missing d"); return; }
  float d = server.arg("d").toFloat();
  settings.distance_threshold = d;
  prefs.putFloat("dist_thr", settings.distance_threshold);
  server.send(200, "text/plain", "OK");
}

// Save all settings
void handleSaveAll() {
  if (server.hasArg("s")) settings.sensitivity = server.arg("s").toInt();
  if (server.hasArg("h")) settings.hold_seconds = server.arg("h").toInt();
  if (server.hasArg("p")) settings.power_save = server.arg("p").toInt() != 0;
  if (server.hasArg("d")) settings.distance_threshold = server.arg("d").toFloat();

  // Отправляем команды
  sendCmdSetSensitivity(settings.sensitivity);
  sendCmdSetHold(settings.hold_seconds);
  sendCmdSetPowerSave(settings.power_save);

  // Сохраняем
  prefs.putUChar("sensitivity", settings.sensitivity);
  prefs.putUChar("hold", settings.hold_seconds);
  prefs.putBool("powersave", settings.power_save);
  prefs.putFloat("dist_thr", settings.distance_threshold);

  server.send(200, "text/plain", "OK");
}

// Очистить лог
void handleClearLog() {
  hexLog = "";
  server.send(200, "text/plain", "OK");
}

/////////////////////
// Setup / Loop    //
/////////////////////
void setup() {
  Serial.begin(115200);
  delay(200);

  // UART для радара
  RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(50);
  Serial.println("Serial1 (Radar) started");

  // Подключаемся к WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi not connected - start AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HLK_LD2402_Config");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  // Preferences (NVS)
  prefs.begin("ld2402", false);
  settings.sensitivity = prefs.getUChar("sensitivity", 1);
  settings.hold_seconds  = prefs.getUChar("hold", 5);
  settings.power_save    = prefs.getBool("powersave", false);
  settings.distance_threshold = prefs.getFloat("dist_thr", 1.0);

  // Инициализация веб-сервера — маршруты
  server.on("/", handleRoot);
  server.on("/events", HTTP_GET, handleEvents);
  server.on("/set_sensitivity", handleSetSensitivity);
  server.on("/set_hold", handleSetHold);
  server.on("/toggle_powersave", handleTogglePower);
  server.on("/read_status", handleReadStatus);
  server.on("/send_hex", handleSendHex);
  server.on("/save_threshold", handleSaveThreshold);
  server.on("/save_all", handleSaveAll);
  server.on("/clear_log", handleClearLog);
  server.begin();
  Serial.println("Web server started");

  // Инициализация истории
  history_distance.reserve(HISTORY_LEN);
  history_energy.reserve(HISTORY_LEN);
  history_time.reserve(HISTORY_LEN);

  // Применим сохранённые настройки к модулю при старте
  sendCmdSetSensitivity(settings.sensitivity);
  delay(100);
  sendCmdSetHold(settings.hold_seconds);
  delay(100);
  sendCmdSetPowerSave(settings.power_save);
}

void loop() {
  // Обработка WebServer
  server.handleClient();

  // Обработка входящих байтов от радара
  handleRadarInput();

  // Отправка событий по SSE и запись истории каждую секунду
  if (millis() - lastSampleMillis >= SAMPLE_INTERVAL) {
    lastSampleMillis = millis();
    pushHistoryPoint();
    // Попытка автоматически занулить presence при отсутствии новых данных
    static unsigned long lastDataTime = 0;
    if (hexLog.length() > 0) lastDataTime = millis();
    if (millis() - lastDataTime > 5000) { // 5s — считаем потерю данных
      lastPresence = false;
    }

    // Если есть подключение SSE — отправляем
    if (eventClientConnected && eventClient) {
      if (!eventClient.connected()) {
        eventClient.stop();
        eventClientConnected = false;
      } else {
        // Сформируем JSON и отправим
        String json = "{";
        json += "\"time\":" + String(millis()) + ",";
        json += "\"distance\":" + String(lastDistance, 2) + ",";
        json += "\"energy\":" + String(lastEnergy, 2) + ",";
        json += "\"presence\":" + String(lastPresence ? "true" : "false") + ",";
        // hexlog — обрезаем (безопасность)
        int startIdx = max(0, (int)hexLog.length() - 800);
        String shortLog = hexLog.substring(startIdx);
        // escape quotes
        shortLog.replace("\\", "\\\\");
        shortLog.replace("\"", "\\\"");
        shortLog.replace("\n", "\\n");
        json += "\"hexlog\":\"" + shortLog + "\"";
        json += "}\n\n";
        eventClient.print("data: ");
        eventClient.print(json);
        // flush
        eventClient.flush();
      }
    }
  }

  // Небольшая задержка
  delay(5);
}
