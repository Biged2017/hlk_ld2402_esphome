/*
  HLK-LD2402 Configuration Tool for ESP32
  ----------------------------------------
  - Инженерное меню для настройки HLK-LD2402
  - Сохранение настроек в самом модуле HLK-LD2402
  - Чтение текущих настроек из HLK-LD2402
  - Вывод информации с датчиков в режиме реального времени
  
  Подключение:
    HLK TX -> ESP32 RX1 (GPIO16)
    HLK RX -> ESP32 TX1 (GPIO17)
    GND общий
    VCC -> 5V (проверьте ревизию модуля!)
*/

#include <WiFi.h>
#include <WebServer.h>           // Встроенный web сервер
#include <ArduinoJson.h>        // Для формирования JSON
#include <vector>

/////////////////////
// Настройки WiFi  //
/////////////////////
const char* WIFI_SSID = "ИМЯ";  // ЗАМЕНИТЕ НА ИМЯ ВАШЕЙ СЕТИ
const char* WIFI_PASS = "ПАРОЛЬ";  // ЗАМЕНИТЕ НА ПАРОЛЬ ВАШЕЙ СЕТИ

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
WiFiClient eventClient;
bool eventClientConnected = false;
unsigned long lastSSECheck = 0;
const unsigned long SSE_CHECK_INTERVAL = 5000; // Проверяем соединение каждые 5 секунд

/////////////////////
// Структуры данных//
/////////////////////
struct RadarSettings {
  uint8_t sensitivity;      // 0=Low,1=Med,2=High
  uint8_t hold_seconds;     // время удержания
  bool power_save;          // энергосбережение включено
  float distance_threshold; // порог срабатывания по расстоянию (m)
} radarSettings;

struct RadarData {
  float distance;      // метры
  float energy;        // условная "энергия"/уровень сигнала
  bool presence;       // наличие цели
  unsigned long timestamp; // время получения данных
} radarData;

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

// Команды для HLK-LD2402
const uint8_t CMD_READ_STATUS[]      = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x01};
const uint8_t CMD_READ_SETTINGS[]    = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x02};
const uint8_t CMD_SAVE_SETTINGS[]    = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x03};
uint8_t CMD_SET_SENSITIVITY[7] = {0xFD,0xFC,0xFB,0xFA,0x05,0x01,0x00};  // last = 0..2
uint8_t CMD_SET_HOLD_TIME[7]   = {0xFD,0xFC,0xFB,0xFA,0x06,0x01,0x05};  // last = seconds
uint8_t CMD_SET_POWERSAVE[7]   = {0xFD,0xFC,0xFB,0xFA,0x07,0x01,0x00};  // last = 0/1
uint8_t CMD_SET_THRESHOLD[8]   = {0xFD,0xFC,0xFB,0xFA,0x08,0x02,0x00,0x00}; // last 2 = threshold in cm

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

void sendCmdSetThreshold(float threshold_meters) {
  // Конвертируем метры в сантиметры
  uint16_t threshold_cm = (uint16_t)(threshold_meters * 100);
  CMD_SET_THRESHOLD[6] = (uint8_t)(threshold_cm >> 8);  // Старший байт
  CMD_SET_THRESHOLD[7] = (uint8_t)(threshold_cm & 0xFF); // Младший байт
  sendCommand(CMD_SET_THRESHOLD, sizeof(CMD_SET_THRESHOLD));
}

void sendCmdReadSettings() {
  sendCommand(CMD_READ_SETTINGS, sizeof(CMD_READ_SETTINGS));
}

void sendCmdSaveSettings() {
  sendCommand(CMD_SAVE_SETTINGS, sizeof(CMD_SAVE_SETTINGS));
}

/////////////////////
// Парсер ответов  //
/////////////////////
// Пробуем распарсить текстовую строку на наличие числа расстояния (метры/см)
bool tryParseAsciiDistance(const String &s, float &outDist) {
  // Примеры строк: "D:123" "DIST=1.23m" "distance=123" "target:1.23" "range:2.5"
  int idx;
  
  // Ищем различные форматы расстояния
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
  
  // Дополнительные форматы для HLK-LD2402
  idx = s.indexOf("target:");
  if (idx >= 0) {
    int start = idx + 7;
    int end = start;
    while (end < s.length() && (isDigit(s[end]) || s[end]=='.')) end++;
    String num = s.substring(start, end);
    outDist = num.toFloat();
    return true;
  }
  
  idx = s.indexOf("range:");
  if (idx >= 0) {
    int start = idx + 6;
    int end = start;
    while (end < s.length() && (isDigit(s[end]) || s[end]=='.')) end++;
    String num = s.substring(start, end);
    outDist = num.toFloat();
    return true;
  }
  
  // Если ничего не найдено, попробуем найти первое число в строке
  for (int i = 0; i < s.length(); i++) {
    if (isDigit(s.charAt(i))) {
      int start = i;
      int end = start;
      bool hasDot = false;
      while (end < s.length() && (isDigit(s[end]) || s[end]=='.')) {
        if (s[end] == '.') hasDot = true;
        end++;
      }
      String num = s.substring(start, end);
      float val = num.toFloat();
      // Если число выглядит как расстояние (0.1-10м или 10-1000см)
      if ((val >= 0.1 && val <= 10.0) || (val >= 10 && val <= 1000)) {
        outDist = hasDot ? val : val / 100.0; // если нет точки, считаем что это см
        return true;
      }
      i = end; // пропускаем обработанное число
    }
  }
  
  return false;
}

// Парсер прихода байтов (вызывается в loop)
void handleRadarInput() {
  static String line = "";
  static std::vector<uint8_t> binaryBuffer;
  
  while (RadarSerial.available()) {
    uint8_t b = RadarSerial.read();
    // Добавляем в HEX-лог (ограничение длины)
    if (hexLog.length() > 4000) hexLog = hexLog.substring(hexLog.length() - 2000);
    hexLog += byteToHex(b) + " ";
    
    // Сохраняем в бинарный буфер для анализа
    binaryBuffer.push_back(b);
    if (binaryBuffer.size() > 100) binaryBuffer.erase(binaryBuffer.begin());
    
    // Если это печатный ASCII — накапливаем строку для парсинга
    if (b == '\n' || b == '\r') {
      if (line.length() > 0) {
        Serial.print("ASCII line: ");
        Serial.println(line);
        
        // Попытка парсинга ASCII distance
        float d;
        if (tryParseAsciiDistance(line, d)) {
          radarData.distance = d;
          radarData.presence = true;
          radarData.timestamp = millis();
          Serial.print("Parsed distance: ");
          Serial.println(d);
          
          // Принудительно отправляем обновление при получении новых данных
          if (eventClientConnected && eventClient && eventClient.connected()) {
            Serial.println("Immediate SSE update triggered by new data");
            sendEventUpdate();
          }
        }
        // Попытка извлечь "energy" или "amp"
        int pos = line.indexOf("E:");
        if (pos >= 0) {
          int s = pos + 2;
          int e = s;
          while (e < line.length() && (isDigit(line[e]) || line[e]=='.')) e++;
          radarData.energy = line.substring(s,e).toFloat();
          Serial.print("Parsed energy: ");
          Serial.println(radarData.energy);
        }
        line = "";
      }
    } else {
      // Добавляем в строковый буфер, если это печатные символы
      if (b >= 32 && b <= 126) {
        line += (char)b;
        // Если строка станет слишком длинной, обрезаем
        if (line.length() > 200) line = line.substring(line.length()-200);
      }
    }
    
    // Проверяем бинарные пакеты HLK-LD2402
    // Формат: FD FC FB FA [LEN] [CMD] [DATA...] [CS]
    if (binaryBuffer.size() >= 6) {
      // Ищем заголовок пакета
      int headerPos = -1;
      for (size_t i = 0; i <= binaryBuffer.size() - 4; i++) {
        if (binaryBuffer[i] == 0xFD && binaryBuffer[i+1] == 0xFC &&
            binaryBuffer[i+2] == 0xFB && binaryBuffer[i+3] == 0xFA) {
          headerPos = i;
          break;
        }
      }
      
      if (headerPos >= 0 && binaryBuffer.size() >= headerPos + 6) {
        uint8_t len = binaryBuffer[headerPos + 4];
        uint8_t cmd = binaryBuffer[headerPos + 5];
        
        // Проверяем, что у нас есть полный пакет
        if (binaryBuffer.size() >= headerPos + 6 + len) {
          // Обрабатываем пакет с данными о расстоянии (CMD = 0x06)
          if (cmd == 0x06 && len >= 4) {
            // Предполагаемый формат: [DIST_H] [DIST_L] [ENERGY] [SPEED]
            uint16_t dist_raw = (binaryBuffer[headerPos + 6] << 8) | binaryBuffer[headerPos + 7];
            radarData.distance = dist_raw / 100.0; // переводим в метры
            radarData.energy = binaryBuffer[headerPos + 8];
            radarData.presence = true;
            radarData.timestamp = millis();
            
            Serial.print("Binary packet - Distance: ");
            Serial.print(radarData.distance);
            Serial.print("m, Energy: ");
            Serial.println(radarData.energy);
            
            // Принудительно отправляем обновление при получении новых данных
            if (eventClientConnected && eventClient && eventClient.connected()) {
              Serial.println("Immediate SSE update triggered by binary data");
              sendEventUpdate();
            }
          }
          // Обрабатываем пакет с настройками (CMD = 0x02)
          else if (cmd == 0x02 && len >= 6) {
            // Предполагаемый формат: [SENSITIVITY] [HOLD_TIME] [POWER_SAVE] [THRESHOLD_H] [THRESHOLD_L]
            radarSettings.sensitivity = binaryBuffer[headerPos + 6];
            radarSettings.hold_seconds = binaryBuffer[headerPos + 7];
            radarSettings.power_save = binaryBuffer[headerPos + 8] != 0;
            uint16_t threshold_raw = (binaryBuffer[headerPos + 9] << 8) | binaryBuffer[headerPos + 10];
            radarSettings.distance_threshold = threshold_raw / 100.0; // переводим в метры
            
            Serial.println("Received settings from radar:");
            Serial.print("  Sensitivity: "); Serial.println(radarSettings.sensitivity);
            Serial.print("  Hold time: "); Serial.print(radarSettings.hold_seconds); Serial.println(" seconds");
            Serial.print("  Power save: "); Serial.println(radarSettings.power_save ? "ON" : "OFF");
            Serial.print("  Distance threshold: "); Serial.print(radarSettings.distance_threshold); Serial.println(" meters");
          }
          
          // Удаляем обработанный пакет из буфера
          binaryBuffer.erase(binaryBuffer.begin(), binaryBuffer.begin() + headerPos + 6 + len);
        }
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
  history_distance.push_back(radarData.distance);
  history_energy.push_back(radarData.energy);
  history_time.push_back(now);
}

/////////////////////
// SSE / Events     //
/////////////////////
// Проверка состояния SSE соединения
bool checkSSEConnection() {
  if (!eventClientConnected || !eventClient) {
    return false;
  }
  
  if (!eventClient.connected()) {
    Serial.println("SSE client disconnected during check");
    eventClient.stop();
    eventClientConnected = false;
    return false;
  }
  
  // Дополнительная проверка - отправляем пинг каждые SSE_CHECK_INTERVAL
  if (millis() - lastSSECheck > SSE_CHECK_INTERVAL) {
    lastSSECheck = millis();
    // Отправляем пустое сообщение для поддержания соединения
    eventClient.print(": ping\n\n");
    eventClient.flush();
  }
  
  return true;
}

// Отправляем JSON пакет с состоянием и частью истории
void sendEventUpdate() {
  if (!checkSSEConnection()) {
    Serial.println("SSE not connected - skipping update");
    return;
  }

  Serial.print("Sending SSE update - Distance: ");
  Serial.print(radarData.distance, 2);
  Serial.print("m, Energy: ");
  Serial.print(radarData.energy, 2);
  Serial.print(", Presence: ");
  Serial.println(radarData.presence ? "true" : "false");

  // Создаем JSON с правильным форматированием для SSE
  String payload = "data: {";
  payload += "\"time\":" + String(millis()) + ",";
  payload += "\"distance\":" + String(radarData.distance,2) + ",";
  payload += "\"energy\":" + String(radarData.energy,2) + ",";
  payload += "\"presence\":" + String(radarData.presence ? "true" : "false") + ",";
  
  // hexlog — обрезаем (безопасность) и экранируем
  int startIdx = max(0, (int)hexLog.length() - 800);
  String shortLog = hexLog.substring(startIdx);
  // escape quotes
  shortLog.replace("\\", "\\\\");
  shortLog.replace("\"", "\\\"");
  shortLog.replace("\n", "\\n");
  shortLog.replace("\r", "\\r");
  payload += "\"hexlog\":\"" + shortLog + "\"";
  payload += "}\n\n";
  
  // Отправляем как Server-Sent Event
  size_t sent = eventClient.print(payload);
  eventClient.flush();
  
  Serial.print("SSE payload sent, bytes: ");
  Serial.println(sent);
  
  // Проверяем, что данные отправились
  if (sent == 0) {
    Serial.println("Failed to send SSE data, client may be disconnected");
    eventClient.stop();
    eventClientConnected = false;
  }
}

// Эндпоинт для событий /events
void handleEvents() {
  Serial.println("SSE client connecting...");
  
  // Устанавливаем заголовки
  WiFiClient client = server.client();
  if (!client) {
    Serial.println("No client available for SSE");
    return;
  }
  
  // Закрываем предыдущее соединение, если есть
  if (eventClientConnected && eventClient) {
    eventClient.stop();
    Serial.println("Previous SSE client disconnected");
  }
  
  // Пометим, что у нас есть подключение
  eventClient = client;
  eventClientConnected = true;
  lastSSECheck = millis();
  
  Serial.println("SSE client connected, sending headers");
  Serial.print("Client remote IP: ");
  Serial.println(client.remoteIP());

  // Отправляем заголовки напрямую клиенту
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: text/event-stream\r\n");
  client.print("Cache-Control: no-cache\r\n");
  client.print("Connection: keep-alive\r\n");
  client.print("Access-Control-Allow-Origin: *\r\n");
  client.print("Access-Control-Allow-Headers: Cache-Control\r\n");
  client.print("\r\n");  // Пустая строка завершает заголовки
  
  // Отправляем начальное сообщение для проверки соединения
  String initMsg = "data: {\"status\":\"connected\",\"time\":" + String(millis()) + "}\n\n";
  size_t sent = client.print(initMsg);
  client.flush();
  
  Serial.print("Initial SSE message sent, bytes: ");
  Serial.println(sent);
}

/////////////////////
// Web endpoints   //
/////////////////////

// Главная страница — HTML/JS интерфейс
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HLK-LD2402 — Инженерное меню</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/normalize/8.0.1/normalize.min.css">
<style>
  body{font-family:Inter,system-ui,-apple-system,Segoe UI,Roboto,"Helvetica Neue",Arial; background:#0f1724; color:#e6eef8; margin:0; padding:12px;}
  .container{max-width:1100px;margin:0 auto;}
  header{display:flex;align-items:center;gap:12px;}
  h1{margin:6px 0 0;font-size:20px;}
  .card{background:linear-gradient(180deg, rgba(255,255,255,0.02), rgba(0,0,0,0.05)); border-radius:12px; padding:14px; box-shadow: 0 6px 18px rgba(2,6,23,0.6); margin-top:12px;}
  .row{display:flex;gap:12px;flex-wrap:wrap;}
  .col{flex:1; min-width:260px;}
  .controls button, .controls input[type=range], .controls input[type=number]{width:100%;}
  label{font-size:13px;color:#9fb0c8;}
  .status{font-weight:600; color:#8ef3a3;}
  .hexlog{font-family:monospace; font-size:12px; max-height:140px; overflow:auto; background:#031022; padding:8px; border-radius:8px;}
  footer{margin-top:16px;color:#9fb0c8;font-size:13px;}
  .tab{padding:8px 16px;cursor:pointer;background:#031022;border-radius:6px 6px 0 0;margin-right:4px;}
  .tab.active{background:#1a2332;}
  .tab-content{display:none;}
  .tab-content.active{display:block;}
</style>
</head>
<body>
<div class="container">
  <header>
    <div>
      <h1>HLK-LD2402 — Инженерное меню настройки</h1>
      <div style="color:#9fb0c8;font-size:13px;">Настройка параметров и сохранение в модуле HLK-LD2402</div>
    </div>
  </header>

  <div style="margin-bottom:12px;">
    <div class="tab active" onclick="showTab('monitoring')">Мониторинг</div>
    <div class="tab" onclick="showTab('settings')">Настройки</div>
    <div class="tab" onclick="showTab('engineering')">Инженерное меню</div>
  </div>

  <!-- Мониторинг -->
  <div id="monitoring" class="tab-content active">
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

        <div class="col">
          <label>Текущие настройки модуля</label>
          <div style="background:#031022;padding:8px;border-radius:6px;font-size:12px;margin-top:5px;">
            <div>Чувствительность: <span id="current_sensitivity">-</span></div>
            <div>Удержание: <span id="current_holdtime">-</span> сек</div>
            <div>Энергосбережение: <span id="current_powersave">-</span></div>
            <div>Порог: <span id="current_threshold">-</span> м</div>
          </div>
          
          <div style="margin-top:15px;">
            <button id="btnReadSettings">Прочитать настройки из модуля</button>
            <button id="btnRefreshData">Обновить данные</button>
          </div>
        </div>
      </div>
    </div>

    <div class="card" style="margin-top:12px;">
      <canvas id="chartDistance" height="120"></canvas>
    </div>
  </div>

  <!-- Настройки -->
  <div id="settings" class="tab-content">
    <div class="card">
      <div class="row">
        <div class="col controls">
          <div class="setting-group">
            <label>Чувствительность (0=Low,1=Med,2=High)</label>
            <div style="display:flex;align-items:center;gap:10px;">
              <input id="sensitivity" type="range" min="0" max="2" step="1" value="1" style="flex:1;">
              <span id="sensitivity_value" style="min-width:30px;text-align:center;">1</span>
            </div>
          </div>

          <div class="setting-group" style="margin-top:15px;">
            <label>Время удержания (сек)</label>
            <div style="display:flex;align-items:center;gap:10px;">
              <input id="holdtime" type="range" min="1" max="30" step="1" value="5" style="flex:1;">
              <span id="holdtime_value" style="min-width:30px;text-align:center;">5</span>
            </div>
          </div>

          <div class="setting-group" style="margin-top:15px;">
            <label>Энергосбережение</label>
            <div style="display:flex;align-items:center;gap:10px;">
              <input id="powersave" type="checkbox" style="width:auto;">
              <span>Включить энергосбережение</span>
            </div>
          </div>

          <div class="setting-group" style="margin-top:15px;">
            <label>Порог срабатывания по расстоянию (м)</label>
            <div style="display:flex;align-items:center;gap:10px;">
              <input id="dist_threshold" type="number" step="0.01" min="0" value="1.0" style="flex:1;">
              <span id="threshold_indicator" style="min-width:60px;padding:4px 8px;background:#031022;border-radius:4px;font-size:12px;">1.00м</span>
            </div>
          </div>

          <div style="display:flex;gap:8px;margin-top:20px;">
            <button id="btnApplySettings">Применить настройки</button>
            <button id="btnSaveToModule">Сохранить в модуль</button>
          </div>
        </div>

        <div class="col">
          <label>Статус операций</label>
          <div id="operation_status" style="background:#031022;padding:8px;border-radius:6px;font-size:12px;margin-top:5px;min-height:100px;">
            Ожидание операций...
          </div>
        </div>
      </div>
    </div>
  </div>

  <!-- Инженерное меню -->
  <div id="engineering" class="tab-content">
    <div class="card">
      <label>Отправка произвольной команды (HEX - пробелы разделяют байты)</label>
      <input id="cmd_hex" style="width:100%;padding:8px;margin-top:6px;" placeholder="FD FC FB FA 05 01 02">
      <div style="display:flex;gap:8px;margin-top:8px;">
        <button id="btnSendHex">Отправить HEX</button>
        <button id="btnClearLog">Очистить лог</button>
        <button id="btnTestSSE">Тестировать SSE</button>
      </div>
    </div>

    <div class="card" style="margin-top:12px;">
      <label>Предустановленные команды</label>
      <div style="display:flex;flex-wrap:wrap;gap:8px;margin-top:8px;">
        <button class="preset-btn" data-cmd="FD FC FB FA 02 01">Прочитать статус</button>
        <button class="preset-btn" data-cmd="FD FC FB FA 02 02">Прочитать настройки</button>
        <button class="preset-btn" data-cmd="FD FC FB FA 02 03">Сохранить настройки</button>
        <button class="preset-btn" data-cmd="FD FC FB FA 05 01 01">Установить чувствительность: Средняя</button>
        <button class="preset-btn" data-cmd="FD FC FB FA 06 01 05">Установить удержание: 5 сек</button>
        <button class="preset-btn" data-cmd="FD FC FB FA 07 01 00">Выключить энергосбережение</button>
      </div>
    </div>
  </div>

  <footer>
    <div>Подключение UART: RX=GPIO16, TX=GPIO17. Настройки сохраняются непосредственно в модуле HLK-LD2402.</div>
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
  const powersave = document.getElementById('powersave');
  const distThreshold = document.getElementById('dist_threshold');
  
  // Элементы для отображения значений
  const sensitivityValue = document.getElementById('sensitivity_value');
  const holdtimeValue = document.getElementById('holdtime_value');
  const thresholdIndicator = document.getElementById('threshold_indicator');
  
  // Элементы для отображения текущих настроек
  const currentSensitivity = document.getElementById('current_sensitivity');
  const currentHoldtime = document.getElementById('current_holdtime');
  const currentPowersave = document.getElementById('current_powersave');
  const currentThreshold = document.getElementById('current_threshold');
  
  // Элемент статуса операций
  const operationStatus = document.getElementById('operation_status');

  // Функция для обновления отображаемых значений
  function updateValueDisplays() {
    sensitivityValue.textContent = sensitivity.value;
    holdtimeValue.textContent = holdtime.value;
    thresholdIndicator.textContent = parseFloat(distThreshold.value).toFixed(2) + 'м';
  }
  
  // Функция для обновления отображения текущих настроек
  function updateCurrentSettings(settings) {
    const sensLevels = ['Low', 'Med', 'High'];
    currentSensitivity.textContent = sensLevels[settings.sensitivity] || '-';
    currentHoldtime.textContent = settings.hold_seconds || '-';
    currentPowersave.textContent = settings.power_save ? 'Вкл' : 'Выкл';
    currentThreshold.textContent = settings.distance_threshold ? settings.distance_threshold.toFixed(2) : '-';
  }
  
  // Функция для добавления сообщения в статус операций
  function addStatusMessage(message, type = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    const color = type === 'error' ? '#ff6b6b' : type === 'success' ? '#8ef3a3' : '#9fb0c8';
    operationStatus.innerHTML += `<div style="color:${color}">[${timestamp}] ${message}</div>`;
    operationStatus.scrollTop = operationStatus.scrollHeight;
  }
  
  // Функция для переключения вкладок
  function showTab(tabName) {
    // Скрываем все вкладки
    document.querySelectorAll('.tab-content').forEach(tab => {
      tab.classList.remove('active');
    });
    document.querySelectorAll('.tab').forEach(tab => {
      tab.classList.remove('active');
    });
    
    // Показываем выбранную вкладку
    document.getElementById(tabName).classList.add('active');
    event.target.classList.add('active');
  }

  // Обновляем значения при изменении ползунков
  sensitivity.oninput = updateValueDisplays;
  holdtime.oninput = updateValueDisplays;
  distThreshold.oninput = updateValueDisplays;
  
  // Инициализация значений при загрузке страницы
  updateValueDisplays();

  // Кнопки
  document.getElementById('btnReadSettings').onclick = () => {
    fetch('/read_settings')
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          updateCurrentSettings(data.settings);
          addStatusMessage('Настройки успешно прочитаны из модуля', 'success');
        } else {
          addStatusMessage('Ошибка чтения настроек: ' + data.error, 'error');
        }
      })
      .catch(error => {
        console.error('Error reading settings:', error);
        addStatusMessage('Ошибка при чтении настроек: ' + error.message, 'error');
      });
  };
  
  document.getElementById('btnRefreshData').onclick = () => {
    fetch('/refresh_data')
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          addStatusMessage('Данные обновлены', 'success');
        } else {
          addStatusMessage('Ошибка обновления данных: ' + data.error, 'error');
        }
      })
      .catch(error => {
        console.error('Error refreshing data:', error);
        addStatusMessage('Ошибка при обновлении данных: ' + error.message, 'error');
      });
  };
  
  document.getElementById('btnApplySettings').onclick = () => {
    const settings = {
      sensitivity: parseInt(sensitivity.value),
      hold_seconds: parseInt(holdtime.value),
      power_save: powersave.checked,
      distance_threshold: parseFloat(distThreshold.value)
    };
    
    addStatusMessage('Применение настроек...', 'info');
    
    fetch('/apply_settings', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(settings)
    })
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          addStatusMessage('Настройки успешно применены к модулю', 'success');
        } else {
          addStatusMessage('Ошибка применения настроек: ' + data.error, 'error');
        }
      })
      .catch(error => {
        console.error('Error applying settings:', error);
        addStatusMessage('Ошибка при применении настроек: ' + error.message, 'error');
      });
  };
  
  document.getElementById('btnSaveToModule').onclick = () => {
    addStatusMessage('Сохранение настроек в модуль...', 'info');
    
    fetch('/save_to_module')
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          addStatusMessage('Настройки успешно сохранены в энергонезависимую память модуля', 'success');
        } else {
          addStatusMessage('Ошибка сохранения настроек: ' + data.error, 'error');
        }
      })
      .catch(error => {
        console.error('Error saving settings:', error);
        addStatusMessage('Ошибка при сохранении настроек: ' + error.message, 'error');
      });
  };

  document.getElementById('btnSendHex').onclick = () => {
    const hexData = document.getElementById('cmd_hex').value;
    fetch('/send_hex?data=' + encodeURIComponent(hexData))
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          addStatusMessage('Команда успешно отправлена', 'success');
        } else {
          addStatusMessage('Ошибка отправки команды: ' + data.error, 'error');
        }
      })
      .catch(error => {
        console.error('Error sending hex:', error);
        addStatusMessage('Ошибка при отправке команды: ' + error.message, 'error');
      });
  };
  
  document.getElementById('btnClearLog').onclick = () => {
    fetch('/clear_log');
    hexlogEl.textContent = '';
    addStatusMessage('Лог очищен', 'info');
  };
  
  document.getElementById('btnTestSSE').onclick = () => {
    fetch('/test_sse')
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          addStatusMessage('Тестовые данные отправлены', 'success');
        } else {
          addStatusMessage('Ошибка тестирования: ' + data.error, 'error');
        }
      })
      .catch(error => {
        console.error('Error testing SSE:', error);
        addStatusMessage('Ошибка при тестировании: ' + error.message, 'error');
      });
  };

  // Предустановленные команды
  document.querySelectorAll('.preset-btn').forEach(btn => {
    btn.onclick = () => {
      const cmd = btn.getAttribute('data-cmd');
      document.getElementById('cmd_hex').value = cmd;
      document.getElementById('btnSendHex').click();
    };
  });

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
  console.log("Setting up EventSource...");
  
  // Проверяем поддержку EventSource в браузере
  if (typeof(EventSource) === "undefined") {
    console.error("Browser does not support Server-Sent Events");
    document.getElementById('presence').textContent = 'Браузер не поддерживает SSE';
    document.getElementById('presence').style.color = '#ff0000';
  } else {
    // Функция для создания EventSource с улучшенной обработкой
    function createEventSource() {
      console.log("Creating new EventSource connection...");
      
      const evtSource = new EventSource('/events');
      
      let lastMessageTime = Date.now();
      const connectionTimeout = 10000; // 10 секунд без сообщений считаем обрывом
      
      const activityCheck = setInterval(function() {
        const now = Date.now();
        if (now - lastMessageTime > connectionTimeout && evtSource.readyState === EventSource.OPEN) {
          console.warn("No SSE messages received for", connectionTimeout/1000, "seconds, connection may be stale");
          document.getElementById('presence').textContent = 'Нет данных (проверка соединения)';
          document.getElementById('presence').style.color = '#ffaa00';
        }
      }, 5000);
      
      evtSource.onopen = function() {
        console.log("SSE connection opened successfully");
        document.getElementById('presence').textContent = 'Подключено к SSE';
        document.getElementById('presence').style.color = '#00ff00';
        lastMessageTime = Date.now();
      };
      
      evtSource.onerror = function(e) {
        console.log("SSE error:", e);
        console.log("ReadyState:", evtSource.readyState);
        
        let errorMsg = 'Ошибка SSE';
        if (evtSource.readyState === EventSource.CONNECTING) {
          errorMsg = 'Подключение к SSE...';
        } else if (evtSource.readyState === EventSource.CLOSED) {
          errorMsg = 'SSE соединение закрыто';
        }
        
        document.getElementById('presence').textContent = errorMsg + ' (' + evtSource.readyState + ')';
        document.getElementById('presence').style.color = '#ff0000';
        
        clearInterval(activityCheck);
        
        if (evtSource.readyState === EventSource.CLOSED) {
          console.log("SSE connection closed, attempting to reconnect in 3 seconds...");
          setTimeout(function() {
            console.log("Attempting to reconnect...");
            evtSource.close();
            createEventSource();
          }, 3000);
        }
      };
      
      evtSource.onmessage = function(e) {
        lastMessageTime = Date.now();
        
        console.log("SSE message received, length:", e.data.length);
        
        if (!e.data || e.data.trim() === '') {
          console.warn("Empty SSE data received");
          return;
        }
        
        if (e.data.startsWith(':')) {
          console.log("Ping message received, ignoring");
          return;
        }
        
        try {
          let jsonData = e.data;
          if (jsonData.startsWith('data: ')) {
            jsonData = jsonData.substring(6);
          }
          
          const o = JSON.parse(jsonData);
          console.log("Parsed SSE data:", o);
          
          if (o.distance !== undefined) {
            distanceEl.textContent = o.distance + ' м';
          }
          
          if (o.energy !== undefined) {
            energyEl.textContent = o.energy;
          }
          
          if (o.presence !== undefined) {
            presenceEl.textContent = o.presence ? 'Да' : 'Нет';
            presenceEl.style.color = o.presence ? '#8ef3a3' : '#ff6b6b';
          }
          
          if (o.hexlog !== undefined) {
            hexlogEl.textContent = o.hexlog;
          }

          // Обновляем график
          if (o.distance !== undefined && o.energy !== undefined) {
            const labels = chart.data.labels;
            const t = new Date().toLocaleTimeString();
            labels.push(t);
            if (labels.length > 120) { labels.shift(); }

            chart.data.datasets[0].data.push(parseFloat(o.distance));
            chart.data.datasets[1].data.push(parseFloat(o.energy));
            if (chart.data.datasets[0].data.length > 120) {
              chart.data.datasets.forEach(ds => ds.data.shift());
            }
            chart.update('none');
          }
        } catch(err){
          console.error("Error parsing SSE data:", err);
          document.getElementById('presence').textContent = 'Ошибка парсинга JSON';
          document.getElementById('presence').style.color = '#ff0000';
        }
      };
      
      return evtSource;
    }
    
    const evtSource = createEventSource();
  }

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

// Чтение настроек из модуля
void handleReadSettings() {
  Serial.println("Reading settings from module...");
  sendCmdReadSettings();
  
  // Ждем ответа от модуля
  delay(500);
  
  // Создаем JSON ответ с текущими настройками
  String json = "{";
  json += "\"success\":true,";
  json += "\"settings\":{";
  json += "\"sensitivity\":" + String(radarSettings.sensitivity) + ",";
  json += "\"hold_seconds\":" + String(radarSettings.hold_seconds) + ",";
  json += "\"power_save\":" + String(radarSettings.power_save ? "true" : "false") + ",";
  json += "\"distance_threshold\":" + String(radarSettings.distance_threshold, 2);
  json += "}";
  json += "}";
  
  server.sendHeader("Content-Type", "application/json");
  server.send(200, "application/json", json);
}

// Обновление данных
void handleRefreshData() {
  // Отправляем команду чтения статуса
  sendCommand(CMD_READ_STATUS, sizeof(CMD_READ_STATUS));
  
  String json = "{\"success\":true}";
  server.sendHeader("Content-Type", "application/json");
  server.send(200, "application/json", json);
}

// Применение настроек к модулю
void handleApplySettings() {
  String body = server.arg("plain");
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, body);
  
  if (error) {
    String json = "{\"success\":false,\"error\":\"Invalid JSON\"}";
    server.sendHeader("Content-Type", "application/json");
    server.send(400, "application/json", json);
    return;
  }
  
  // Применяем настройки
  if (doc.containsKey("sensitivity")) {
    uint8_t sensitivity = doc["sensitivity"];
    if (sensitivity > 2) sensitivity = 2;
    radarSettings.sensitivity = sensitivity;
    sendCmdSetSensitivity(sensitivity);
    delay(100);
  }
  
  if (doc.containsKey("hold_seconds")) {
    uint8_t hold_seconds = doc["hold_seconds"];
    if (hold_seconds < 1) hold_seconds = 1;
    if (hold_seconds > 30) hold_seconds = 30;
    radarSettings.hold_seconds = hold_seconds;
    sendCmdSetHold(hold_seconds);
    delay(100);
  }
  
  if (doc.containsKey("power_save")) {
    bool power_save = doc["power_save"];
    radarSettings.power_save = power_save;
    sendCmdSetPowerSave(power_save);
    delay(100);
  }
  
  if (doc.containsKey("distance_threshold")) {
    float distance_threshold = doc["distance_threshold"];
    if (distance_threshold < 0) distance_threshold = 0;
    if (distance_threshold > 10) distance_threshold = 10;
    radarSettings.distance_threshold = distance_threshold;
    sendCmdSetThreshold(distance_threshold);
    delay(100);
  }
  
  String json = "{\"success\":true}";
  server.sendHeader("Content-Type", "application/json");
  server.send(200, "application/json", json);
}

// Сохранение настроек в модуль
void handleSaveToModule() {
  Serial.println("Saving settings to module...");
  sendCmdSaveSettings();
  
  // Ждем ответа от модуля
  delay(500);
  
  String json = "{\"success\":true}";
  server.sendHeader("Content-Type", "application/json");
  server.send(200, "application/json", json);
}

// Send arbitrary HEX command
void handleSendHex() {
  if (!server.hasArg("data")) { 
    String json = "{\"success\":false,\"error\":\"Missing data\"}";
    server.sendHeader("Content-Type", "application/json");
    server.send(400, "application/json", json);
    return; 
  }
  
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
    String json = "{\"success\":true}";
    server.sendHeader("Content-Type", "application/json");
    server.send(200, "application/json", json);
  } else {
    String json = "{\"success\":false,\"error\":\"PARSE_ERROR\"}";
    server.sendHeader("Content-Type", "application/json");
    server.send(400, "application/json", json);
  }
}

// Очистить лог
void handleClearLog() {
  hexLog = "";
  String json = "{\"success\":true}";
  server.sendHeader("Content-Type", "application/json");
  server.send(200, "application/json", json);
}

// Тестирование SSE
void handleTestSSE() {
  Serial.println("SSE test requested - sending test data");
  
  // Отправляем тестовые данные
  radarData.distance = 2.5;
  radarData.energy = 75;
  radarData.presence = true;
  radarData.timestamp = millis();
  
  // Добавляем тестовую запись в HEX лог
  hexLog += "TEST_DATA ";
  
  String json = "{\"success\":true}";
  server.sendHeader("Content-Type", "application/json");
  server.send(200, "application/json", json);
}

/////////////////////
// Setup / Loop    //
/////////////////////
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== HLK-LD2402 Configuration Tool Starting ===");
  Serial.println("Debug mode enabled - check Serial Monitor for details");

  // UART для радара
  RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(50);
  Serial.println("Serial1 (Radar) started");
  Serial.print("Radar RX pin: ");
  Serial.println(RADAR_RX_PIN);
  Serial.print("Radar TX pin: ");
  Serial.println(RADAR_TX_PIN);
  Serial.print("Radar baud rate: ");
  Serial.println(RADAR_BAUD);

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

  // Инициализация настроек по умолчанию
  radarSettings.sensitivity = 1;  // Medium
  radarSettings.hold_seconds = 5;
  radarSettings.power_save = false;
  radarSettings.distance_threshold = 1.0;
  
  // Инициализация данных
  radarData.distance = 0.0;
  radarData.energy = 0.0;
  radarData.presence = false;
  radarData.timestamp = 0;

  // Инициализация веб-сервера — маршруты
  server.on("/", handleRoot);
  server.on("/events", HTTP_GET, handleEvents);
  server.on("/read_settings", handleReadSettings);
  server.on("/refresh_data", handleRefreshData);
  server.on("/apply_settings", HTTP_POST, handleApplySettings);
  server.on("/save_to_module", handleSaveToModule);
  server.on("/send_hex", handleSendHex);
  server.on("/clear_log", handleClearLog);
  server.on("/test_sse", handleTestSSE);
  server.begin();
  Serial.println("Web server started");
  Serial.print("Server IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("Open http://" + WiFi.localIP().toString() + " in your browser");
  Serial.println("SSE endpoint available at: http://" + WiFi.localIP().toString() + "/events");

  // Инициализация истории
  history_distance.reserve(HISTORY_LEN);
  history_energy.reserve(HISTORY_LEN);
  history_time.reserve(HISTORY_LEN);

  // Пробуем прочитать текущие настройки из модуля при старте
  Serial.println("Reading current settings from module...");
  sendCmdReadSettings();
  delay(500);
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
      radarData.presence = false;
    }

    // Если есть подключение SSE — отправляем
    if (checkSSEConnection()) {
      Serial.println("SSE client connected, sending update...");
      sendEventUpdate();
    } else {
      Serial.println("No SSE client connected");
    }
  }

  // Дополнительная проверка состояния SSE соединения
  if (eventClientConnected && eventClient) {
    if (!eventClient.connected()) {
      Serial.println("SSE client disconnected in main loop");
      eventClient.stop();
      eventClientConnected = false;
    }
  }

  // Небольшая задержка
  delay(5);
}
