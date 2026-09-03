// RoboControl.ino
// Программа для ESP8266, которая запускает веб-интерфейс и WebSocket-сервер.
// Она получает команды управления от браузера и передаёт их по UART
// на контроллер моторов STM32. При потере связи робот останавливается.

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#define NODEBUG_WEBSOCKETS
#include <WebSocketsServer.h>

// ======= Wi-Fi =======
// Параметры подключения к Wi-Fi в режиме станции STA. При неудаче подключения
// ESP8266 запускает собственную точку доступа (AP) с именем "RobotAP".
// const char* ssid = "Redmi_9A";
const char* ssid = "rtk26-28";
const char* password = "96444335020";
void debugPrint(const String& message) {
  Serial.println(message);
}
const unsigned long WIFI_CONNECT_TIMEOUT = 15000; // Время ожидания подключения, мс

// ======= HTTP- и WebSocket-серверы =======
ESP8266WebServer server(80);
WebSocketsServer webSocket(81);

// ======= Heartbeat / Safety =======
// Если команда не поступает дольше SAFETY_TIMEOUT, отправляется команда остановки.
const unsigned long SAFETY_TIMEOUT = 1000; // ms
unsigned long lastCommandTime = 0;  // Время получения последней команды, мс
int16_t currentVx = 0;
int16_t currentVy = 0;
int16_t currentVz = 0;

// ======= Вспомогательные функции =======
// Определяет Content-Type по расширению файла для HTTP-ответа.
String getContentType(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".htm"))  return "text/html";
  if (path.endsWith(".js"))   return "application/javascript";
  if (path.endsWith(".css"))  return "text/css";
  if (path.endsWith(".png"))  return "image/png";
  if (path.endsWith(".jpg"))  return "image/jpeg";
  if (path.endsWith(".ico"))  return "image/x-icon";
  if (path.endsWith(".json")) return "application/json";
  return "text/plain";
}

// ======= UART для связи со STM32 =======
// Serial (UART0) используется для передачи команд на STM32.
// UART0 ESP8266: TX = GPIO1, RX = GPIO3. RX STM32 подключается к TX ESP8266,
// а TX STM32 — к RX ESP8266. Земля устройств должна быть общей.
// Формат кадра: 0xAA 0x55 vx vy vz checksum, значения команд -100..100.
void sendToStm(int16_t vx, int16_t vy, int16_t vz) {

  uint8_t payload[3] = {
    (uint8_t)(int8_t)vx,
    (uint8_t)(int8_t)vy,
    (uint8_t)(int8_t)vz
  };
  uint8_t checksum = payload[0] ^ payload[1] ^ payload[2];

  Serial.write(0xAA);
  Serial.write(0x55);
  Serial.write(payload, sizeof(payload));
  Serial.write(checksum);
}

void processMotion(int16_t vx, int16_t vy, int16_t vz) {
  currentVx = constrain(vx, -100, 100);
  currentVy = constrain(vy, -100, 100);
  currentVz = constrain(vz, -100, 100);
  lastCommandTime = millis();
  sendToStm(currentVx, currentVy, currentVz);
}

// ======= Обработчики HTTP-запросов =======
// Отправляет index.html из LittleFS для загрузки веб-интерфейса.
void handleRoot() {
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(500, "text/plain", "index.html not found");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}

// Обрабатывает запросы к файлам: если файл найден в LittleFS, отправляет его,
// иначе возвращает ошибку 404.
void handleNotFound() {
  String path = server.uri();
  if (path == "/") { handleRoot(); return; }
  if (LittleFS.exists(path)) {
    File f = LittleFS.open(path, "r");
    if (!f) {
      server.send(500, "text/plain", "File open failed");
      return;
    }
    server.streamFile(f, getContentType(path));
    f.close();
    return;
  }
  server.send(404, "text/plain", "Not found");
}

// ======= WebSocket =======
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT && length > 0) {
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      webSocket.sendTXT(num, "ERROR");
      return;
    }

    int vx = doc["vx"] | 0;
    int vy = doc["vy"] | 0;
    int vz = doc["vz"] | 0;
    processMotion(vx, vy, vz);
    webSocket.sendTXT(num, "OK");
  }
}

// ======= Инициализация и главный цикл =======
void setup() {
  // Встроенный светодиод ESP8266 обычно включается уровнем LOW.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Инициализация последовательного порта Serial (UART0).
  Serial.begin(57600); // UART0: TX=GPIO1, RX=GPIO3, двоичная передача на STM32
  debugPrint("ESP boot");
  delay(100);

  // Подключение файловой системы LittleFS для веб-сервера.
  if (!LittleFS.begin()) {
    if (!LittleFS.format() || !LittleFS.begin()) {
      // Не удалось подключить или отформатировать LittleFS.
    }
  }

  // Подключение к указанной Wi-Fi-сети в режиме STA. При неудаче
  // запускается собственная точка доступа для управления роботом.
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  debugPrint("WiFi: connecting to " + String(ssid));
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < WIFI_CONNECT_TIMEOUT) {
    delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    debugPrint("WiFi: STA failed, starting AP RobotAP");
    // При неудаче подключения запускается точка доступа ESP8266.
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RobotAP");
    debugPrint("WiFi: AP IP " + WiFi.softAPIP().toString());
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    debugPrint("WiFi: connected, IP " + WiFi.localIP().toString());
  }

  // HTTP-сервер отдаёт файлы веб-интерфейса, а WebSocket принимает команды.
  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  debugPrint("HTTP server: started");

  // Запуск WebSocket-сервера.
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  debugPrint("WebSocket server: started on port 81");

  lastCommandTime = millis();
}

void loop() {
  // Обработка HTTP-запросов и WebSocket-соединений.
  server.handleClient();
  webSocket.loop();

  // При потере связи остановить робота.
  if (millis() - lastCommandTime > SAFETY_TIMEOUT) {
    if (currentVx != 0 || currentVy != 0 || currentVz != 0)
      processMotion(0, 0, 0);
  }
}