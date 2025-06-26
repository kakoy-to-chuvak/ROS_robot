#ifdef __cplusplus
extern "C" {
#endif
  uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <driver/uart.h>
#include "esp_task_wdt.h"
#include "driver/pcnt.h"
#include <math.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>  // Библиотека для работы с адресной светодиодной лентой

/* ---------- Wi-Fi ---------- */
#define WIFI_SSID "robotx"
#define WIFI_PASS "78914040"

/* ---------- Пины и параметры робота ---------- */
// Двигатели (H-мосты)
#define MOTOR_LA 33
#define MOTOR_LB 32
#define MOTOR_RA 27
#define MOTOR_RB 26

// Энкодеры (PCNT)
#define ENCODER_RA 18
#define ENCODER_RB 19
#define ENCODER_LA 34
#define ENCODER_LB 35

// Описание механики
#define WHEEL_D 0.044f  // диаметр колеса, м
#define BASE_L 0.1f     // база (расстояние между колёсами), м
#define TICKS_REV 2930  // количество тиков энкодера на оборот колеса
#define BATTERY_COEF 0.0025074626865672f
constexpr float MM_PER_TICK = WHEEL_D * M_PI * 1000.0f / TICKS_REV;  // мм за один тик

/* ---------- Глобальные переменные состояния ---------- */
volatile uint8_t dutyLA = 0, dutyLB = 0, dutyRA = 0, dutyRB = 0;  // текущие ШИМ для H-мостов
volatile int32_t encTotL = 0, encTotR = 0;                        // накопленные тики энкодера
volatile int32_t prevEncL = 0, prevEncR = 0;
volatile float speedL = 0.0f, speedR = 0.0f;               // измеренные скорости, мм/с
volatile float tgtL = 0.0f, tgtR = 0.0f;                   // целевые скорости, мм/с
volatile float odomX = 0.0f, odomY = 0.0f, odomTh = 0.0f;  // одометрия: положение робота (м, м, рад)
volatile float kp = 0.f, ki = 0.f, kd = 0.f, kff = 1.f;    // PID коэффициенты

volatile uint32_t lastCmdMs = 0;

// Режим выравнивания (для прямолинейного движения)
bool alignMode = false;
float alignSign = 1.0f;
int32_t alignRefL = 0, alignRefR = 0;
#define kAlign 1.0f  // коэффициент P-контроллера выравнивания (мм -> мм/с)

/* ---------- Настройки лидара ---------- */
#define LIDAR_RX_PIN 17  // lidar TX -> ESP RX
#define LIDAR_TX_PIN 16  // lidar RX (не обязательно использовать)
#define LIDAR_BAUD 115200

static const uint8_t HDR[4] = { 0x55, 0xAA, 0x03, 0x08 };
#define BODY_LEN 32      // байт в теле пакета лидара (8 точек по 4 байта)
#define INTENSITY_MIN 2  // минимальное значение интенсивности для учёта точки
#define MAX_SPREAD_DEG 20.0
#define FRAME_LEN 20   // длина упакованных данных на каждую порцию (2 байта нач.угол, 2 байта кон.угол, 8*2 байта дистанции)
#define MAX_FRAMES 64  // макс. количество порций на один полный оборот (64*20 ≈ 1280 байт)
static uint8_t scanBuf[MAX_FRAMES * FRAME_LEN];
static uint8_t *wr = scanBuf;
static uint8_t frameCount = 0;
static float prevStartAngle = -1;

/* ---------- Дальномер ---------- */
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool is_ranging = 1;
unsigned int measured_distance = 0;

/* ---------- Светодиодная лента ----------*/
#define LED1_PIN 14  // Пин, к которому подключена лента
#define LED1_COUNT 8
#define LED2_PIN 25  // Пин, к которому подключена лента
#define LED2_COUNT 10
#define LED_TYPE (NEO_GRB + NEO_KHZ800)                       // Тип светодиодов (формат данных и частота)
Adafruit_NeoPixel strip_up(LED1_COUNT, LED1_PIN, LED_TYPE);   // Создаём объект для управления лентой
Adafruit_NeoPixel strip_hor(LED2_COUNT, LED2_PIN, LED_TYPE);  // Создаём объект для управления лентой
byte show1 = 0;
byte show2 = 1;


/* ---------- Статистика ---------- */
volatile uint32_t stat_rx = 0;  // принятых 20-байтных пакетов от LDS
volatile uint32_t stat_tx = 0;  // переданных полных сканов по WS
float battery_charge = 0;
float cpu_temp = 0;  // Конвертация в °C
float cpu_load = 0;  // Конвертация в °C
uint32_t total_ram = 0;
uint32_t free_ram = 0;
uint32_t used_ram = 0;


/* ---------- Вспомогательные функции ---------- */
inline float decodeAngle(uint16_t raw) {
  // Декодирует угол (двухбайтное значение) из формата LDS
  float a = (raw - 0xA000) / 64.0f;
  if (a < 0) a += 360.0f;
  else if (a >= 360) a -= 360.0f;
  return a;
}
bool readBytes(HardwareSerial &serial, uint8_t *dst, size_t n, uint32_t timeout = 300) {
  uint32_t t0 = millis();
  for (size_t i = 0; i < n; ++i) {
    while (!serial.available()) {
      if (millis() - t0 > timeout) return false;
      vTaskDelay(1);
      esp_task_wdt_reset();
    }
    dst[i] = serial.read();
  }
  return true;
}
bool waitLidarHeader(HardwareSerial &serial) {
  uint8_t pos = 0;
  uint32_t t0 = millis();
  while (true) {
    if (serial.available()) {
      if (uint8_t(serial.read()) == HDR[pos]) {
        if (++pos == 4) return true;
      } else {
        pos = 0;
      }
    }
    if (millis() - t0 > 200) return false;
    esp_task_wdt_reset();
  }
}
inline uint16_t crc16(uint16_t crc, uint8_t v) {
  crc ^= v;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}



/* ---------- Настройка дальномера ---------- */
bool setupRangefinder(Adafruit_VL53L0X *_rangefinder, uint32_t _timing_budget) {
  Serial.println("Starting ranging.....");
  if (_rangefinder->begin()) {
    _rangefinder->configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    _rangefinder->setMeasurementTimingBudgetMicroSeconds(_timing_budget);
    _rangefinder->startRangeContinuous();
    Serial.println("Ranging started successfully!");
    return 1;
  } else {
    Serial.println("Ranging not started");
    return 0;
  }
}


/* ---------- Настройка ленты ---------- */
void statOnStrip(Adafruit_NeoPixel *_strip, float stat, uint8_t ledcount, byte minGcolor, byte maxGcolor) {
  int reqLeds = ceil(ledcount * stat);
  short colorG = minGcolor + ceil((maxGcolor - minGcolor) * stat);
  uint32_t color = _strip->Color(255 - colorG, colorG, 0);
  _strip->show();
  for (int i = 0; i < reqLeds; i++) {
    _strip->setPixelColor(i, color);
  }
}

/* ---------- Настройка WebSocket и HTTP ---------- */
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *wsClient = nullptr;


// Обработчик событий WebSocket
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    // Новый клиент подключился
    if (wsClient && wsClient->status() == WS_CONNECTED) {
      // Закрываем предыдущего клиента, если был
      wsClient->close();
    }
    wsClient = client;
    //wsClient->printf("[WS] Connected (id=%u)\n", client->id());
    wsClient->client()->setNoDelay(true);  // отключаем алгоритм Нэгла для минимальной задержки
    Serial.printf("[WS] Client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    if (client == wsClient) {
      wsClient = nullptr;
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
  } else if (type == WS_EVT_DATA) {
    // NEW: входящее сообщение от клиента (ROS2), может содержать команду на движение
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == 4) {
      // Ожидаем 4-байтовое бинарное сообщение: [int16_t left_mm_s, int16_t right_mm_s]
      int16_t left = data[0] | (data[1] << 8);
      int16_t right = data[2] | (data[3] << 8);
      // Устанавливаем новые целевые скорости колес
      tgtL = (float)left;
      tgtR = (float)right;
      lastCmdMs = millis();  // ← обновили «пинг»
      // Включаем режим выравнивания, если |vL|≈|vR| и не ноль (робот едет прямо или крутится на месте)
      if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
        alignMode = true;
        alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
        alignRefL = encTotL;
        alignRefR = encTotR;
      } else {
        alignMode = false;
      }
      // Можно отправить подтверждение или лог (не обязательно)
      Serial.printf("[WS] Cmd: left=%d, right=%d\n", left, right);
    }
    // Если нужно обработать текстовые сообщения или другие бинарные команды, добавить тут
  }
}



// Настройка HTTP-роутов
void setupRoutes() {
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Формируем JSON с текущим состоянием робота
    char json[512];
    snprintf(json, sizeof(json),
             "{\"duty\":{\"MOTOR_LA\":%u,\"MOTOR_LB\":%u,\"MOTOR_RA\":%u,\"(MOTOR_RB\":%u},"
             "\"enc\":{\"left\":%ld,\"right\":%ld},"
             "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
             "\"target\":{\"left\":%.1f,\"right\":%.1f},"
             "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f},"
             "\"rangefinder\":{\"value\":%lu},"
             "\"robot\":{\"battery_charge\":%.2f,\"cpu_load\":%.2f,\"used_ram\":%llu,\"cpu_temp\":%.2f}}",
             dutyLA, dutyLB, dutyRA, dutyRB,
             encTotL, encTotR,
             speedL, speedR,
             tgtL, tgtR,
             odomX, odomY, odomTh,
             measured_distance,
             battery_charge, cpu_load, used_ram, cpu_temp);
    request->send(200, "application/json", json);
  });
  server.on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("l")) tgtL = request->getParam("l")->value().toFloat();
    if (request->hasParam("r")) tgtR = request->getParam("r")->value().toFloat();
    lastCmdMs = millis();  // ← обновили «пинг»
    // Управление alignMode аналогично, как выше
    if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
      alignMode = true;
      alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
      alignRefL = encTotL;
      alignRefR = encTotR;
    } else {
      alignMode = false;
    }
    request->send(200, "text/plain", "ok");
  });
  server.on("/setCoeff", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp")) kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki")) ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd")) kd = request->getParam("kd")->value().toFloat();
    if (request->hasParam("kff")) kff = request->getParam("kff")->value().toFloat();
    request->send(200, "text/plain", "ok");
  });
  server.on("/resetEnc", HTTP_GET, [](AsyncWebServerRequest *request) {
    encTotL = encTotR = 0;
    prevEncL = prevEncR = 0;
    speedL = speedR = 0;
    alignMode = false;
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    request->send(200, "text/plain", "enc reset");
  });
  server.on("/resetOdom", HTTP_GET, [](AsyncWebServerRequest *request) {
    odomX = odomY = odomTh = 0;
    request->send(200, "text/plain", "odom reset");
  });
  // Тестовая главная страница (необязательно)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain",
                  "Endpoints: /state /setSpeed /setCoeff /resetEnc /resetOdom");
  });
}



/* ---------- Управление моторами (PWM) ---------- */
inline void setPWM(uint8_t pin, uint8_t value) {
  analogWrite(pin, value);
  // Обновляем глобальные duty-переменные для мониторинга
  switch (pin) {
    case MOTOR_LA: dutyLA = value; break;
    case MOTOR_LB: dutyLB = value; break;
    case MOTOR_RA: dutyRA = value; break;
    case MOTOR_RB: dutyRB = value; break;
  }
}
inline void stopMotors() {
  setPWM(MOTOR_LA, 0);
  setPWM(MOTOR_LB, 0);
  setPWM(MOTOR_RA, 0);
  setPWM(MOTOR_RB, 0);
}



/* ---------- Инициализация счетчиков PCNT ---------- */
void pcntInit(pcnt_unit_t unit, gpio_num_t pulse_pin, gpio_num_t ctrl_pin) {
  pcnt_config_t cfg = {};
  cfg.unit = unit;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.pulse_gpio_num = pulse_pin;
  cfg.ctrl_gpio_num = ctrl_pin;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = -32768;
  pcnt_unit_config(&cfg);
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}
inline int16_t readEncoder(pcnt_unit_t unit) {
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  pcnt_counter_clear(unit);
  return count;
}



/* ---------- PID-регулятор скорости ---------- */
void updatePID() {
  static uint32_t prevMillis = millis();
  static float iTermL = 0, iTermR = 0;
  static float prevErrorL = 0, prevErrorR = 0;
  static float lastTgtL = 0, lastTgtR = 0;

  uint32_t now = millis();
  float dt = (now - prevMillis) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  prevMillis = now;

  // Сброс интегральной части при изменении целевой скорости или переходе через 0
  if (tgtL != lastTgtL || fabs(tgtL) < 1.0f) {
    iTermL = 0;
    prevErrorL = 0;
  }
  if (tgtR != lastTgtR || fabs(tgtR) < 1.0f) {
    iTermR = 0;
    prevErrorR = 0;
  }
  lastTgtL = tgtL;
  lastTgtR = tgtR;

  // Выравнивание (P-регулятор) – корректирует цель при езде строго прямо или повороте на месте
  float corr = 0.0f;
  if (alignMode) {
    int32_t dL = encTotL - alignRefL;
    int32_t dR = encTotR - alignRefR;
    float diff_mm = (float)(dL - alignSign * dR) * MM_PER_TICK;  // разница пройденного пути (мм)
    corr = kAlign * diff_mm;                                     // мм/с коррекция
  }
  float tgtCorrL = tgtL - corr;
  float tgtCorrR = tgtR + alignSign * corr;

  // PID для левого и правого колеса
  float errorL = tgtCorrL - speedL;
  float errorR = tgtCorrR - speedR;
  iTermL += errorL * dt;
  iTermR += errorR * dt;
  // ограничиваем накопление интегральной ошибки
  const float I_LIMIT = 300.0f;
  if (iTermL > I_LIMIT) iTermL = I_LIMIT;
  if (iTermL < -I_LIMIT) iTermL = -I_LIMIT;
  if (iTermR > I_LIMIT) iTermR = I_LIMIT;
  if (iTermR < -I_LIMIT) iTermR = -I_LIMIT;
  float dTermL = (errorL - prevErrorL) / dt;
  float dTermR = (errorR - prevErrorR) / dt;
  prevErrorL = errorL;
  prevErrorR = errorR;
  // Управляющее воздействие
  float outputL = kp * errorL + ki * iTermL + kd * dTermL + kff * tgtCorrL;
  float outputR = kp * errorR + ki * iTermR + kd * dTermR + kff * tgtCorrR;
  // Ограничиваем PWM
  if (outputL > 255) outputL = 255;
  if (outputL < -255) outputL = -255;
  if (outputR > 255) outputR = 255;
  if (outputR < -255) outputR = -255;
  // Задаём PWM на моторах (A-вперёд, B-назад)
  uint8_t pwmLA, pwmLB, pwmRA, pwmRB;
  if (outputL >= 0) {
    pwmLA = (uint8_t)outputL;
    pwmLB = 0;
  } else {
    pwmLA = 0;
    pwmLB = (uint8_t)(-outputL);
  }
  if (outputR >= 0) {
    pwmRA = (uint8_t)outputR;
    pwmRB = 0;
  } else {
    pwmRA = 0;
    pwmRB = (uint8_t)(-outputR);
  }
  setPWM(MOTOR_LA, pwmLA);
  setPWM(MOTOR_LB, pwmLB);
  setPWM(MOTOR_RA, pwmRA);
  setPWM(MOTOR_RB, pwmRB);
}



/* ---------- Задача чтения Лидара (поток на Core 1) ---------- */
void lidarTask(void *param) {
  esp_task_wdt_add(NULL);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  uint8_t body[BODY_LEN];
  while (true) {
    esp_task_wdt_reset();
    vTaskDelay(1);
    if (!waitLidarHeader(Serial2)) {
      continue;
    }
    if (!readBytes(Serial2, body, BODY_LEN)) {
      continue;
    }
    // Распарсить порцию точек лидара
    float startDeg = decodeAngle(body[2] | (body[3] << 8));
    uint8_t offset = 4;
    uint16_t dist[8];
    uint8_t quality[8];
    for (int i = 0; i < 8; ++i) {
      dist[i] = body[offset] | (body[offset + 1] << 8);
      quality[i] = body[offset + 2];
      offset += 3;
    }
    float endDeg = decodeAngle(body[offset] | (body[offset + 1] << 8));
    if (endDeg < startDeg) endDeg += 360.0f;
    if (endDeg - startDeg > MAX_SPREAD_DEG) {
      // Пропускаем пакет, если слишком большой разрыв (ошибка)
      continue;
    }
    // Упаковываем 20 байт в общий буфер скана
    uint16_t s = (uint16_t)(startDeg * 100 + 0.5f);
    uint16_t e = (uint16_t)(endDeg * 100 + 0.5f);
    *wr++ = s & 0xFF;
    *wr++ = s >> 8;
    *wr++ = e & 0xFF;
    *wr++ = e >> 8;
    for (int i = 0; i < 8; ++i) {
      uint16_t d = (quality[i] >= INTENSITY_MIN) ? dist[i] : 0;
      *wr++ = d & 0xFF;
      *wr++ = d >> 8;
    }
    frameCount++;
    stat_rx++;
    if (frameCount >= MAX_FRAMES) {
      frameCount = 0;
      wr = scanBuf;
      prevStartAngle = -1;  // сброс для корректного «перескока» угла
      continue;             // переходим к следующему пакету
    }
    // Проверяем перескок через 0° (начало нового круга)
    if (prevStartAngle >= 0.0f && startDeg < prevStartAngle && frameCount >= 30) {
      size_t scanSize = frameCount * FRAME_LEN;
      // Вычисляем CRC для всего скана
      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < scanSize; ++i) {
        crc = crc16(crc, scanBuf[i]);
      }
      scanBuf[scanSize] = crc & 0xFF;
      scanBuf[scanSize + 1] = crc >> 8;
      // Отправляем по WebSocket, если клиент подключен
      if (wsClient && wsClient->canSend()) {
        wsClient->binary(scanBuf, scanSize + 2);
        stat_tx++;
      }
      // Сбрасываем буфер для следующего оборота
      wr = scanBuf;
      frameCount = 0;
    }
    prevStartAngle = startDeg;
  }
}



/* ---------- Обработчик команд ---------- */
void commandHandler() {
  String command = Serial.readString();
  command.trim();
  if (command.indexOf("getIp") == 0) {
    Serial.println(WiFi.localIP());
  } else if (command.indexOf("getBatt") == 0) {
    Serial.println(battery_charge);
  }
}



/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP32 Lidar+Motor Bridge ===");
  // Wi-Fi подключение
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  // Запуск дальномера
  if (is_ranging) is_ranging = setupRangefinder(&lox, 100000);
  if (!is_ranging) measured_distance = -1;
  // Настройка GPIO
  pinMode(MOTOR_LA, OUTPUT);
  pinMode(MOTOR_LB, OUTPUT);
  pinMode(MOTOR_RA, OUTPUT);
  pinMode(MOTOR_RB, OUTPUT);
  stopMotors();
  // PCNT для энкодеров
  pcntInit(PCNT_UNIT_0, (gpio_num_t)ENCODER_RA, (gpio_num_t)ENCODER_RB);
  pcntInit(PCNT_UNIT_1, (gpio_num_t)ENCODER_LA, (gpio_num_t)ENCODER_LB);
  // Запуск веб-сервера и WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  setupRoutes();
  server.begin();
  // Запуск задачи лидара на ядре 0
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, nullptr, 2, nullptr, 0);

  strip_hor.begin();
  strip_up.begin();
  strip_hor.show();
  strip_up.show();

  battery_charge = analogRead(39) * BATTERY_COEF;
  total_ram = ESP.getHeapSize();
}



/* ---------- LOOP ---------- */
void loop() {
  static uint32_t t10 = 0, t20 = 0, t2000 = 0;
  uint32_t now = millis();

  // Каждые 10 мс: считываем энкодеры, обновляем одометрию
  if (now - t10 >= 10) {
    t10 = now;
    int16_t dR = readEncoder(PCNT_UNIT_0);
    int16_t dL = readEncoder(PCNT_UNIT_1);
    encTotR += dR;
    encTotL += dL;
    // Расстояние, пройденное каждым колесом за 10 мс (в метрах)
    float sR = dR * MM_PER_TICK / 1000.0f;
    float sL = dL * MM_PER_TICK / 1000.0f;
    // Обновляем одометрические координаты (в глобальной системе odom)
    float ds = 0.5f * (sR + sL);
    float dth = (sR - sL) / BASE_L;
    float midTh = odomTh + 0.5f * dth;
    odomX += ds * cosf(midTh);
    odomY += ds * sinf(midTh);
    odomTh += dth;
    // Нормализуем угол odomTh в [-pi, pi]
    if (odomTh > M_PI) odomTh -= 2 * M_PI;
    if (odomTh < -M_PI) odomTh += 2 * M_PI;
  }

  //получение значения дальномера
  if (is_ranging && lox.isRangeComplete()) {
    measured_distance = lox.readRange();
  }

  // Каждые 20 мс: вычисляем текущие скорости колес (мм/с)
  if (now - t20 >= 20) {
    float dt = (now - t20) * 0.001f;
    t20 = now;
    speedL = (encTotL - prevEncL) * MM_PER_TICK / dt;
    speedR = (encTotR - prevEncR) * MM_PER_TICK / dt;
    prevEncL = encTotL;
    prevEncR = encTotR;
  }

  // Каждые 20 мс: обновляем PID-регулятор и ШИМ моторов
  static uint32_t tPID = 0;
  if (now - tPID >= 20) {
    tPID = now;
    updatePID();
  }

  static uint32_t tStat = 0;
  static uint16_t idleCounter = 0;
  idleCounter++;
  if (now - tStat >= 1000) {
    unsigned long count = idleCounter;
    idleCounter = 0;
    cpu_temp = (temprature_sens_read() - 32) / 1.8;  // Конвертация в °C

    free_ram = ESP.getFreeHeap();
    used_ram = total_ram - free_ram;

    static unsigned long maxIdle = 0;
    if (count > maxIdle) maxIdle = count;
    cpu_load = 1.0 - (float)count / maxIdle;
    battery_charge = (battery_charge * 20 + analogRead(39) * BATTERY_COEF) / 21.0;
    switch (show1) {
      case 0: statOnStrip(&strip_up, battery_charge / 8.4, 8, 0, 180); break;
      case 1: statOnStrip(&strip_up, cpu_load, 8, 180, 0); break;
      case 2: statOnStrip(&strip_up, cpu_temp / 100.0, 8, 180, 0); break;
      case 3: statOnStrip(&strip_up, used_ram / total_ram, 8, 180, 0); break;
    }

    statOnStrip(&strip_hor, cpu_load, 10, 180, 0);

    tStat = now;
  }


  // Обслуживание клиентов WebSocket (освобождение памяти для отключившихся)
  ws.cleanupClients();

  if (Serial.available()) {
    commandHandler();
  }
}