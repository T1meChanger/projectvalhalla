#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_BUTTON_MODULE // Включение модуля управления кнопками
#include <DabbleESP32.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <FastLED.h> // Подключение библиотеки для управления светодиодами WS2812B

#define LED_PIN 26 // Пин, к которому подключены светодиоды WS2812B
#define NUM_LEDS 1 // Количество светодиодов

CRGB leds[NUM_LEDS]; // Объявление массива для управления свето диодами

// Определение пинов управления моторами через драйвер MX1508
const int motorPins[2][2] = {
  {22, 23},  // Пины управления направлением и скоростью для мотора 1
  {24, 25}   // Пины управления направлением и скоростью для мотора 2
};

// Пин подключения SDA и SCL для ESP32 (может отличаться в зависимости от используемой платы)
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int MAX_MOTOR_SPEED = 250;

bool isSnifferActive = false; // Флаг активации сниффера Bluetooth

// Адрес устройства, с которого происходит управление
uint8_t controlDeviceAddress[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

void rotateMotors(MotorDirection directions[], int speeds[]) {
  for (int i = 0; i < 2; ++i) {
    digitalWrite(motorPins[i][0], directions[i] == MOTOR_FORWARD ? HIGH : LOW);
    // Используем analogWrite() для управления скоростью
    analogWrite(motorPins[i][1], abs(speeds[i]));
  }
}

void setUpPinModes() {
  for (int i = 0; i < 2; ++i) {
    pinMode(motorPins[i][0], OUTPUT);  // Направление
    pinMode(motorPins[i][1], OUTPUT);  // Скорость (управление ШИМ)
  }
  rotateMotors((MotorDirection[]){MOTOR_STOP, MOTOR_STOP}, (int[]){0, 0}); // Остановить моторы при инициализации
}

void setup() {
  Serial.begin(9600);
  
  setUpPinModes();
  
  // Инициализация датчика цвета
  if (!tcs.begin()) {
    Serial.println("Не удалось найти датчик цвета, проверьте подключение!");
    while (1);
  }
  
  // Инициализация светодиода WS2812B
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  
  Dabble.begin("абоба"); // Инициализация Dabble

  // Регистрация обработчика для кнопки в Dabble
  Button.attach(ButtonPin, onButtonPress); // Подключение кнопки к Dabble

  // Инициализация Bluetooth
  esp_bt_controller_init();
  esp_bt_controller_enable(ESP_BT_MODE_BTDM);

  // Включение режима сниффера Bluetooth
  esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);

  // Регистрация колбэка для обработки найденных устройств
  esp_bt_gap_register_callback([](esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    if (event == ESP_BT_GAP_DISC_RES_EVT) {
      for (int i = 0; i < param->disc_res.num_prop; i++) {
        if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR) {
          // Проверка адреса устройства
          bool isControlDevice = true;
          for (int j = 0; j < 6; j++) {
            if (param->disc_res.bda[0][j] != controlDeviceAddress[j]) {
              isControlDevice = false;
              break;
            }
          }
          if (!isControlDevice) {
            // Если адрес устройства не соответствует устройству управления, выводим его в Serial Monitor
            Serial.print("Found device: ");
            Serial.println(param->disc_res.bda[0]);
          }
          break;
        }
      }
    }
  });

  // Начать сканирование устройств
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}

void loop() {
  int motorSpeeds[2] = {0, 0};
  MotorDirection motorDirections[2] = {MOTOR_STOP, MOTOR_STOP};
  
  Dabble.processInput(); // Обработка ввода Dabble
  
  // Чтение цвета
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  
  // Преобразование сырых данных в значения цвета
  float colorTemp, lux;
  uint16_t hue, saturation;
  tcs.calculateColorTemperature(r, g, b, &colorTemp);
  tcs.calculateLux(r, g, b, &lux);
  tcs.calculateColor(r, g, b, &hue, &saturation);
  
  // Вывод результатов
  Serial.print("Color Temp: "); Serial.print(colorTemp); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, 2); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r); Serial.print(" ");
  Serial.print("G: "); Serial.print(g); Serial.print(" ");
  Serial.print("B: "); Serial.print(b); Serial.print(" ");
  Serial.print("C: "); Serial.print(c); Serial.print(" ");
  Serial.print("H: "); Serial.print(hue); Serial.print(" ");
  Serial.print("S: "); Serial.print(saturation); Serial.print(" ");
  Serial.println(" ");
  
  // Отображение цвета на светодиоде WS2812B
  leds[0] = CRGB(r, g, b);
  FastLED.show();
  
  if (isSnifferActive) {
    // Если сниффер Bluetooth активен, обработка сигналов Bluetooth здесь
    // Например, можно добавить код для сканирования и обработки найденных устройств
    // Также можно остановить сканирование Bluetooth при необходимости
  }
  
  if (GamePad.isUpPressed()) {
    for (int i = 0; i < 2; ++i)
      motorSpeeds[i] = MAX_MOTOR_SPEED;
  } 
  else if (GamePad.isDownPressed()) {
    for (int i = 0; i < 2; ++i)
      motorSpeeds[i] = -MAX_MOTOR_SPEED;
  } 
  else if (GamePad.isLeftPressed()) {
    motorSpeeds[0] = MAX_MOTOR_SPEED;
    motorSpeeds[1] = -MAX_MOTOR_SPEED;
  } 
  else if (GamePad.isRightPressed()) {
    motorSpeeds[0] = -MAX_MOTOR_SPEED;
    motorSpeeds[1] = MAX_MOTOR_SPEED;
  }

  for (int i = 0; i < 2; ++i) {
    motorDirections[i] = motorSpeeds[i] > 0 ? MOTOR_FORWARD : (motorSpeeds[i] < 0 ? MOTOR_BACKWARD : MOTOR_STOP);
  }

  rotateMotors(motorDirections, motorSpeeds);
}

// Обработчик нажатия на кнопку
void onButtonPress() {
  if (Button.isPressed()) {
    isSnifferActive = !isSnifferActive; // Переключение состояния сниффера Bluetooth
    if (isSnifferActive) {
      // Включение режима сниффера Bluetooth
      esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
      esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    } else {
      // Отключение режима сниффера Bluetooth
      esp_bt_gap_stop_discovery();
    }
  }
}