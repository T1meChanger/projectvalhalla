#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <FastLED.h> // Подключение библиотеки для управления светодиодами WS2812B

#define LED_PIN 26 // Пин, к которому подключены светодиоды WS2812B
#define NUM_LEDS 1 // Количество светодиодов

CRGB leds[NUM_LEDS]; // Объявление массива для управления свето диодами

// Определение пинов управления моторами через драйвер MX1508
const int motorPins[3][3] = {
  {22, 23}  // Пины управления направлением и скоростью для мотора 1
  {24, 25}   // Пины управления направлением и скоростью для мотора 2
  {30, 31} //Пины на подьем
};

// Пин подключения SDA и SCL для ESP32
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int MAX_MOTOR_SPEED = 250;

Servo servo1;
Servo servo2; //объявление сервоприводов
int angle;

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

    servo1.attach(28);//привязываем сервопривод к аналоговому выходу 27
  servo2.attach(29);//привязываем сервопривод к аналоговому выходу 21

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

   if (GamePad.isTrainglePressed()) {
   for(angle=30; angle<=90; angle++){
   servo1.write(angle);
   delay(20);
   }
  } 
  else if (GamePad.isCrossPressed()) {
    for(angle=90; angle>=30; angle--){
   servo1.write(angle);
   delay(20);
    }
  } 
  else if (GamePad.isSquarePressed()) {
    for(angle=30; angle<=90; angle++){
   servo2.write(angle);
   delay(20);
   }
  } 
  else if (GamePad.isCirclePressed()) {
    for(angle=90; angle>=30; angle--){
   servo2.write(angle);
   delay(20);
    }
  }
  if (GamePad.isSelectPressed()) {

  }
//ПОДЪЕМ КРЫШКИ
// Номера пинов для подключения мотора
#define MOTOR_PIN_STEP  2
#define MOTOR_PIN_DIR   3

// Количество шагов на один оборот мотора
#define STEPS_PER_REVOLUTION 200

// Количество оборотов, на которые нужно прокрутить мотор
#define TARGET_REVOLUTIONS 0.3

// Инициализация экземпляра мотора (AccelStepper)
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_PIN_STEP, MOTOR_PIN_DIR);

// Флаг, указывающий направление движения мотора
bool forwardDirection = true;

// Переменная для отслеживания состояния кнопки "select"
bool selectButtonPressed = false;

void setup() {
  // Инициализация библиотеки Dabble
  Dabble.begin();

  // Настройка скорости и ускорения мотора
  stepper.setMaxSpeed(1000); // Указать максимальную скорость в шагах в секунду
  stepper.setAcceleration(500); // Указать ускорение в шагах в секунду^2
  
  // Вычисление количества шагов для прокрутки на 0.3 оборота
  int targetSteps = TARGET_REVOLUTIONS * STEPS_PER_REVOLUTION;
  
  // Перемещение мотора на заданное количество шагов в начальном направлении
  stepper.move(targetSteps);
}

void loop() {
  // Обновление состояния мотора
  stepper.run();

  // Проверка нажатия кнопки "select" через Dabble
  if (Dabble.getJoystickSelect()) {
    if (!selectButtonPressed) {
      // Изменяем состояние кнопки "select" и направление мотора
      selectButtonPressed = true;
      forwardDirection = !forwardDirection;
      
      // Вычисляем количество шагов для обратного движения
      int targetSteps = (forwardDirection ? 1 : -1) * TARGET_REVOLUTIONS * STEPS_PER_REVOLUTION;
      
      // Перемещаем мотор в новом направлении
      stepper.move(targetSteps);
    }
  } else {
    // Сбрасываем флаг, когда кнопка "select" отпущена
    selectButtonPressed = false;
  }
}
}
