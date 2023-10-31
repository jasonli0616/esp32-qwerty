#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

const int Y_AXIS_PIN = 2;
const int X_AXIS_PIN = 4;

void setup() {
  Dabble.begin("ESP32-QWERTY");

  pinMode(Y_AXIS_PIN, OUTPUT);
  pinMode(X_AXIS_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("READY");
}

void loop() {
  Dabble.processInput();

  double yAxis = map(GamePad.gety_axis(), -7, 7, 0, 255);
  double xAxis = map(GamePad.getx_axis(), -7, 7, 0, 255);

  analogWrite(Y_AXIS_PIN, yAxis);
  analogWrite(X_AXIS_PIN, xAxis);

}