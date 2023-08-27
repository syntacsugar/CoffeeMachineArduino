#include <HX711.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

double kp = 0.6;
double ki = 0.15;
double kd = 0.06;

double setpoint1 = 25.0;
double setpoint2 = 30.0;
int tempPin1 = A0;
int tempPin2 = A1;

int buttonPin1 = 8;
int buttonPin2 = 9;
int outputPin1 = 10;
int outputPin2 = 11;

const int flowMeterPin = 7;
volatile int pulseCount = 0;
double flowRate = 0.0;
unsigned long lastPulseTime = 0;

const int loadCellDoutPin = 6;
const int loadCellSckPin = 7;
HX711 scale;

double input1, input2;
double output1, output2;
PID pid1(&input1, &output1, &setpoint1, kp, ki, kd, DIRECT);
PID pid2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT);

unsigned long previousMillis = 0;
const long interval = 1000;

double totalVolume = 0.0;

void setup() {
  Serial.begin(9600);
  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);

  pinMode(flowMeterPin, INPUT_PULLUP);

  lcd.begin(16, 2);

  attachInterrupt(digitalPinToInterrupt(flowMeterPin), countPulse, FALLING);

  scale.begin(loadCellDoutPin, loadCellSckPin);
  scale.set_scale();
  scale.tare();
}

void loop() {
  unsigned long currentMillis = millis();

  readSerialCommands();

  int sensorValue1 = analogRead(tempPin1);
  input1 = (sensorValue1 * 5.0 / 1023.0 - 0.5) * 100.0;

  int sensorValue2 = analogRead(tempPin2);
  input2 = (sensorValue2 * 5.0 / 1023.0 - 0.5) * 100.0;

  flowRate = pulseCount * 0.01;

  double weight = scale.get_units();

  unsigned long currentTime = millis();
  double elapsedTime = (currentTime - lastPulseTime) / 1000.0;
  double volume = flowRate * elapsedTime;
  totalVolume += volume;

  lastPulseTime = currentTime;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    pid1.Compute();
    pid2.Compute();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp1: ");
    lcd.print(input1);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Temp2: ");
    lcd.print(input2);
    lcd.print("C");

    Serial.print("Temp1: ");
    Serial.print(input1);
    Serial.print("C, Temp2: ");
    Serial.print(input2);
    Serial.print("C, Button1: ");
    Serial.print(digitalRead(buttonPin1) == LOW ? "Pressed" : "Released");
    Serial.print(", Button2: ");
    Serial.print(digitalRead(buttonPin2) == LOW ? "Pressed" : "Released");
    Serial.print(", Output1: ");
    Serial.print(digitalRead(outputPin1) == HIGH ? "On" : "Off");
    Serial.print(", Output2: ");
    Serial.print(digitalRead(outputPin2) == HIGH ? "On" : "Off");
    Serial.print(", Flow Rate: ");
    Serial.print(flowRate);
    Serial.print(" L/min, Total Volume: ");
    Serial.print(totalVolume);
    Serial.println(" L");

    lcd.setCursor(0, 1);
    lcd.print("Volume: ");
    lcd.print(totalVolume);
    lcd.print("L");
  }

  if (digitalRead(buttonPin1) == LOW) {
    digitalWrite(outputPin1, HIGH);
  } else {
    digitalWrite(outputPin1, LOW);
  }

  if (digitalRead(buttonPin2) == LOW) {
    digitalWrite(outputPin2, HIGH);
  } else {
    digitalWrite(outputPin2, LOW);
  }
}

void readSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("setpoint1:")) {
      setpoint1 = command.substring(10).toDouble();
    } else if (command.startsWith("setpoint2:")) {
      setpoint2 = command.substring(10).toDouble();
    } else if (command.startsWith("kp:")) {
      kp = command.substring(3).toDouble();
      pid1.SetTunings(kp, ki, kd);
      pid2.SetTunings(kp, ki, kd);
    } else if (command.startsWith("ki:")) {
      ki = command.substring(3).toDouble();
      pid1.SetTunings(kp, ki, kd);
      pid2.SetTunings(kp, ki, kd);
    } else if (command.startsWith("kd:")) {
      kd = command.substring(3).toDouble();
      pid1.SetTunings(kp, ki, kd);
      pid2.SetTunings(kp, ki, kd);
    }
  }
}

void countPulse() {
  pulseCount++;
}
