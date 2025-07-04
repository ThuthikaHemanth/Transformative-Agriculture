// Blynk configuration

#define BLYNK_TEMPLATE_ID "TMPL3d7vveBVl"
#define BLYNK_TEMPLATE_NAME "Watering and Cutting"
#define BLYNK_AUTH_TOKEN "-J0TxFNV-x3s2X59OAvdPVDgyIwwce29"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h> // For Servo control
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


// Motor Pins
#define ENA 34
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 5
#define ENB 13

// Relay Pins
#define relay1_pin 32 //18
#define relay2_pin 33 //19

// Button Pins
#define button1_pin 18 //34
#define button2_pin 19 //35

// Temperature and Humidity sensor and soil moisture sensor
#define DHTPIN 25 //Connect Out pin to D25 in ESP32
#define SOIL_MOISTURE_PIN 12 //Connect Out pin to D12 in ESP32
#define DHTTYPE DHT22  
DHT dht(DHTPIN, DHTTYPE);

// Servo Pins
Servo servo1;
Servo servo2;
int servoPin1 = 35;
int servoPin2 = 4;

// Blynk Variables
char auth[] = "-J0TxFNV-x3s2X59OAvdPVDgyIwwce29"; // Enter your Blynk auth token
char ssid[] = "HEMANTH"; // WiFi name
char pass[] = "12345678"; // WiFi password

int x = 50, y = 50, Speed = 0, servoPosition1 = 90, pos1 = 90;
int relay1_state = 0, relay2_state = 0;

// Initialize Blynk timer
BlynkTimer timer;

// Blynk Virtual Pins
#define button1_vpin V5
#define button2_vpin V6

// Motor Control Functions
void carForward();
void carBackward();
void carLeft();
void carRight();
void carStop();

// Relay Control Functions
void control_relay(int relay);

// Function to read joystick and slider values
void smartcar() {
  if (y > 70) {
    carForward();
    Serial.println("carForward");
  } else if (y < 30) {
    carBackward();
    Serial.println("carBackward");
  } else if (x < 30) {
    carLeft();
    Serial.println("carLeft");
  } else if (x > 70) {
    carRight();
    Serial.println("carRight");
  } else {
    carStop();
    Serial.println("carStop");
  }
}

// Relay State Sync
BLYNK_CONNECTED() {
  Blynk.syncVirtual(button1_vpin);
  Blynk.syncVirtual(button2_vpin);
}

// Button State Write Handlers
BLYNK_WRITE(button1_vpin) {
  relay1_state = param.asInt();
  digitalWrite(relay1_pin, relay1_state);
}

BLYNK_WRITE(button2_vpin) {
  relay2_state = param.asInt();
  digitalWrite(relay2_pin, relay2_state);
}

// Joystick and Slider Handlers
BLYNK_WRITE(V0) { x = param[0].asInt(); }
BLYNK_WRITE(V1) { y = param[0].asInt(); }
BLYNK_WRITE(V2) { Speed = param.asInt(); }
BLYNK_WRITE(V3) { servoPosition1 = param.asInt(); }
BLYNK_WRITE(V4) { pos1 = param.asInt(); servo2.write(pos1); }


void sendSensor()
{
  int soilmoisture = analogRead(SOIL_MOISTURE_PIN);  // Read from the soil moisture sensor
  int soilmoisturepercentage = map(soilmoisture, 3500, 4095, 100, 0);
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V9, soilmoisturepercentage);
    Blynk.virtualWrite(V7, t);
    Blynk.virtualWrite(V8, h);
    Serial.print("Soil Moisture : ");
    Serial.print(soilmoisturepercentage);
    Serial.print("Temperature : ");
    Serial.print(t);
    Serial.print("Humidity : ");
    Serial.println(h);
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Relay pins
  pinMode(relay1_pin, OUTPUT);
  pinMode(relay2_pin, OUTPUT);

  // Button pins
  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);

  // Initialize relay states
  digitalWrite(relay1_pin, HIGH);
  digitalWrite(relay2_pin, HIGH);

  // Attach servos
  servo1.attach(servoPin1, 500, 2500);
  servo2.attach(servoPin2, 500, 2500);
  servo1.write(servoPosition1);

  // Connect to Blynk
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  timer.setInterval(100L, sendSensor);

  // Relay button listener
  timer.setInterval(200L, []() {
    if (digitalRead(button1_pin) == LOW) {
      control_relay(1);
      Blynk.virtualWrite(button1_vpin, relay1_state);
    }
    if (digitalRead(button2_pin) == LOW) {
      control_relay(2);
      Blynk.virtualWrite(button2_vpin, relay2_state);
    }
  });
}

void loop() {
  Blynk.run();
  timer.run();
  smartcar();
  if (servoPosition1 == 1) {
    for (int i = 0; i <= 180; i += 180) {
      servo1.write(i);
      delay(150);
    }
    for (int i = 180; i >= 0; i -= 180) {
      servo1.write(i);
      delay(150);
    }
  }
}

// Motor Movement Functions
void carForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}

void carBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}

void carLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}

void carRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}

void carStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Relay Control Function
void control_relay(int relay) {
  if (relay == 1) {
    relay1_state = !relay1_state;
    digitalWrite(relay1_pin, relay1_state);
  } else if (relay == 2) {
    relay2_state = !relay2_state;
    digitalWrite(relay2_pin, relay2_state);
  }
}
