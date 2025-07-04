#define BLYNK_PRINT Serial

// Blynk configuration
#define BLYNK_TEMPLATE_ID "TMPL3dfUzulds"
#define BLYNK_TEMPLATE_NAME "wifi controlled car"
#define BLYNK_AUTH_TOKEN "YAGURN6gKanjDxQc61O5hh2fzW9FbEpN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h> // For Servo control
#include <NewPing.h> //For UltraSonic Sensor


// Motor Pins
#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 13

// Relay Pins
#define relay1_pin 34 //18
#define relay2_pin 35 //19

// Button Pins
#define button1_pin 18 //34
#define button2_pin 19 //35

// UltraSonic Sensor Pins
#define ECHO_PIN 15     // D2
#define TRIGGER_PIN 16 // D1
#define MAX_DISTANCE 200  // Maximum distance we want to measure (in centimeters)

// Servo Pins
Servo servo1;
Servo servo2;
int servoPin1 = 33;
int servoPin2 = 32;

// Blynk Variables
char auth[] = "YAGURN6gKanjDxQc61O5hh2fzW9FbEpN"; // Enter your Blynk auth token
char ssid[] = "HEMANTH"; // WiFi name
char pass[] = "12345678"; // WiFi password

int x = 50, y = 50, Speed = 0, servoPosition1 = 90, pos1 = 90;
int relay1_state = 0, relay2_state = 0;

// Initialize NewPing library
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

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
  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensor);
}

void sendSensor() {
  // Measure distance
  unsigned int distance = sonar.ping_cm();

  // Send distance to Blynk app
  Blynk.virtualWrite(V7, distance);  // Gauge widget
  Blynk.virtualWrite(V8, distance);  // Label widget

  // Print distance to Serial Monitor for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void loop() {
  Blynk.run();
  timer.run();
  smartcar();
  if (servoPosition1 == 1) {
    for (int i = 0; i <= 90; i += 90) {
      servo1.write(i);
      delay(150);
    }
    for (int i = 90; i >= 0; i -= 90) {
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
