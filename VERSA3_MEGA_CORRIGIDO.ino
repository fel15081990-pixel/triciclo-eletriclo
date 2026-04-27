// VERSA3_MEGA_CORRIGIDO.ino

#include <Arduino.h>

// Constants and variables
const int RAIN_SENSOR_PIN = 2;
const int CURRENT_SENSOR_PIN = A0;
const int SPEED_LIMITER_PIN = 3;
const int DEBOUNCE_TIME = 50; // milliseconds
const int ALARM_RESET_TIME = 5000; // milliseconds

volatile int odometer = 0;
volatile int speed = 0;
volatile float current = 0.0;
bool alarmActive = false;
unsigned long lastKeyPressTime = 0;
unsigned long lastRainSensorCheck = 0;
unsigned long lastCurrentCheck = 0;
unsigned long alarmResetTime = 0;

void setup() {
    pinMode(RAIN_SENSOR_PIN, INPUT);
    pinMode(CURRENT_SENSOR_PIN, INPUT);
    pinMode(SPEED_LIMITER_PIN, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    checkRainSensor();
    handleCurrentSensor();
    // Handle speed limiter logic
    enforceSpeedLimiter();
    // Processing RPM
    processRPM();
    // Reset alarm if needed
    autoResetAlarm();
}

void checkRainSensor() {
    if (millis() - lastRainSensorCheck >= DEBOUNCE_TIME) {
        if (digitalRead(RAIN_SENSOR_PIN) == HIGH) {
            // Rain detected
            alarmActive = true;
        } else {
            alarmActive = false;
        }
        lastRainSensorCheck = millis();
    }
}

void handleCurrentSensor() {
    if (millis() - lastCurrentCheck >= DEBOUNCE_TIME) {
        current = analogRead(CURRENT_SENSOR_PIN);
        if (current < 0) {
            // Handle negative current
            current = 0;
        }
        lastCurrentCheck = millis();
    }
}

void processRPM() {
    // Chained RPM processing logic would go here
}

void enforceSpeedLimiter() {
    if (speed > 30) { // Example speed limit
        digitalWrite(SPEED_LIMITER_PIN, LOW);
    } else {
        digitalWrite(SPEED_LIMITER_PIN, HIGH);
    }
}

void autoResetAlarm() {
    if (alarmActive && (millis() - alarmResetTime > ALARM_RESET_TIME)) {
        alarmActive = false;
        alarmResetTime = millis();
    }
}