#include <Arduino.h>
#include <task.h>
#include <queue.h>
#include <cstdio>

QueueHandle_t xLoggerQueue;
SemaphoreHandle_t sensorSemaphore;
SemaphoreHandle_t buttonSemaphore;

const int trafficLightRed = 6;
const int trafficLightYellow = 5;
const int trafficLightGreen = 4;

const int pedestrianLightRed = 2;
const int pedestrianLightGreen = 1;

const int buzzer = 0;
const int NOTE_C4 = 262;

const int rangeFinderEcho = 17;
const int rangeFinderTrig = 18;
const float soundSpeed = 0.034;

const int button = 10;
//queue struct
enum Cause { NORMAL, BUTTON, SENSOR };

typedef struct {
    Cause cause;
} LightCommand;

//Light transitions
void redToGreenTraffic(const int redLedPin, const int yellowLedPin, const int greenLedPin) {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(yellowLedPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
}

void greenToRedTraffic(const int redLedPin, const int yellowLedPin, const int greenLedPin) {
    digitalWrite(greenLedPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
}

void redToGreenPedestrian(const int redLed, const int greenLed) {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
}

void greenToRedPedestrian(const int redLed, const int greenLed) {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
}

void triggerLightSwitch() {
    if (digitalRead(pedestrianLightGreen) == HIGH) {
        redToGreenTraffic(trafficLightRed, trafficLightYellow, trafficLightGreen);
        greenToRedPedestrian(pedestrianLightRed, pedestrianLightGreen);
    } else {
        greenToRedTraffic(trafficLightRed, trafficLightYellow, trafficLightGreen);
        redToGreenPedestrian(pedestrianLightRed, pedestrianLightGreen);
    }
}

void IRAM_ATTR handlePedestrianButtonISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

[[noreturn]] void taskChangeLightsFromPedestrianButtonISR(void *pv) {
    for (;;) {
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            if (digitalRead(trafficLightGreen) == HIGH) {
                vTaskDelay(pdMS_TO_TICKS(300));
                triggerLightSwitch();
                LightCommand cmd = {BUTTON};
                xQueueSend(xLoggerQueue, &cmd, 0);
            } else {
                Serial.printf("Button pressed, but pedestrian already have green light.\n");
            }
        }
    }
}

[[noreturn]] void taskDefaultLightBehaviour(void *pv) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        triggerLightSwitch();
        LightCommand cmd = {NORMAL};
        xQueueSend(xLoggerQueue, &cmd, 0);
    }
}

[[noreturn]] void taskLogger(void *pv) {
    LightCommand receivedCmd;
    for (;;) {
        if (xQueueReceive(xLoggerQueue, &receivedCmd, portMAX_DELAY) == pdPASS) {
            switch (receivedCmd.cause) {
                case NORMAL: Serial.printf("Received Normal light switch command.\n"); break;
                case BUTTON: Serial.printf("Received Button light switch command.\n"); break;
                case SENSOR: Serial.printf("Received Sensor light switch command.\n"); break;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(trafficLightRed, OUTPUT);
    pinMode(trafficLightYellow, OUTPUT);
    pinMode(trafficLightGreen, OUTPUT);
    pinMode(pedestrianLightRed, OUTPUT);
    pinMode(pedestrianLightGreen, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(rangeFinderEcho, INPUT);
    pinMode(rangeFinderTrig, OUTPUT);
    pinMode(button, INPUT_PULLUP);

    digitalWrite(trafficLightGreen, HIGH);
    digitalWrite(pedestrianLightRed, HIGH);

    sensorSemaphore = xSemaphoreCreateBinary();
    buttonSemaphore = xSemaphoreCreateBinary();
    xLoggerQueue = xQueueCreate(5, sizeof(LightCommand));
    attachInterrupt(digitalPinToInterrupt(button), handlePedestrianButtonISR, FALLING);

    xTaskCreate(taskDefaultLightBehaviour, "default", 2048, nullptr, 1, nullptr);
    xTaskCreate(taskLogger, "logger", 2048, nullptr, 1, nullptr);
    xTaskCreate(taskChangeLightsFromPedestrianButtonISR, "changeLightsButtonISR", 2048, nullptr, 2, nullptr);
}

void loop() {}