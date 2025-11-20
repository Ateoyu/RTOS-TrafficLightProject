#include <Arduino.h>
#include <task.h>
#include <queue.h>
#include <cstdio>

QueueHandle_t xLoggerQueue;
SemaphoreHandle_t sensorSemaphore;
SemaphoreHandle_t buttonSemaphore;
TaskHandle_t xDefaultLightTaskHandle;
TaskHandle_t xBuzzerTaskHandle;
TimerHandle_t xLightTimer;

struct TrafficLightPins {
    int red;
    int yellow;
    int green;
};

struct PedestrianLightPins {
    int red;
    int green;
};

constexpr TrafficLightPins traffic = {6, 5, 4};
constexpr PedestrianLightPins pedestrian = {2, 1};

constexpr int buzzer = 0;
constexpr int NOTE_C4 = 262;

constexpr int rangeFinderEcho = 17;
constexpr int rangeFinderTrig = 18;
constexpr float soundSpeed = 0.034;

constexpr int button = 10;

//queue struct
enum Cause { NORMAL, BUTTON };

typedef struct {
    Cause cause;
} LightCommand;

//Light transitions
void redToGreenTraffic(const int redLed, const int yellowLed, const int greenLed) {
    digitalWrite(redLed, HIGH);
    digitalWrite(yellowLed, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(redLed, LOW);
    digitalWrite(yellowLed, LOW);
    digitalWrite(greenLed, HIGH);
}

void greenToRedTraffic(const int redLed, const int yellowLed, const int greenLed) {
    digitalWrite(greenLed, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(greenLed, LOW);
    digitalWrite(yellowLed, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(yellowLed, LOW);
    digitalWrite(redLed, HIGH);
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
    if (digitalRead(pedestrian.green) == HIGH) {
        redToGreenTraffic(traffic.red, traffic.yellow, traffic.green);
        greenToRedPedestrian(pedestrian.red, pedestrian.green);
        xTaskNotifyGive(xBuzzerTaskHandle);
    } else {
        greenToRedTraffic(traffic.red, traffic.yellow, traffic.green);
        redToGreenPedestrian(pedestrian.red, pedestrian.green);
        xTaskNotifyGive(xBuzzerTaskHandle);
    }
}

void IRAM_ATTR handlePedestrianButtonISR() {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vLightTimerCallback(TimerHandle_t xTimer) {
    xTaskNotifyGive(xDefaultLightTaskHandle);
}

[[noreturn]] void taskChangeLightsFromPedestrianButtonISR(void *pv) {
    for (;;) {
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            if (digitalRead(traffic.green) == HIGH) {
                vTaskDelay(pdMS_TO_TICKS(300));
                xTimerReset(xLightTimer, 0);
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
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        triggerLightSwitch();
        LightCommand cmd = {NORMAL};
        xQueueSend(xLoggerQueue, &cmd, 0);
    }
}

[[noreturn]] void taskBuzzer(void *pv) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (digitalRead(pedestrian.green) == HIGH) {
            digitalWrite(buzzer, HIGH);
            vTaskDelay(pdMS_TO_TICKS(150));
            digitalWrite(buzzer, LOW);
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        digitalWrite(buzzer, LOW);
    }
}


[[noreturn]] void taskLogger(void *pv) {
    LightCommand receivedCmd;
    for (;;) {
        if (xQueueReceive(xLoggerQueue, &receivedCmd, portMAX_DELAY) == pdPASS) {
            switch (receivedCmd.cause) {
                case NORMAL: Serial.printf("Received Normal light switch command.\n"); break;
                case BUTTON: Serial.printf("Received Button light switch command.\n"); break;
                // case SENSOR: Serial.printf("Received Sensor light switch command.\n"); break;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(traffic.red, OUTPUT);
    pinMode(traffic.yellow, OUTPUT);
    pinMode(traffic.green, OUTPUT);

    pinMode(pedestrian.red, OUTPUT);
    pinMode(pedestrian.green, OUTPUT);

    pinMode(buzzer, OUTPUT);

    pinMode(rangeFinderEcho, INPUT);
    pinMode(rangeFinderTrig, OUTPUT);

    pinMode(button, INPUT_PULLUP);

    digitalWrite(traffic.green, HIGH);
    digitalWrite(pedestrian.red, HIGH);

    sensorSemaphore = xSemaphoreCreateBinary();
    buttonSemaphore = xSemaphoreCreateBinary();
    xLoggerQueue = xQueueCreate(5, sizeof(LightCommand));
    attachInterrupt(digitalPinToInterrupt(button), handlePedestrianButtonISR, FALLING);

    xTaskCreate(taskLogger, "logger", 2048, nullptr, 1, nullptr);
    xTaskCreate(taskChangeLightsFromPedestrianButtonISR, "changeLightsButtonISR", 2048, nullptr, 2, nullptr);
    xTaskCreate(taskDefaultLightBehaviour, "default", 2048, nullptr, 1, &xDefaultLightTaskHandle);
    xTaskCreate(taskBuzzer, "buzzer", 2048, nullptr, 1, &xBuzzerTaskHandle);
    // Create 10-second auto reload timer
    xLightTimer = xTimerCreate(
        "LightTimer",
        pdMS_TO_TICKS(10000),
        pdTRUE,
        nullptr,
        vLightTimerCallback
    );
    xTimerStart(xLightTimer, 0);
}

void loop() {}
