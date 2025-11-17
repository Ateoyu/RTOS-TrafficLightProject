#include <Arduino.h>
#include <task.h>
#include <queue.h>
#include <cstdio>

QueueHandle_t xQueue;
SemaphoreHandle_t semaphoreSensor;
SemaphoreHandle_t semaphoreButton;

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
int currentButtonState;
int lastButtonState = HIGH;

volatile bool pedestrianCrossingOn = false;

//queue struct
enum Cause { NORMAL, BUTTON, SENSOR };

typedef struct {
    Cause cause;
} LightCommand;

//Light transitions
void redToGreen(const int redLedPin, const int amberLedPin, const int greenLedPin) {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(amberLedPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(redLedPin, LOW);
    digitalWrite(amberLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
}

void greenToRed(const int redLedPin, const int amberLedPin, const int greenLedPin) {
    digitalWrite(greenLedPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(greenLedPin, LOW);
    digitalWrite(amberLedPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(600));
    digitalWrite(amberLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
}

// ISR Handling
void IRAM_ATTR pedestrianButton() {
    currentButtonState = digitalRead(button);

    if (!pedestrianCrossingOn && lastButtonState == LOW && currentButtonState == HIGH) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(semaphoreButton, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    lastButtonState = currentButtonState;
}

void vehicleSensor() { //TODO:
    // Clears the rangeFinderTrig
    digitalWrite(rangeFinderTrig, LOW);
    delayMicroseconds(2);
    // Sets the rangeFinderTrig on HIGH state for 10 micro seconds
    digitalWrite(rangeFinderTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(rangeFinderTrig, LOW);

    // Reads the rangeFinderEcho, returns the sound wave travel time in microseconds
    unsigned long duration = pulseIn(rangeFinderEcho, HIGH);

    // Calculate the distance
    float distanceCm = duration * soundSpeed / 2;

    // Prints the distance in the Serial Monitor
    Serial.print("Distance (cm): ");
    Serial.println(distanceCm);
}

// every 10s change from pedestrian -> car (add tasks to queue)
// if pedestrianButton sends ISR, change after 1s instead of 10s (execute next switch from queue)
// if vehicleSensor sends ISR, change after 3s instead of 10s (execute next switch from queue)
// if pedestrianCrossing on == buzzer buzz
// log all events into Serial via taskLogger

// queue works on bool,
// true -> green light for car, red for pedestrian
// false -> red light for car, green for pedestrian
// differentiate how through struct entry, labelling what caused the change to decide how long to wait till change.

//TASK 1 (change lights)
void changeLights(void *pvParameters) {
    LightCommand cmd;

    for (;;) {
        if (xQueueReceive(xQueue, &cmd, portMAX_DELAY) == pdPASS) {
            pedestrianCrossingOn = !pedestrianCrossingOn;

            if (pedestrianCrossingOn) {
                // Pedestrians green
                greenToRed(trafficLightRed, trafficLightYellow, trafficLightGreen);
                digitalWrite(pedestrianLightRed, LOW);
                digitalWrite(pedestrianLightGreen, HIGH);
            } else {
                // Cars green
                digitalWrite(pedestrianLightGreen, LOW);
                digitalWrite(pedestrianLightRed, HIGH);
                redToGreen(trafficLightRed, trafficLightYellow, trafficLightGreen);
            }
        }
    }
}

//TASK 2 (buzzer)
void buzzerSound(void *pvParameters) {
    for (;;) {
        if (pedestrianCrossingOn) {
            tone(buzzer, NOTE_C4, 100);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//TASK 3 (main controller)
void taskController(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(semaphoreButton, portMAX_DELAY) == pdTRUE) {
            LightCommand cmd = {BUTTON};
            xQueueSend(xQueue, &cmd, 0);
        } else if (xSemaphoreTake(semaphoreSensor, portMAX_DELAY) == pdTRUE) {
            LightCommand cmd = {SENSOR};
            xQueueSend(xQueue, &cmd, 0);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10000));
            LightCommand cmd = {NORMAL};
            xQueueSend(xQueue, &cmd, 0);
        }
    }
}

//TASK 4 (logger)
void taskLogger(void *pvParameters) {
    LightCommand log;

    for (;;) {
        if (xQueueReceive(xQueue, &log, portMAX_DELAY) == pdPASS) {
            Serial.printf("LOG → Cause: %d, pedestrianCrossingOn: %d\n",
                          log.cause, pedestrianCrossingOn);
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

    xQueue = xQueueCreate(5, sizeof(LightCommand));
    semaphoreSensor = xSemaphoreCreateBinary();
    semaphoreButton = xSemaphoreCreateBinary();

    if (xQueue != nullptr && semaphoreSensor != nullptr && semaphoreButton != nullptr) {
        attachInterrupt(digitalPinToInterrupt(button), pedestrianButton, CHANGE);

        xTaskCreate(changeLights, "changeLights", 2000, nullptr, 1, nullptr);
        xTaskCreate(buzzerSound, "buzzer", 1000, nullptr, 1, nullptr);
        xTaskCreate(taskController, "controller", 2000, nullptr, 2, nullptr);
        xTaskCreate(taskLogger, "logger", 2000, nullptr, 1, nullptr);
    } else {
        printf("Failed to create queue\n");
    }
}

void loop() {
}
