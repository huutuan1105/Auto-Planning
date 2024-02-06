#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include "semphr.h"

int moisturePin = A0; // khai bao chan A0 cua cam bien do am
float moisture = 0.0;
float currentMoisture = 0.0;
unsigned long pumpStartTime = 0;

// Khai báo Task Handlers
TaskHandle_t measureMoistureTaskHandler;
TaskHandle_t controlPumpTaskHandler;

SemaphoreHandle_t moistureSemaphore;

// Prototype của các Task
void measureMoistureTask(void *pvParameters);
void controlPumpTask(void *pvParameters);

void setup() {
  
  pinMode(10, OUTPUT);
  pinMode(A0, INPUT);

  xTaskCreate(measureMoistureTask, "MeasureMoistureTask", 128, NULL, 1, &measureMoistureTaskHandler);
  xTaskCreate(controlPumpTask, "ControlPumpTask", 128, NULL, 2, &controlPumpTaskHandler);

  moistureSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(moistureSemaphore);

  Serial.begin(9600);
}
void loop() {

}
void measureMoistureTask(void *pvParameter) {
  while (1) {
    int sensorValue = analogRead(moisturePin);// đọc giá trị analog của cảm biến
    moisture = map(sensorValue, 0, 1023, 0, 100); // chuyển giá trị analog sang %
    moisture = 100 - moisture;
    Serial.print("moisture: ");// in giá trị moisture
    Serial.print(moisture);
    Serial.print("% ");

    xSemaphoreTake(moistureSemaphore, portMAX_DELAY);
    currentMoisture = moisture;
    xSemaphoreGive(moistureSemaphore);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void controlPumpTask(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    xSemaphoreTake(moistureSemaphore, portMAX_DELAY);
    currentMoisture = moisture;
    xSemaphoreGive(moistureSemaphore);

    Serial.print("currentMoisture = ");
    Serial.println(currentMoisture);

    if (currentMoisture < 60) {
      if (currentMoisture == 0) {
        digitalWrite(10, LOW);
      } else {
        vTaskSuspend(measureMoistureTaskHandler); 
        digitalWrite(10, HIGH);// bật máy bơm
        pumpStartTime = millis();

        vTaskDelay(4000 / portTICK_PERIOD_MS); // bơm trong 4s

        digitalWrite(10, LOW);//tắt

        vTaskResume(measureMoistureTaskHandler);
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
