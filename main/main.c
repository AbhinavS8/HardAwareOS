#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_system.h"

SemaphoreHandle_t xPauseSemaphore = NULL;  // for pausing tasks

extern uint8_t temprature_sens_read();

typedef struct {
    float cpu_usage;
    float temperature;
    float voltage;
} SystemStats;

// Constants
#define VOLTAGE_THRESHOLD 3.0 
#define ADC_CHANNEL ADC1_CHANNEL_0  // ADC1 Channel 0 (GPIO36 on ESP32)
#define MAX_ADC_READING 4095.0
#define MAX_VOLTAGE 3.3 

float get_voltage() {
    int raw = adc1_get_raw(ADC_CHANNEL);
    return (raw / MAX_ADC_READING) * MAX_VOLTAGE;
}

float get_internal_temp() {
    uint8_t raw_temp = temprature_sens_read();
    return (raw_temp - 32) / 1.8;
}

// use internal thermo 
void temperature_check_task() {
    SystemStats stats;

    while (1) {
        stats.temperature = get_internal_temp();
        stats.cpu_usage = (float)uxTaskGetNumberOfTasks();
        printf("Temp: %.2f°C | Voltage: %.2fV | Active Tasks: %.2f\n",
               stats.temperature, get_voltage(), stats.cpu_usage);

        if (stats.temperature >= 70) {
            printf("HIGH TEMPERATURE DETECTED! PAUSING ALL TASKS...\n");
            xSemaphoreTake(xPauseSemaphore, portMAX_DELAY);

            while (get_internal_temp() >= 70) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            printf("Temperature normalized. Resuming all tasks.\n");
            xSemaphoreGive(xPauseSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Voltage monitoring, checking for low voltage
void voltage_check_task() {
    while (1) {
        float voltage = get_voltage();
        
        if (voltage < VOLTAGE_THRESHOLD) {
            printf("LOW VOLTAGE DETECTED! (%.2fV) PAUSING ALL TASKS...\n", voltage);
            xSemaphoreTake(xPauseSemaphore, portMAX_DELAY);

            while (get_voltage() < VOLTAGE_THRESHOLD) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            printf("Voltage restored. Resuming all tasks.\n");
            xSemaphoreGive(xPauseSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Example workload task for simulating
void high_load_task(void *param) {
    while (1) {
        xSemaphoreTake(xPauseSemaphore, portMAX_DELAY);
        printf("[High Load] Processing data...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Another example workload task
void another_task(void *param) {
    while (1) {
        xSemaphoreTake(xPauseSemaphore, portMAX_DELAY);
        printf("[Another Task] Running...\n");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void app_main() {
    xPauseSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xPauseSemaphore);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);  // Full-scale 3.3V range

    printf("\nTime(ms) for program initiation: %lld\n", esp_timer_get_time() / 1000);

    xTaskCreate(high_load_task, "High Load Task", 2048, NULL, 1, NULL);
    xTaskCreate(another_task, "Another Task", 2048, NULL, 1, NULL);
    xTaskCreate(temperature_check_task, "Temp Watch Task", 2048, NULL, 2, NULL);
    xTaskCreate(voltage_check_task, "Voltage Watch Task", 2048, NULL, 2, NULL);

    printf("\nProgram completion: %lld\n", esp_timer_get_time() / 1000);
}


