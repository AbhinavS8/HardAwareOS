#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern uint8_t temprature_sens_read();  

float get_internal_temp() {
    uint8_t raw_temp = temprature_sens_read();
    return (raw_temp - 32) / 1.8;  
}

// Struct to hold system resource usage
typedef struct {
    float cpu_usage;
    float temperature;
} SystemStats;

QueueHandle_t systemQueue;
TaskHandle_t highLoadTaskHandle = NULL;  // Persistent task handle

// Simulated high-load task
void high_load_task(void *pvParameter) {
    while (1) {
        printf("[High Load] Processing data...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Simulating workload
    }
}

// Task manager to monitor temperature and suspend/resume tasks
void task_manager(void *pvParameter) {
    SystemStats stats;

    while (1) {
        stats.temperature = get_internal_temp();
        stats.cpu_usage = uxTaskGetNumberOfTasks();  // Count active tasks

        printf("Temp: %.2f°C | Active Tasks: %.2f\n", stats.temperature, stats.cpu_usage);

        if (stats.temperature > 65.0) {
            if (highLoadTaskHandle != NULL) {
                printf("Overheating! Suspending high-load task...\n");
                vTaskSuspend(highLoadTaskHandle);
            }
        } else {
            if (highLoadTaskHandle == NULL) {
                printf("Resuming high-load task...\n");
                xTaskCreate(high_load_task, "high_load_task", 2048, NULL, 5, &highLoadTaskHandle);
            } else {
                vTaskResume(highLoadTaskHandle);
            }
        }

        xQueueSend(systemQueue, &stats, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Monitor every 5 seconds
    }
}

// Main function to start tasks
void app_main() {
    esp_task_wdt_init(10, true);  
    esp_task_wdt_add(NULL);  
    
    systemQueue = xQueueCreate(5, sizeof(SystemStats));
    xTaskCreate(task_manager, "task_manager", 4096, NULL, 5, NULL);

    // Keep app_main running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/adc.h"
// #include "esp_system.h"
// #include "esp_log.h"

// #define TEMP_THRESHOLD_HIGH 60.0
// #define VOLTAGE_THRESHOLD_LOW 2.5
// #define CPU_LOAD_HIGH 80

// // Task handles
// TaskHandle_t sensorTaskHandle;
// TaskHandle_t loggingTaskHandle;

// // ----------------- Hardware Monitoring -----------------
// float get_temperature() {
//     return (temprature_sens_read() - 32) / 1.8;
// }

// float get_voltage() {
//     int val = adc1_get_raw(ADC1_CHANNEL_0);
//     return val * (3.3 / 4095); // 12-bit ADC with 3.3V reference
// }

// int get_cpu_load() {
//     return uxTaskGetNumberOfTasks(); // Approximate CPU load based on task count
// }

// // ----------------- Tasks -----------------
// void sensorTask(void *param) {
//     while (1) {
//         float temperature = get_temperature();
//         float voltage = get_voltage();
//         int cpuLoad = get_cpu_load();

//         ESP_LOGI("Sensor", "Temperature: %.2f°C, Voltage: %.2fV, CPU Load: %d%%", 
//                  temperature, voltage, cpuLoad);

//         // Dynamic priority adjustment
//         if (temperature > TEMP_THRESHOLD_HIGH) {
//             ESP_LOGW("Sensor", "High temperature detected! Lowering priority.");
//             vTaskPrioritySet(sensorTaskHandle, tskIDLE_PRIORITY);
//         } else {
//             vTaskPrioritySet(sensorTaskHandle, 3);
//         }

//         if (voltage < VOLTAGE_THRESHOLD_LOW) {
//             ESP_LOGW("Sensor", "Low voltage! Suspending non-critical tasks.");
//             vTaskSuspend(loggingTaskHandle); // Suspend logging if voltage is low
//         } else {
//             vTaskResume(loggingTaskHandle);
//         }

//         if (cpuLoad > CPU_LOAD_HIGH) {
//             ESP_LOGW("Sensor", "High CPU load! Slowing down sampling.");
//             vTaskDelay(pdMS_TO_TICKS(2000)); // Increase delay
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(1000)); // Normal delay
//         }
//     }
// }

// void loggingTask(void *param) {
//     while (1) {
//         ESP_LOGI("Logger", "Logging data...");
//         vTaskDelay(pdMS_TO_TICKS(3000));
//     }
// }

// // ----------------- Main Setup -----------------
// void app_main(void) {
//     ESP_LOGI("Main", "Starting Hardware-Aware Scheduler");

//     // ADC setup for voltage reading
//     adc1_config_width(ADC_WIDTH_BIT_12);
//     adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

//     // Create tasks
//     xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 3, &sensorTaskHandle);
//     xTaskCreate(loggingTask, "Logging Task", 4096, NULL, 2, &loggingTaskHandle);
// }

