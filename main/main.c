#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

SemaphoreHandle_t xSemaphore = NULL;
extern uint8_t temprature_sens_read();

float get_internal_temp() {

    uint8_t raw_temp = temprature_sens_read();
    return (raw_temp - 32) / 1.8;
}


void temperature_check_task() {

}

void voltage_check_task() {
    
}

void simul_task() {
    printf("[High Load] Processing data...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Simulated workload
    vTaskDelete(NULL);
}

void app_main() {

    xSemaphore = xSemaphoreCreateBinary();
    printf("\nTimer output in milliseconds program initiation: %lld\n", esp_timer_get_time() / 1000);

    xTaskCreate(simul_task,"sample task", 2048, NULL, 1, NULL);

    printf("\nProgram completion: %lld\n", esp_timer_get_time() / 1000);
}



// #include <stdio.h>
// #include "esp_system.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"
// #include "esp_task_wdt.h"

// extern uint8_t temprature_sens_read();

// // Function to get internal temperature
// float get_internal_temp() {
//     uint8_t raw_temp = temprature_sens_read();
//     return (raw_temp - 32) / 1.8;
// }

// // Struct to hold system stats
// typedef struct {
//     float cpu_usage;
//     float temperature;
// } SystemStats;

// QueueHandle_t systemQueue;
// SemaphoreHandle_t taskSemaphore;  // Binary Semaphore
// TaskHandle_t highLoadTaskHandle = NULL;

// // High-load task waits on the semaphore before running
// void high_load_task(void *pvParameter) {
//     esp_task_wdt_add(NULL);  // Register task to watchdog

//     while (1) {
//         if (xSemaphoreTake(taskSemaphore, portMAX_DELAY) == pdTRUE) {
//             printf("[High Load] Processing data...\n");
//             vTaskDelay(pdMS_TO_TICKS(1000));  // Simulated workload
//         }
//     }
// }

// // Task manager monitors temperature and controls tasks
// void task_manager(void *pvParameter) {
//     esp_task_wdt_add(NULL);  // Register task to watchdog

//     SystemStats stats;

//     while (1) {
//         stats.temperature = get_internal_temp();
//         stats.cpu_usage = (float)uxTaskGetNumberOfTasks();
//         printf("Temp: %.2f°C | Active Tasks: %.2f\n", stats.temperature, stats.cpu_usage);

//         if (stats.temperature > 65.0) {
//             printf("🔥 Overheating detected! Stopping high-load task... 🔥\n");
//             xSemaphoreGive(taskSemaphore);  // Release to let high_load_task stop
//         } else {
//             printf("✅ Resuming high-load task\n");
//             xSemaphoreTake(taskSemaphore, 0);  // Allow high_load_task to run
//         }

//         // Send system stats to queue
//         xQueueSend(systemQueue, &stats, pdMS_TO_TICKS(1000));

//         esp_task_wdt_reset();  // Reset watchdog
//         vTaskDelay(pdMS_TO_TICKS(5000));  // Monitor every 5 seconds
//     }
// }

// // Main function
// void app_main() {
//     // Configure the watchdog timer
//     esp_task_wdt_config_t wdt_config = {
//         .timeout_ms = 10000,  // 10-second timeout
//         .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
//         .trigger_panic = true
//     };

//     esp_task_wdt_init(&wdt_config);
//     esp_task_wdt_add(NULL);  // Register main loop

//     systemQueue = xQueueCreate(5, sizeof(SystemStats));
//     taskSemaphore = xSemaphoreCreateBinary();  // Initialize binary semaphore

//     xTaskCreate(task_manager, "task_manager", 4096, NULL, 5, NULL);
//     xTaskCreate(high_load_task, "high_load_task", 2048, NULL, 5, &highLoadTaskHandle);

//     while (1) {
//         esp_task_wdt_reset();  // Reset watchdog for main loop
//         vTaskDelay(pdMS_TO_TICKS(1000));  // Allow FreeRTOS to manage tasks
//     }
// }
