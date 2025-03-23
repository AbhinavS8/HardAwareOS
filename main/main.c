#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_system.h"

SemaphoreHandle_t xPauseSemaphore = NULL;  // Global semaphore for pausing tasks
extern uint8_t temprature_sens_read();

typedef struct {
    float cpu_usage;
    float temperature;
} SystemStats;

// Function to get internal ESP32 temperature
float get_internal_temp() {
    uint8_t raw_temp = temprature_sens_read();
    return (raw_temp - 32) / 1.8;
}

// Temperature monitoring task
void temperature_check_task() {
    SystemStats stats;

    while (1) {
        stats.temperature = get_internal_temp();
        stats.cpu_usage = (float)uxTaskGetNumberOfTasks();
        printf("Temp: %.2f°C | Active Tasks: %.2f\n", stats.temperature, stats.cpu_usage);

        if (stats.temperature >= 70) {
            printf("HIGH TEMPERATURE DETECTED! PAUSING ALL TASKS...\n");

            // Take the semaphore to pause all tasks
            xSemaphoreTake(xPauseSemaphore, portMAX_DELAY);

            // Wait until temperature goes below 70°C
            while (get_internal_temp() >= 70) {
                vTaskDelay(pdMS_TO_TICKS(2000));  // Delay and re-check temperature
            }

            // Give the semaphore back to resume all tasks
            printf("Temperature normalized. Resuming all tasks.\n");
            xSemaphoreGive(xPauseSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // Check temperature every 2 seconds
    }
}

// Example workload task
void high_load_task(void *param) {
    while (1) {
        // Wait for the semaphore (blocks execution if temperature is high)
        xSemaphoreTake(xPauseSemaphore, portMAX_DELAY);

        printf("[High Load] Processing data...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Simulated workload

        // Release the semaphore for other tasks to use
        xSemaphoreGive(xPauseSemaphore);
    }
}

// Another example workload task
void another_task(void *param) {
    while (1) {
        // Wait for the semaphore (blocks execution if temperature is high)
        xSemaphoreTake(xPauseSemaphore, portMAX_DELAY);

        printf("[Another Task] Running...\n");
        vTaskDelay(pdMS_TO_TICKS(1500));

        // Release the semaphore for other tasks to use
        xSemaphoreGive(xPauseSemaphore);
    }
}

void app_main() {
    // Create binary semaphore (starts unlocked)
    xPauseSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xPauseSemaphore);  // Ensure tasks start executing

    printf("\nTime(ms) for program initiation: %lld\n", esp_timer_get_time() / 1000);

    xTaskCreate(high_load_task, "High Load Task", 2048, NULL, 1, NULL);
    xTaskCreate(another_task, "Another Task", 2048, NULL, 1, NULL);
    xTaskCreate(temperature_check_task, "Temp Watch Task", 2048, NULL, 2, NULL);

    printf("\nProgram completion: %lld\n", esp_timer_get_time() / 1000);
}

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include "esp_system.h"

// SemaphoreHandle_t xSemaphore = NULL;
// extern uint8_t temprature_sens_read();

// typedef struct {
//     float cpu_usage;
//     float temperature;
// } SystemStats;

// float get_internal_temp() {

//     uint8_t raw_temp = temprature_sens_read();
//     return (raw_temp - 32) / 1.8;
// }


// void temperature_check_task() {
//     SystemStats stats;
//     while (1) {
//         stats.temperature = get_internal_temp();
//         stats.cpu_usage = (float)uxTaskGetNumberOfTasks();
//         printf("Temp: %.2f°C | Active Tasks: %.2f\n", stats.temperature, stats.cpu_usage);

//         if (stats.temperature >= 70) {
//             if (xSemaphoreTake(xSemaphore, 0) == pdTRUE) {
//                 xSemaphoreTake(xSemaphore);
//             }
//             printf("HIGH TEMPERATURES. PAUSING EXECUTION");
            
//             // for (int i=0;i<10;i++) {

//             //     vTaskDelay(pdMS_TO_TICKS(2000));
//             //     if (get_internal_temp() < 70) {break;}
            
//             // }
            
//         } else {

//         }
//         vTaskDelay(2000/ portTICK_PERIOD_MS);
//     }
// }

// void voltage_check_task() {
    
// }

// void simul_task() {
//     while(1) {
//         if (xSemaphoreTake(xSemaphore, 0) == pdTRUE) {
//             xSemaphoreTake(xSemaphore, portMAX_DELAY);
//         }
//         printf("[High Load] Processing data...\n");
//         vTaskDelay(1000 / portTICK_PERIOD_MS); //workload sim
//     }
//         vTaskDelete(NULL);
    
// }

// void app_main() {

//     xSemaphore = xSemaphoreCreateBinary();

//     printf("\nTime(ms) for program initiation: %lld\n", esp_timer_get_time() / 1000);

//     xTaskCreate(simul_task,"sample task", 2048, NULL, 1, NULL);
//     xTaskCreate(temperature_check_task,"temp watch task", 2048, NULL, 2, NULL);

//     // xSemaphoreTake(xSemaphore, portMAX_DELAY);

//     // for (int i = 10; i >= 0; i--) {
//     //     printf("Restarting in %d seconds...\n", i);
//     //     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     // }
//     printf("\nProgram completion: %lld\n", esp_timer_get_time() / 1000);

//     // printf("Restarting now.\n");
//     // fflush(stdout);
//     // esp_restart();
// }



// // #include <stdio.h>
// // #include "esp_system.h"
// // #include "freertos/FreeRTOS.h"
// // #include "freertos/task.h"
// // #include "freertos/queue.h"
// // #include "freertos/semphr.h"
// // #include "esp_task_wdt.h"

// // extern uint8_t temprature_sens_read();

// // // Function to get internal temperature
// // float get_internal_temp() {
// //     uint8_t raw_temp = temprature_sens_read();
// //     return (raw_temp - 32) / 1.8;
// // }

// // // Struct to hold system stats
// // typedef struct {
// //     float cpu_usage;
// //     float temperature;
// // } SystemStats;

// // QueueHandle_t systemQueue;
// // SemaphoreHandle_t taskSemaphore;  // Binary Semaphore
// // TaskHandle_t highLoadTaskHandle = NULL;

// // // High-load task waits on the semaphore before running
// // void high_load_task(void *pvParameter) {
// //     esp_task_wdt_add(NULL);  // Register task to watchdog

// //     while (1) {
// //         if (xSemaphoreTake(taskSemaphore, portMAX_DELAY) == pdTRUE) {
// //             printf("[High Load] Processing data...\n");
// //             vTaskDelay(pdMS_TO_TICKS(1000));  // Simulated workload
// //         }
// //     }
// // }

// // // Task manager monitors temperature and controls tasks
// // void task_manager(void *pvParameter) {
// //     esp_task_wdt_add(NULL);  // Register task to watchdog

//     // SystemStats stats;

// //     while (1) {
// //         stats.temperature = get_internal_temp();
// //         stats.cpu_usage = (float)uxTaskGetNumberOfTasks();
// //         printf("Temp: %.2f°C | Active Tasks: %.2f\n", stats.temperature, stats.cpu_usage);

// //         if (stats.temperature > 65.0) {
// //             printf("🔥 Overheating detected! Stopping high-load task... 🔥\n");
// //             xSemaphoreGive(taskSemaphore);  // Release to let high_load_task stop
// //         } else {
// //             printf("✅ Resuming high-load task\n");
// //             xSemaphoreTake(taskSemaphore, 0);  // Allow high_load_task to run
// //         }

// //         // Send system stats to queue
// //         xQueueSend(systemQueue, &stats, pdMS_TO_TICKS(1000));

// //         esp_task_wdt_reset();  // Reset watchdog
// //         vTaskDelay(pdMS_TO_TICKS(5000));  // Monitor every 5 seconds
// //     }
// // }

// // // Main function
// // void app_main() {
// //     // Configure the watchdog timer
// //     esp_task_wdt_config_t wdt_config = {
// //         .timeout_ms = 10000,  // 10-second timeout
// //         .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
// //         .trigger_panic = true
// //     };

// //     esp_task_wdt_init(&wdt_config);
// //     esp_task_wdt_add(NULL);  // Register main loop

// //     systemQueue = xQueueCreate(5, sizeof(SystemStats));
// //     taskSemaphore = xSemaphoreCreateBinary();  // Initialize binary semaphore

// //     xTaskCreate(task_manager, "task_manager", 4096, NULL, 5, NULL);
// //     xTaskCreate(high_load_task, "high_load_task", 2048, NULL, 5, &highLoadTaskHandle);

// //     while (1) {
// //         esp_task_wdt_reset();  // Reset watchdog for main loop
// //         vTaskDelay(pdMS_TO_TICKS(1000));  // Allow FreeRTOS to manage tasks
// //     }
// // }
