#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Default stack size for the tasks
#define DEFAULT_STACK_SIZE 2048

// Define states
enum state { IDLE=1, MORSE_MODE };
enum state programState = IDLE;

// Morse symbols
#define MORSE_DOT '.'
#define MORSE_DASH '-'
#define MORSE_SPACE ' '
#define MORSE_END "\n"

// Queue for morse symbols
QueueHandle_t morseQueue;

// Threshold values for IMU detection
#define GYRO_THRESHOLD_DOT 30.0
#define GYRO_THRESHOLD_DASH 30.0
#define ACCEL_THRESHOLD_SPACE 1.5
#define ROTATION_DETECT_TIME_MS 200

// Message buffer
#define MAX_MESSAGE_LENGTH 256
char messageBuffer[MAX_MESSAGE_LENGTH];
int messageIndex = 0;

// Button interrupt handler
static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (programState == IDLE) {
        programState = MORSE_MODE;
        printf("__Morse mode ENABLED__\n");
        rgb_led_write(0, 255, 0);
    } else {
        programState = IDLE;
        printf("__Morse mode DISABLED__\n");
        rgb_led_write(255, 0, 0);
        
        if (messageIndex > 0) {
            messageBuffer[messageIndex] = '\0';
            printf("%s  %s", messageBuffer, MORSE_END);
            messageIndex = 0;
        }
    }
}

static void imu_task(void *arg) {
    (void)arg;
    
    float ax, ay, az, gx, gy, gz, temp;
    
    printf("IMU task started\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Initialize IMU
    printf("Initializing IMU...\n");
    int init_result = init_ICM42670();
    if (init_result != 0) {
        printf("ERROR: IMU init failed with code %d\n", init_result);
        // Continue anyway to see what happens
    } else {
        printf("IMU initialized successfully\n");
    }
    
    ICM42670_start_with_default_values();
    printf("IMU fully started!\n");

    vTaskDelay(pdMS_TO_TICKS(500));
    
    TickType_t rotationStartTime = 0;
    bool rotatingYaxis = false;
    bool rotatingZaxis = false;
    
    for(;;) {
        // Read sensor data
        int read_result = ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp);

        if (read_result != 0) {
            // Error reading sensor
            printf("ERROR: Read failed with code %d\n", read_result);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;  // Skip this iteration and try again
        }
        
        // Print sensor values for debugging
        printf("SENSOR: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f\n", 
               ax, ay, az, gx, gy, gz);
        
        if (programState == MORSE_MODE) {
            TickType_t currentTime = xTaskGetTickCount();
            
            // Y-axis rotation = DOT
            if (gy > GYRO_THRESHOLD_DOT && !rotatingYaxis) {
                rotatingYaxis = true;
                rotationStartTime = currentTime;
            }
            else if (rotatingYaxis && gy < GYRO_THRESHOLD_DOT / 2) {
                TickType_t elapsed = currentTime - rotationStartTime;
                if (elapsed >= pdMS_TO_TICKS(ROTATION_DETECT_TIME_MS)) {
                    char symbol = MORSE_DOT;
                    xQueueSend(morseQueue, &symbol, 0);
                    printf("__Detected: DOT__\n");
                }
                rotatingYaxis = false;
            }
            
            // Z-axis rotation = DASH
            if ((gz > GYRO_THRESHOLD_DASH || gz < -GYRO_THRESHOLD_DASH) && !rotatingZaxis) {
                rotatingZaxis = true;
                rotationStartTime = currentTime;
            }
            else if (rotatingZaxis && (gz < GYRO_THRESHOLD_DASH / 2 && gz > -GYRO_THRESHOLD_DASH / 2)) {
                TickType_t elapsed = currentTime - rotationStartTime;
                if (elapsed >= pdMS_TO_TICKS(ROTATION_DETECT_TIME_MS)) {
                    char symbol = MORSE_DASH;
                    xQueueSend(morseQueue, &symbol, 0);
                    printf("__Detected: DASH__\n");
                }
                rotatingZaxis = false;
            }
            
            // X-axis acceleration = SPACE
            if (ax > ACCEL_THRESHOLD_SPACE) {
                char symbol = MORSE_SPACE;
                xQueueSend(morseQueue, &symbol, 0);
                printf("__Detected: SPACE__\n");
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Communication task
static void communication_task(void *arg) {
    (void)arg;
    char symbol;
    
    for(;;) {
        if (xQueueReceive(morseQueue, &symbol, portMAX_DELAY) == pdTRUE) {
            if (messageIndex < MAX_MESSAGE_LENGTH - 1) {
                messageBuffer[messageIndex++] = symbol;
                printf("%c", symbol);
                fflush(stdout);
                blink_led(1);
            }
        }
    }
}

int main(void) {
    stdio_init_all();
    
    // Wait for USB serial connection
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000);  // Extra delay after connection
    
    printf("\n\n=== MORSE CODE MESSENGER ===\n");
    printf("Starting initialization...\n");
    
    init_hat_sdk();
    sleep_ms(300);
    
    printf("HAT SDK initialized\n");
    
    init_led();
    init_rgb_led();
    init_sw1();
    
    printf("Peripherals initialized\n");

    rgb_led_write(255, 0, 0);
    
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &btn_fxn);
    
    printf("Button interrupt configured\n");
    
    morseQueue = xQueueCreate(50, sizeof(char));
    
    if (morseQueue == NULL) {
        printf("ERROR: Failed to create queue!\n");
        return 0;
    }
    printf("Queue created successfully\n");
    
    printf("Creating IMU task...\n");
    TaskHandle_t imuTaskHandle = NULL;
    BaseType_t result = xTaskCreate(imu_task, "IMU_Task", DEFAULT_STACK_SIZE, NULL, 1, &imuTaskHandle);
    
    if (result != pdPASS) {
        printf("ERROR: IMU Task creation failed! Result: %d\n", result);
        return 0;
    }
    printf("IMU task created successfully\n");
    
    printf("Creating communication task...\n");
    TaskHandle_t commTaskHandle = NULL;
    result = xTaskCreate(communication_task, "Comm_Task", DEFAULT_STACK_SIZE, NULL, 1, &commTaskHandle);
    
    if (result != pdPASS) {
        printf("ERROR: Communication Task creation failed! Result: %d\n", result);
        return 0;
    }
    printf("Communication task created successfully\n");
    
    printf("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();
    
    printf("ERROR: Scheduler returned!\n");
    return 0;
}