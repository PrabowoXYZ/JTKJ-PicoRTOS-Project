#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"
#include <hardware/i2c.h>

// Default stack size for the tasks
#define DEFAULT_STACK_SIZE 2048

// Debug output control - set to 0 to disable debug messages
#define DEBUG_ENABLED 0

#if DEBUG_ENABLED
    #define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...) 
#endif

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

// Threshold values for tilt detection (using accelerometer)
#define TILT_RIGHT_THRESHOLD 0.6    // ax > 0.6 = tilted right (DASH)
#define TILT_LEFT_THRESHOLD -0.6    // ax < -0.6 = tilted left (DOT)
#define TILT_FORWARD_THRESHOLD -0.6 // ay < -0.6 = tilted forward (SPACE)
#define TILT_HOLD_TIME_MS 300       // Must hold tilt for 300ms
#define TILT_COOLDOWN_MS 800        // Cooldown between detections

// Message buffer
#define MAX_MESSAGE_LENGTH 256
char messageBuffer[MAX_MESSAGE_LENGTH];
int messageIndex = 0;

/* =========================
 *  BUTTON HANDLER
 * ========================= */
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
            printf("Final Message: %s %s\n", messageBuffer, MORSE_END);
            messageIndex = 0;
        }
    }
}

/* =========================
 *  IMU HELPER FUNCTIONS
 * ========================= */
static bool verify_sensor_ready() {
    uint8_t who_am_i_reg = 0x75;
    uint8_t who = 0;
    uint8_t addresses[] = {0x68, 0x69};
    
    for (int i = 0; i < 2; i++) {
        DEBUG_PRINT("  Checking address 0x%02X...\n", addresses[i]);
        int write_result = i2c_write_blocking(i2c_default, addresses[i], &who_am_i_reg, 1, true);
        if (write_result == 1) {
            int read_result = i2c_read_blocking(i2c_default, addresses[i], &who, 1, false);
            if (read_result == 1) {
                DEBUG_PRINT("  WHO_AM_I: 0x%02X\n", who);
                if (who == 0x67) {
                    DEBUG_PRINT("  ✓ Sensor ready at address 0x%02X\n", addresses[i]);
                    return true;
                }
            }
        }
    }
    DEBUG_PRINT("  ✗ Sensor not found\n");
    return false;
}

static int initialize_imu() {
    DEBUG_PRINT("Waiting for system stabilization (1000ms)...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    DEBUG_PRINT("Verifying I2C communication before init...\n");
    if (!verify_sensor_ready()) {
        printf("ERROR: Sensor not detected before init!\n");
        blink_led(10);
        return -1;
    }
    
    DEBUG_PRINT("Initializing IMU (soft reset)...\n");
    int init_result = init_ICM42670();
    if (init_result != 0) {
        printf("ERROR: IMU init failed with code %d\n", init_result);
        blink_led(5);
        return -2;
    }
    DEBUG_PRINT("IMU initialized successfully\n");
    
    DEBUG_PRINT("Waiting after soft reset (1000ms)...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    DEBUG_PRINT("Verifying I2C communication after reset...\n");
    if (!verify_sensor_ready()) {
        printf("ERROR: Sensor not responding after reset!\n");
        blink_led(8);
        return -3;
    }
    
    DEBUG_PRINT("Additional stabilization delay (500ms)...\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    return 0;
}

static int configure_sensors() {
    DEBUG_PRINT("Configuring accelerometer (100Hz, ±4g)...\n");
    int accel_result = ICM42670_startAccel(100, 4);
    DEBUG_PRINT("Accelerometer config returned: %d\n", accel_result);
    if (accel_result != 0) {
        printf("ERROR: Accelerometer config failed with code %d\n", accel_result);
        blink_led(3);
        return -1;
    }
    DEBUG_PRINT("✓ Accelerometer configured\n");
    
    DEBUG_PRINT("Waiting between configs (300ms)...\n");
    vTaskDelay(pdMS_TO_TICKS(300));
    
    DEBUG_PRINT("Configuring gyroscope (100Hz, ±250dps)...\n");
    int gyro_result = ICM42670_startGyro(100, 250);
    DEBUG_PRINT("Gyroscope config returned: %d\n", gyro_result);
    if (gyro_result != 0) {
        printf("ERROR: Gyroscope config failed with code %d\n", gyro_result);
        blink_led(4);
        return -2;
    }
    DEBUG_PRINT("✓ Gyroscope configured\n");
    
    DEBUG_PRINT("Waiting before low-noise mode (300ms)...\n");
    vTaskDelay(pdMS_TO_TICKS(300));
    
    DEBUG_PRINT("Enabling Low-Noise mode...\n");
    int ln_result = ICM42670_enable_accel_gyro_ln_mode();
    DEBUG_PRINT("Low-Noise mode returned: %d\n", ln_result);
    if (ln_result != 0) {
        printf("ERROR: Low-Noise mode failed with code %d\n", ln_result);
        blink_led(6);
        return -3;
    }
    DEBUG_PRINT("✓ Low-Noise mode enabled\n");
    
    return 0;
}

/* =========================
 *  MOTION DETECTION
 * ========================= */
static void detect_tilt_motion(float ax, float ay, 
                               TickType_t *tiltStartTime,
                               TickType_t *lastDetectionTime,
                               bool *tiltingRight,
                               bool *tiltingLeft,
                               bool *tiltingForward) {
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t timeSinceLastDetection = currentTime - *lastDetectionTime;
    
    if (timeSinceLastDetection < pdMS_TO_TICKS(TILT_COOLDOWN_MS)) {
        return;
    }
    
    // TILT RIGHT = DASH
    if (ax > TILT_RIGHT_THRESHOLD && !*tiltingRight && !*tiltingLeft && !*tiltingForward) {
        *tiltingRight = true;
        *tiltStartTime = currentTime;
        DEBUG_PRINT("→ Tilt RIGHT detected (ax=%.2f)...\n", ax);
    }
    else if (*tiltingRight && ax > TILT_RIGHT_THRESHOLD) {
        TickType_t elapsed = currentTime - *tiltStartTime;
        if (elapsed >= pdMS_TO_TICKS(TILT_HOLD_TIME_MS)) {
            char symbol = MORSE_DASH;
            xQueueSend(morseQueue, &symbol, 0);
            
            if (messageIndex < MAX_MESSAGE_LENGTH - 1) {
                messageBuffer[messageIndex++] = symbol;
                messageBuffer[messageIndex] = '\0';
            }
            
            printf("__Detected: DASH__ (Message: %s)\n", messageBuffer);
            *lastDetectionTime = currentTime;
            *tiltingRight = false;
        }
    }
    else if (*tiltingRight && ax <= TILT_RIGHT_THRESHOLD) {
        *tiltingRight = false;
    }
    
    // TILT LEFT = DOT
    if (ax < TILT_LEFT_THRESHOLD && !*tiltingRight && !*tiltingLeft && !*tiltingForward) {
        *tiltingLeft = true;
        *tiltStartTime = currentTime;
        DEBUG_PRINT("← Tilt LEFT detected (ax=%.2f)...\n", ax);
    }
    else if (*tiltingLeft && ax < TILT_LEFT_THRESHOLD) {
        TickType_t elapsed = currentTime - *tiltStartTime;
        if (elapsed >= pdMS_TO_TICKS(TILT_HOLD_TIME_MS)) {
            char symbol = MORSE_DOT;
            xQueueSend(morseQueue, &symbol, 0);
            
            if (messageIndex < MAX_MESSAGE_LENGTH - 1) {
                messageBuffer[messageIndex++] = symbol;
                messageBuffer[messageIndex] = '\0';
            }
            
            printf("__Detected: DOT__ (Message: %s)\n", messageBuffer);
            *lastDetectionTime = currentTime;
            *tiltingLeft = false;
        }
    }
    else if (*tiltingLeft && ax >= TILT_LEFT_THRESHOLD) {
        *tiltingLeft = false;
    }
    
    // TILT FORWARD = SPACE
    if (ay < TILT_FORWARD_THRESHOLD && !*tiltingRight && !*tiltingLeft && !*tiltingForward) {
        *tiltingForward = true;
        *tiltStartTime = currentTime;
        DEBUG_PRINT("↓ Tilt FORWARD detected (ay=%.2f)...\n", ay);
    }
    else if (*tiltingForward && ay < TILT_FORWARD_THRESHOLD) {
        TickType_t elapsed = currentTime - *tiltStartTime;
        if (elapsed >= pdMS_TO_TICKS(TILT_HOLD_TIME_MS)) {
            char symbol = MORSE_SPACE;
            xQueueSend(morseQueue, &symbol, 0);
            
            if (messageIndex < MAX_MESSAGE_LENGTH - 1) {
                messageBuffer[messageIndex++] = symbol;
                messageBuffer[messageIndex] = '\0';
            }
            
            printf("__Detected: SPACE__ (Message: %s)\n", messageBuffer);
            *lastDetectionTime = currentTime;
            *tiltingForward = false;
        }
    }
    else if (*tiltingForward && ay >= TILT_FORWARD_THRESHOLD) {
        *tiltingForward = false;
    }
}

/* =========================
 *  IMU TASK
 * ========================= */
static void imu_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temp;
    
    DEBUG_PRINT("IMU task started\n");
    
    // Initialize IMU
    if (initialize_imu() != 0) {
        vTaskDelete(NULL);
        return;
    }
    
    // Configure sensors
    if (configure_sensors() != 0) {
        vTaskDelete(NULL);
        return;
    }
    
    printf("✓✓✓ IMU fully configured and operational! ✓✓✓\n");
    blink_led(2);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // State variables
    TickType_t tiltStartTime = 0;
    TickType_t lastDetectionTime = 0;
    bool tiltingRight = false;
    bool tiltingLeft = false;
    bool tiltingForward = false;
    
    printf("\n=== MOTION DETECTION ACTIVE ===\n");
    printf("INSTRUCTIONS:\n");
    printf("- Tilt RIGHT (ax > 0.6) = DASH (-)\n");
    printf("- Tilt LEFT (ax < -0.6) = DOT (.)\n");
    printf("- Tilt FORWARD (ay < -0.6) = SPACE\n");
    printf("Press button to enable/disable Morse mode\n\n");
    
    // Main loop
    while(1) {
        int read_result = ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp);
        
        if (read_result != 0) {
            printf("ERROR: Read failed with code %d\n", read_result);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        static int print_counter = 0;
        if (print_counter++ % 20 == 0) {
            DEBUG_PRINT("SENSOR: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f temp=%.1f°C\n", 
                       ax, ay, az, gx, gy, gz, temp);
        }
        
        if (programState == MORSE_MODE) {
            detect_tilt_motion(ax, ay, &tiltStartTime, &lastDetectionTime,
                             &tiltingRight, &tiltingLeft, &tiltingForward);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* =========================
 *  MAIN
 * ========================= */
int main(void) {
    stdio_init_all();
    
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    sleep_ms(500);
    
    printf("\n\n=== MORSE CODE MESSENGER ===\n");
    DEBUG_PRINT("Starting initialization...\n");
    
    init_hat_sdk();
    sleep_ms(1000);
    DEBUG_PRINT("HAT SDK initialized\n");
    
    init_led();
    init_rgb_led();
    init_sw1();
    DEBUG_PRINT("Peripherals initialized\n");

    rgb_led_write(255, 0, 0);
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &btn_fxn);
    DEBUG_PRINT("Button interrupt configured\n");
    
    morseQueue = xQueueCreate(50, sizeof(char));
    if (morseQueue == NULL) {
        printf("ERROR: Failed to create queue!\n");
        return 0;
    }
    DEBUG_PRINT("Queue created successfully\n");
    
    DEBUG_PRINT("Creating IMU task...\n");
    TaskHandle_t imuTaskHandle = NULL;
    BaseType_t result = xTaskCreate(imu_task, "IMU_Task", DEFAULT_STACK_SIZE, NULL, 1, &imuTaskHandle);
    
    if (result != pdPASS) {
        printf("ERROR: IMU Task creation failed! Result: %d\n", result);
        return 0;
    }
    DEBUG_PRINT("IMU task created successfully\n");
    
    DEBUG_PRINT("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();
    
    printf("ERROR: Scheduler returned!\n");
    return 0;
}