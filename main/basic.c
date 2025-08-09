#include <esp_log.h>
#include "pn532.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "ir_sensors.h"

static const char *TAG = "railway-controller";

// PN532 Configuration
#define PN532_HSU_BUS_GPIO_TX    (17)  // GPIO17 for TX (ESP32 UART2 TX)
#define PN532_HSU_BUS_GPIO_RX    (16)  // GPIO16 for RX (ESP32 UART2 RX)  
#define PN532_UART_NUM           UART_NUM_2
#define PN532_BAUD_RATE_CODE     4     // 115200 baud (default - no baud change needed)

// Motor Controller Configuration
#define MOTOR_SPEED_PIN          (25)  // GPIO25 for PWM output (0-3.3V)
#define MOTOR_PWM_CHANNEL        LEDC_CHANNEL_0
#define MOTOR_PWM_TIMER          LEDC_TIMER_0
#define MOTOR_PWM_FREQUENCY      1000  // 1kHz PWM frequency
#define MOTOR_PWM_RESOLUTION     LEDC_TIMER_12_BIT  // 12-bit resolution (0-4095)

// Direction Control Configuration
#define MOTOR_DIRECTION_PIN      (26)  // GPIO26 for relay control (0V/3.3V)

// Speed definitions (0-4095 for 12-bit PWM)
#define SPEED_STOP               0      // 0V
#define SPEED_MEDIUM             2048   // ~1.65V (half of 3.3V)
#define SPEED_FULL               4095   // ~3.3V

// Direction definitions
#define DIRECTION_FORWARD        0      // 0V to relay (forward)
#define DIRECTION_REVERSE        1      // 3.3V to relay (reverse)

// Speed ramping configuration
#define SPEED_RAMP_STEPS         50     // Number of steps over 10 seconds
#define SPEED_RAMP_DELAY_MS      200    // 200ms between steps (50 * 200ms = 10 seconds)

// Global variables for speed ramping and train tracking
static uint32_t current_speed = 0;      // Current actual speed
static uint32_t target_speed = 0;       // Target speed to reach
static TaskHandle_t speed_ramp_task_handle = NULL;
static uint8_t current_train_id = 0;    // Currently assigned train ID

// Function prototypes
void motor_controller_init(void);
void set_train_speed(uint32_t speed_level);
void set_train_speed_smooth(uint32_t speed_level);
void set_train_direction(uint8_t direction);
void handle_rfid_card(char *card_id, uint8_t *raw_uid, int uid_len);
void speed_ramp_task(void *pvParameters);
uint32_t get_current_speed(void);

void motor_controller_init(void)
{
    // Configure PWM timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = MOTOR_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    // Configure PWM channel
    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = MOTOR_PWM_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_SPEED_PIN,
        .duty = 0,  // Start at 0 (stopped)
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    // Configure direction control GPIO
    gpio_config_t direction_config = {
        .pin_bit_mask = (1ULL << MOTOR_DIRECTION_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&direction_config));
    
    // Start in forward direction (0V to relay)
    gpio_set_level(MOTOR_DIRECTION_PIN, DIRECTION_FORWARD);

    ESP_LOGI(TAG, "Motor controller initialized on GPIO%d", MOTOR_SPEED_PIN);
    ESP_LOGI(TAG, "Direction relay control on GPIO%d", MOTOR_DIRECTION_PIN);
    ESP_LOGI(TAG, "Speed range: 0V (stop) to 3.3V (full speed)");
    ESP_LOGI(TAG, "Direction: 0V (forward) / 3.3V (reverse)");
}

void set_train_speed(uint32_t speed_level)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL, speed_level));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL));
    
    current_speed = speed_level;  // Update current speed tracker
    float voltage = (speed_level * 3.3f) / 4095.0f;
    ESP_LOGI(TAG, "Train speed set to %lu (%.2fV)", speed_level, voltage);
}

void set_train_speed_smooth(uint32_t speed_level)
{
    target_speed = speed_level;
    ESP_LOGI(TAG, "Setting target speed to %lu (ramping over 10 seconds)", speed_level);
    
    // If the speed ramp task doesn't exist, create it
    if (speed_ramp_task_handle == NULL) {
        xTaskCreate(speed_ramp_task, "SpeedRamp", 2048, NULL, 5, &speed_ramp_task_handle);
    }
    // If it exists but is suspended/finished, resume it
    else {
        vTaskResume(speed_ramp_task_handle);
    }
}

void speed_ramp_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Speed ramp task started");
    
    while (1) {
        // Calculate the difference between current and target speed
        int32_t speed_diff = (int32_t)target_speed - (int32_t)current_speed;
        
        if (speed_diff == 0) {
            // We've reached the target, suspend this task
            ESP_LOGI(TAG, "Target speed reached, suspending ramp task");
            vTaskSuspend(NULL);  // Suspend this task
            continue;
        }
        
        // Calculate step size based on remaining distance and remaining time
        // Use a fixed step size for consistent ramping speed
        int32_t step_size;
        if (abs(speed_diff) < SPEED_RAMP_STEPS) {
            // If we're close to target, just go there directly
            step_size = speed_diff;
        } else {
            // Fixed step size for consistent ramping
            step_size = (speed_diff > 0) ? (4095 / SPEED_RAMP_STEPS) : -(4095 / SPEED_RAMP_STEPS);
            
            // But don't overshoot the target
            if (speed_diff > 0 && step_size > speed_diff) {
                step_size = speed_diff;
            } else if (speed_diff < 0 && step_size < speed_diff) {
                step_size = speed_diff;
            }
        }
        
        // Calculate new speed
        uint32_t new_speed = current_speed + step_size;
        
        // Bounds checking
        if (new_speed > 4095) new_speed = 4095;
        
        // Apply the speed change
        set_train_speed(new_speed);
        
        // Wait before next step
        vTaskDelay(pdMS_TO_TICKS(SPEED_RAMP_DELAY_MS));
    }
}

void set_train_direction(uint8_t direction)
{
    gpio_set_level(MOTOR_DIRECTION_PIN, direction);
    ESP_LOGI(TAG, "Train direction set to %s (%.1fV)", 
             direction == DIRECTION_FORWARD ? "FORWARD" : "REVERSE",
             direction == DIRECTION_FORWARD ? 0.0f : 3.3f);
}

uint32_t get_current_speed(void)
{
    return current_speed;
}

void handle_rfid_card(char *card_id, uint8_t *raw_uid, int uid_len)
{
    ESP_LOGI(TAG, "RFID Card detected: %s", card_id);
    
    // Assign a train ID based on the card (you could use a lookup table)
    current_train_id = raw_uid[0] % 10;  // Simple train ID assignment
    assign_train_to_track(current_train_id, card_id, raw_uid);
    
    // Check the first byte of the UID for speed control
    // Check the second byte for direction control (if available)
    if (uid_len > 0) {
        uint8_t speed_byte = raw_uid[0];  // Use first byte of UID for speed
        uint8_t direction_byte = uid_len > 1 ? raw_uid[1] : 0x80;  // Use second byte for direction (default forward)
        
        // Direction control based on second byte
        if (direction_byte < 0x80) {
            ESP_LOGI(TAG, "Setting direction: FORWARD");
            set_train_direction(DIRECTION_FORWARD);
        } else {
            ESP_LOGI(TAG, "Setting direction: REVERSE");
            set_train_direction(DIRECTION_REVERSE);
        }
        
        // Speed control based on first byte (same as before)
        if (speed_byte < 0x20) {
            // Low UID range = STOP
            ESP_LOGI(TAG, "STOP ZONE - Emergency brake!");
            set_train_speed(SPEED_STOP);  // Use smooth ramping
        }
        else if (speed_byte < 0x50) {
            // Low UID range = STOP
            ESP_LOGI(TAG, "STOP ZONE - Slow to stop");
            set_train_speed_smooth(SPEED_STOP);  // Use smooth ramping
        }
        else if (speed_byte < 0x80) {
            // Medium UID range = MEDIUM SPEED
            ESP_LOGI(TAG, "MEDIUM SPEED ZONE - Cruising speed");
            set_train_speed_smooth(SPEED_MEDIUM);  // Use smooth ramping
        } 
        else {
            // High UID range = FULL SPEED
            ESP_LOGI(TAG, "FULL SPEED ZONE - Express service!");
            set_train_speed_smooth(SPEED_FULL);  // Use smooth ramping
        }
        
        ESP_LOGI(TAG, "Control based on UID - Speed: 0x%02X, Direction: 0x%02X", speed_byte, direction_byte);
        ESP_LOGI(TAG, "Train ID %d now assigned to track", current_train_id);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Railway Controller Starting - PN532 + Motor Control + IR Sensors");
    
    // Initialize motor controller first
    motor_controller_init();
    
    // Initialize IR sensors
    ir_sensors_init();
    
    // Start with train stopped and in forward direction
    set_train_direction(DIRECTION_FORWARD);
    set_train_speed(SPEED_STOP);  // Use direct speed setting for initialization
    
    // Give the PN532 more time to boot up and stabilize
    ESP_LOGI(TAG, "Waiting for PN532 to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Using UART%d, TX: GPIO%d, RX: GPIO%d, Baud code: %d", 
             PN532_UART_NUM, PN532_HSU_BUS_GPIO_TX, PN532_HSU_BUS_GPIO_RX, PN532_BAUD_RATE_CODE);
    
    // Initialize the PN532
    pn532_t *nfc = pn532_init((int8_t)PN532_UART_NUM, (uint8_t)PN532_BAUD_RATE_CODE, 
                              (int8_t)PN532_HSU_BUS_GPIO_TX, (int8_t)PN532_HSU_BUS_GPIO_RX, 0);
    
    if (!nfc) {
        ESP_LOGE(TAG, "Failed to initialize PN532");
        ESP_LOGE(TAG, "Motor controller will remain at STOP");
        return;
    }
    
    ESP_LOGI(TAG, "PN532 initialized successfully");
    ESP_LOGI(TAG, "Railway automation active - place RFID cards near reader for train assignment");
    ESP_LOGI(TAG, "Speed zones: UID[0] < 0x50 = STOP, 0x50-0x7F = MEDIUM, >= 0x80 = FULL");
    ESP_LOGI(TAG, "Direction zones: UID[1] < 0x80 = FORWARD, >= 0x80 = REVERSE");
    ESP_LOGI(TAG, "IR sensors will provide automatic track control");
    
    char last_card_id[21] = {0};  // Remember last card to avoid spam
    // uint64_t last_debug_time = 0;    // For periodic IR sensor debugging (disabled - no sensors)
    
    while (1) {
        // Debug IR sensors every 5 seconds (DISABLED - no sensors connected)
        /*
        uint64_t now = esp_timer_get_time() / 1000000;  // Get seconds
        if ((now - last_debug_time) > 5) {
            debug_all_ir_sensors();
            last_debug_time = now;
        }
        */
        // Check IR sensors for train detection (DISABLED - no sensors connected yet)
        /*
        if (current_train_id > 0) {  // Only check if we have an assigned train
            train_tracker_t *train = get_current_train();
            if (ir_sensor_triggered(IR_SENSOR_1_PIN)) {
                handle_train_detection(IR_SENSOR_1_PIN, train);
            }
            if (ir_sensor_triggered(IR_SENSOR_2_PIN)) {
                handle_train_detection(IR_SENSOR_2_PIN, train);
            }
            if (ir_sensor_triggered(IR_SENSOR_3_PIN)) {
                handle_train_detection(IR_SENSOR_3_PIN, train);
            }
            if (ir_sensor_triggered(IR_SENSOR_4_PIN)) {
                handle_train_detection(IR_SENSOR_4_PIN, train);
            }
        }
        */
        
        // Check for RFID cards (for train assignment)
        int card_count = pn532_Cards(nfc);
        
        if (card_count > 0) {
            // Get the card ID
            char id_text[21];
            uint8_t *nfcid = pn532_nfcid(nfc, id_text);
            
            if (nfcid && nfcid[0] > 0) {
                // Only process if it's a different card than last time
                if (strcmp(id_text, last_card_id) != 0) {
                    strcpy(last_card_id, id_text);
                    
                    ESP_LOGI(TAG, "New card detected: %s", id_text);
                    ESP_LOG_BUFFER_HEX_LEVEL(TAG, &nfcid[1], nfcid[0], ESP_LOG_INFO);
                    
                    // Handle the RFID card for train assignment and initial control
                    handle_rfid_card(id_text, &nfcid[1], nfcid[0]);
                }
                
                // Wait a bit to avoid spam
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else if (card_count < 0) {
            ESP_LOGW(TAG, "Error reading cards: %s", pn532_err_to_name(pn532_lasterr(nfc)));
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // No cards present - clear the last card memory after a delay
            if (strlen(last_card_id) > 0) {
                // Card was removed, wait a bit then clear
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (pn532_Cards(nfc) == 0) {  // Double-check no card present
                    ESP_LOGI(TAG, "Card removed - IR sensors continue to control train %d", current_train_id);
                    memset(last_card_id, 0, sizeof(last_card_id));
                }
            }
            
            // Quick check for new cards and sensors
            vTaskDelay(pdMS_TO_TICKS(50));  // Faster polling for IR sensors
        }
    }
    
    // Cleanup (this code won't be reached in this example)
    pn532_end(nfc);
}
