#include "ir_sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "IR_SENSORS";

// Global train tracker (assuming single train for now, expandable)
train_tracker_t current_train = {0};

void ir_sensors_init(void)
{
    // Configure all IR sensor pins as inputs with pullup
    gpio_config_t ir_config = {
        .pin_bit_mask = (1ULL << IR_SENSOR_1_PIN) | 
                       (1ULL << IR_SENSOR_2_PIN) | 
                       (1ULL << IR_SENSOR_3_PIN) | 
                       (1ULL << IR_SENSOR_4_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    // IR sensors typically need pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE       // We'll poll for simplicity
    };
    ESP_ERROR_CHECK(gpio_config(&ir_config));
    
    ESP_LOGI(TAG, "IR sensors initialized:");
    ESP_LOGI(TAG, "  Platform stop: GPIO%d", IR_SENSOR_1_PIN);
    ESP_LOGI(TAG, "  Siding entry: GPIO%d", IR_SENSOR_2_PIN);
    ESP_LOGI(TAG, "  Junction: GPIO%d", IR_SENSOR_3_PIN);
    ESP_LOGI(TAG, "  End bumper: GPIO%d", IR_SENSOR_4_PIN);
}

bool ir_sensor_triggered(gpio_num_t sensor_pin)
{
    // IR photointerrupter typically pulls LOW when beam is broken
    return (gpio_get_level(sensor_pin) == 0);
}

void handle_train_detection(gpio_num_t sensor_pin, train_tracker_t *train)
{
    uint64_t now = esp_timer_get_time() / 1000;  // Get milliseconds
    
    // Debounce - ignore triggers within 100ms of last detection
    if ((now - train->last_detection) < 100) {
        return;
    }
    
    train->last_detection = now;
    train->last_sensor = sensor_pin;
    
    const char *sensor_name;
    switch (sensor_pin) {
        case IR_SENSOR_1_PIN:
            sensor_name = "Platform Stop";
            // Auto-stop at platform
            ESP_LOGI(TAG, "Train %d at platform - stopping", train->train_id);
            // Call your existing speed control function
            extern void set_train_speed_smooth(uint32_t speed_level);
            set_train_speed_smooth(0);  // Stop the train
            break;
            
        case IR_SENSOR_2_PIN:
            sensor_name = "Siding Entry";
            // Reduce speed for siding
            ESP_LOGI(TAG, "Train %d entering siding - reducing speed", train->train_id);
            // Call function to get current speed from basic.c
            extern uint32_t get_current_speed(void);
            if (get_current_speed() > 1024) {  // If going fast, slow down
                extern void set_train_speed_smooth(uint32_t speed_level);
                set_train_speed_smooth(1024);  // Quarter speed
            }
            break;
            
        case IR_SENSOR_3_PIN:
            sensor_name = "Junction";
            ESP_LOGI(TAG, "Train %d at junction", train->train_id);
            ESP_LOGI(TAG, "DEBUG: GPIO%d level = %d", sensor_pin, gpio_get_level(sensor_pin));
            // Could trigger point motor control here
            break;
            
        case IR_SENSOR_4_PIN:
            sensor_name = "End Bumper";
            ESP_LOGI(TAG, "Train %d at end of line - emergency stop!", train->train_id);
            // Emergency stop
            extern void set_train_speed(uint32_t speed_level);
            set_train_speed(0);  // Immediate stop, no ramping
            break;
            
        default:
            sensor_name = "Unknown";
            break;
    }
    
    ESP_LOGI(TAG, "Train detected at %s (GPIO%d)", sensor_name, sensor_pin);
}

void assign_train_to_track(uint8_t train_id, char *card_id, uint8_t *raw_uid)
{
    current_train.train_id = train_id;
    
    ESP_LOGI(TAG, "Train %d assigned to track (RFID: %s)", train_id, card_id);
    ESP_LOGI(TAG, "IR sensors now tracking this train");
    
    // You could store additional train characteristics here
    // based on the RFID UID (speed limits, horn sounds, etc.)
}

train_tracker_t* get_current_train(void)
{
    return &current_train;
}

void debug_all_ir_sensors(void)
{
    ESP_LOGI(TAG, "=== IR Sensor Debug ===");
    ESP_LOGI(TAG, "GPIO%d (Platform): %d", IR_SENSOR_1_PIN, gpio_get_level(IR_SENSOR_1_PIN));
    ESP_LOGI(TAG, "GPIO%d (Siding):   %d", IR_SENSOR_2_PIN, gpio_get_level(IR_SENSOR_2_PIN));
    ESP_LOGI(TAG, "GPIO%d (Junction): %d", IR_SENSOR_3_PIN, gpio_get_level(IR_SENSOR_3_PIN));
    ESP_LOGI(TAG, "GPIO%d (End):      %d", IR_SENSOR_4_PIN, gpio_get_level(IR_SENSOR_4_PIN));
    ESP_LOGI(TAG, "=== (0=triggered, 1=clear) ===");
}
