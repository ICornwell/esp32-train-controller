#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include "driver/gpio.h"
#include "esp_log.h"

// IR Sensor Configuration
#define IR_SENSOR_1_PIN         (32)    // GPIO32 for platform stop
#define IR_SENSOR_2_PIN         (33)    // GPIO33 for siding entry
#define IR_SENSOR_3_PIN         (34)    // GPIO34 for junction detection
#define IR_SENSOR_4_PIN         (35)    // GPIO35 for end-of-line bumper

// Track occupancy states
typedef enum {
    TRACK_CLEAR = 0,
    TRACK_OCCUPIED = 1
} track_state_t;

// Train tracking structure
typedef struct {
    uint8_t train_id;           // Which train (from RFID assignment)
    uint32_t current_speed;     // Current speed setting
    uint8_t direction;          // Current direction
    uint64_t last_detection;    // Timestamp of last IR sensor trigger
    gpio_num_t last_sensor;     // Which sensor last detected this train
} train_tracker_t;

// Function prototypes
void ir_sensors_init(void);
bool ir_sensor_triggered(gpio_num_t sensor_pin);
void handle_train_detection(gpio_num_t sensor_pin, train_tracker_t *train);
void assign_train_to_track(uint8_t train_id, char *card_id, uint8_t *raw_uid);
train_tracker_t* get_current_train(void);
void debug_all_ir_sensors(void);

#endif // IR_SENSORS_H
