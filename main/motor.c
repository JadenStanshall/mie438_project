// motor.c
#include "motor.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

#define DIR_PIN     GPIO_NUM_32
#define STEP_PIN    GPIO_NUM_33

#define MS1_PIN     GPIO_NUM_12
#define MS2_PIN     GPIO_NUM_14
#define MS3_PIN     GPIO_NUM_27

#define STEPS_PER_REV 200 // 1/2 stepping

static const char *TAG = "MOTOR";

static void delay_us(uint32_t us) {
    ets_delay_us(us);
}

void stepper_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) |
                        (1ULL << MS1_PIN) | (1ULL << MS2_PIN) | (1ULL << MS3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Set 1/2 microstepping: MS1 = HIGH, MS2 = LOW, MS3 = LOW
    gpio_set_level(MS1_PIN, 1);
    gpio_set_level(MS2_PIN, 0);
    gpio_set_level(MS3_PIN, 0);

    // Set initial direction pin state (optional)
    gpio_set_level(DIR_PIN, 0);
}

void stepper_move(float rpm, int direction) {
    float steps_per_minute = rpm * STEPS_PER_REV;
    float step_delay_us = (60.0 * 1000000.0) / steps_per_minute;

    gpio_set_level(DIR_PIN, direction);

    gpio_set_level(STEP_PIN, 1);
    delay_us((steps_per_minute));
    gpio_set_level(STEP_PIN, 0);
    delay_us((steps_per_minute));

    // ESP_LOGI(TAG, "delay = %f ", step_delay_us / 2);
}
