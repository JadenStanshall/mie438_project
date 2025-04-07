#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define DIR_PIN     32
#define STEP_PIN    33

#define MS1_PIN     12
#define MS2_PIN     14
#define MS3_PIN     27

#define DELAY_US    5500
// min speed = 5500
// max speed = 800

#define STEPS_PER_REV 200 // full = 200, half = 400

static const char *TAG = "STEPPER_TEST";

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) | (1ULL << MS1_PIN) | (1ULL << MS2_PIN) | (1ULL << MS3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // config half step
    gpio_set_level(MS1_PIN, 0);
    gpio_set_level(MS2_PIN, 0);
    gpio_set_level(MS3_PIN, 0);

    gpio_set_level(DIR_PIN, 1);  // Set fixed direction

    float rpm = (60.0f * 1000000.0f) / (STEPS_PER_REV * 2.0f * DELAY_US);
    ESP_LOGI(TAG, "Running motor at delay %d us (%.2f RPM)", DELAY_US, rpm);

    while (1) {
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(DELAY_US);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(DELAY_US);
    }
}
