#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FREERTOS.h"
#include "freertos/task.h"
// gpio config struct
gpio_config_t g_config = {
    .pin_bit_mask = 1ULL << GPIO_NUM_16, // IO 16
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};
void app_main(void){
    ESP_ERROR_CHECK(gpio_config(&g_config)); // Returns ESP_OK on success or ESP_ERR_INVALID_ARG for Param Error
    // Every 10 Seconds Beep.
    while(1) {
        gpio_set_level(GPIO_NUM_16, 1); // high
        vTaskDelay(pdMS_TO_TICKS(200)); // Sleep 200ms
        gpio_set_level(GPIO_NUM_16, 0); // low
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}
