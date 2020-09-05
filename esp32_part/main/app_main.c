#include "driver/gpio.h"
#include "nvs_flash.h"



#define LED_GPIO GPIO_NUM_2

esp_err_t start_rest_server(void);
void start_wifi(void);
void start_ble(void);

void app_main(void)
{
    gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    start_wifi();
    ESP_ERROR_CHECK(start_rest_server());
    start_ble();
    gpio_set_level(LED_GPIO, 0);
}
