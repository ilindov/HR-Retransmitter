#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"



#define EXAMPLE_ESP_WIFI_SSID      "MyWiFi"
#define EXAMPLE_ESP_WIFI_PASS      ""
// #define EXAMPLE_ESP_MAXIMUM_RETRY  0
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "HRRetransmitter WiFi station";
static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            // s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        // } else {
            // xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        // }
        // ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    wifi_ant_config_t ant_config;
    ESP_ERROR_CHECK(esp_wifi_get_ant(&ant_config));
    ESP_LOGI(TAG, "rx_ant_mode: %d, tx_ant_mode: %d, rx_ant_default: %d, enabled_ant0: %d, enabled_ant1: %d", 
            ant_config.rx_ant_mode, ant_config.tx_ant_mode, ant_config.rx_ant_default, ant_config.enabled_ant0, ant_config.enabled_ant1);
    
    wifi_ant_gpio_config_t ant_gpio_config;
    ESP_ERROR_CHECK(esp_wifi_get_ant_gpio(&ant_gpio_config));

    for (int i = 0; i <= 3; i++) {
        ESP_LOGI(TAG, "I: %d, gpio_select: %d; gpio_num: %d",
            i,
            ant_gpio_config.gpio_cfg[i].gpio_select,
            ant_gpio_config.gpio_cfg[i].gpio_num);
    }

        

    /*{
        .rx_ant_mode = WIFI_ANT_MODE_ANT0,
        .tx_ant_mode = WIFI_ANT_MODE_ANT1,
        .enabled_ant0 = 1,
        .enabled_ant1 = 1;
    };*/

    // ESP_ERROR_CHECK(esp_wifi_set_ant());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "WiFi_init_sta finished.");

    wifi_country_t wifi_set_country = {
        .cc = "BG",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_set_country));


    wifi_country_t wifi_country;
    ESP_ERROR_CHECK(esp_wifi_get_country(&wifi_country));
    ESP_LOGI(TAG, "Country: %s, max_tx_power: %d", wifi_country.cc, wifi_country.max_tx_power);

    int8_t tx_power;
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&tx_power));
    ESP_LOGI(TAG, "Device  max_tx_power: %d", tx_power);

    wifi_ps_type_t ps_type;
    ESP_ERROR_CHECK(esp_wifi_get_ps(&ps_type));
    ESP_LOGI(TAG, "Current PS: %d", ps_type);

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    ESP_ERROR_CHECK(esp_wifi_get_ps(&ps_type));
    ESP_LOGI(TAG, "Set PS: %d", ps_type);    

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_got_ip));
    vEventGroupDelete(s_wifi_event_group);
}

void start_wifi(void)
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}
