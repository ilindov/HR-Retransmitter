#include <string.h>
#include <time.h>
#include "esp_http_server.h"
#include "esp_log.h"



#define REST_CHECK(a, str, goto_tag, ...)                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(a))                                                                      \
        {                                                                              \
            ESP_LOGE(REST_TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                             \
        }                                                                              \
    } while (0)


void update_hr(uint8_t);

static const char *REST_TAG = "HRRetransmitter-rest";
time_t prev_time = 0;


/* Simple handler for putting HR data */
static esp_err_t puthr_put_handler(httpd_req_t *req)
{
    uint8_t hr = (uint8_t)atoi(strchr(req->uri, '?') + sizeof(char));
    ESP_LOGI(REST_TAG, "HR: %d - %ld\n", hr, time(NULL) - prev_time);
    prev_time = time(NULL);
    update_hr(hr);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Connection", "Keep-Alive");
    httpd_resp_send(req, NULL, (ssize_t)NULL);
    return ESP_OK;
}

esp_err_t start_rest_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // config.max_open_sockets = 7;
    // config.max_uri_handlers = 1;
    // config.backlog_conn     = 20;
    config.lru_purge_enable = true;
    // config.recv_wait_timeout= 10;

    ESP_LOGI(REST_TAG, "Starting HTTP Server");
    REST_CHECK(httpd_start(&server, &config) == ESP_OK, "Start server failed", err);

    /* URI handler for fetching heart rate data */
    httpd_uri_t puthr_put_uri = {
        .uri = "/puthr",
        .method = HTTP_PUT,
        .handler = puthr_put_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &puthr_put_uri);

    return ESP_OK;
err:
    return ESP_FAIL;
}
