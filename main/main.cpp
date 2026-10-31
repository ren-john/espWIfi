// ESP32-S3 camera with persistent AP/client selection and browser provisioning.
#include <cstdio>
#include <atomic>
#include <cstdlib>
#include "esp_timer.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "cJSON.h"
#include "mbedtls/base64.h"

static const char *TAG = "FatigueWiFi";
static constexpr char AP_SSID[] = "FatigueSense_Cam";
static constexpr char AP_PASSWORD[] = "safedriving";
static constexpr int64_t RECOVERY_DELAY_US = 30LL * 1000000;
static constexpr int64_t RETRY_DELAY_US = 5LL * 1000000;

// One committed NVS blob keeps credentials and requested mode consistent.
// Wi-Fi driver's own credential storage is disabled (WIFI_STORAGE_RAM).
struct NetworkSettings {
    uint32_t version = 1;
    uint8_t client_mode = 0;
    char ssid[33] = {};
    char password[65] = {};
};
static NetworkSettings settings;
// Stable hardware identity used by the enterprise Android app. Register this
// exact value as the vehicle's assigned device ID in the backend.
static char device_id[24] = {};
static esp_netif_t *sta_netif = nullptr;
static std::atomic<bool> station_ready{false};
static std::atomic<bool> station_disconnected{true};
static std::atomic<bool> hotspot_active{false};
static std::atomic<int> disconnect_reason{0};
static std::atomic<bool> camera_ready{false};
static std::atomic<int64_t> restart_at{0};

static bool valid_credentials(const char *ssid, const char *password) {
    const size_t ssid_length = std::strlen(ssid);
    const size_t password_length = std::strlen(password);
    if (ssid_length == 0 || ssid_length > 32) return false;
    // Open network, WPA passphrase, or a 64-digit raw PSK.
    return password_length == 0 || (password_length >= 8 && password_length <= 63) ||
        (password_length == 64 && std::strspn(password, "0123456789abcdefABCDEF") == 64);
}

static void load_settings() {
    nvs_handle_t handle;
    esp_err_t result = nvs_open("fatigue_wifi", NVS_READONLY, &handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) return;
    ESP_ERROR_CHECK(result);
    NetworkSettings saved = {};
    size_t length = sizeof(saved);
    result = nvs_get_blob(handle, "settings", &saved, &length);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) return;
    if (result != ESP_OK || length != sizeof(saved) || saved.version != 1 ||
        saved.client_mode > 1 || saved.ssid[32] != '\0' || saved.password[64] != '\0' ||
        (saved.ssid[0] && !valid_credentials(saved.ssid, saved.password)) ||
        (saved.client_mode && !saved.ssid[0])) {
        ESP_LOGW(TAG, "Invalid saved network settings; starting hotspot");
        return;
    }
    settings = saved;
}

static esp_err_t save_settings(const NetworkSettings &next) {
    nvs_handle_t handle;
    esp_err_t result = nvs_open("fatigue_wifi", NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;
    result = nvs_set_blob(handle, "settings", &next, sizeof(next));
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result;
}

static void wifi_event_handler(void *, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        station_ready.store(false);
        disconnect_reason.store(static_cast<wifi_event_sta_disconnected_t *>(data)->reason);
        station_disconnected.store(true);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        station_ready.store(false);
        // Start a fresh DHCP negotiation on the next connection attempt.
        esp_wifi_disconnect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto *event = static_cast<ip_event_got_ip_t *>(data);
        station_disconnected.store(false);
        disconnect_reason.store(0);
        station_ready.store(true);
        ESP_LOGI(TAG, "Client connected. device_id=%s | Control: http://" IPSTR "/ | Stream: http://" IPSTR ":81/stream",
                 device_id, IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.ip));
    }
}

// This task alone changes runtime Wi-Fi modes. HTTP only commits the next boot's
// settings, so callbacks cannot race each other or interrupt their own response.
static void network_task(void *) {
    int64_t offline_since = esp_timer_get_time();
    int64_t next_attempt = 0;
    int64_t attempt_started = 0;
    bool connecting = false;
    while (true) {
        const int64_t now = esp_timer_get_time();
        const int64_t reboot = restart_at.load();
        if (reboot && now >= reboot) esp_restart();
        if (settings.client_mode) {
            if (station_ready.load()) {
                offline_since = now;
                connecting = false;
            } else {
                if (!hotspot_active.load() && now - offline_since >= RECOVERY_DELAY_US) {
                    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
                    hotspot_active.store(true);
                    ESP_LOGW(TAG, "Recovery hotspot ready: %s; http://192.168.4.1/wifi", AP_SSID);
                }
                if (connecting && station_disconnected.load()) {
                    connecting = false;
                    next_attempt = now + RETRY_DELAY_US;
                }
                // Also recover when association succeeds but DHCP never does.
                if (connecting && now - attempt_started >= RECOVERY_DELAY_US) {
                    esp_wifi_disconnect();
                    connecting = false;
                    next_attempt = now + RETRY_DELAY_US;
                }
                if (!connecting && now >= next_attempt) {
                    station_disconnected.store(false);
                    esp_err_t result = esp_wifi_connect();
                    connecting = result == ESP_OK;
                    attempt_started = now;
                    next_attempt = now + RETRY_DELAY_US;
                    if (result != ESP_OK) {
                        station_disconnected.store(true);
                        ESP_LOGW(TAG, "Wi-Fi connect: %s; retrying", esp_err_to_name(result));
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void init_wifi() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    sta_netif = esp_netif_create_default_wifi_sta();
    if (!ap || !sta_netif) abort();
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, "fatiguesense-cam"));
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr));

    // Configure both interfaces before starting; only the requested mode runs.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t ap_config = {};
    std::memcpy(ap_config.ap.ssid, AP_SSID, sizeof(AP_SSID) - 1);
    ap_config.ap.ssid_len = sizeof(AP_SSID) - 1;
    std::memcpy(ap_config.ap.password, AP_PASSWORD, sizeof(AP_PASSWORD));
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    if (settings.client_mode) {
        wifi_config_t sta_config = {};
        std::memcpy(sta_config.sta.ssid, settings.ssid, std::strlen(settings.ssid));
        std::memcpy(sta_config.sta.password, settings.password, std::strlen(settings.password));
        sta_config.sta.threshold.authmode = settings.password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        sta_config.sta.pmf_cfg.capable = true;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(settings.client_mode ? WIFI_MODE_STA : WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));
    uint8_t station_mac[6] = {};
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, station_mac));
    std::snprintf(device_id, sizeof(device_id), "esp32s3-%02x%02x%02x%02x%02x%02x",
                  station_mac[0], station_mac[1], station_mac[2],
                  station_mac[3], station_mac[4], station_mac[5]);
    ESP_LOGI(TAG, "Enterprise device_id: %s", device_id);
    hotspot_active.store(!settings.client_mode);
    if (!settings.client_mode) ESP_LOGI(TAG, "Hotspot ready: %s | http://192.168.4.1/", AP_SSID);
    if (xTaskCreate(network_task, "network", 4096, nullptr, 4, nullptr) != pdPASS) abort();
}

// Settings pages use the hotspot password for HTTP Basic login (user: admin).
// HTTP is intended for a trusted local network, not Internet exposure.
static bool authorize_settings(httpd_req_t *req) {
    char header[160] = {};
    unsigned char decoded[96] = {};
    size_t decoded_length = 0;
    char expected[sizeof(AP_PASSWORD) + 6] = {};
    std::snprintf(expected, sizeof(expected), "admin:%s", AP_PASSWORD);
    if (httpd_req_get_hdr_value_str(req, "Authorization", header, sizeof(header)) == ESP_OK &&
        std::strncmp(header, "Basic ", 6) == 0 &&
        mbedtls_base64_decode(decoded, sizeof(decoded), &decoded_length,
            reinterpret_cast<const unsigned char *>(header + 6), std::strlen(header + 6)) == 0 &&
        decoded_length == std::strlen(expected) &&
        std::memcmp(decoded, expected, decoded_length) == 0) return true;
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"FatigueSense Wi-Fi\"");
    httpd_resp_send(req, "Log in with admin and the ESP hotspot password.", HTTPD_RESP_USE_STRLEN);
    return false;
}

static esp_err_t wifi_page_handler(httpd_req_t *req) {
    if (!authorize_settings(req)) return ESP_OK;
    extern const char wifi_html[] asm("_binary_wifi_setup_html_start");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, wifi_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t wifi_status_handler(httpd_req_t *req) {
    if (!authorize_settings(req)) return ESP_OK;
    esp_netif_ip_info_t info = {};
    esp_netif_get_ip_info(sta_netif, &info);
    char address[16] = {};
    if (station_ready.load()) std::snprintf(address, sizeof(address), IPSTR, IP2STR(&info.ip));
    cJSON *json = cJSON_CreateObject();
    if (!json) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    cJSON_AddStringToObject(json, "device_id", device_id);
    cJSON_AddStringToObject(json, "mode", settings.client_mode ? "sta" : "ap");
    cJSON_AddStringToObject(json, "ssid", settings.ssid);
    cJSON_AddBoolToObject(json, "connected", station_ready.load());
    cJSON_AddBoolToObject(json, "hotspot_active", hotspot_active.load());
    cJSON_AddBoolToObject(json, "camera_ready", camera_ready.load());
    cJSON_AddBoolToObject(json, "restarting", restart_at.load() != 0);
    cJSON_AddNumberToObject(json, "disconnect_reason", disconnect_reason.load());
    cJSON_AddStringToObject(json, "ip", address);
    char *body = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (!body) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    cJSON_free(body);
    return result;
}

static esp_err_t wifi_config_handler(httpd_req_t *req) {
    if (!authorize_settings(req)) return ESP_OK;
    if (restart_at.load()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_send(req, "Restart already scheduled", HTTPD_RESP_USE_STRLEN);
    }
    // Require JSON and a custom header: browsers cannot submit a cross-origin
    // form to this endpoint. No CORS/OPTIONS handler grants cross-origin access.
    char content_type[64] = {}, requested_with[32] = {};
    if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type)) != ESP_OK ||
        std::strcmp(content_type, "application/json") != 0 ||
        httpd_req_get_hdr_value_str(req, "X-Requested-With", requested_with, sizeof(requested_with)) != ESP_OK ||
        std::strcmp(requested_with, "FatigueSense") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON and X-Requested-With: FatigueSense required");
    }
    char body[1024] = {};
    if (req->content_len == 0 || req->content_len >= sizeof(body))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body size");
    size_t received = 0;
    while (received < req->content_len) {
        int count = httpd_req_recv(req, body + received, req->content_len - received);
        if (count == HTTPD_SOCK_ERR_TIMEOUT)
            return httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Request timed out");
        if (count <= 0) return ESP_FAIL;
        received += count;
    }
    cJSON *json = cJSON_ParseWithLengthOpts(body, received + 1, nullptr, true);
    if (!cJSON_IsObject(json)) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected JSON object");
    }
    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(json, "mode");
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(json, "ssid");
    const cJSON *password = cJSON_GetObjectItemCaseSensitive(json, "password");
    const cJSON *forget = cJSON_GetObjectItemCaseSensitive(json, "forget");
    NetworkSettings next = settings;
    bool valid = cJSON_IsString(mode) && (!forget || cJSON_IsBool(forget));
    if (valid && std::strcmp(mode->valuestring, "ap") == 0) {
        next.client_mode = 0;
        if (cJSON_IsTrue(forget)) {
            std::memset(next.ssid, 0, sizeof(next.ssid));
            std::memset(next.password, 0, sizeof(next.password));
        }
    } else if (valid && std::strcmp(mode->valuestring, "sta") == 0 && !cJSON_IsTrue(forget)) {
        next.client_mode = 1;
        if (ssid || password) {
            valid = cJSON_IsString(ssid) && cJSON_IsString(password) &&
                valid_credentials(ssid->valuestring, password->valuestring);
            if (valid) {
                std::memset(next.ssid, 0, sizeof(next.ssid));
                std::memset(next.password, 0, sizeof(next.password));
                std::strcpy(next.ssid, ssid->valuestring);
                std::strcpy(next.password, password->valuestring);
            }
        } else valid = valid_credentials(next.ssid, next.password);
    } else valid = false;
    cJSON_Delete(json);
    if (!valid) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
        "Use mode ap or sta; SSID 1-32 bytes; password empty, 8-63 bytes, or 64 hex digits");
    if (save_settings(next) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not save settings; no restart");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    // The control server serializes all config requests. Keep current settings
    // immutable; the networking task reads the new blob on the next boot.
    esp_err_t result = httpd_resp_send(req, "{\"saved\":true,\"restart_seconds\":2}", HTTPD_RESP_USE_STRLEN);
    restart_at.store(esp_timer_get_time() + 2000000);
    return result;
}
static constexpr gpio_num_t RED_LED = GPIO_NUM_1;
static constexpr gpio_num_t GREEN_LED = GPIO_NUM_2;

static void init_leds() {
    ESP_ERROR_CHECK(gpio_set_level(RED_LED, 0));
    ESP_ERROR_CHECK(gpio_set_level(GREEN_LED, 0));
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << RED_LED) | (1ULL << GREEN_LED);
    config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&config));
}

static void init_buzzer() {
    // Camera uses timer/channel 0; the existing transistor-driven buzzer is GPIO21.
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_1;
    timer.freq_hz = 100;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channel = {};
    channel.gpio_num = 21;
    channel.speed_mode = LEDC_LOW_SPEED_MODE;
    channel.channel = LEDC_CHANNEL_1;
    channel.timer_sel = LEDC_TIMER_1;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

// Only this task writes buzzer duty. App alerts never write LED pins.
static std::atomic<int> buzzer_mode{0}; // 0 off, 1 fatigue ramp, 2 three beeps, 3 browser beep
static void app_buzzer_task(void *) {
    int previous = 0;
    int64_t started = 0;
    while (true) {
        int mode = buzzer_mode.load();
        if (mode != previous) { started = esp_timer_get_time(); previous = mode; }
        int elapsed = static_cast<int>((esp_timer_get_time() - started) / 1000);
        int duty = 0;
        bool finished = false;
        if (mode == 1) duty = elapsed >= 2500 ? 1023 : 40 + 983 * elapsed / 2500;
        if (mode == 2) {
            duty = (elapsed < 200 || (elapsed >= 350 && elapsed < 550) ||
                    (elapsed >= 700 && elapsed < 900)) ? 1023 : 0;
            finished = elapsed >= 1500;
        }
        if (mode == 3) { duty = elapsed < 500 ? 1023 : 0; finished = elapsed >= 500; }
        if (finished) { int expected = mode; buzzer_mode.compare_exchange_strong(expected, 0); }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static esp_err_t health_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "ok", 2);
}

static esp_err_t app_alert_handler(httpd_req_t *req) {
    char query[128], type[16] = {}, sequence[16] = {}, outputs[16] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "type", type, sizeof(type)) != ESP_OK ||
        httpd_query_key_value(query, "seq", sequence, sizeof(sequence)) != ESP_OK ||
        httpd_query_key_value(query, "outputs", outputs, sizeof(outputs)) != ESP_OK ||
        std::strcmp(outputs, "buzzer") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "type, seq and outputs=buzzer required");
    }
    if (!sequence[0] || std::strspn(sequence, "0123456789") != std::strlen(sequence))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid seq");
    unsigned long long parsed = std::strtoull(sequence, nullptr, 10);
    if (parsed == 0 || parsed > UINT32_MAX)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid seq");
    int mode;
    if (std::strcmp(type, "fatigue") == 0) mode = 1;
    else if (std::strcmp(type, "yawn_fatigue") == 0) mode = 2;
    else if (std::strcmp(type, "clear") == 0 || std::strcmp(type, "yawn") == 0) mode = 0;
    else return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown alert type");
    static uint32_t last_seq = 0;
    uint32_t seq = static_cast<uint32_t>(parsed);
    if ((seq >> 16) == (last_seq >> 16) && seq <= last_seq)
        return httpd_resp_send(req, "stale", 5);
    last_seq = seq;
    // A yawn-only notification must not interrupt an ongoing three-beep alert.
    if (!(std::strcmp(type, "yawn") == 0 && buzzer_mode.load() == 2))
        buzzer_mode.store(mode);
    ESP_LOGI(TAG, "App buzzer-only alert: %s seq=%lu", type, static_cast<unsigned long>(seq));
    return health_handler(req);
}

static esp_err_t output_handler(httpd_req_t *req) {
    const char *label = static_cast<const char *>(req->user_ctx);
    const bool all = std::strcmp(label, "B4: All three") == 0;
    const bool red = all || std::strcmp(label, "B1: Red LED") == 0;
    const bool green = all || std::strcmp(label, "B2: Green LED") == 0;
    const bool buzz = all || std::strcmp(label, "B3: Buzzer") == 0;
    ESP_LOGI(TAG, "%s: ON for 500 ms", label);
    ESP_ERROR_CHECK(gpio_set_level(RED_LED, red));
    ESP_ERROR_CHECK(gpio_set_level(GREEN_LED, green));
    buzzer_mode.store(buzz ? 3 : 0);
    // Only the control server waits; the stream has its own server task.
    // Always turn off locally, even if the browser disconnects during the beep.
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(gpio_set_level(RED_LED, 0));
    ESP_ERROR_CHECK(gpio_set_level(GREEN_LED, 0));
    buzzer_mode.store(0);
    ESP_LOGI(TAG, "%s: OFF", label);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Test complete - outputs off", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t init_camera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 11;
    config.pin_d1 = 9;
    config.pin_d2 = 8;
    config.pin_d3 = 10;
    config.pin_d4 = 12;
    config.pin_d5 = 18;
    config.pin_d6 = 17;
    config.pin_d7 = 16;
    config.pin_xclk = 15;
    config.pin_pclk = 13;
    config.pin_vsync = 6;
    config.pin_href = 7;
    config.pin_sccb_sda = 4;
    config.pin_sccb_scl = 5;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA; // 128x128 diagnostic stream
    config.jpeg_quality = 20;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    return esp_camera_init(&config);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    if (!camera_ready.load()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "Camera initialization failed; Wi-Fi setup is still available.", HTTPD_RESP_USE_STRLEN);
    }
    esp_err_t result = httpd_resp_set_type(
        req, "multipart/x-mixed-replace;boundary=frame");
    if (result != ESP_OK) return result;
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ESP_LOGI(TAG, "Browser stream connected");
    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame) {
            ESP_LOGE(TAG, "Camera capture failed");
            return ESP_FAIL;
        }
        if (!frame->buf || frame->format != PIXFORMAT_JPEG || frame->len < 4 ||
            frame->buf[0] != 0xff || frame->buf[1] != 0xd8 ||
            frame->buf[frame->len - 2] != 0xff || frame->buf[frame->len - 1] != 0xd9) {
            esp_camera_fb_return(frame);
            ESP_LOGE(TAG, "Invalid JPEG; ending stream");
            return ESP_FAIL;
        }
        char header[96];
        int length = std::snprintf(header, sizeof(header),
            "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            static_cast<unsigned>(frame->len));
        if (length < 0 || static_cast<size_t>(length) >= sizeof(header)) {
            esp_camera_fb_return(frame);
            return ESP_FAIL;
        }
        result = httpd_resp_send_chunk(req, header, length);
        if (result == ESP_OK) {
            result = httpd_resp_send_chunk(req,
                reinterpret_cast<const char *>(frame->buf), frame->len);
        }
        esp_camera_fb_return(frame);
        if (result != ESP_OK) {
            ESP_LOGI(TAG, "Browser stream ended: %s", esp_err_to_name(result));
            return result;
        }
        // Limit load to below 10 FPS; never delay while holding a frame buffer.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req,
        "<!doctype html><html><head><meta name='viewport' content='width=device-width'>"
        "<title>Camera</title></head><body><h2>Camera stream</h2>"
        "<p><a href='/wifi'>Wi-Fi settings / change connection mode</a></p>"
        "<img id='camera' style='max-width:100%;height:auto' alt='Camera stream'>"
        "<p>Each button activates its outputs for 0.5 seconds.</p>"
        "<style>button{padding:16px;margin:6px;font-size:18px}</style>"
        "<button type='button' data-path='/red'>B1: Red LED</button>"
        "<button type='button' data-path='/green'>B2: Green LED</button>"
        "<button type='button' data-path='/buzzer'>B3: Buzzer</button>"
        "<button type='button' data-path='/all'>B4: All three</button>"
        "<p id='status' role='status'></p>"
        "<script>"
        "document.getElementById('camera').src='http://'+location.hostname+':81/stream';"
        "const buttons=document.querySelectorAll('button'),status=document.getElementById('status');"
        "buttons.forEach(button=>button.onclick=async()=>{"
        "buttons.forEach(b=>b.disabled=true);status.textContent=button.textContent+' running...';"
        "try{const r=await fetch(button.dataset.path,{method:'POST',signal:AbortSignal.timeout(5000)});"
        "if(!r.ok)throw new Error('HTTP '+r.status);status.textContent=await r.text();}"
        "catch(e){status.textContent='Request failed. Check the ESP32 log.';}"
        "finally{buttons.forEach(b=>b.disabled=false);}});"
        "</script>"
        "</body></html>", HTTPD_RESP_USE_STRLEN);
}

static void start_camera_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    config.max_uri_handlers = 12;
    httpd_handle_t server = nullptr;
    ESP_ERROR_CHECK(httpd_start(&server, &config));
    httpd_uri_t page = {};
    page.uri = "/";
    page.method = HTTP_GET;
    page.handler = page_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &page));
    static const char *paths[] = {"/red", "/green", "/buzzer", "/all"};
    static const char *labels[] = {"B1: Red LED", "B2: Green LED", "B3: Buzzer", "B4: All three"};
    for (int i = 0; i < 4; ++i) {
        httpd_uri_t output = {};
        output.uri = paths[i];
        output.method = HTTP_POST;
        output.handler = output_handler;
        output.user_ctx = const_cast<char *>(labels[i]);
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &output));
    }

    httpd_uri_t alert = {};
    alert.uri = "/alert";
    alert.method = HTTP_GET;
    alert.handler = app_alert_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &alert));
    httpd_uri_t health = {};
    health.uri = "/health";
    health.method = HTTP_GET;
    health.handler = health_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &health));

    httpd_uri_t wifi_page = {};
    wifi_page.uri = "/wifi";
    wifi_page.method = HTTP_GET;
    wifi_page.handler = wifi_page_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &wifi_page));
    httpd_uri_t wifi_status = {};
    wifi_status.uri = "/api/wifi";
    wifi_status.method = HTTP_GET;
    wifi_status.handler = wifi_status_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &wifi_status));
    httpd_uri_t wifi_config = {};
    wifi_config.uri = "/api/wifi";
    wifi_config.method = HTTP_POST;
    wifi_config.handler = wifi_config_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &wifi_config));

    // A continuous stream blocks its HTTP server task, so use a second server.
    config.server_port = 81;
    config.ctrl_port = 32769;
    httpd_handle_t stream_server = nullptr;
    ESP_ERROR_CHECK(httpd_start(&stream_server, &config));
    httpd_uri_t stream = {};
    stream.uri = "/stream";
    stream.method = HTTP_GET;
    stream.handler = stream_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(stream_server, &stream));
    ESP_LOGI(TAG, "HTTP ready: control :80, Wi-Fi setup :80/wifi, MJPEG :81/stream (one stream at a time)");
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "FatigueSense: camera streaming with configurable Wi-Fi");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    load_settings();
    init_wifi();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t camera_result = init_camera();
    if (camera_result != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s; Wi-Fi settings remain available",
                 esp_err_to_name(camera_result));
    } else {
        camera_ready.store(true);
        ESP_LOGI(TAG, "Camera initialized");
    }
    init_leds();
    init_buzzer();
    if (xTaskCreate(app_buzzer_task, "app_buzzer", 2048, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not start buzzer task");
        return;
    }
    start_camera_server();
    // Wi-Fi, DHCP and HTTP continue in ESP-IDF's background tasks.
}


