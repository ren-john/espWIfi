idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_http_server esp_timer esp_netif driver json mbedtls esp32-camera
    EMBED_FILES "wifi_setup.html"
 )
