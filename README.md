# FatigueSense ESP32-S3 Camera

ESP-IDF firmware for the FatigueSense ESP32-S3 camera. The firmware starts a setup access point, accepts Wi-Fi credentials through a local page, joins the configured phone hotspot in station mode, and exposes camera/control and Wi-Fi recovery endpoints.

## Build

This project uses ESP-IDF 5.4.4 and targets `esp32s3`.

```powershell
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

The ESP-IDF component manager uses `main/idf_component.yml`; `dependencies.lock` records the resolved camera and JPEG components.

## Wi-Fi setup

On first boot, connect to `FatigueSense_Cam` with password `safedriving`, then open `http://192.168.4.1/wifi`. Enter the phone hotspot SSID and password and save. The ESP restarts into station mode and retries until the hotspot is available. If it cannot connect, it starts the setup hotspot again after about 30 seconds.

The control page is served on port 80. The MJPEG camera stream is on port 81 at `/stream`. `/wifi` and `/api/wifi` provide the local setup/status interface.

## Source layout

- `main/main.cpp`: Wi-Fi state machine, camera stream, control endpoints, and device startup.
- `main/wifi_setup.html`: embedded Wi-Fi setup and recovery page.
- `main/CMakeLists.txt`: ESP-IDF component registration and HTML embedding.
- `sdkconfig`: current ESP-IDF build configuration; `sdkconfig.defaults` contains project defaults.
