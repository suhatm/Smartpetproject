cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smartpetring_power_ui)

target_sources(app PRIVATE
    src/app/main.c
    src/app/sensor_hub_bridge.c
    src/app/recorder.c
    src/bluetooth/ble_sensor_hub.c
    src/hardware/power_control.c
    src/hardware/status_led.c
    src/hardware/imu_sensor.c
    src/hardware/app_watchdog.c
    src/hardware/imu_stream.c
    src/hardware/temp_sensor.c
    src/hardware/sd_store.c
    src/hardware/mic_pdm.c
    src/hardware/body_imu.c
    src/hardware/qvar_sensor.c
    src/hardware/pvdf_adc.c
    src/hardware/batt_gauge.c
)

target_include_directories(app PRIVATE
    src/app
    src/bluetooth
    src/hardware
)
