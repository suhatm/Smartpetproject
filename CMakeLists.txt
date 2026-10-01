cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smartpetring_power_ui)

target_sources(app PRIVATE
    src/app/main.c
    src/app/test_mailbox.c
    src/bluetooth/ble_led_service.c
    src/bluetooth/ble_imu_service.c
    src/hardware/power_control.c
    src/hardware/status_led.c
    src/hardware/imu_sensor.c
    src/hardware/imu_stream.c
)

target_include_directories(app PRIVATE
    src/bluetooth
    src/hardware
)
