# _Low-power ESP32 BTHome BLE Broadcaster_
This project builds on the work of Christos Baltatzidis's BTHome implementation for Arduino, but removes the Arduino requirement, which makes it suitable for use in ESP-IDF. This project also includes example code for doing deep sleep, with wakeup and "pairing" mode provided by a GPIO wakeup.

## How to use
This is my first time working with ESP-IDF in any real capacity, so please do your best to open and configure it. I want to expand the configuration settings but who knows man who knows.

## Credits
[esp-nimble-cpp](https://github.com/h2zero/esp-nimble-cpp): used as the NimBLE interface.
[BTHome](https://github.com/Chreece/BTHomeV2-ESP32-example): Christos Baltatzidis's BTHomeV2 implementation was used as the basis for the esp-idf component.
[tweeny](https://github.com/mobius3/tweeny): The excellent C++ tweening library for the nice LED pulse
[ESP-IDF Components library](https://github.com/UncleRus/esp-idf-lib): used for interfacing with sensors etc
