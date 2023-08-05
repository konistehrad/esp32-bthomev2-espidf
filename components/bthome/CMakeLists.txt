cmake_minimum_required(VERSION 3.5)

idf_component_register(
  REQUIRED_IDF_TARGETS
    "esp32"
    "esp32s3"
    "esp32c3"
  INCLUDE_DIRS
    "src"
  SRCS
    "src/BTHome.cpp"
  REQUIRES
    bt
    nvs_flash
    esp-nimble-cpp
    mbedtls
)

  