#pragma once
#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "camera_pin.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"

#define TEST_ESP_OK(ret) assert(ret == ESP_OK)
#define TEST_ASSERT_NOT_NULL(ret) assert(ret != NULL)

extern bool auto_jpeg_support;

static const char *TAG = "camera";
esp_err_t init_camera(uint32_t, pixformat_t, framesize_t, uint8_t);

#endif