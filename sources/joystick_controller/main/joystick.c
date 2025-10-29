#include <string.h>
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "joystick.h"
#include "ros.h"

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN 256
#define Z_PIN_GPIO 14
#define ADC_MAX_VALUE 4095
#define DEAD_ZONE_THRESHOLD 0.1f

// ADC channels for X and Y axes (GPIO12, GPIO13)
static adc_channel_t channel[2] = {ADC_CHANNEL_4, ADC_CHANNEL_5};

static TaskHandle_t s_task_handle;
static const char *TAG = "joystick";
static joystick_input_t joystick_input = {0};
joystick_input_t *joystick_controller = &joystick_input;
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;
        // ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        // ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i,
        //          adc_pattern[i].channel);
        // ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static float scale_joystick_value(uint32_t raw_value)
{
    // Simple scaling: 0-1 range based on ADC_MAX_VALUE
    return (float)raw_value / (float)ADC_MAX_VALUE;
}

static void gpio_init(void)
{
    // Configure Z pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << Z_PIN_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "GPIO%d configured as input with pullup", Z_PIN_GPIO);
}

void joystick_task(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    gpio_init();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t),
                        &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    while (1)
    {

        /**
         * This is to show you the way to use the ADC continuous mode driver event
         * callback. This `ulTaskNotifyTake` will block when the data processing in
         * the task is fast. However in this example, the data processing (print) is
         * slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still
         * just call `adc_continuous_read()` here in a loop, with/without a certain
         * block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK)
            {
                uint32_t x_val = 0, y_val = 0;

                // Read ADC values
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);

                    if (chan_num == 4)
                        x_val = data;
                    if (chan_num == 5)
                        y_val = data;
                }

                // Scale to 0-1 range
                float scaled_x = scale_joystick_value(x_val);
                float scaled_y = scale_joystick_value(y_val);

                int z_val = gpio_get_level(Z_PIN_GPIO);

                // update global joystick state
                joystick_controller->x = scaled_x;
                joystick_controller->y = scaled_y;
                joystick_controller->z = (double)z_val;
                //TODO: send to a queue from this task?
                publish_twist_msg();

                vTaskDelay(100);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}