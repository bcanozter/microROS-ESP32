
// Standard libraries
#include <unistd.h>
#include <stdio.h>
#include <string.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP32
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"

// microROS
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "micro_ros_utilities/string_utilities.h"
#include "micro_ros_utilities/type_utilities.h"
#include "uros_network_interfaces.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Header files
#include "camera_pin.h"
#include "camera.h"

bool auto_jpeg_support;
static const char *TASK_TAG = "uros_camera_publisher";
#define RCCHECK(fn)                                                                             \
    {                                                                                           \
        rcl_ret_t temp_rc = fn;                                                                 \
        if ((temp_rc != RCL_RET_OK))                                                            \
        {                                                                                       \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                                  \
        }                                                                                       \
    }
#define RCSOFTCHECK(fn)                                                                           \
    {                                                                                             \
        rcl_ret_t temp_rc = fn;                                                                   \
        if ((temp_rc != RCL_RET_OK))                                                              \
        {                                                                                         \
            ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                         \
    }

rcl_publisher_t publisher;
sensor_msgs__msg__CompressedImage image;
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
        {
            image.data.size = frame->len;
            memcpy(image.data.data, frame->buf, frame->len);
            image.format = micro_ros_string_utilities_set(image.format, "jpeg");
            RCSOFTCHECK(rcl_publish(&publisher, &image, NULL));
            esp_camera_fb_return(frame);
        }
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "camera_publisher", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
        "/camera/image/compressed"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    //! rclc_timer_init_default is deprecated. using defaul2 instead.
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // IMPORTANT: https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/
    static micro_ros_utilities_memory_conf_t conf = {0};

    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;

    micro_ros_utilities_memory_rule_t rules[] = {
        //Maximum data capacity, ~100KB
        {"data", 102400},
    };
    conf.rules = rules;
    conf.n_rules = sizeof(rules) / sizeof(rules[0]);
    bool success = micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
        &image,
        conf);
    if (!success)
    {
        ESP_LOGE(TASK_TAG, "Unable to create message memory. Abort.");
        RCCHECK(rcl_publisher_fini(&publisher, &node));
        RCCHECK(rcl_node_fini(&node));
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main()
{
    // Init camera
    TEST_ESP_OK(init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_QVGA, 2));

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(micro_ros_task,
                TASK_TAG,
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}
