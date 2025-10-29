#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int32.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

#define GPIO_INPUT_IO_0 13
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

rcl_publisher_t publisher;
static QueueHandle_t gpio_evt_queue = NULL;
std_msgs__msg__UInt32 msg;

//
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_init_options_t init_options;
rclc_executor_t executor;
//

#define DEBOUNCE_TIME_MS 50
static volatile uint32_t last_isr_time = 0;
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_isr_time > DEBOUNCE_TIME_MS)
    {
        last_isr_time = current_time;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
}

void gpio_interrupt_handler(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            while (gpio_get_level(io_num) == 0)
            {
                printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, 0);
                msg.data = 0;
                RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    RCSOFTCHECK(rcl_init_options_fini(&init_options));
}

void micro_ros_task(void *arg)
{
    allocator = rcl_get_default_allocator();
    while (1)
    {
        printf("Connecting to micro-ros-agent.\n");
        init_options = rcl_get_zero_initialized_init_options();
        RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        // Static Agent IP and port can be used instead of autodisvery.
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
        // RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
        RCSOFTCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

        bool connected = rmw_uros_ping_agent(100, 3) == RMW_RET_OK;
        if (!connected)
        {
            printf("Unable to ping micro-ros-agent, retrying...\n");
            RCSOFTCHECK(rcl_init_options_fini(&init_options));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        node = rcl_get_zero_initialized_node();

        RCCHECK(rclc_node_init_default(&node, "reconnection_example_node", "", &support));

        publisher = rcl_get_zero_initialized_publisher();

        RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
            "reconnection_example_publisher"));

        executor = rclc_executor_get_zero_initialized_executor();

        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        printf("Connected to micro-ros-agent.\n");
        while (1)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK)
            {
                // Break this while loop.
                printf("Lost connection to micro-ros-agent\n");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // This executes because we lost connection to micro-ros-agent.
        destroy_entities();
    }
    destroy_entities();
    vTaskDelete(NULL);
}

void app_main(void)
{
    gpio_config_t io_conf = {};
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    xTaskCreate(gpio_interrupt_handler,
                "gpio_task",
                2048,
                NULL,
                10,
                NULL);
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
}
