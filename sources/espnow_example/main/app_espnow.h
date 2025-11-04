#ifndef __APP_ESPNOW_H__
#define __APP_ESPNOW_H__

#include "esp_now.h"

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA

#define ESPNOW_QUEUE_SIZE 6
#define ESPNOW_PAYLOAD_HEAD_LEN (4)
#define ESPNOW_MAXDELAY (512)
#if defined(ESP_NOW_MAX_DATA_LEN_V2)
#define ESPNOW_PAYLOAD_MAX_LEN (1470)
#else
#define ESPNOW_PAYLOAD_MAX_LEN (250)
#endif
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum
{
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_send_cb_t;

typedef struct
{
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_recv_cb_t;

typedef union
{
    espnow_send_cb_t send_cb;
    espnow_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct
{
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum
{
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

typedef struct
{
    uint32_t seq;
    uint8_t payload[0];
} __attribute__((packed)) app_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct esp_now_msg_send
{
    uint32_t retry_times;
    uint32_t max_retry;
    uint32_t msg_len;
    void *sent_msg;
} espnow_send_param_t;

void espnow_task(void);
esp_err_t espnow_data_parse(const uint8_t *, uint16_t);
#endif