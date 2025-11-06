#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "app_espnow.h"

static const char *TAG = "espnow_example";

static QueueHandle_t s_espnow_recv_queue = NULL;

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t espnow_payload[ESPNOW_PAYLOAD_MAX_LEN];
static uint32_t current_seq = 0;
static espnow_send_param_t *sent_msgs;
static SemaphoreHandle_t sent_msgs_mutex = NULL;

esp_err_t app_espnow_create_peer(uint8_t dst_mac[ESP_NOW_ETH_ALEN])
{
    esp_err_t ret = ESP_FAIL;
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        return ESP_ERR_NO_MEM;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));

    esp_now_get_peer(dst_mac, peer);
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    // memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
    memcpy(peer->peer_addr, dst_mac, ESP_NOW_ETH_ALEN);

    if (esp_now_is_peer_exist(dst_mac) == false)
    {
        ret = esp_now_add_peer(peer);
    }
    else
    {
        ret = esp_now_mod_peer(peer);
    }
    free(peer);

    return ret;
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGW(TAG, "Send OK to " MACSTR " %s %d", MAC2STR(mac_addr), __func__, __LINE__);
    }
    else
    {
        ESP_LOGW(TAG, "Send Fail %s %d ", __func__, __LINE__);
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = recv_info->src_addr;
    uint8_t *des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (espnow_data_parse(data, len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Receive espnow_data_parse error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr))
    {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    }
    else
    {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL)
    {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_espnow_recv_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

void espnow_data_prepare(uint8_t *buf, const uint8_t *payload, size_t payload_len, bool seq_init)
{
    app_espnow_data_t *temp = (app_espnow_data_t *)buf;
    if (seq_init)
    {
        temp->seq = 0;
        current_seq = 0;
    }
    else
    {
        temp->seq = ++current_seq;
    }
    ESP_LOGW(TAG, "free heap %" PRIu32 ", minimum %" PRIu32 "", esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    memcpy(temp->payload, payload, payload_len);
}

esp_err_t espnow_data_parse(const uint8_t *data, uint16_t data_len)
{
    esp_err_t ret = ESP_OK;
    app_espnow_data_t *buf = (app_espnow_data_t *)data;

    if (data_len < sizeof(app_espnow_data_t))
    {
        ESP_LOGD(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        ret = ESP_FAIL;
    }

    return ret;
}

esp_err_t esp_now_send_broadcast(const uint8_t *payload, size_t payload_len, bool seq_init)
{
    esp_err_t ret = ESP_OK;
    uint8_t *buf = calloc(1, payload_len + ESPNOW_PAYLOAD_HEAD_LEN);
    espnow_data_prepare(buf, payload, payload_len, seq_init);
    app_espnow_create_peer(s_broadcast_mac);
    ret = esp_now_send(s_broadcast_mac, buf, payload_len + ESPNOW_PAYLOAD_HEAD_LEN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error: %d [%s %d]", ret, __func__, __LINE__);
    }

    xSemaphoreTake(sent_msgs_mutex, portMAX_DELAY);
    sent_msgs->retry_times = 0;
    sent_msgs->max_retry = 1;
    sent_msgs->msg_len = payload_len + ESPNOW_PAYLOAD_HEAD_LEN;
    sent_msgs->sent_msg = buf;
    xSemaphoreGive(sent_msgs_mutex);
    return ret;
}

void espnow_task_main(void *pvParameter)
{
    espnow_event_t evt;

    while (xQueueReceive(s_espnow_recv_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        switch (evt.id)
        {
        case ESPNOW_RECV_CB:
        {
            espnow_recv_cb_t *recv_cb = &evt.info.recv_cb;
            app_espnow_data_t *buf = (app_espnow_data_t *)recv_cb->data;
            uint32_t recv_seq = buf->seq;
            memset(espnow_payload, 0x0, ESPNOW_PAYLOAD_MAX_LEN);
            memcpy(espnow_payload, buf->payload, recv_cb->data_len - ESPNOW_PAYLOAD_HEAD_LEN);
            ESP_LOGI(TAG, "Receive broadcast data from: " MACSTR ", len: %d, recv_seq: %" PRIu32 ", current_seq: %" PRIu32 "",
                     MAC2STR(recv_cb->mac_addr),
                     recv_cb->data_len,
                     recv_seq,
                     current_seq);
            free(recv_cb->data);
            recv_cb->data = NULL;
            break;
        }
        default:
            ESP_LOGE(TAG, "Callback type error: %d", evt.id);
            break;
        }
    }
}

static esp_err_t espnow_init(void)
{
    esp_err_t ret = ESP_FAIL;
    espnow_send_param_t *send_param;

    s_espnow_recv_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_recv_queue == NULL)
    {
        ESP_LOGE(TAG, "Create queue fail");
        return ret;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
#endif
    /* Set primary master key. */
    // ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    if (app_espnow_create_peer(s_broadcast_mac) != ESP_OK)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        esp_now_unregister_send_cb();
        vSemaphoreDelete(s_espnow_recv_queue);
        s_espnow_recv_queue = NULL;
        return ESP_FAIL;
    }

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_espnow_recv_queue);
        s_espnow_recv_queue = NULL;
        esp_now_deinit();
        return ret;
    }
    sent_msgs = (espnow_send_param_t *)malloc(sizeof(espnow_send_param_t));
    sent_msgs->max_retry = 0;
    sent_msgs->msg_len = 0;
    sent_msgs->retry_times = 0;
    sent_msgs->sent_msg = NULL;
    sent_msgs_mutex = xSemaphoreCreateMutex();

    xTaskCreate(espnow_task_main, "espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->sent_msg);
    free(send_param);
    vQueueDelete(s_espnow_recv_queue);
    s_espnow_recv_queue = NULL;
    esp_now_deinit();
}

static void esp_now_send_timer_cb(TimerHandle_t timer)
{
    xSemaphoreTake(sent_msgs_mutex, portMAX_DELAY);
    if (sent_msgs->max_retry > sent_msgs->retry_times)
    {
        sent_msgs->retry_times++;
        if (sent_msgs->sent_msg)
        {
            app_espnow_create_peer(s_broadcast_mac);
            esp_err_t ret = esp_now_send(s_broadcast_mac, sent_msgs->sent_msg, sent_msgs->msg_len);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Send error: %d [%s %d]", ret, __func__, __LINE__);
            }
        }
    }
    else
    {
        if (sent_msgs->max_retry)
        {
            sent_msgs->retry_times = 0;
            sent_msgs->max_retry = 0;
            sent_msgs->msg_len = 0;
            if (sent_msgs->sent_msg)
            {
                free(sent_msgs->sent_msg);
                sent_msgs->sent_msg = NULL;
            }
        }
    }
    xSemaphoreGive(sent_msgs_mutex);
}

static void debug_timer_cb(TimerHandle_t timer)
{
    uint8_t sta_mac[6] = {0};
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    char msg[100];
    sprintf(msg, "From[" MACSTR "] Hello World!!\n", MAC2STR(sta_mac));
    esp_now_send_broadcast((const uint8_t *)msg, sizeof(msg), true);
}

void espnow_task(void)
{
    espnow_init();
    TimerHandle_t esp_now_send_timer = xTimerCreate("esp_now_send_timer", 100 / portTICK_PERIOD_MS, pdTRUE,
                                                    NULL, esp_now_send_timer_cb);
    xTimerStart(esp_now_send_timer, portMAX_DELAY);

    // TimerHandle_t debug_timer = xTimerCreate("debug_timer", 5000 / portTICK_PERIOD_MS, pdTRUE,
    //     NULL, debug_timer_cb);
    // xTimerStart(debug_timer, portMAX_DELAY);
}