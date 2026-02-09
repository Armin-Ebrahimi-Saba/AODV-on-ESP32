/*
 * espnow_unicast_main.c
 * Networked Embedded Systems
 * Telecommunication Networks Group
 * TU Berlin 2023
 * Based on example codes of Espressif
 */

#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/idf_additions.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "esp_console.h"

#include "espnow_example.h"
#include "espnow_aodv.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_receiver";

/* static */ QueueHandle_t s_example_espnow_queue;
/* static */ QueueHandle_t aodv_rx_queue;
/* static */ QueueHandle_t aodv_tx_queue;

/* static */ uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static uint8_t local_mac[ESP_NOW_ETH_ALEN];



void init_console(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = PROMPT_STR "-->";

    esp_console_register_help_command();
    const esp_console_cmd_t send_cmd = {
        .command = "send",
        .help = "Sendet eine AODV Nachricht",
        .hint = "<mac> <msg>",
        .func = esp_handle_user_input};
    ESP_ERROR_CHECK(esp_console_cmd_register(&send_cmd));
    // register_system_common();

    // #if SOC_LIGHT_SLEEP_SUPPORTED
    //     register_system_light_sleep();
    // #endif
    // #if SOC_LIGHT_DEEP_SUPPORTED
    //     register_system_deep_sleep();
    // #endif

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}




void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESPNOW_WIFI_IF, local_mac));
    ESP_LOGI(TAG, "My MAC Address: "MACSTR"", MAC2STR(local_mac));
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
    
    uint8_t *mac_addr = info->des_addr;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // if (xQueueSendFromISR(aodv_tx_queue, &evt, &xHigherPriorityTaskWoken) != pdTRUE) {
    //     ESP_LOGW(TAG, "Send send queue fail");
    // }
    //  ESP_LOGI(TAG, "Succesfully put a message in the queue.");
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    ESP_LOGI(TAG, "Received a packet from "MACSTR"", MAC2STR(mac_addr));
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(aodv_rx_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Prepare ESPNOW data to be sent. */
static void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    /* Fill all remaining bytes after the data with random values */
    //esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint16_t *seq, uint8_t **payload)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;
    uint8_t payload_len = data_len - sizeof(example_espnow_data_t);

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *seq = buf->seq_num;
    *payload = malloc(payload_len);
    if (*payload == NULL) {
        ESP_LOGE(TAG, "Malloc payload buffer");
        return -1;
    }
    memcpy(*payload, buf->payload, payload_len);
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

uint8_t add_peer(uint8_t* mac_addr, bool encrypt)
{
    /* If MAC address does not exist in peer list, add it to peer list. */
    if (esp_now_is_peer_exist(mac_addr) == false) {
        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL) {
            ESP_LOGE(TAG, "Malloc peer information fail");
            esp_now_deinit();
            vTaskDelete(NULL);
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = encrypt;
        if(encrypt){
            memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
        }
        memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
        ESP_LOGI(TAG, "See peer for the first time and add it to my list.");
        free(peer);
        return 1;
    } else {
        return 0;
    }
}

static void send_broadcast(void *pvParameter)
{
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    example_espnow_data_prepare(send_param);
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
    vTaskDelete(NULL);
}

void handle_aodv_msg(const uint8_t* src_addr, 
                  uint8_t *data, int len)
{
    if (len < 1) return;

    switch (data[0]) {
        case AODV_RREQ:
            aodv_handle_rreq(src_addr, (aodv_rreq_t*)data);
            break;
        case AODV_RREP:
            aodv_handle_rrep(src_addr, (aodv_rrep_t*)data);
            break;
        case AODV_RERR:
            aodv_handle_rerr(src_addr, (aodv_rerr_t*)data);
            break;
    }
}

static void beacon_task(void* pvParameter) {
    ESP_LOGI(TAG, "Start broadcasting beacons every 10 seconds.");
    example_espnow_send_param_t *send_param;
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        esp_now_deinit();
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->len = sizeof(example_espnow_data_t) + 25;
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        esp_now_deinit();
    }
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    memcpy(buf->payload, "Hello World: I am in NES", 25);
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);
    while (1) {
        vTaskDelay(7000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending beacon...");
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
            ESP_LOGE(TAG, "Send error");
            free(send_param->buffer);
            free(send_param);
            vTaskDelete(NULL);
        }
    }
}


static void main_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint16_t recv_seq = 0;
    int ret;
    bool is_broadcast = false;

    uint8_t local_mac[6];
    uint8_t other_mac[6] = { 0x34, 0x85, 0x18, 0xb9, 0x1a, 0xb4 };
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, local_mac);

    while (true) {
        if (xQueueReceive(aodv_rx_queue, &evt, 100/portTICK_PERIOD_MS) == pdTRUE) {
            example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
            uint8_t* payload = NULL;

            ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_seq, &payload);
            free(recv_cb->data);
            if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d, data: %s", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len, payload);
                
                /*********************************************/
                /*************** STEP 1 **********************/
                /* Remove the comment in the next line to    */
                /* add the broadcast node to your peer list  */
                /*********************************************/
                uint8_t is_new_peer = add_peer(recv_cb->mac_addr, true);

                if (is_new_peer != 0) {
                    uint8_t* mac_addr = malloc(ESP_NOW_ETH_ALEN);
                    memcpy(mac_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    ESP_LOGI(TAG, "Broadcasting RREQ to find a route to "MACSTR"", MAC2STR(recv_cb->mac_addr));
                    // xTaskCreatePinnedToCore(aodv_send_rreq, "aodv_send_rreq", 4096, mac_addr, 5, NULL, 0);
                    aodv_send_rreq(mac_addr);
                    free(mac_addr);
                }

                if (payload[0] == AODV_RREQ) {
                    handle_aodv_msg(recv_cb->mac_addr, payload, recv_cb->data_len - sizeof(example_espnow_data_t));
                    aodv_rreq_t* rreq = (aodv_rreq_t*) payload;
                    ESP_LOGI(TAG, "RREQ >>> Type: %d | Hop: %d | ID: %" PRIu32 " | Src: "MACSTR" (Seq: %" PRIu32 ") | Dest: "MACSTR" (Seq: %" PRIu32 ")",
                        rreq->type,
                        rreq->hop_count,
                        rreq->rreq_id,
                        MAC2STR(rreq->src),
                        rreq->src_seq,
                        MAC2STR(rreq->dest),
                        rreq->dest_seq);
                }

            } else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d, data: %s", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len, payload);
                if (payload[0] == AODV_RREP) {
                    handle_aodv_msg(recv_cb->mac_addr, payload, recv_cb->data_len - sizeof(example_espnow_data_t));
                    aodv_rrep_t* rrep = (aodv_rrep_t*) payload;
                    ESP_LOGI(TAG, "RREP >>> Type: %d | Hop: %d | Dest: "MACSTR" (Seq: %" PRIu32 ") | Origin: "MACSTR" ",
                        rrep->type,
                        rrep->hop_count,
                        MAC2STR(rrep->dest),
                        rrep->dest_seq,
                        MAC2STR(rrep->origin)
                    );
                }
            } else {
                ESP_LOGE(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
            }
            free(payload);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);

    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;
    
    // s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    // if (s_example_espnow_queue == NULL) {
    //     ESP_LOGE(TAG, "Create mutex fail");
    //     return ESP_FAIL;
    // }
    aodv_rx_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (aodv_rx_queue == NULL) {
        ESP_LOGE(TAG, "Create rx mutex fail");
        return ESP_FAIL;
    }

    aodv_tx_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (aodv_tx_queue == NULL) {
        ESP_LOGE(TAG, "Create tx mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    add_peer(broadcast_mac, false);
    
    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->len = sizeof(example_espnow_data_t) + 25;
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        esp_now_deinit();
        return ESP_FAIL;
    }
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    memcpy(buf->payload, "Hello World: I am in NES", 25);
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);
    
    xTaskCreate(main_task, "main", 4096, NULL, 4, NULL);
    xTaskCreatePinnedToCore(beacon_task, "beacon", 4096, NULL, 3, NULL, 0);

    init_console();
    return ESP_OK;
}

void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    ESP_LOGI(TAG, "-----------Deinitializing ESPNOW-----------");
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    vSemaphoreDelete(aodv_rx_queue);
    vSemaphoreDelete(aodv_tx_queue);
    esp_now_deinit();
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    aodv_init();
    example_espnow_init();
    
    /*********************************************/
    /*************** STEP 3 **********************/
    /* Remove the comment in the next line to    */
    /* go to periodic sleep                      */
    /*********************************************/
    /*while(1){
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //esp_wdt_reset();
        esp_wifi_stop();
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(1000));
        esp_light_sleep_start();
        esp_wifi_start();
    }*/
}
