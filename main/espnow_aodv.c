#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/projdefs.h"
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

#include <inttypes.h>

#include "espnow_example.h"
#include "espnow_aodv.h"


static const char* TAG = "AODV";
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t local_mac[ESP_NOW_ETH_ALEN];
static uint32_t local_seq_num = 0;
static aodv_route_t routing_table[AODV_MAX_ROUTES];
static aodv_rreq_cache_entry_t rreq_cache[AODV_MAX_ROUTES];
static aodv_data_t buffer;

extern QueueHandle_t s_example_espnow_queue;
extern uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX];


int esp_handle_user_input(int argc, char **argv)
{

    uint8_t adr[ESP_NOW_ETH_ALEN];
    sscanf(argv[1], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
           &adr[0],
           &adr[1],
           &adr[2],
           &adr[3],
           &adr[4],
           &adr[5]);

    ESP_LOGI(TAG, "adresse: ...%d...", adr);
    aodv_send_data(adr);
    return 0;
}


static void print_table_task(void *pvParameter)
{
    while (1) {
        vTaskDelay(7000/portTICK_PERIOD_MS);
        int64_t now = esp_timer_get_time();
        bool empty = true;
        ESP_LOGI(TAG, "My MAC Address: "MACSTR"", MAC2STR(local_mac));
        ESP_LOGI(TAG, "=========================== AODV ROUTING TABLE ============================");
        ESP_LOGI(TAG, "| %-17s | %-17s | %-3s | %-5s | %-8s | %-6s |", 
                "Dest IP", "Next Hop", "Hop", "Seq", "TTL(ms)", "Valid");
        ESP_LOGI(TAG, "|-------------------|-------------------|-----|-------|----------|--------|");

        for (int i = 0; i < AODV_MAX_ROUTES; i++) {
            // Option: Remove the 'if' to see invalid/empty slots too
             if (routing_table[i].valid) {
                empty = false;
                
                // Calculate remaining time in milliseconds
                int64_t remaining_time = (routing_table[i].expiry_time - now) / 1000;
                if (remaining_time < 0) remaining_time = 0;

                ESP_LOGI(TAG, "| "MACSTR" | "MACSTR" | %3d | %5" PRIu32 " | %8" PRId64 " | %s  |",
                        MAC2STR(routing_table[i].dest),
                        MAC2STR(routing_table[i].next_hop),
                        routing_table[i].hop_count,
                        routing_table[i].dest_seq,
                        remaining_time,
                        routing_table[i].valid ? "TRUE " : "FALSE");
            }
        }
        if (empty) {
            ESP_LOGI(TAG, "|                  (Table is Empty)                 ");
        }
        ESP_LOGI(TAG, "===========================================================================");
    }
}

static int64_t aodv_get_time_us(void) {
    return esp_timer_get_time();
}

static void mac_addr_copy(uint8_t *dst, const uint8_t *src) {
    memcpy(dst, src, ESP_NOW_ETH_ALEN);
}

static bool mac_addr_equal(const uint8_t *addr1, const uint8_t *addr2) {
    return memcmp(addr1, addr2, ESP_NOW_ETH_ALEN) == 0;
}

static bool is_originator(const uint8_t *originator)
{
    return memcmp(originator, local_mac, ESP_NOW_ETH_ALEN) == 0;
}

static void delete_route(const uint8_t idx) {
    memset(&routing_table[idx], 0, sizeof(aodv_route_t));
}

static aodv_route_t *aodv_find_route(const uint8_t *dest)
{
    for (int i = 0; i < AODV_MAX_ROUTES; i++) {
        if (routing_table[i].valid &&
            memcmp(routing_table[i].dest, dest, ESP_NOW_ETH_ALEN) == 0) {
            return &routing_table[i];
        }
    }
    return NULL;
}

static aodv_route_t *aodv_get_reverse_route(uint8_t *originator)
{
    aodv_route_t *route = aodv_find_route(originator);

    if (!route || !route->valid) {
        return NULL;
    }

    return route;
}

static aodv_route_t *aodv_alloc_route_entry(void)
{
    // index of the last route entry to be returned
    // we never delete a route for the sake of demo
    static uint8_t i = AODV_MAX_ROUTES - 1; 
    i = (i+1)%AODV_MAX_ROUTES;
    memset(&routing_table[i], 0, sizeof(aodv_route_t));
    return &routing_table[i];
}

void aodv_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    /* Fill all remaining bytes after the data with random values */
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

void aodv_update_route(
    const uint8_t *dest,
    const uint8_t *next_hop,
    uint8_t hop_count,
    uint32_t dest_seq
) {
    aodv_route_t *route = aodv_find_route(dest);

    if (!route) {

        route = aodv_alloc_route_entry();
        if (!route){
         return; // routing table full
        }
    }
    if (!route->valid ||
        dest_seq > route->dest_seq ||
        (dest_seq == route->dest_seq && hop_count < route->hop_count)) {

        memcpy(route->dest, dest, ESP_NOW_ETH_ALEN);
        memcpy(route->next_hop, next_hop, ESP_NOW_ETH_ALEN);
        route->hop_count = hop_count;
        route->dest_seq = dest_seq;
        route->valid = true;
        route->expiry_time = esp_timer_get_time() + AODV_ROUTE_TIMEOUT_MS * 1000;
        ESP_LOGI(TAG, "Hop: %" PRIu8 " | Seq: %" PRIu32 " | Expiry: %" PRId64 " | Valid: %s | Dest: "MACSTR" | Next Hop: "MACSTR"", 
            route->hop_count, route->dest_seq, route->expiry_time, route->valid ? "true" : "false", MAC2STR(route->dest), MAC2STR(route->next_hop));
    }
}

void aodv_update_route2(
    const uint8_t *dest,
    uint32_t dest_seq
) {
    aodv_route_t *route = aodv_find_route(dest);

    if (!route) {
        route = aodv_alloc_route_entry();
        if (!route) return; // routing table full
    }

    if (!route->valid ||
        dest_seq > route->dest_seq) {
            
        route->dest_seq = dest_seq;
        route->valid = true;
        route->expiry_time = esp_timer_get_time() + AODV_ROUTE_TIMEOUT_MS * 1000;
        ESP_LOGI(TAG, "Hop: %" PRIu8 " | Seq: %" PRIu32 " | Expiry: %" PRId64 " | Valid: %s | Dest: "MACSTR" | Next Hop: "MACSTR"", 
            route->hop_count, route->dest_seq, route->expiry_time, route->valid ? "true" : "false", MAC2STR(route->dest), MAC2STR(route->next_hop));
    }
}

void aodv_send_rreq(void* pvParameter)
{
    static uint32_t rreq_id = 0;

    ESP_LOGI(TAG, "Sending RREQ message to "MACSTR".", MAC2STR((uint8_t*)pvParameter));
    example_espnow_send_param_t *send_param;
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->len = sizeof(example_espnow_data_t) + sizeof(aodv_rreq_t);
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
    }
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    aodv_rreq_t rreq = {
        .type = AODV_RREQ,
        .hop_count = 0,
        .rreq_id = rreq_id++,
        .src_seq = ++local_seq_num,
        .dest_seq = 0 // TODO: must be taken from the old entry in routing table if it is already there
    };
    // esp_wifi_get_mac(WIFI_IF_STA, rreq.src);
    memcpy(rreq.src, local_mac, ESP_NOW_ETH_ALEN);
    // set the final destination
    memcpy(rreq.dest, (uint8_t*)pvParameter, ESP_NOW_ETH_ALEN);
    memcpy(buf->payload, &rreq, sizeof(rreq));

    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    aodv_espnow_data_prepare(send_param);

    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        free(send_param->buffer);
        free(send_param);
    }
    free(send_param->buffer);
    free(send_param);
}
void aodv_send_data(void* pvParameter)
{

    ESP_LOGI(TAG, "Sending RDATA message to "MACSTR".", MAC2STR((uint8_t*)pvParameter));
    example_espnow_send_param_t *send_param;
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->len = sizeof(example_espnow_data_t) + sizeof(aodv_data_t);
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
    }
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    aodv_data_t rdata = {
        .type = AODV_DATA,
        .dest_seq = 0 // TODO: must be taken from the old entry in routing table if it is already there
    };
    // set the final destination
    memcpy(rdata.dest, (uint8_t*)pvParameter, ESP_NOW_ETH_ALEN);
    memset(rdata.message, 0, 6);
    memcpy(rdata.message, "Hello", 5);
    memcpy(buf->payload, &rdata, sizeof(rdata));

    // memcpy(buf->payload, "HELLO THIS IS A RREQ MESSAGE", 28);
    aodv_route_t *route = aodv_find_route(rdata.dest); 
    if(route != NULL){

    memcpy(send_param->dest_mac, route->next_hop, ESP_NOW_ETH_ALEN);

    aodv_espnow_data_prepare(send_param);

    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Sending data...");
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        free(send_param->buffer);
        free(send_param);
        // vTaskDelete(NULL);
    }
    free(send_param->buffer);
    free(send_param);
    // vTaskDelete(NULL);
    }
    else{
        buffer = rdata;
        aodv_send_rreq(rdata.dest);
    }
}

int aodv_send_rrep(
    const uint8_t *next_hop,
    uint8_t *originator,
    uint8_t *dest,
    uint32_t dest_seq,
    uint8_t hop_count
)
{
    ESP_LOGI(TAG,
        "Sending an RREP:\n"
        "  next hop    = "MACSTR"\n"
        "  originator  = "MACSTR"\n"
        "  dest        = "MACSTR"\n",
        MAC2STR(next_hop),
        MAC2STR(dest),
        MAC2STR(originator)
    );

    example_espnow_send_param_t *send_param;
    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->len = sizeof(example_espnow_data_t) + sizeof(aodv_rrep_t);
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        return ESP_FAIL;
    }
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    /* Create the RREP message and write it to the payload */
    /* BEGIN */
    aodv_rrep_t rrep;

    memset(&rrep, 0, sizeof(rrep));

    if (local_seq_num < dest_seq)
        local_seq_num = dest_seq;

    rrep.dest_seq = dest_seq;
    rrep.type = AODV_RREP;
    rrep.hop_count = hop_count;
    memcpy(rrep.dest, dest, ESP_NOW_ETH_ALEN);
    memcpy(rrep.origin, originator, ESP_NOW_ETH_ALEN);

    memcpy(buf->payload, &rrep, sizeof(rrep));
    /* END */

    memcpy(send_param->dest_mac, next_hop, ESP_NOW_ETH_ALEN);
    aodv_espnow_data_prepare(send_param);


    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        free(send_param->buffer);
        free(send_param);
    }
    free(send_param->buffer);
    free(send_param);
    return ESP_OK;
}

int aodv_send_rerr(uint8_t *dest)
{
    aodv_route_t *route = aodv_find_route(dest);
    if (!route) return ESP_FAIL;

    local_seq_num = (local_seq_num > route->dest_seq ?
                     local_seq_num : route->dest_seq) + 1;

    aodv_rerr_t rerr = {
        .type = AODV_RERR,
        .dest_seq = local_seq_num
    };

    memcpy(rerr.dest, dest, ESP_NOW_ETH_ALEN);

    route->valid = false;

    esp_now_send(broadcast_mac,
                 (uint8_t *)&rerr,
                 sizeof(rerr));
    return ESP_OK;
}

/* Insert RREQ messages to the cache so that
later already seen messages can be detected */
static void aodv_cache_rreq(uint8_t *orig_addr, uint32_t rreq_id) {
    // Find free slot or oldest entry
    int oldest_idx = 0;
    int64_t oldest_time = INT64_MAX;
    
    for (int i = 0; i < AODV_MAX_ROUTES; i++) {
        if (rreq_cache[i].timestamp == 0) {
            oldest_idx = i;
            break;
        }
        if (rreq_cache[i].timestamp < oldest_time) {
            oldest_time = rreq_cache[i].timestamp;
            oldest_idx = i;
        }
    }
    
    mac_addr_copy(rreq_cache[oldest_idx].orig_addr, orig_addr);
    rreq_cache[oldest_idx].rreq_id = rreq_id;
    rreq_cache[oldest_idx].timestamp = aodv_get_time_us();
}

/* RREQ Cache Management */
static bool aodv_is_rreq_cached(uint8_t *orig_addr, uint32_t rreq_id) {
    int64_t now = aodv_get_time_us();
    
    for (int i = 0; i < AODV_MAX_ROUTES; i++) {
        if (mac_addr_equal(rreq_cache[i].orig_addr, orig_addr) &&
                rreq_cache[i].rreq_id == rreq_id &&
                (now - rreq_cache[i].timestamp) < (AODV_PATH_DISCOVERY_TIME * 1000)) {
            return true;
        }
    }
    return false;
}


// sender is the previous hop and the originator of the rreq!
int aodv_handle_rreq(const uint8_t *sender, aodv_rreq_t *rreq)
{
    // ignore this if it is from us
    if (mac_addr_equal(rreq->src, local_mac)) {
        ESP_LOGD(TAG, "This RREQ msg is from us, ignoring");
        return ESP_OK; 
    }

    // Check if we've seen this RREQ before
    if (aodv_is_rreq_cached(rreq->src, rreq->rreq_id)) {
        ESP_LOGD(TAG, "RREQ already processed, ignoring");
        return ESP_OK;
    }    

    ESP_LOGI(TAG,
        "Handling an RREQ:\n"
        "  sender      = "MACSTR"\n"
        "  originator  = "MACSTR"\n"
        "  dest        = "MACSTR"\n",
        MAC2STR(sender),
        MAC2STR(rreq->src),
        MAC2STR(rreq->dest)
    );

    aodv_cache_rreq(rreq->src, rreq->rreq_id);
    rreq->hop_count++;

    ESP_LOGI(TAG,
        "RREQ>>> Inserting Route:"
    );
    aodv_update_route(
        rreq->src,
        sender,
        rreq->hop_count,
        rreq->src_seq
    );

    // TODO can replaced by adding a route oneself with hopcount of 0
    // check if we are the destination
    if (memcmp(rreq->dest, local_mac, ESP_NOW_ETH_ALEN) == 0) {
        aodv_send_rrep(sender, rreq->src, rreq->dest, rreq->dest_seq, 0);
        return ESP_OK;
    }

    aodv_route_t *route = aodv_find_route(rreq->dest);
    if (route && route->valid &&
        route->dest_seq >= rreq->dest_seq) {
        // Send RREP on behalf of destination
        aodv_send_rrep(sender, rreq->src, rreq->dest,
            rreq->dest_seq, route->hop_count);
        return ESP_OK;
    }
    

    // TODO this can be simplified by using the old send_param
    example_espnow_send_param_t *send_param;
    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    send_param->len = sizeof(example_espnow_data_t) + sizeof(aodv_rreq_t);
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        return ESP_FAIL;
    }
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    memcpy(buf->payload, rreq, sizeof(*rreq));

    aodv_espnow_data_prepare(send_param);

    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        free(send_param->buffer);
        free(send_param);
        return ESP_FAIL;
    }
    free(send_param->buffer);
    free(send_param);
    return ESP_OK;
}

int aodv_handle_rrep(const uint8_t *sender, aodv_rrep_t *rrep)
{
    if (mac_addr_equal(rrep->dest, local_mac)) {
        aodv_send_data(buffer.dest);
        return ESP_OK;
    }

    ESP_LOGI(TAG,
        "Handling an RREP:\n"
        "  sender      = "MACSTR"\n"
        "  originator  = "MACSTR"\n"
        "  dest        = "MACSTR"\n",
        MAC2STR(sender),
        MAC2STR(rrep->dest),
        MAC2STR(rrep->origin)
    );

    ESP_LOGI(TAG,
        "RREP>>> Inserting Route:"
    );
    rrep->hop_count++;
    aodv_update_route(
        rrep->dest,
        sender,
        rrep->hop_count,
        rrep->dest_seq
    );

    if (!is_originator(rrep->origin)) {
        // TODO: can remove this reverse_route function
        aodv_route_t* rroute = aodv_get_reverse_route(rrep->origin);
        if (rroute == NULL) {
            ESP_LOGE(TAG, "Non existing reverse route to "MACSTR"", MAC2STR(rrep->origin));
            return ESP_FAIL;
        }

        example_espnow_send_param_t *send_param;
        /* Initialize sending parameters. */
        send_param = malloc(sizeof(example_espnow_send_param_t));
        if (send_param == NULL) {
            ESP_LOGE(TAG, "Malloc send parameter fail");
            return ESP_FAIL;
        }
        memset(send_param, 0, sizeof(example_espnow_send_param_t));
        send_param->len = sizeof(example_espnow_data_t) + sizeof(aodv_rrep_t);
        send_param->buffer = malloc(send_param->len);
        if (send_param->buffer == NULL) {
            ESP_LOGE(TAG, "Malloc send buffer fail");
            free(send_param);
            return ESP_FAIL;
        }
        example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

        memcpy(buf->payload, rrep, sizeof(*rrep));

        memcpy(send_param->dest_mac, rroute->next_hop, ESP_NOW_ETH_ALEN);
        aodv_espnow_data_prepare(send_param);

        ESP_LOGI(TAG, "Forwarding RREP msg to "MACSTR"", MAC2STR(send_param->dest_mac));
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
            ESP_LOGE(TAG, "Send error");
            free(send_param->buffer);
            free(send_param);
            return ESP_FAIL;
        }
        free(send_param->buffer);
        free(send_param);
        return ESP_OK;
    } 
    ESP_LOGI(TAG, "Found a route to "MACSTR" with %3d hops", MAC2STR(rrep->dest), rrep->hop_count);
    return ESP_OK;
}

int aodv_handle_rerr(const uint8_t *sender, aodv_rerr_t *rerr)
{
    aodv_route_t *route = aodv_find_route(rerr->dest);
    if (!route) return ESP_FAIL;

    if (route->dest_seq <= rerr->dest_seq) {
        route->valid = false;
        esp_now_send(broadcast_mac,
                     (uint8_t *)rerr,
                     sizeof(*rerr));
    }
    return ESP_OK;
}

int aodv_handle_data(const uint8_t *sender, aodv_data_t *rdata){
    if (mac_addr_equal(rdata->dest, local_mac)) {
        ESP_LOGI(TAG, "MESSAGE: %s", rdata->message);
        return ESP_OK;
    }

    ESP_LOGI(TAG,
        "RDATA>>> Inserting Route:"
    );
    aodv_update_route2(
        rdata->dest,
        rdata->dest_seq
    );
    if (1) {
        // TODO: can remove this reverse_route function
        aodv_route_t* rroute = aodv_get_reverse_route(rdata->dest);
        if (rroute == NULL) {
            return ESP_FAIL;
        }

        example_espnow_send_param_t *send_param;
        /* Initialize sending parameters. */
        send_param = malloc(sizeof(example_espnow_send_param_t));
        if (send_param == NULL) {
            ESP_LOGE(TAG, "Malloc send parameter fail");
            return ESP_FAIL;
        }
        memset(send_param, 0, sizeof(example_espnow_send_param_t));
        send_param->len = sizeof(example_espnow_data_t) + sizeof(aodv_data_t);
        send_param->buffer = malloc(send_param->len);
        if (send_param->buffer == NULL) {
            ESP_LOGE(TAG, "Malloc send buffer fail");
            free(send_param);
            return ESP_FAIL;
        }
        example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

        memcpy(buf->payload, rdata, sizeof(*rdata));

        memcpy(send_param->dest_mac, rroute->next_hop, ESP_NOW_ETH_ALEN);
        aodv_espnow_data_prepare(send_param);

        ESP_LOGI(TAG, "Forwarding RDATA msg to "MACSTR"", MAC2STR(send_param->dest_mac));
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
            ESP_LOGE(TAG, "Send error");
            free(send_param->buffer);
            free(send_param);
            return ESP_FAIL;
        }
        free(send_param->buffer);
        free(send_param);
        return ESP_OK;
    } 
    return ESP_OK;
}

void aodv_init() {
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    if (err != ESP_OK) {
        ESP_LOGE("AODV", "Failed to get MAC address");
    }
    ESP_LOGI(TAG, "Local MAC address: "MACSTR"", MAC2STR(local_mac));
    xTaskCreatePinnedToCore(print_table_task, "print_table_task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(aodv_maintenance_task, "aodv_maintenance", 4096, NULL, 5, NULL, 1);
}

static void aodv_periodic_task() {
    int64_t now = esp_timer_get_time();
    // Check route timeouts
    for (size_t i = 0; i < AODV_MAX_ROUTES; i++) {
        if (routing_table[i].dest[0] == 0) // TODO: check for all bytes
            continue;
        if (!routing_table[i].valid) {
            ESP_LOGI(TAG, "Resending RREQ to "MACSTR"",
                     MAC2STR(routing_table[i].dest));
            aodv_send_rreq(routing_table[i].dest);
            delete_route(i);
        }
        else if (routing_table[i].expiry_time <= now) { // check if expired
            ESP_LOGI(TAG, "Route expired: dest="MACSTR"",
                     MAC2STR(routing_table[i].dest));
            routing_table[i].valid = false;
        }
    }
}

void aodv_maintenance_task(void *pvParameter) {
    while (1) {
        aodv_periodic_task();
        vTaskDelay(9000/portTICK_PERIOD_MS);
    }
}