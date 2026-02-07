#ifndef AODV_H
#define AODV_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_now.h"
#include "espnow_aodv.h"

#define AODV_MAX_ROUTES             16
#define AODV_ROUTE_TIMEOUT_MS       15000

#define AODV_NET_DIAMETER            35
#define AODV_NODE_TRAVERSAL_TIME     40      // milliseconds
#define AODV_NET_TRAVERSAL_TIME      (AODV_NODE_TRAVERSAL_TIME * AODV_NET_DIAMETER)
#define AODV_PATH_DISCOVERY_TIME     (2 * AODV_NET_TRAVERSAL_TIME)

typedef struct {
    uint8_t dest[ESP_NOW_ETH_ALEN];        // MAC address
    uint8_t next_hop[ESP_NOW_ETH_ALEN];
    uint8_t hop_count;
    uint32_t dest_seq;
    int64_t expiry_time;    // esp_timer_get_time()
    bool valid;
} aodv_route_t;

typedef enum {
    AODV_RREQ = 1,
    AODV_RREP = 2,
    AODV_RERR = 3
} aodv_msg_type_t;

// based on RFC 3561 
typedef struct {
    uint8_t type;              // AODV_RREQ 
    uint8_t hop_count;
    uint32_t rreq_id;
    uint8_t src[ESP_NOW_ETH_ALEN];
    uint32_t src_seq;
    uint8_t dest[ESP_NOW_ETH_ALEN];
    uint32_t dest_seq;
} __attribute__((packed)) aodv_rreq_t;

typedef struct {
    uint8_t type;               // AODV_RREP
    uint8_t hop_count;          // number of hops to destination
    uint8_t dest[ESP_NOW_ETH_ALEN];            // destination of original RREQ
    uint32_t dest_seq;          // destination sequence number
    uint8_t origin[ESP_NOW_ETH_ALEN];          // originator of RREQ
} __attribute__((packed)) aodv_rrep_t;

typedef struct {
    uint8_t type;              // AODV_RERR
    uint8_t dest[ESP_NOW_ETH_ALEN];           // unreachable destination
    uint32_t dest_seq;
} __attribute__((packed)) aodv_rerr_t;

/* RREQ Cache Entry */
typedef struct {
    uint8_t orig_addr[ESP_NOW_ETH_ALEN];
    uint32_t rreq_id;
    int64_t timestamp;                       // Timestamp in microseconds
} __attribute__((packed)) aodv_rreq_cache_entry_t;

/* =======================
 * Public API
 * ======================= */

void example_espnow_deinit(example_espnow_send_param_t *send_param);
void aodv_espnow_data_prepare(example_espnow_send_param_t *send_param);
void aodv_maintenance_task(void* pvParameter);

void aodv_send_rreq(void* pvParameter);
int aodv_send_rrep(
    const uint8_t *next_hop,
    uint8_t *originator,
    uint8_t *dest,
    uint32_t dest_seq,
    uint8_t hop_count
);
int aodv_send_rerr(uint8_t *dest);

int aodv_handle_rreq(const uint8_t *sender, aodv_rreq_t *rreq);
int aodv_handle_rrep(const uint8_t *sender, aodv_rrep_t *rrep);
int aodv_handle_rerr(const uint8_t *sender, aodv_rerr_t *rerr);

void aodv_init();

#endif /* AODV_H */
