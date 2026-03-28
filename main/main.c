/*
 * Renpho ES-CS20M BLE Scale Client for ESP32 (ESP-IDF + NimBLE)
 *
 * Uses the Lefu/0x1A10 protocol:
 *   Service 0x1A10:
 *     0x2A10 - Notify: weight data and status from scale
 *     0x2A11 - Write:  commands to scale
 *
 * Handshake:
 *   1. Subscribe to notifications on 0x2A10
 *   2. Send user profile command (0x97) to 0x2A11
 *   3. Scale sends 0x11 (start), then 0x14 weight frames, then 0x11 (stop)
 *
 * Frame format: 55 AA [cmd] 00 [payload_len] [payload...] [checksum]
 */

#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "SCALE";

/* Lefu frame header */
#define LEFU_HDR_0  0x55
#define LEFU_HDR_1  0xAA

/* Command opcodes (written to 0x2A11) */
#define CMD_USER_PROFILE  0x97

/* Notification opcodes (received on 0x2A10) */
#define NTF_START_STOP  0x11
#define NTF_WEIGHT      0x14

/* State machine */
typedef enum {
    STATE_IDLE,
    STATE_SCANNING,
    STATE_CONNECTED,
    STATE_MEASURING,
} scale_state_t;

static scale_state_t state = STATE_IDLE;
static uint16_t conn_handle;

/* Last weight seen during measurement (used when scale signals "stop") */
static float last_weight_kg;

/* Characteristic handles */
static uint16_t notify_val_handle;
static uint16_t notify_cccd_handle;
static uint16_t write_val_handle;
static uint16_t svc_start_handle;
static uint16_t svc_end_handle;

/* UUIDs */
static const ble_uuid16_t svc_uuid    = BLE_UUID16_INIT(0x1A10);
static const ble_uuid16_t notify_uuid = BLE_UUID16_INIT(0x2A10);
static const ble_uuid16_t write_uuid  = BLE_UUID16_INIT(0x2A11);
static const ble_uuid16_t cccd_uuid   = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);

static int gap_event_handler(struct ble_gap_event *event, void *arg);
static void start_scan(void);

/* =========================================================================
 * LEFU PROTOCOL
 * ========================================================================= */

static void lefu_send_cmd(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[64];
    frame[0] = LEFU_HDR_0;
    frame[1] = LEFU_HDR_1;
    frame[2] = cmd;
    frame[3] = 0x00;
    frame[4] = payload_len;
    if (payload_len > 0) {
        memcpy(&frame[5], payload, payload_len);
    }

    uint8_t sum = 0;
    int total = 5 + payload_len;
    for (int i = 0; i < total; i++) sum += frame[i];
    frame[total] = sum;

    int rc = ble_gattc_write_no_rsp_flat(conn_handle, write_val_handle,
                                          frame, total + 1);
    if (rc != 0) {
        ESP_LOGE(TAG, "Write cmd 0x%02X failed: %d", cmd, rc);
    }
}

static void send_user_profile(void)
{
    time_t now = time(NULL);
    uint32_t ts = (uint32_t)now;

    uint8_t payload[] = {
        0x01, 0x00, 0x00,
        (uint8_t)((ts >> 24) & 0xFF),
        (uint8_t)((ts >> 16) & 0xFF),
        (uint8_t)((ts >> 8) & 0xFF),
        (uint8_t)(ts & 0xFF),
        0x01, 0x06
    };
    lefu_send_cmd(CMD_USER_PROFILE, payload, sizeof(payload));
    state = STATE_MEASURING;
}

/* =========================================================================
 * NOTIFICATION HANDLER
 * ========================================================================= */

static void handle_notification(const uint8_t *data, uint16_t len)
{
    if (len < 6 || data[0] != LEFU_HDR_0 || data[1] != LEFU_HDR_1) {
        return;
    }

    uint8_t cmd = data[2];
    uint8_t payload_len = data[4];
    const uint8_t *payload = &data[5];

    switch (cmd) {

    case NTF_START_STOP:
        if (payload_len > 0 && payload[0] == 0x00 && last_weight_kg > 0) {
            /* Measurement stopped — last_weight_kg is the final reading */
            ESP_LOGI(TAG, ">>> FINAL WEIGHT: %.2f kg (%.1f lbs) <<<",
                     last_weight_kg, last_weight_kg * 2.20462f);
            /* TODO: POST weight via HTTP here */
            last_weight_kg = 0;
        }
        break;

    case NTF_WEIGHT:
        if (payload_len >= 5) {
            uint16_t raw = ((uint16_t)payload[3] << 8) | payload[4];
            last_weight_kg = raw / 100.0f;
        }
        break;

    default:
        break;
    }
}

/* =========================================================================
 * GATT DISCOVERY
 * ========================================================================= */

static int on_write_complete(uint16_t conn_handle_cb,
                              const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr, void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Write failed: %s (status=%d)", (const char *)arg, error->status);
    }
    return 0;
}

static void subscribe_and_start(void)
{
    uint8_t val[2] = {0x01, 0x00};
    int rc = ble_gattc_write_flat(conn_handle, notify_cccd_handle,
                                   val, sizeof(val),
                                   on_write_complete, "subscribe");
    if (rc != 0) {
        ESP_LOGE(TAG, "Subscribe failed: %d", rc);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    send_user_profile();
}

static int on_descriptor_discovered(uint16_t conn_handle_cb,
                                     const struct ble_gatt_error *error,
                                     uint16_t chr_val_handle,
                                     const struct ble_gatt_dsc *dsc, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (notify_cccd_handle == 0) {
            ESP_LOGE(TAG, "CCCD for 0x2A10 not found");
            return 0;
        }
        subscribe_and_start();
        return 0;
    }
    if (error->status != 0) return 0;

    if (ble_uuid_cmp(&dsc->uuid.u, &cccd_uuid.u) == 0 &&
        notify_cccd_handle == 0 && dsc->handle > notify_val_handle) {
        notify_cccd_handle = dsc->handle;
    }
    return 0;
}

static int on_characteristic_discovered(uint16_t conn_handle_cb,
                                         const struct ble_gatt_error *error,
                                         const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        int rc = ble_gattc_disc_all_dscs(conn_handle_cb,
                                          svc_start_handle, svc_end_handle,
                                          on_descriptor_discovered, NULL);
        if (rc != 0) ESP_LOGE(TAG, "Descriptor discovery failed: %d", rc);
        return 0;
    }
    if (error->status != 0) return 0;

    if (ble_uuid_cmp(&chr->uuid.u, &notify_uuid.u) == 0) {
        notify_val_handle = chr->val_handle;
    } else if (ble_uuid_cmp(&chr->uuid.u, &write_uuid.u) == 0) {
        write_val_handle = chr->val_handle;
    }
    return 0;
}

static int on_service_discovered(uint16_t conn_handle_cb,
                                  const struct ble_gatt_error *error,
                                  const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (svc_start_handle == 0) {
            ESP_LOGE(TAG, "Service 0x1A10 not found");
            return 0;
        }
        int rc = ble_gattc_disc_all_chrs(conn_handle_cb,
                                          svc_start_handle, svc_end_handle,
                                          on_characteristic_discovered, NULL);
        if (rc != 0) ESP_LOGE(TAG, "Characteristic discovery failed: %d", rc);
        return 0;
    }
    if (error->status != 0) return 0;

    if (ble_uuid_cmp(&svc->uuid.u, &svc_uuid.u) == 0) {
        svc_start_handle = svc->start_handle;
        svc_end_handle = svc->end_handle;
    }
    return 0;
}

/* =========================================================================
 * GAP EVENT HANDLER
 * ========================================================================= */

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data) != 0) {
            return 0;
        }

        char name_buf[32] = {0};
        if (fields.name != NULL && fields.name_len > 0) {
            int n = fields.name_len < 31 ? fields.name_len : 31;
            memcpy(name_buf, fields.name, n);
        }

        if (!strstr(name_buf, "CS20M") && !strstr(name_buf, "Renpho") &&
            !strstr(name_buf, "RENPHO")) {
            return 0;
        }

        ESP_LOGI(TAG, "Found scale: %s", name_buf);
        ble_gap_disc_cancel();

        int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                                  30000, NULL, gap_event_handler, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Connect failed: %d", rc);
            start_scan();
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        start_scan();
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "Connection failed: %d", event->connect.status);
            start_scan();
            return 0;
        }
        conn_handle = event->connect.conn_handle;
        state = STATE_CONNECTED;
        last_weight_kg = 0;

        notify_val_handle = 0;
        notify_cccd_handle = 0;
        write_val_handle = 0;
        svc_start_handle = 0;
        svc_end_handle = 0;

        ble_gattc_disc_all_svcs(conn_handle, on_service_discovered, NULL);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        state = STATE_IDLE;
        start_scan();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (event->notify_rx.attr_handle == notify_val_handle) {
            uint16_t data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
            uint8_t data[64];
            if (data_len > sizeof(data)) data_len = sizeof(data);
            os_mbuf_copydata(event->notify_rx.om, 0, data_len, data);
            handle_notification(data, data_len);
        }
        return 0;
    }

    default:
        return 0;
    }
}

/* =========================================================================
 * SCANNING & INIT
 * ========================================================================= */

static void start_scan(void)
{
    struct ble_gap_disc_params params = {0};
    params.passive = 0;
    params.filter_duplicates = 1;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                           &params, gap_event_handler, NULL);
    if (rc != 0) ESP_LOGE(TAG, "Scan failed: %d", rc);
    state = STATE_SCANNING;
}

static void on_ble_hs_sync(void)
{
    ble_hs_util_ensure_addr(0);
    start_scan();
}

static void on_ble_hs_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset: %d", reason);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_LOGI(TAG, "ES-CS20M Scale Client starting");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    ble_hs_cfg.sync_cb = on_ble_hs_sync;
    ble_hs_cfg.reset_cb = on_ble_hs_reset;

    nimble_port_freertos_init(nimble_host_task);
}
