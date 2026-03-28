/*
 * Renpho ES-CS20M BLE Scale Client for ESP32 (ESP-IDF + NimBLE)
 *
 * Uses the Lefu/0x1A10 protocol:
 *   Service 0x1A10:
 *     0x2A10 - Notify: scale sends weight data and ACKs here
 *     0x2A11 - Write:  we send commands here
 *
 * Handshake:
 *   1. Subscribe to notifications on 0x2A10
 *   2. Write user profile command (0x97) to 0x2A11
 *   3. Wait for 0x17 ACK on 0x2A10
 *   4. Write config command (0x96) to 0x2A11
 *   5. Wait for 0x16 ACK on 0x2A10
 *   6. Write start-measurement command (0x90) to 0x2A11
 *   7. Receive weight frames (0x14) and BIA frames (0x18/0x19) on 0x2A10
 *
 * Frame format: 55 AA [cmd] 00 [len] [payload...] [checksum]
 * Checksum = sum of all preceding bytes mod 256.
 */

#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "QN_SCALE";

/* =========================================================================
 * LEFU PROTOCOL CONSTANTS
 * ========================================================================= */

/* Frame header */
#define LEFU_HDR_0  0x55
#define LEFU_HDR_1  0xAA

/* Command opcodes (written to 0x2A11) */
#define CMD_USER_PROFILE  0x97
#define CMD_CONFIG        0x96
#define CMD_START_MEAS    0x90
#define CMD_INTERIM_ACK   0x99

/* Notification opcodes (received on 0x2A10) */
#define NTF_START_STOP    0x11
#define NTF_WEIGHT        0x14
#define NTF_HISTORY_ACK   0x16
#define NTF_PROFILE_ACK   0x17
#define NTF_FINAL_BIA     0x18
#define NTF_INTERIM_BIA   0x19

/* =========================================================================
 * STATE
 * ========================================================================= */

typedef enum {
    STATE_IDLE,
    STATE_SCANNING,
    STATE_CONNECTED,
    STATE_DISC_COMPLETE,
    STATE_SUBSCRIBED,
    STATE_WAIT_PROFILE_ACK,   /* sent 0x97, waiting for 0x17 */
    STATE_WAIT_CONFIG_ACK,    /* sent 0x96, waiting for 0x16 */
    STATE_MEASURING,          /* sent 0x90, receiving weight data */
} scale_state_t;

static scale_state_t state = STATE_IDLE;
static uint16_t conn_handle;

/* Characteristic handles for the 0x1A10 service */
static uint16_t notify_val_handle;   /* 0x2A10 - notifications from scale */
static uint16_t notify_cccd_handle;  /* CCCD for 0x2A10 */
static uint16_t write_val_handle;    /* 0x2A11 - commands to scale */

/* Service handle range */
static uint16_t svc_start_handle;
static uint16_t svc_end_handle;

/* UUIDs */
static const ble_uuid16_t svc_uuid    = BLE_UUID16_INIT(0x1A10);
static const ble_uuid16_t notify_uuid = BLE_UUID16_INIT(0x2A10);
static const ble_uuid16_t write_uuid  = BLE_UUID16_INIT(0x2A11);
static const ble_uuid16_t cccd_uuid   = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);

/* Forward declarations */
static int gap_event_handler(struct ble_gap_event *event, void *arg);
static void start_scan(void);

/* =========================================================================
 * LEFU FRAME BUILDER
 * ========================================================================= */

/* Build a Lefu frame: 55 AA [cmd] 00 [len] [payload...] [checksum]
 * Returns total frame length. buf must be large enough. */
static int lefu_build_frame(uint8_t *buf, uint8_t cmd,
                             const uint8_t *payload, uint8_t payload_len)
{
    buf[0] = LEFU_HDR_0;
    buf[1] = LEFU_HDR_1;
    buf[2] = cmd;
    buf[3] = 0x00;
    buf[4] = payload_len;
    if (payload_len > 0) {
        memcpy(&buf[5], payload, payload_len);
    }

    /* Checksum = sum of all preceding bytes mod 256 */
    uint8_t sum = 0;
    int total = 5 + payload_len;
    for (int i = 0; i < total; i++) {
        sum += buf[i];
    }
    buf[total] = sum;
    return total + 1;
}

/* Send a Lefu command to the write characteristic */
static void lefu_send_cmd(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[64];
    int len = lefu_build_frame(frame, cmd, payload, payload_len);

    char hex[128] = {0};
    for (int i = 0; i < len && i < 40; i++) {
        sprintf(hex + (i * 3), "%02X ", frame[i]);
    }
    ESP_LOGI(TAG, "TX cmd=0x%02X [%d bytes]: %s", cmd, len, hex);

    int rc = ble_gattc_write_no_rsp_flat(conn_handle, write_val_handle,
                                          frame, len);
    if (rc != 0) {
        ESP_LOGE(TAG, "Write failed: %d", rc);
    }
}

/* =========================================================================
 * HANDSHAKE COMMANDS
 * ========================================================================= */

/* Step 1: Send user profile (cmd 0x97) */
static void send_user_profile(void)
{
    time_t now = time(NULL);
    uint32_t ts = (uint32_t)now;

    uint8_t payload[] = {
        0x01, 0x00, 0x00,
        (uint8_t)((ts >> 24) & 0xFF),  /* timestamp big-endian */
        (uint8_t)((ts >> 16) & 0xFF),
        (uint8_t)((ts >> 8) & 0xFF),
        (uint8_t)(ts & 0xFF),
        0x01, 0x06
    };
    ESP_LOGI(TAG, "Sending user profile (0x97)");
    lefu_send_cmd(CMD_USER_PROFILE, payload, sizeof(payload));
    state = STATE_WAIT_PROFILE_ACK;
}

/* Step 2: Send config (cmd 0x96) after receiving 0x17 ACK */
static void send_config(void)
{
    /* Default user: male, born 1990-01-01, 170cm, 75kg, standard mode */
    uint16_t year = 1990;
    uint16_t height_mm = 1700;  /* 170cm in mm... well, openScale uses mm */
    uint16_t weight = 7500;     /* 75.00 kg * 100 */

    uint8_t payload[] = {
        0x11,                            /* sex: 0x11=male, 0x21=female */
        (uint8_t)((year >> 8) & 0xFF),   /* year BE */
        (uint8_t)(year & 0xFF),
        0x01,                            /* month */
        0x01,                            /* day */
        (uint8_t)((height_mm >> 8) & 0xFF),  /* height BE */
        (uint8_t)(height_mm & 0xFF),
        0x00, 0x00,                      /* reserved */
        (uint8_t)((weight >> 8) & 0xFF), /* weight BE */
        (uint8_t)(weight & 0xFF),
        0xAA,                            /* mode: 0xAA=standard, 0x6A=athlete */
        0x01, 0x05
    };
    ESP_LOGI(TAG, "Sending config (0x96)");
    lefu_send_cmd(CMD_CONFIG, payload, sizeof(payload));
    state = STATE_WAIT_CONFIG_ACK;
}

/* Step 3: Send start measurement (cmd 0x90) after receiving 0x16 ACK */
static void send_start_measurement(void)
{
    uint8_t payload[] = { 0x01, 0x00, 0x01, 0x00 };  /* byte[2]=0x01 enables BIA */
    ESP_LOGI(TAG, "Sending start measurement (0x90)");
    lefu_send_cmd(CMD_START_MEAS, payload, sizeof(payload));
    state = STATE_MEASURING;
}

/* =========================================================================
 * NOTIFICATION HANDLER
 * ========================================================================= */

static void handle_notification(const uint8_t *data, uint16_t len)
{
    char hex[128] = {0};
    for (int i = 0; i < len && i < 40; i++) {
        sprintf(hex + (i * 3), "%02X ", data[i]);
    }
    ESP_LOGI(TAG, "RX [%d bytes]: %s", len, hex);

    /* Lefu frames: 55 AA [cmd] 00 [len] [payload...] [checksum]
     * Minimum frame is 6 bytes (header + cmd + 00 + len=0 + checksum) */
    if (len < 6 || data[0] != LEFU_HDR_0 || data[1] != LEFU_HDR_1) {
        ESP_LOGI(TAG, "Not a Lefu frame, ignoring");
        return;
    }

    uint8_t cmd = data[2];
    uint8_t payload_len = data[4];
    const uint8_t *payload = &data[5];

    switch (cmd) {

    case NTF_PROFILE_ACK:  /* 0x17 */
        ESP_LOGI(TAG, "Profile ACK received (0x17)");
        if (state == STATE_WAIT_PROFILE_ACK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            send_config();
        }
        break;

    case NTF_HISTORY_ACK:  /* 0x16 */
        ESP_LOGI(TAG, "Config ACK received (0x16)");
        if (state == STATE_WAIT_CONFIG_ACK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            send_start_measurement();
        }
        break;

    case NTF_START_STOP:  /* 0x11 */
        if (payload_len > 0 && payload[0] == 0x01) {
            ESP_LOGI(TAG, "Scale says: measurement started");
        } else {
            ESP_LOGI(TAG, "Scale says: measurement stopped");
        }
        break;

    case NTF_WEIGHT: {  /* 0x14 */
        /* Weight frame: payload bytes 3-4 = weight uint16 BE / 100 = kg */
        if (payload_len >= 5) {
            uint16_t raw = ((uint16_t)payload[3] << 8) | payload[4];
            float weight_kg = raw / 100.0f;
            float weight_lbs = weight_kg * 2.20462f;
            bool stable = (payload_len >= 6 && payload[5] != 0x00);

            if (stable) {
                ESP_LOGI(TAG, ">>> FINAL WEIGHT: %.2f kg (%.1f lbs) <<<",
                         weight_kg, weight_lbs);
            } else {
                ESP_LOGI(TAG, "settling... %.2f kg (%.1f lbs)",
                         weight_kg, weight_lbs);
            }
        }
        break;
    }

    case NTF_FINAL_BIA: {  /* 0x18 */
        ESP_LOGI(TAG, "Final BIA data received");
        if (payload_len >= 10) {
            uint16_t raw_w = ((uint16_t)payload[5] << 8) | payload[6];
            uint16_t r1 = ((uint16_t)payload[7] << 8) | payload[8];
            uint16_t r2 = ((uint16_t)payload[9] << 8) | payload[10];
            float weight_kg = raw_w / 100.0f;
            ESP_LOGI(TAG, "  weight=%.2f kg, r1=%d ohm, r2=%d ohm",
                     weight_kg, r1, r2);
        }
        break;
    }

    case NTF_INTERIM_BIA:  /* 0x19 */
        ESP_LOGI(TAG, "Interim BIA data received");
        /* Send interim ACK (0x99) */
        {
            uint8_t ack_payload[] = { 0x01 };
            lefu_send_cmd(CMD_INTERIM_ACK, ack_payload, sizeof(ack_payload));
        }
        break;

    default:
        ESP_LOGI(TAG, "Unhandled Lefu cmd: 0x%02X", cmd);
        break;
    }
}

/* =========================================================================
 * GATT DISCOVERY
 * ========================================================================= */

static int on_write_complete(uint16_t conn_handle_cb,
                              const struct ble_gatt_error *error,
                              struct ble_gatt_attr *attr,
                              void *arg)
{
    const char *desc = (const char *)arg;
    if (error->status == 0) {
        ESP_LOGI(TAG, "Write OK: %s", desc);
    } else {
        ESP_LOGE(TAG, "Write FAILED: %s (status=%d)", desc, error->status);
    }
    return 0;
}

static void subscribe_and_start(void)
{
    ESP_LOGI(TAG, "Subscribing to 0x2A10 notifications (CCCD handle=%d)",
             notify_cccd_handle);

    uint8_t val[2] = {0x01, 0x00};  /* enable notifications */
    int rc = ble_gattc_write_flat(conn_handle, notify_cccd_handle,
                                   val, sizeof(val),
                                   on_write_complete, "subscribe 0x2A10");
    if (rc != 0) {
        ESP_LOGE(TAG, "Subscribe failed: %d", rc);
        return;
    }

    /* Give the scale a moment, then start the handshake */
    vTaskDelay(pdMS_TO_TICKS(300));
    send_user_profile();
}

/* Descriptor discovery — find the CCCD for 0x2A10 */
static int on_descriptor_discovered(uint16_t conn_handle_cb,
                                     const struct ble_gatt_error *error,
                                     uint16_t chr_val_handle,
                                     const struct ble_gatt_dsc *dsc,
                                     void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Descriptor discovery complete");
        ESP_LOGI(TAG, "  notify_val=%d cccd=%d write_val=%d",
                 notify_val_handle, notify_cccd_handle, write_val_handle);

        if (notify_cccd_handle == 0) {
            ESP_LOGE(TAG, "Could not find CCCD for 0x2A10!");
            return 0;
        }
        state = STATE_DISC_COMPLETE;
        subscribe_and_start();
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "Descriptor discovery error: %d", error->status);
        return 0;
    }

    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&dsc->uuid.u, uuid_str);
    ESP_LOGI(TAG, "  descriptor: %s at handle %d", uuid_str, dsc->handle);

    if (ble_uuid_cmp(&dsc->uuid.u, &cccd_uuid.u) == 0) {
        /* The CCCD immediately after 0x2A10 (notify_val_handle) belongs to it */
        if (notify_cccd_handle == 0 && dsc->handle > notify_val_handle) {
            notify_cccd_handle = dsc->handle;
            ESP_LOGI(TAG, "Found 0x2A10 CCCD at handle %d", dsc->handle);
        }
    }
    return 0;
}

/* Characteristic discovery */
static int on_characteristic_discovered(uint16_t conn_handle_cb,
                                         const struct ble_gatt_error *error,
                                         const struct ble_gatt_chr *chr,
                                         void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Characteristic discovery complete, finding descriptors...");
        int rc = ble_gattc_disc_all_dscs(conn_handle_cb,
                                          svc_start_handle, svc_end_handle,
                                          on_descriptor_discovered, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Descriptor discovery failed to start: %d", rc);
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "Characteristic discovery error: %d", error->status);
        return 0;
    }

    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid_str);

    if (ble_uuid_cmp(&chr->uuid.u, &notify_uuid.u) == 0) {
        notify_val_handle = chr->val_handle;
        ESP_LOGI(TAG, "Found 0x2A10 (notify) at handle %d", chr->val_handle);
    } else if (ble_uuid_cmp(&chr->uuid.u, &write_uuid.u) == 0) {
        write_val_handle = chr->val_handle;
        ESP_LOGI(TAG, "Found 0x2A11 (write) at handle %d", chr->val_handle);
    } else {
        ESP_LOGI(TAG, "Found characteristic %s at handle %d", uuid_str, chr->val_handle);
    }

    return 0;
}

/* Service discovery */
static int on_service_discovered(uint16_t conn_handle_cb,
                                  const struct ble_gatt_error *error,
                                  const struct ble_gatt_svc *svc,
                                  void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (svc_start_handle == 0) {
            ESP_LOGE(TAG, "Service 0x1A10 not found!");
            return 0;
        }
        ESP_LOGI(TAG, "Discovering characteristics [%d-%d]...",
                 svc_start_handle, svc_end_handle);
        int rc = ble_gattc_disc_all_chrs(conn_handle_cb,
                                          svc_start_handle, svc_end_handle,
                                          on_characteristic_discovered, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Characteristic discovery failed: %d", rc);
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
        return 0;
    }

    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&svc->uuid.u, uuid_str);
    ESP_LOGI(TAG, "Discovered service: %s [handles %d-%d]",
             uuid_str, svc->start_handle, svc->end_handle);

    if (ble_uuid_cmp(&svc->uuid.u, &svc_uuid.u) == 0) {
        svc_start_handle = svc->start_handle;
        svc_end_handle = svc->end_handle;
        ESP_LOGI(TAG, "  ^^^ Target service 0x1A10!");
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
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                          event->disc.length_data);
        if (rc != 0) return 0;

        const uint8_t *addr = event->disc.addr.val;
        char name_buf[32] = {0};
        if (fields.name != NULL && fields.name_len > 0) {
            int copy_len = fields.name_len < 31 ? fields.name_len : 31;
            memcpy(name_buf, fields.name, copy_len);
        }

        /* Only log devices with names to reduce noise */
        if (name_buf[0] != '\0') {
            ESP_LOGI(TAG, "Scan: \"%s\" [%02X:%02X:%02X:%02X:%02X:%02X] rssi=%d",
                     name_buf,
                     addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
                     event->disc.rssi);
        }

        /* Match by name */
        bool match = false;
        if (strstr(name_buf, "CS20M") || strstr(name_buf, "Renpho") ||
            strstr(name_buf, "RENPHO") || strstr(name_buf, "QN-Scale")) {
            match = true;
        }
        if (!match) return 0;

        ESP_LOGI(TAG, "*** Found scale! Connecting... ***");
        ble_gap_disc_cancel();

        rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                              30000, NULL, gap_event_handler, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Connect failed: %d", rc);
            start_scan();
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan complete, restarting...");
        start_scan();
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "Connection failed: %d", event->connect.status);
            start_scan();
            return 0;
        }
        conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connected! conn_handle=%d", conn_handle);
        state = STATE_CONNECTED;

        /* Reset handles */
        notify_val_handle = 0;
        notify_cccd_handle = 0;
        write_val_handle = 0;
        svc_start_handle = 0;
        svc_end_handle = 0;

        ESP_LOGI(TAG, "Starting service discovery...");
        int rc = ble_gattc_disc_all_svcs(conn_handle, on_service_discovered, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Service discovery failed: %d", rc);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected: reason=%d", event->disconnect.reason);
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
        } else {
            ESP_LOGI(TAG, "Notification from handle %d",
                     event->notify_rx.attr_handle);
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU: %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

/* =========================================================================
 * SCANNING
 * ========================================================================= */

static void start_scan(void)
{
    struct ble_gap_disc_params scan_params = {0};
    scan_params.passive = 0;           /* active scan to get names */
    scan_params.filter_duplicates = 1;
    scan_params.itvl = 0;
    scan_params.window = 0;

    ESP_LOGI(TAG, "Starting BLE scan...");
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                           &scan_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Scan failed: %d", rc);
    }
    state = STATE_SCANNING;
}

/* =========================================================================
 * NimBLE HOST CALLBACKS
 * ========================================================================= */

static void on_ble_hs_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE address error: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE ready");
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

/* =========================================================================
 * ENTRY POINT
 * ========================================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "ES-CS20M Scale Client starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init() failed: %d", ret);
        return;
    }

    ble_hs_cfg.sync_cb = on_ble_hs_sync;
    ble_hs_cfg.reset_cb = on_ble_hs_reset;

    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "Init complete. Step on the scale to begin.");
}
