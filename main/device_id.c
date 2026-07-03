#include "device_id.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"

static const char *TAG = "devid";

/* "IDL0-" + 4 hex chars + NUL = 10 bytes. */
static char s_device_name[10] = "IDL0-0000";
static uint8_t s_device_id[6] = {0};

void idl0_device_id_init(void)
{
    uint8_t mac[6] = {0};
    esp_err_t err = esp_efuse_mac_get_default(mac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_efuse_mac_get_default failed (%s); using fallback name",
                 esp_err_to_name(err));
        return;  /* keep the "IDL0-0000" default */
    }

    snprintf(s_device_name, sizeof(s_device_name), "IDL0-%02X%02X", mac[4], mac[5]);
    memcpy(s_device_id, mac, sizeof(s_device_id));
    ESP_LOGI(TAG, "device name: %s (MAC %02X:%02X:%02X:%02X:%02X:%02X)",
             s_device_name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

const char *idl0_device_name(void)
{
    return s_device_name;
}

const uint8_t *idl0_device_id_bytes(void)
{
    return s_device_id;
}
