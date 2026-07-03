/* See ota.h. */

#include "ota.h"

#include <stdlib.h>

#include "esp_log.h"
#include "esp_ota_ops.h"

static const char *TAG = "ota";

struct idl0_ota_session {
    esp_ota_handle_t handle;
    const esp_partition_t *target;
    size_t written;
    bool aborted;
};

static bool s_pending_verify = false;

void idl0_ota_init(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        ESP_LOGW(TAG, "no running partition (factory?)");
        return;
    }
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_err_t err = esp_ota_get_state_partition(running, &state);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_ota_get_state_partition: %s", esp_err_to_name(err));
        return;
    }
    s_pending_verify = (state == ESP_OTA_IMG_PENDING_VERIFY);
    ESP_LOGI(TAG, "running partition '%s', state=%d, pending_verify=%d",
             running->label, (int)state, (int)s_pending_verify);
}

bool idl0_ota_pending_verify(void)
{
    return s_pending_verify;
}

idl0_ota_session_t *idl0_ota_begin(size_t expected_size)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        ESP_LOGE(TAG, "no next OTA partition");
        return NULL;
    }
    ESP_LOGI(TAG, "OTA target '%s' subtype=0x%02x size=%lu bytes",
             target->label, (unsigned)target->subtype,
             (unsigned long)target->size);

    idl0_ota_session_t *s = calloc(1, sizeof(*s));
    if (s == NULL) {
        return NULL;
    }
    s->target = target;
    size_t image_size = expected_size > 0 ? expected_size : OTA_WITH_SEQUENTIAL_WRITES;
    esp_err_t err = esp_ota_begin(target, image_size, &s->handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        free(s);
        return NULL;
    }
    return s;
}

bool idl0_ota_write(idl0_ota_session_t *s, const void *data, size_t len)
{
    if (s == NULL || s->aborted) return false;
    esp_err_t err = esp_ota_write(s->handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
        s->aborted = true;
        return false;
    }
    s->written += len;
    return true;
}

bool idl0_ota_end(idl0_ota_session_t *s)
{
    if (s == NULL) return false;
    if (s->aborted) {
        esp_ota_abort(s->handle);
        free(s);
        return false;
    }
    esp_err_t err = esp_ota_end(s->handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "image validation failed (SHA-256 mismatch)");
        } else {
            ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        }
        free(s);
        return false;
    }
    err = esp_ota_set_boot_partition(s->target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        free(s);
        return false;
    }
    ESP_LOGI(TAG, "OTA wrote %lu bytes; boot set to '%s'",
             (unsigned long)s->written, s->target->label);
    free(s);
    return true;
}

void idl0_ota_abort(idl0_ota_session_t *s)
{
    if (s == NULL) return;
    esp_ota_abort(s->handle);
    free(s);
}

bool idl0_ota_mark_valid(void)
{
    if (!s_pending_verify) {
        return false;
    }
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_mark_app_valid_cancel_rollback: %s",
                 esp_err_to_name(err));
        return false;
    }
    s_pending_verify = false;
    ESP_LOGI(TAG, "OTA image marked valid");
    return true;
}
