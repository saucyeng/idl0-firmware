#include "mode_state.h"

static EventGroupHandle_t s_group = NULL;

EventGroupHandle_t idl0_mode_event_group(void)
{
    if (s_group == NULL) {
        s_group = xEventGroupCreate();
    }
    return s_group;
}

void idl0_mode_set_bits(EventBits_t bits)
{
    xEventGroupSetBits(idl0_mode_event_group(), bits);
}

void idl0_mode_clear_bits(EventBits_t bits)
{
    xEventGroupClearBits(idl0_mode_event_group(), bits);
}

EventBits_t idl0_mode_get_bits(void)
{
    return xEventGroupGetBits(idl0_mode_event_group());
}
