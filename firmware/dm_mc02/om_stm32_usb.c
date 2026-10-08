#include "om_stm32_usb.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include <string.h>
/* The pinned usb_device.h declares init only, not the generated handle. */
extern USBD_HandleTypeDef hUsbDeviceHS;

static om_cdc transport;
static uint8_t attempted, ready, was_configured;
static const uint8_t *owned;
static size_t owned_length;

static uint32_t enter(void *context) {
    (void)context;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return key;
}
static void leave(void *context, uint32_t key) {
    (void)context;
    __DMB();
    __set_PRIMASK(key);
}
static int submit(void *context, const uint8_t *data, size_t length) {
    uint32_t key = enter(context);
    int result = -1;
    /* Use the class slot used by the pinned middleware's DataIn callback.
     * Interrupt masking also closes the disconnect/submit lifetime race. */
    USBD_CDC_HandleTypeDef *cdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassDataCmsit[hUsbDeviceHS.classId];
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED || !cdc) {
        result = 1;
    } else if (cdc->TxState) {
        result = 1;
    } else if (length <= OM_MAX_FRAME && !owned) {
        if (USBD_CDC_SetTxBuffer(&hUsbDeviceHS, (uint8_t *)data,
                                (uint32_t)length) == USBD_OK) {
            owned = data;
            owned_length = length;
            uint8_t status = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
            if (status == USBD_OK) result = 0;
            else {
                /* Fail terminally: the endpoint may have retained its pointer. */
                result = -1;
            }
        }
    }
    leave(context, key);
    return result;
}

int om_stm32_usb_init(RNG_HandleTypeDef *rng, const uint8_t digest[32],
                      const om_joint *joints) {
    /* USB and all callbacks must be stopped during this call. */
    if (attempted) return -1;
    attempted = 1;
    if (!rng || !digest || !joints ||
        hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) return -1;
    uint8_t nonce[16];
    for (unsigned i = 0; i < 4; ++i) {
        uint32_t word;
        if (HAL_RNG_GenerateRandomNumber(rng, &word) != HAL_OK) return -1;
        for (unsigned k = 0; k < 4; ++k) nonce[4*i+k] = (uint8_t)(word >> (8*k));
    }
    if (om_cdc_init(&transport, (om_cdc_io){NULL, enter, leave, submit},
                    nonce, digest, joints)) return -1;
    ready = 1;
    return 0;
}
void om_stm32_usb_receive(const uint8_t *data, size_t length) {
    if (ready) om_cdc_receive(&transport, data, length);
}
void om_stm32_usb_complete(const uint8_t *data, size_t length) {
    if (!ready) return;
    uint32_t key = enter(NULL);
    USBD_CDC_HandleTypeDef *cdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassDataCmsit[hUsbDeviceHS.classId];
    if (owned && data == owned && length == owned_length && cdc && !cdc->TxState) {
        owned = NULL;
        owned_length = 0;
        om_cdc_tx_complete(&transport);
    } else {
        om_cdc_link_fault(&transport);
    }
    leave(NULL, key);
}
void om_stm32_usb_disconnect(void) {
    if (ready) om_cdc_link_fault(&transport);
    /* Preserve buffers and session even if endpoints are subsequently reopened. */
}
void om_stm32_usb_poll(void) {
    if (!ready) return;
    uint32_t key = enter(NULL);
    uint8_t configured = hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED;
    if (configured) was_configured = 1;
    else if (was_configured) om_cdc_link_fault(&transport);
    leave(NULL, key);
    om_cdc_poll(&transport, HAL_GetTick());
}
int om_stm32_usb_faulted(void) {
    return !ready || transport.transport_fault || transport.core.fault;
}
int om_stm32_usb_status_provider(om_status_provider provider, void *context) {
    if(!ready || !provider || transport.core.status ||
       hUsbDeviceHS.dev_state==USBD_STATE_CONFIGURED) return -1;
    transport.core.status=provider;
    transport.core.status_context=context;
    return 0;
}
void om_stm32_usb_observation_fault(void) {
    if(ready) om_fault(&transport.core,9);
}
