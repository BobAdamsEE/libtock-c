#pragma once

#include "../../tock.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVER_NUM_CAN 0x20007

// Upcall numbers.
#define LIBTOCK_CAN_UPCALL_ENABLED           0
#define LIBTOCK_CAN_UPCALL_DISABLED          1
#define LIBTOCK_CAN_UPCALL_MESSAGE_SENT      2
#define LIBTOCK_CAN_UPCALL_MESSAGE_RECEIVED  3
#define LIBTOCK_CAN_UPCALL_STOPPED           4
#define LIBTOCK_CAN_UPCALL_ERROR             5

// Error kinds reported through the error upcall.
#define LIBTOCK_CAN_ERROR_TX 100
#define LIBTOCK_CAN_ERROR_RX 101

bool libtock_can_driver_exists(void);

returncode_t libtock_can_set_upcall_enabled(subscribe_upcall callback, void* opaque);
returncode_t libtock_can_set_upcall_disabled(subscribe_upcall callback, void* opaque);
returncode_t libtock_can_set_upcall_message_sent(subscribe_upcall callback, void* opaque);
returncode_t libtock_can_set_upcall_message_received(subscribe_upcall callback, void* opaque);
returncode_t libtock_can_set_upcall_stopped(subscribe_upcall callback, void* opaque);
returncode_t libtock_can_set_upcall_error(subscribe_upcall callback, void* opaque);

returncode_t libtock_can_set_readonly_allow_tx_buffer(const uint8_t* buffer, uint32_t len);
returncode_t libtock_can_set_readwrite_allow_rx_buffer(uint8_t* buffer, uint32_t len);

returncode_t libtock_can_command_enable(void);
returncode_t libtock_can_command_disable(void);
returncode_t libtock_can_command_send_standard(uint32_t id, uint32_t len);
returncode_t libtock_can_command_send_extended(uint32_t id, uint32_t len);
returncode_t libtock_can_command_start_receive(void);
returncode_t libtock_can_command_stop_receive(void);
returncode_t libtock_can_command_subscribe_standard(uint32_t id, uint32_t mask);
returncode_t libtock_can_command_subscribe_extended(uint32_t id, uint32_t mask);
returncode_t libtock_can_command_clear_subscriptions(void);
returncode_t libtock_can_command_subscription_capacity(uint32_t* capacity);

#ifdef __cplusplus
}
#endif
