#include "can_syscalls.h"

bool libtock_can_driver_exists(void) {
  return driver_exists(DRIVER_NUM_CAN);
}

static returncode_t set_upcall(uint32_t which, subscribe_upcall callback, void* opaque) {
  subscribe_return_t sval = subscribe(DRIVER_NUM_CAN, which, callback, opaque);
  return tock_subscribe_return_to_returncode(sval);
}

returncode_t libtock_can_set_upcall_enabled(subscribe_upcall callback, void* opaque) {
  return set_upcall(LIBTOCK_CAN_UPCALL_ENABLED, callback, opaque);
}

returncode_t libtock_can_set_upcall_disabled(subscribe_upcall callback, void* opaque) {
  return set_upcall(LIBTOCK_CAN_UPCALL_DISABLED, callback, opaque);
}

returncode_t libtock_can_set_upcall_message_sent(subscribe_upcall callback, void* opaque) {
  return set_upcall(LIBTOCK_CAN_UPCALL_MESSAGE_SENT, callback, opaque);
}

returncode_t libtock_can_set_upcall_message_received(subscribe_upcall callback, void* opaque) {
  return set_upcall(LIBTOCK_CAN_UPCALL_MESSAGE_RECEIVED, callback, opaque);
}

returncode_t libtock_can_set_upcall_stopped(subscribe_upcall callback, void* opaque) {
  return set_upcall(LIBTOCK_CAN_UPCALL_STOPPED, callback, opaque);
}

returncode_t libtock_can_set_upcall_error(subscribe_upcall callback, void* opaque) {
  return set_upcall(LIBTOCK_CAN_UPCALL_ERROR, callback, opaque);
}

returncode_t libtock_can_set_readonly_allow_tx_buffer(const uint8_t* buffer, uint32_t len) {
  allow_ro_return_t aval = allow_readonly(DRIVER_NUM_CAN, 0, (const void*) buffer, len);
  return tock_allow_ro_return_to_returncode(aval);
}

returncode_t libtock_can_set_readwrite_allow_rx_buffer(uint8_t* buffer, uint32_t len) {
  allow_rw_return_t aval = allow_readwrite(DRIVER_NUM_CAN, 0, (void*) buffer, len);
  return tock_allow_rw_return_to_returncode(aval);
}

returncode_t libtock_can_command_enable(void) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 3, 0, 0);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_disable(void) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 4, 0, 0);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_send_standard(uint32_t id, uint32_t len) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 5, id, len);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_send_extended(uint32_t id, uint32_t len) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 6, id, len);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_start_receive(void) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 7, 0, 0);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_stop_receive(void) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 8, 0, 0);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_subscribe_standard(uint32_t id, uint32_t mask) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 10, id, mask);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_subscribe_extended(uint32_t id, uint32_t mask) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 11, id, mask);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_clear_subscriptions(void) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 12, 0, 0);
  return tock_command_return_novalue_to_returncode(cval);
}

returncode_t libtock_can_command_subscription_capacity(uint32_t* capacity) {
  syscall_return_t cval = command(DRIVER_NUM_CAN, 13, 0, 0);
  return tock_command_return_u32_to_returncode(cval, capacity);
}
