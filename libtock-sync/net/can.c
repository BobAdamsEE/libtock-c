#include "can.h"

#include <libtock-sync/services/alarm.h>
#include <libtock/net/syscalls/can_syscalls.h>

struct can_result {
  bool fired;
  returncode_t ret;
};

static struct can_result enable_result   = {.fired = false};
static struct can_result disable_result  = {.fired = false};
static struct can_result send_result     = {.fired = false};
static struct can_result stop_result     = {.fired = false};

// Set by the receive upcall; cleared by the reader once it has looked.
static bool rx_notified = false;

static void enable_cb(returncode_t ret) {
  enable_result.fired = true;
  enable_result.ret   = ret;
}

static void disable_cb(returncode_t ret) {
  disable_result.fired = true;
  disable_result.ret   = ret;
}

static void send_cb(returncode_t ret) {
  send_result.fired = true;
  send_result.ret   = ret;
}

static void stop_cb(returncode_t ret) {
  stop_result.fired = true;
  stop_result.ret   = ret;
}

static void received_cb(returncode_t ret __attribute__ ((unused)),
                        uint32_t offset  __attribute__ ((unused)),
                        uint32_t id      __attribute__ ((unused))) {
  rx_notified = true;
}

bool libtocksync_can_exists(void) {
  return libtock_can_driver_exists();
}

returncode_t libtocksync_can_subscribe_id(uint32_t id, bool extended, uint32_t mask) {
  return libtock_can_subscribe_id(id, extended, mask);
}

returncode_t libtocksync_can_clear_subscriptions(void) {
  return libtock_can_clear_subscriptions();
}

returncode_t libtocksync_can_enable(void) {
  enable_result.fired = false;

  returncode_t ret = libtock_can_enable(enable_cb);
  if (ret != RETURNCODE_SUCCESS) return ret;

  yield_for(&enable_result.fired);
  return enable_result.ret;
}

returncode_t libtocksync_can_disable(void) {
  disable_result.fired = false;

  returncode_t ret = libtock_can_disable(disable_cb);
  if (ret != RETURNCODE_SUCCESS) return ret;

  yield_for(&disable_result.fired);
  return disable_result.ret;
}

returncode_t libtocksync_can_send(uint32_t       id,
                                  bool           extended,
                                  const uint8_t* data,
                                  uint32_t       length,
                                  uint32_t       timeout_ms) {
  send_result.fired = false;

  returncode_t ret = libtock_can_send(id, extended, data, length, send_cb);
  if (ret != RETURNCODE_SUCCESS) return ret;

  ret = libtocksync_alarm_yield_for_with_timeout(&send_result.fired, timeout_ms);
  if (ret != RETURNCODE_SUCCESS) return ret;

  return send_result.ret;
}

returncode_t libtocksync_can_start_receive(libtock_can_rx_t* rx) {
  rx_notified = false;
  return libtock_can_start_receive(rx, received_cb);
}

returncode_t libtocksync_can_stop_receive(void) {
  stop_result.fired = false;

  returncode_t ret = libtock_can_stop_receive(stop_cb);
  if (ret != RETURNCODE_SUCCESS) return ret;

  yield_for(&stop_result.fired);
  return stop_result.ret;
}

returncode_t libtocksync_can_read_frame(libtock_can_rx_t*    rx,
                                        libtock_can_frame_t* frame,
                                        uint32_t             timeout_ms) {
  if (rx == NULL || frame == NULL) return RETURNCODE_EINVAL;

  // Anything already buffered is returned without yielding. Several frames can
  // be appended between schedulings, and the kernel coalesces their upcalls
  // into one, so waiting for a fresh upcall per frame would strand the rest.
  if (libtock_can_rx_next(rx, frame)) return RETURNCODE_SUCCESS;

  rx_notified = false;
  returncode_t ret = libtocksync_alarm_yield_for_with_timeout(&rx_notified, timeout_ms);
  if (ret != RETURNCODE_SUCCESS) return ret;

  if (libtock_can_rx_next(rx, frame)) return RETURNCODE_SUCCESS;
  return RETURNCODE_FAIL;
}
