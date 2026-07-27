#include "can.h"

#include <string.h>

#include "syscalls/can_syscalls.h"

// Header field offsets, see can.h. Version (bytes 0..2) is always zero and is
// never read back, so it has no accessor here.
#define HDR_FLAGS   2
#define HDR_OFFSET  4

#define HDR_FLAG_EXCEEDED 0x01

#define CHUNK_ID     0
#define CHUNK_LEN    4
#define CHUNK_FLAGS  5
#define CHUNK_DATA   8

#define CHUNK_FLAG_EXTENDED 0x01

// Multi-byte fields are assembled byte by byte rather than cast through a
// pointer: the buffer comes from the caller and need not be aligned, and Tock
// maps process memory such that an unaligned word access faults.
static uint32_t read_u32_le(const uint8_t* p) {
  return (uint32_t) p[0]
         | ((uint32_t) p[1] << 8)
         | ((uint32_t) p[2] << 16)
         | ((uint32_t) p[3] << 24);
}

// -- Upcall trampolines -----------------------------------------------------

static void can_enabled_upcall(int status,
                               __attribute__ ((unused)) int unused1,
                               __attribute__ ((unused)) int unused2,
                               void*                        opaque) {
  libtock_can_callback_enabled cb = (libtock_can_callback_enabled) opaque;
  if (cb != NULL) cb(tock_status_to_returncode(status));
}

static void can_disabled_upcall(int status,
                                __attribute__ ((unused)) int unused1,
                                __attribute__ ((unused)) int unused2,
                                void*                        opaque) {
  libtock_can_callback_disabled cb = (libtock_can_callback_disabled) opaque;
  if (cb != NULL) cb(tock_status_to_returncode(status));
}

static void can_sent_upcall(int status,
                            __attribute__ ((unused)) int unused1,
                            __attribute__ ((unused)) int unused2,
                            void*                        opaque) {
  libtock_can_callback_message_sent cb = (libtock_can_callback_message_sent) opaque;
  if (cb != NULL) cb(tock_status_to_returncode(status));
}

static void can_stopped_upcall(int status,
                               __attribute__ ((unused)) int unused1,
                               __attribute__ ((unused)) int unused2,
                               void*                        opaque) {
  libtock_can_callback_stopped cb = (libtock_can_callback_stopped) opaque;
  if (cb != NULL) cb(tock_status_to_returncode(status));
}

static void can_received_upcall(int status, int offset, int id, void* opaque) {
  libtock_can_callback_message_received cb = (libtock_can_callback_message_received) opaque;
  if (cb != NULL) cb(tock_status_to_returncode(status), (uint32_t) offset, (uint32_t) id);
}

static void can_error_upcall(int kind, int code,
                             __attribute__ ((unused)) int unused2,
                             void*                        opaque) {
  libtock_can_callback_error cb = (libtock_can_callback_error) opaque;
  if (cb != NULL) cb((uint32_t) kind, (uint32_t) code);
}

// -- API --------------------------------------------------------------------

bool libtock_can_exists(void) {
  return libtock_can_driver_exists();
}

returncode_t libtock_can_subscribe_id(uint32_t id, bool extended, uint32_t mask) {
  if (extended) {
    return libtock_can_command_subscribe_extended(id, mask);
  }
  return libtock_can_command_subscribe_standard(id, mask);
}

returncode_t libtock_can_clear_subscriptions(void) {
  return libtock_can_command_clear_subscriptions();
}

returncode_t libtock_can_subscription_capacity(uint32_t* capacity) {
  return libtock_can_command_subscription_capacity(capacity);
}

returncode_t libtock_can_enable(libtock_can_callback_enabled cb) {
  returncode_t ret = libtock_can_set_upcall_enabled(can_enabled_upcall, cb);
  if (ret != RETURNCODE_SUCCESS) return ret;
  return libtock_can_command_enable();
}

returncode_t libtock_can_disable(libtock_can_callback_disabled cb) {
  returncode_t ret = libtock_can_set_upcall_disabled(can_disabled_upcall, cb);
  if (ret != RETURNCODE_SUCCESS) return ret;
  return libtock_can_command_disable();
}

returncode_t libtock_can_send(uint32_t                          id,
                              bool                              extended,
                              const uint8_t*                    data,
                              uint32_t                          length,
                              libtock_can_callback_message_sent cb) {
  if (length > LIBTOCK_CAN_MAX_DLC) return RETURNCODE_ESIZE;

  returncode_t ret = libtock_can_set_readonly_allow_tx_buffer(data, length);
  if (ret != RETURNCODE_SUCCESS) return ret;

  ret = libtock_can_set_upcall_message_sent(can_sent_upcall, cb);
  if (ret != RETURNCODE_SUCCESS) return ret;

  if (extended) {
    return libtock_can_command_send_extended(id, length);
  }
  return libtock_can_command_send_standard(id, length);
}

returncode_t libtock_can_start_receive(libtock_can_rx_t*                     rx,
                                       libtock_can_callback_message_received cb) {
  if (rx == NULL || rx->buffer == NULL) return RETURNCODE_EINVAL;
  if (rx->length < LIBTOCK_CAN_RX_BUFFER_SIZE(2)) return RETURNCODE_ESIZE;

  returncode_t ret = libtock_can_rx_reset(rx);
  if (ret != RETURNCODE_SUCCESS) return ret;

  ret = libtock_can_set_upcall_message_received(can_received_upcall, cb);
  if (ret != RETURNCODE_SUCCESS) return ret;

  return libtock_can_command_start_receive();
}

returncode_t libtock_can_stop_receive(libtock_can_callback_stopped cb) {
  returncode_t ret = libtock_can_set_upcall_stopped(can_stopped_upcall, cb);
  if (ret != RETURNCODE_SUCCESS) return ret;
  return libtock_can_command_stop_receive();
}

returncode_t libtock_can_set_error_callback(libtock_can_callback_error cb) {
  return libtock_can_set_upcall_error(can_error_upcall, cb);
}

// -- Receive buffer helpers -------------------------------------------------

void libtock_can_rx_init(libtock_can_rx_t* rx, uint8_t* buffer, uint32_t length) {
  rx->buffer = buffer;
  rx->length = length;
  rx->next   = 0;
  memset(buffer, 0, length);
}

uint32_t libtock_can_rx_available(const libtock_can_rx_t* rx) {
  if (rx == NULL || rx->buffer == NULL || rx->length < LIBTOCK_CAN_RX_HEADER_SIZE) return 0;
  return read_u32_le(&rx->buffer[HDR_OFFSET]) / LIBTOCK_CAN_RX_CHUNK_SIZE;
}

bool libtock_can_rx_get(const libtock_can_rx_t* rx, uint32_t index, libtock_can_frame_t* frame) {
  if (frame == NULL || index >= libtock_can_rx_available(rx)) return false;

  uint32_t start = LIBTOCK_CAN_RX_HEADER_SIZE + index * LIBTOCK_CAN_RX_CHUNK_SIZE;
  if (start + LIBTOCK_CAN_RX_CHUNK_SIZE > rx->length) return false;

  const uint8_t* chunk = &rx->buffer[start];
  frame->id       = read_u32_le(&chunk[CHUNK_ID]);
  frame->extended = (chunk[CHUNK_FLAGS] & CHUNK_FLAG_EXTENDED) != 0;
  frame->length   = chunk[CHUNK_LEN] > LIBTOCK_CAN_MAX_DLC ? LIBTOCK_CAN_MAX_DLC : chunk[CHUNK_LEN];
  memcpy(frame->data, &chunk[CHUNK_DATA], LIBTOCK_CAN_MAX_DLC);
  return true;
}

bool libtock_can_rx_next(libtock_can_rx_t* rx, libtock_can_frame_t* frame) {
  if (!libtock_can_rx_get(rx, rx->next, frame)) return false;
  rx->next++;
  return true;
}

bool libtock_can_rx_overflowed(const libtock_can_rx_t* rx) {
  if (rx == NULL || rx->buffer == NULL || rx->length < LIBTOCK_CAN_RX_HEADER_SIZE) return false;
  // Flags are big endian, so the low bits are in the second byte.
  return (rx->buffer[HDR_FLAGS + 1] & HDR_FLAG_EXCEEDED) != 0;
}

returncode_t libtock_can_rx_reset(libtock_can_rx_t* rx) {
  if (rx == NULL || rx->buffer == NULL) return RETURNCODE_EINVAL;
  memset(rx->buffer, 0, rx->length);
  rx->next = 0;
  // Re-allow so the kernel picks up the cleared header and offset.
  return libtock_can_set_readwrite_allow_rx_buffer(rx->buffer, rx->length);
}
