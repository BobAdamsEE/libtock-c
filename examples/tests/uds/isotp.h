/* ISO-TP (ISO 15765-2) for the runtime UDS server.
 *
 * The bootloader has a Rust implementation of the same transport
 * (`bootloader/src/isotp.rs`). This one is separate on purpose: it sits on the
 * CAN *syscall* driver rather than the HIL, and userspace gives us blocking
 * reads, so a callback state machine buys nothing here. The wire behaviour is
 * the same and the two are tested against the same host.
 *
 *   single frame      0x0L data...        L <= 7
 *   first frame       0x1H LL data...     length = HLL, 12 bits
 *   consecutive frame 0x2S data...        S = sequence number, wraps 0..15
 *   flow control      0x3F BS STmin       F = 0 clear-to-send, 1 wait, 2 overflow
 *
 * Scope matches how the server actually behaves: one connection, half duplex,
 * physical addressing only, classic CAN.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <libtock/net/can.h>
#include <libtock/tock.h>

// Largest message either direction. The server's longest request is a
// WriteDataByIdentifier and its longest response an identification string, so
// this is generous; it bounds the receive buffer below and is what a first
// frame is checked against before we grant flow control.
#define ISOTP_MAX_MESSAGE 128

// Frames each receive buffer holds. A 128-byte message is 1 first frame plus
// 18 consecutive frames, so this leaves room for a burst to land while the
// process is descheduled.
#define ISOTP_RX_FRAMES 32

// Frames that can be carried over across a buffer swap. See `isotp.c`; in
// practice this is nought or one.
#define ISOTP_CARRIED 8

typedef struct {
  uint32_t rx_id;  // identifier requests and flow control arrive on
  uint32_t tx_id;  // identifier responses and flow control go out on

  // Which of the two receive buffers the kernel is currently appending to.
  uint8_t active;

  // Frames rescued from the other buffer during a swap, oldest first.
  libtock_can_frame_t carried[ISOTP_CARRIED];
  uint8_t carried_head;
  uint8_t carried_count;

  // Set if the kernel ever ran out of room and dropped a frame. Sticky for the
  // life of the process: it answers "has this transport lost anything", which
  // a self-test wants to know even after the buffer recovered.
  bool overflowed;

  // Where the last failed transfer gave up, and what the driver said. A
  // segmented transfer has several ways to fail that all surface as one error
  // code at the call site, and on a bus they are hard to tell apart after the
  // fact.
  uint8_t stage;
  returncode_t last_error;
} isotp_t;

#define ISOTP_STAGE_NONE             0
#define ISOTP_STAGE_SEND_SINGLE      1
#define ISOTP_STAGE_SEND_FIRST       2
#define ISOTP_STAGE_NO_FLOW_CONTROL  3
#define ISOTP_STAGE_FLOW_REFUSED     4
#define ISOTP_STAGE_SEND_CONSECUTIVE 5

const char* isotp_stage_name(uint8_t stage);

// Set up the transport and start receiving. Owns its receive buffers.
returncode_t isotp_init(isotp_t* tp, uint32_t rx_id, uint32_t tx_id);

// Wait up to `timeout_ms` for one complete message.
//
// Returns RETURNCODE_SUCCESS with `*len` set, RETURNCODE_FAIL if nothing
// arrived in time, or RETURNCODE_ESIZE if the sender announced more than
// `capacity` -- in which case flow control has already refused it.
returncode_t isotp_recv(isotp_t* tp, uint8_t* msg, size_t capacity, size_t* len,
                        uint32_t timeout_ms);

// Send `len` bytes as one message, segmenting and honouring the peer's flow
// control.
returncode_t isotp_send(isotp_t* tp, const uint8_t* msg, size_t len);
