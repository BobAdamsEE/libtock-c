#pragma once

#include "../tock.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum payload of a classic CAN frame.
#define LIBTOCK_CAN_MAX_DLC 8

// Receive buffer layout.
//
// The kernel fills the buffer as a "streaming process slice": an 8-byte header
// followed by one fixed-size chunk per received frame.
//
//   header  [0..2)  version, must be 0
//           [2..4)  flags, big endian; bit 0 = exceeded, bit 1 = halt
//           [4..8)  offset: bytes of chunk data written so far
//   chunk   [0..4)  identifier, little endian
//           [4]     data length
//           [5]     flags, bit 0 = extended (29-bit) identifier
//           [6..8)  reserved
//           [8..16) data
//
// The identifier lives in the chunk rather than only in the upcall argument,
// so a burst of frames that arrives before the process is scheduled can still
// be demultiplexed.
#define LIBTOCK_CAN_RX_HEADER_SIZE 8
#define LIBTOCK_CAN_RX_CHUNK_SIZE  16

// Bytes needed to hold `frames` received frames.
#define LIBTOCK_CAN_RX_BUFFER_SIZE(frames) \
  (LIBTOCK_CAN_RX_HEADER_SIZE + (frames) * LIBTOCK_CAN_RX_CHUNK_SIZE)

typedef struct {
  uint32_t id;
  bool     extended;
  uint8_t  length;
  uint8_t  data[LIBTOCK_CAN_MAX_DLC];
} libtock_can_frame_t;

// A receive buffer plus the caller's read position within it.
//
// The kernel only ever appends; `next` records how many frames this process
// has already consumed, so frames are not re-read after a partial drain.
typedef struct {
  uint8_t* buffer;
  uint32_t length;
  uint32_t next;
} libtock_can_rx_t;

// Callback signatures.
typedef void (*libtock_can_callback_enabled)(returncode_t);
typedef void (*libtock_can_callback_disabled)(returncode_t);
typedef void (*libtock_can_callback_message_sent)(returncode_t);
typedef void (*libtock_can_callback_stopped)(returncode_t);

// - `offset`: total bytes of chunk data now in the buffer.
// - `id`: identifier of the frame that triggered this upcall. With several
//   frames pending this is only the most recent one; read identifiers from the
//   chunks instead.
typedef void (*libtock_can_callback_message_received)(returncode_t, uint32_t offset, uint32_t id);

// - `kind`: LIBTOCK_CAN_ERROR_TX or LIBTOCK_CAN_ERROR_RX.
typedef void (*libtock_can_callback_error)(uint32_t kind, uint32_t code);

bool libtock_can_exists(void);

// -- Configuration ----------------------------------------------------------

// Register interest in an identifier range. A frame is delivered when
// `(received & mask) == (id & mask)`. An all-ones mask selects one identifier.
//
// Nothing is received until at least one subscription exists.
returncode_t libtock_can_subscribe_id(uint32_t id, bool extended, uint32_t mask);

// Drop every subscription this process holds.
returncode_t libtock_can_clear_subscriptions(void);

// How many subscriptions this process may hold at once.
returncode_t libtock_can_subscription_capacity(uint32_t* capacity);

// -- Operation --------------------------------------------------------------

returncode_t libtock_can_enable(libtock_can_callback_enabled cb);
returncode_t libtock_can_disable(libtock_can_callback_disabled cb);

// Transmit one frame. `data` must stay valid until the callback fires.
returncode_t libtock_can_send(uint32_t                          id,
                              bool                              extended,
                              const uint8_t*                    data,
                              uint32_t                          length,
                              libtock_can_callback_message_sent cb);

// Begin receiving into `rx`. The buffer must be at least
// `LIBTOCK_CAN_RX_BUFFER_SIZE(2)` bytes and stay valid until reception stops.
returncode_t libtock_can_start_receive(libtock_can_rx_t*                    rx,
                                       libtock_can_callback_message_received cb);

returncode_t libtock_can_stop_receive(libtock_can_callback_stopped cb);

returncode_t libtock_can_set_error_callback(libtock_can_callback_error cb);

// -- Receive buffer helpers -------------------------------------------------

// Prepare a receive buffer and record it in `rx`. Does not allow it to the
// kernel; `libtock_can_start_receive` does that.
void libtock_can_rx_init(libtock_can_rx_t* rx, uint8_t* buffer, uint32_t length);

// Number of frames currently in the buffer, consumed or not.
uint32_t libtock_can_rx_available(const libtock_can_rx_t* rx);

// Copy frame `index` out of the buffer. Returns false if it is not present.
bool libtock_can_rx_get(const libtock_can_rx_t* rx, uint32_t index, libtock_can_frame_t* frame);

// Copy the next unconsumed frame and advance. Returns false when none remain.
bool libtock_can_rx_next(libtock_can_rx_t* rx, libtock_can_frame_t* frame);

// True if the kernel ran out of room and dropped at least one frame.
bool libtock_can_rx_overflowed(const libtock_can_rx_t* rx);

// Discard everything in the buffer and start again from empty. Re-allows the
// buffer, so the kernel sees the cleared header.
returncode_t libtock_can_rx_reset(libtock_can_rx_t* rx);

#ifdef __cplusplus
}
#endif
