#pragma once

#include <libtock/net/can.h>

#ifdef __cplusplus
extern "C" {
#endif

bool libtocksync_can_exists(void);

// Register interest in an identifier range. Nothing is received until at least
// one subscription exists. See `libtock_can_subscribe_id`.
returncode_t libtocksync_can_subscribe_id(uint32_t id, bool extended, uint32_t mask);
returncode_t libtocksync_can_clear_subscriptions(void);

returncode_t libtocksync_can_enable(void);
returncode_t libtocksync_can_disable(void);

// Transmit one frame and wait for it to leave, or for `timeout_ms` to expire.
//
// A timeout here usually means nothing acknowledged the frame: CAN needs a
// second node on the bus, so a lone board can never complete a transmission.
returncode_t libtocksync_can_send(uint32_t       id,
                                  bool           extended,
                                  const uint8_t* data,
                                  uint32_t       length,
                                  uint32_t       timeout_ms);

returncode_t libtocksync_can_start_receive(libtock_can_rx_t* rx);
returncode_t libtocksync_can_stop_receive(void);

// Take the next unconsumed frame, waiting up to `timeout_ms` for one to
// arrive.
//
// Frames already sitting in the buffer are returned without waiting, so a
// burst that arrived while the process was descheduled is drained one call at
// a time rather than being collapsed into a single notification.
//
// Returns RETURNCODE_SUCCESS with `frame` filled in, or RETURNCODE_FAIL if the
// timeout expired first.
returncode_t libtocksync_can_read_frame(libtock_can_rx_t*    rx,
                                        libtock_can_frame_t* frame,
                                        uint32_t             timeout_ms);

#ifdef __cplusplus
}
#endif
