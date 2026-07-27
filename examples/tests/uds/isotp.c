#include "isotp.h"

#include <string.h>

#include <libtock-sync/net/can.h>
#include <libtock-sync/services/alarm.h>
#include <libtock/net/syscalls/can_syscalls.h>
#include <libtock/peripherals/syscalls/alarm_syscalls.h>

#define FRAME_LEN 8
#define SF_MAX    7  // payload bytes beside one PCI byte
#define FF_DATA   6  // payload bytes beside two PCI bytes
#define CF_DATA   7

#define PCI_SF   0x00
#define PCI_FF   0x10
#define PCI_CF   0x20
#define PCI_FC   0x30
#define PCI_MASK 0xF0

#define FS_CTS   0
#define FS_WAIT  1
#define FS_OVFLW 2

// Flow control this end grants as receiver: send the whole message without
// waiting. Same choice, and the same reasoning, as the bootloader's ISO-TP --
// the server assembles a complete message before acting on it, so there is
// nothing to push back against, and a stop-and-wait round trip per block costs
// milliseconds against a USB-attached host.
#define RX_BLOCK_SIZE 0
#define RX_STMIN      0

// ISO 15765-2 network layer timers.
#define N_BS_MS 1000  // sender waiting for flow control
#define N_CR_MS 1000  // receiver waiting for the next consecutive frame

// How long a single frame may take to leave. Generous on purpose: another
// process may hold the driver's transmit slot, and the capsule queues us
// behind it rather than refusing.
#define TX_FRAME_TIMEOUT_MS 500

static uint8_t tx_frame[FRAME_LEN] __attribute__((aligned(4)));

// Two receive buffers, swapped by `reclaim`. See the comment there for why one
// is not enough.
#define RX_STORE_LEN LIBTOCK_CAN_RX_BUFFER_SIZE(ISOTP_RX_FRAMES)
static uint8_t rx_store[2][RX_STORE_LEN] __attribute__((aligned(8)));
static libtock_can_rx_t rx_slot[2];

returncode_t isotp_init(isotp_t* tp, uint32_t rx_id, uint32_t tx_id) {
  tp->rx_id         = rx_id;
  tp->tx_id         = tx_id;
  tp->active        = 0;
  tp->carried_head  = 0;
  tp->carried_count = 0;
  tp->overflowed    = false;
  tp->stage         = ISOTP_STAGE_NONE;
  tp->last_error    = RETURNCODE_SUCCESS;

  libtock_can_rx_init(&rx_slot[0], rx_store[0], RX_STORE_LEN);
  libtock_can_rx_init(&rx_slot[1], rx_store[1], RX_STORE_LEN);

  return libtocksync_can_start_receive(&rx_slot[0]);
}

const char* isotp_stage_name(uint8_t stage) {
  switch (stage) {
    case ISOTP_STAGE_SEND_SINGLE:      return "single frame";
    case ISOTP_STAGE_SEND_FIRST:       return "first frame";
    case ISOTP_STAGE_NO_FLOW_CONTROL:  return "no flow control";
    case ISOTP_STAGE_FLOW_REFUSED:     return "flow control refused";
    case ISOTP_STAGE_SEND_CONSECUTIVE: return "consecutive frame";
    default:                           return "none";
  }
}

static uint32_t elapsed_ms(uint32_t start_ticks) {
  uint32_t now = 0;
  libtock_alarm_command_read(&now);
  // Unsigned subtraction gives the right answer across a counter wrap.
  return libtock_alarm_ticks_to_ms(now - start_ticks);
}

// Give the kernel a fresh buffer, without destroying anything already in the
// one it was using.
//
// The kernel only ever appends, so a buffer fills up and then drops
// everything; something has to recycle it. Clearing it in place -- which is
// what `libtock_can_rx_reset` does -- is destructive: a frame arriving between
// the "is it drained?" check and the memset is wiped out. That cost about one
// request in forty under sustained traffic, and a lost request looks exactly
// like a server that stopped answering.
//
// Swapping buffers has no such window. The allow syscall is the atomic point:
// once it returns the kernel appends to the new buffer, and whatever is in the
// old one is complete and can be read at leisure. Frames that landed there
// just before the swap are older than anything in the new buffer, so they are
// carried over and handed out first.
static void reclaim(isotp_t* tp) {
  libtock_can_rx_t* old = &rx_slot[tp->active];
  libtock_can_rx_t* fresh = &rx_slot[tp->active ^ 1];

  // Note this asks whether *this* buffer overflowed, not `tp->overflowed`,
  // which is sticky for the life of the process: keying the decision off the
  // sticky flag would swap buffers on every single read once one frame had
  // ever been lost.
  bool full = libtock_can_rx_overflowed(old);
  if (full) tp->overflowed = true;
  if (!full && libtock_can_rx_available(old) < ISOTP_RX_FRAMES / 2) return;
  // No room to rescue stragglers into; try again after the caller drains.
  if (tp->carried_count > ISOTP_CARRIED - 2) return;

  memset(fresh->buffer, 0, fresh->length);
  fresh->next = 0;
  if (libtock_can_set_readwrite_allow_rx_buffer(fresh->buffer, fresh->length)
      != RETURNCODE_SUCCESS) {
    return;
  }
  tp->active ^= 1;

  libtock_can_frame_t frame;
  while (tp->carried_count < ISOTP_CARRIED && libtock_can_rx_next(old, &frame)) {
    uint8_t slot = (uint8_t)((tp->carried_head + tp->carried_count) % ISOTP_CARRIED);
    tp->carried[slot] = frame;
    tp->carried_count++;
  }
  if (libtock_can_rx_overflowed(old)) tp->overflowed = true;
}

// Take the oldest carried-over frame, if there is one.
static bool take_carried(isotp_t* tp, libtock_can_frame_t* frame) {
  if (tp->carried_count == 0) return false;
  *frame = tp->carried[tp->carried_head];
  tp->carried_head = (uint8_t)((tp->carried_head + 1) % ISOTP_CARRIED);
  tp->carried_count--;
  return true;
}

// Read the next frame addressed to us, ignoring anything else, until
// `timeout_ms` has passed in total.
static returncode_t read_frame(isotp_t* tp, libtock_can_frame_t* frame, uint32_t timeout_ms) {
  uint32_t start = 0;
  libtock_alarm_command_read(&start);

  for (;;) {
    // Frames rescued from a swapped-out buffer are the oldest we hold, so they
    // come out before anything the kernel has since appended.
    if (take_carried(tp, frame)) {
      if (frame->id == tp->rx_id) return RETURNCODE_SUCCESS;
      continue;
    }

    reclaim(tp);

    uint32_t used = elapsed_ms(start);
    if (used >= timeout_ms) return RETURNCODE_FAIL;

    libtock_can_rx_t* rx = &rx_slot[tp->active];
    returncode_t ret = libtocksync_can_read_frame(rx, frame, timeout_ms - used);
    if (libtock_can_rx_overflowed(rx)) tp->overflowed = true;
    if (ret != RETURNCODE_SUCCESS) return RETURNCODE_FAIL;

    // The subscription is an exact match, so this should never reject
    // anything; it is here because assembling a message from the wrong
    // identifier would be silently wrong rather than loudly broken.
    if (frame->id == tp->rx_id) return RETURNCODE_SUCCESS;
  }
}

static returncode_t send_frame(isotp_t* tp, const uint8_t* data) {
  memcpy(tx_frame, data, FRAME_LEN);
  returncode_t ret = libtocksync_can_send(tp->tx_id, true, tx_frame, FRAME_LEN,
                                          TX_FRAME_TIMEOUT_MS);
  if (ret != RETURNCODE_SUCCESS) tp->last_error = ret;
  return ret;
}

static returncode_t send_flow_control(isotp_t* tp, uint8_t fs) {
  uint8_t frame[FRAME_LEN] = {0};
  frame[0] = PCI_FC | fs;
  frame[1] = RX_BLOCK_SIZE;
  frame[2] = RX_STMIN;
  return send_frame(tp, frame);
}

// -- Receive ----------------------------------------------------------------

// Collect consecutive frames after a first frame has been accepted.
static returncode_t recv_consecutive(isotp_t* tp, uint8_t* msg, size_t total, size_t* len) {
  size_t offset      = FF_DATA;
  uint8_t expect_sn = 1;

  while (offset < total) {
    libtock_can_frame_t frame;
    if (read_frame(tp, &frame, N_CR_MS) != RETURNCODE_SUCCESS) return RETURNCODE_FAIL;

    if ((frame.data[0] & PCI_MASK) != PCI_CF) return RETURNCODE_FAIL;

    // A gap means frames were lost. Abandoning is the only safe move: the
    // assembled message would otherwise be silently wrong.
    if ((frame.data[0] & 0x0F) != expect_sn) return RETURNCODE_FAIL;
    expect_sn = (uint8_t)((expect_sn + 1) & 0x0F);

    size_t take = total - offset;
    if (take > CF_DATA) take = CF_DATA;
    memcpy(&msg[offset], &frame.data[1], take);
    offset += take;
  }

  *len = total;
  return RETURNCODE_SUCCESS;
}

returncode_t isotp_recv(isotp_t* tp, uint8_t* msg, size_t capacity, size_t* len,
                        uint32_t timeout_ms) {
  libtock_can_frame_t frame;
  if (read_frame(tp, &frame, timeout_ms) != RETURNCODE_SUCCESS) return RETURNCODE_FAIL;

  switch (frame.data[0] & PCI_MASK) {
    case PCI_SF: {
      size_t n = frame.data[0] & 0x0F;
      if (n == 0 || n > SF_MAX || n > capacity) return RETURNCODE_FAIL;
      memcpy(msg, &frame.data[1], n);
      *len = n;
      return RETURNCODE_SUCCESS;
    }

    case PCI_FF: {
      size_t total = (size_t)((frame.data[0] & 0x0F) << 8) | frame.data[1];
      // Zero here is the ISO 15765-2:2016 escape encoding for messages beyond
      // 4095 bytes. Nothing this server answers comes close, so refusing is
      // honest and keeps the parser small.
      if (total == 0 || total > capacity || total > ISOTP_MAX_MESSAGE) {
        (void)send_flow_control(tp, FS_OVFLW);
        return RETURNCODE_ESIZE;
      }
      memcpy(msg, &frame.data[2], FF_DATA);

      if (send_flow_control(tp, FS_CTS) != RETURNCODE_SUCCESS) return RETURNCODE_FAIL;
      return recv_consecutive(tp, msg, total, len);
    }

    default:
      // A consecutive or flow control frame with no transfer in progress. The
      // tail of a transfer we already gave up on, most likely; drop it.
      return RETURNCODE_FAIL;
  }
}

// -- Transmit ---------------------------------------------------------------

// Wait for the peer's flow control, returning its block size and separation
// time. Frames that are not flow control are ignored: a tester that reissues
// its request mid-response must not be mistaken for an answer.
static returncode_t await_flow_control(isotp_t* tp, uint8_t* bs, uint32_t* stmin_ms) {
  for (;;) {
    libtock_can_frame_t frame;
    if (read_frame(tp, &frame, N_BS_MS) != RETURNCODE_SUCCESS) {
      tp->stage = ISOTP_STAGE_NO_FLOW_CONTROL;
      return RETURNCODE_FAIL;
    }
    if ((frame.data[0] & PCI_MASK) != PCI_FC) continue;

    uint8_t fs = frame.data[0] & 0x0F;
    if (fs == FS_WAIT) continue;
    if (fs != FS_CTS) {  // overflow, or reserved
      tp->stage = ISOTP_STAGE_FLOW_REFUSED;
      return RETURNCODE_FAIL;
    }

    *bs = frame.data[1];
    // STmin 0x01..0x7F is milliseconds; 0xF1..0xF9 is 100..900 us, which the
    // alarm cannot express, so it rounds up to 1 ms. Erring long is harmless.
    uint8_t raw = frame.data[2];
    if (raw == 0) {
      *stmin_ms = 0;
    } else if (raw <= 0x7F) {
      *stmin_ms = raw;
    } else if (raw >= 0xF1 && raw <= 0xF9) {
      *stmin_ms = 1;
    } else {
      *stmin_ms = 0;
    }
    return RETURNCODE_SUCCESS;
  }
}

returncode_t isotp_send(isotp_t* tp, const uint8_t* msg, size_t len) {
  if (len == 0 || len > ISOTP_MAX_MESSAGE) return RETURNCODE_ESIZE;

  uint8_t frame[FRAME_LEN];
  tp->stage = ISOTP_STAGE_NONE;

  if (len <= SF_MAX) {
    memset(frame, 0, sizeof(frame));
    frame[0] = PCI_SF | (uint8_t)len;
    memcpy(&frame[1], msg, len);
    returncode_t ret = send_frame(tp, frame);
    if (ret != RETURNCODE_SUCCESS) tp->stage = ISOTP_STAGE_SEND_SINGLE;
    return ret;
  }

  memset(frame, 0, sizeof(frame));
  frame[0] = PCI_FF | (uint8_t)((len >> 8) & 0x0F);
  frame[1] = (uint8_t)(len & 0xFF);
  memcpy(&frame[2], msg, FF_DATA);
  returncode_t ret = send_frame(tp, frame);
  if (ret != RETURNCODE_SUCCESS) {
    tp->stage = ISOTP_STAGE_SEND_FIRST;
    return ret;
  }

  uint8_t bs        = 0;
  uint32_t stmin_ms = 0;
  ret = await_flow_control(tp, &bs, &stmin_ms);
  if (ret != RETURNCODE_SUCCESS) return ret;

  size_t offset      = FF_DATA;
  uint8_t sn         = 1;
  uint8_t in_block   = bs;

  while (offset < len) {
    size_t take = len - offset;
    if (take > CF_DATA) take = CF_DATA;

    memset(frame, 0, sizeof(frame));
    frame[0] = PCI_CF | (sn & 0x0F);
    memcpy(&frame[1], &msg[offset], take);
    ret = send_frame(tp, frame);
    if (ret != RETURNCODE_SUCCESS) {
      tp->stage = ISOTP_STAGE_SEND_CONSECUTIVE;
      return ret;
    }

    offset += take;
    sn      = (uint8_t)((sn + 1) & 0x0F);

    if (stmin_ms > 0 && offset < len) libtocksync_alarm_delay_ms(stmin_ms);

    if (bs != 0 && in_block > 0) in_block--;
    if (bs != 0 && in_block == 0 && offset < len) {
      // Block exhausted: the receiver has to authorise the next one, and may
      // change the block size and separation time while doing so.
      ret = await_flow_control(tp, &bs, &stmin_ms);
      if (ret != RETURNCODE_SUCCESS) return ret;
      in_block = bs;
    }
  }

  return RETURNCODE_SUCCESS;
}
