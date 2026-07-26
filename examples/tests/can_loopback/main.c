/* CAN Test App for SAMV71 MCAN driver.
 *
 * Mode is selected at compile time:
 *   - Default: internal loopback (no external bus needed)
 *   - Define CAN_NORMAL_MODE=1: normal mode for external bus testing
 *
 * In loopback mode: sends one frame and verifies the looped-back data.
 * In normal mode:   sends periodic frames and prints any received frames.
 */

#include <string.h>
#include <libtock/tock.h>
#include <libtock/interface/console.h>

#ifndef CAN_NORMAL_MODE
#define CAN_NORMAL_MODE 0
#endif

#define CAN_DRIVER_NUM  0x20007

#define CMD_SET_BITRATE      1
#define CMD_SET_OP_MODE      2
#define CMD_ENABLE           3
#define CMD_DISABLE          4
#define CMD_SEND_STD         5
#define CMD_START_RECEIVE    7
#define CMD_STOP_RECEIVE     8

#define SUB_ENABLE           0
#define SUB_MESSAGE_SENT     2
#define SUB_MESSAGE_RECEIVED 3
#define SUB_RECEIVED_STOPPED 4
#define SUB_TX_ERROR         5

#define OP_LOOPBACK   0
#define OP_NORMAL     3

static volatile bool     enabled_done;
static volatile bool     tx_done;
static volatile bool     rx_done;
static volatile bool     stopped_done;
static volatile uint32_t enable_status;
static volatile uint32_t tx_status;
static volatile uint32_t rx_id;

static uint8_t rx_buf[4 + 8 * 8] __attribute__((aligned(4)));
static uint8_t tx_buf[8] __attribute__((aligned(4)));

static void write_done(returncode_t ret __attribute__((unused)),
                       uint32_t bytes __attribute__((unused))) {}

static void print(const char* msg) {
  libtock_console_write((const uint8_t*)msg, strlen(msg), write_done);
  yield();
}

static char hex_char(uint8_t nibble) {
  return nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
}

static void print_hex_byte(uint8_t b) {
  char buf[3];
  buf[0] = hex_char(b >> 4);
  buf[1] = hex_char(b & 0xF);
  buf[2] = ' ';
  libtock_console_write((const uint8_t*)buf, 3, write_done);
  yield();
}

static void print_u32(uint32_t val) {
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (val >> (i * 4)) & 0xF;
    char c = hex_char(nibble);
    libtock_console_write((const uint8_t*)&c, 1, write_done);
    yield();
  }
}

static void enable_cb(int status, int a2 __attribute__((unused)),
                      int a3 __attribute__((unused)), void* ud __attribute__((unused))) {
  enable_status = (uint32_t)status;
  enabled_done = true;
}

static void tx_cb(int status, int a2 __attribute__((unused)),
                  int a3 __attribute__((unused)), void* ud __attribute__((unused))) {
  tx_status = (uint32_t)status;
  tx_done = true;
}

static void rx_cb(int status __attribute__((unused)), int offset __attribute__((unused)),
                  int id, void* ud __attribute__((unused))) {
  rx_id = (uint32_t)id;
  rx_done = true;
}

static void stopped_cb(int a1 __attribute__((unused)), int a2 __attribute__((unused)),
                       int a3 __attribute__((unused)), void* ud __attribute__((unused))) {
  stopped_done = true;
}

static void error_cb(int a1 __attribute__((unused)), int a2 __attribute__((unused)),
                     int a3 __attribute__((unused)), void* ud __attribute__((unused))) {
}

static returncode_t can_cmd(uint32_t cmd, int arg1, int arg2) {
  syscall_return_t cval = command(CAN_DRIVER_NUM, cmd, arg1, arg2);
  return tock_command_return_novalue_to_returncode(cval);
}

static int setup_can(void) {
  returncode_t ret;

  if (!driver_exists(CAN_DRIVER_NUM)) {
    print("[CAN] Driver not found\r\n");
    return -1;
  }
  print("[CAN] Driver found\r\n");

  subscribe(CAN_DRIVER_NUM, SUB_ENABLE, enable_cb, NULL);
  subscribe(CAN_DRIVER_NUM, SUB_MESSAGE_SENT, tx_cb, NULL);
  subscribe(CAN_DRIVER_NUM, SUB_MESSAGE_RECEIVED, rx_cb, NULL);
  subscribe(CAN_DRIVER_NUM, SUB_RECEIVED_STOPPED, stopped_cb, NULL);
  subscribe(CAN_DRIVER_NUM, SUB_TX_ERROR, error_cb, NULL);

  allow_readwrite(CAN_DRIVER_NUM, 0, rx_buf, sizeof(rx_buf));
  allow_readonly(CAN_DRIVER_NUM, 0, tx_buf, sizeof(tx_buf));

  ret = can_cmd(CMD_SET_BITRATE, 500000, 0);
  if (ret != RETURNCODE_SUCCESS) {
    print("[CAN] Set bitrate FAILED\r\n");
    return -1;
  }
  print("[CAN] Bitrate: 500 kbps\r\n");

#if CAN_NORMAL_MODE
  ret = can_cmd(CMD_SET_OP_MODE, OP_NORMAL, 0);
  print("[CAN] Mode: Normal (external bus)\r\n");
#else
  ret = can_cmd(CMD_SET_OP_MODE, OP_LOOPBACK, 0);
  print("[CAN] Mode: Loopback (internal)\r\n");
#endif
  if (ret != RETURNCODE_SUCCESS) {
    print("[CAN] Set mode FAILED\r\n");
    return -1;
  }

  enabled_done = false;
  ret = can_cmd(CMD_ENABLE, 0, 0);
  if (ret != RETURNCODE_SUCCESS) {
    print("[CAN] Enable FAILED\r\n");
    return -1;
  }
  yield_for((bool*)&enabled_done);
  if (enable_status != 0) {
    print("[CAN] Enable callback error\r\n");
    return -1;
  }
  print("[CAN] Enabled\r\n");
  return 0;
}

static int run_loopback_test(void) {
  returncode_t ret;

  tx_buf[0] = 0xDE; tx_buf[1] = 0xAD; tx_buf[2] = 0xBE; tx_buf[3] = 0xEF;
  tx_buf[4] = 0xCA; tx_buf[5] = 0xFE; tx_buf[6] = 0xBA; tx_buf[7] = 0xBE;
  allow_readonly(CAN_DRIVER_NUM, 0, tx_buf, sizeof(tx_buf));

  memset(rx_buf, 0, sizeof(rx_buf));
  allow_readwrite(CAN_DRIVER_NUM, 0, rx_buf, sizeof(rx_buf));

  rx_done = false;
  ret = can_cmd(CMD_START_RECEIVE, 0, 0);
  if (ret != RETURNCODE_SUCCESS) { print("[CAN] Start RX FAILED\r\n"); return -1; }

  tx_done = false;
  ret = can_cmd(CMD_SEND_STD, 0x123, 8);
  if (ret != RETURNCODE_SUCCESS) { print("[CAN] Send FAILED\r\n"); return -1; }

  yield_for((bool*)&tx_done);
  yield_for((bool*)&rx_done);

  print("[CAN] RX data: ");
  for (int i = 8; i < 16; i++) print_hex_byte(rx_buf[i]);
  print("\r\n");

  if (rx_buf[8] == 0xDE && rx_buf[9] == 0xAD &&
      rx_buf[10] == 0xBE && rx_buf[11] == 0xEF) {
    print("[CAN] PASS - loopback OK\r\n");
    return 0;
  }
  print("[CAN] FAIL - data mismatch\r\n");
  return -1;
}

static void run_external_test(void) {
  returncode_t ret;
  uint32_t tx_count = 0;

  memset(rx_buf, 0, sizeof(rx_buf));
  allow_readwrite(CAN_DRIVER_NUM, 0, rx_buf, sizeof(rx_buf));

  rx_done = false;
  ret = can_cmd(CMD_START_RECEIVE, 0, 0);
  if (ret != RETURNCODE_SUCCESS) { print("[CAN] Start RX FAILED\r\n"); return; }
  print("[CAN] Listening for frames...\r\n");
  print("[CAN] Sending test frames (ID 0x123) every ~2s\r\n");
  print("[CAN] Connect PCAN-USB at 500 kbps to verify\r\n\r\n");

  while (1) {
    // Prepare TX data with counter
    tx_buf[0] = 0xCA; tx_buf[1] = 0xFE;
    tx_buf[2] = (uint8_t)(tx_count >> 8);
    tx_buf[3] = (uint8_t)(tx_count & 0xFF);
    tx_buf[4] = 0xDE; tx_buf[5] = 0xAD;
    tx_buf[6] = 0xBE; tx_buf[7] = 0xEF;
    allow_readonly(CAN_DRIVER_NUM, 0, tx_buf, sizeof(tx_buf));

    tx_done = false;
    ret = can_cmd(CMD_SEND_STD, 0x123, 8);
    if (ret == RETURNCODE_SUCCESS) {
      yield_for((bool*)&tx_done);
      print("[TX] #");
      print_u32(tx_count);
      if (tx_status == 0) {
        print(" OK\r\n");
      } else {
        print(" ERR\r\n");
      }
    } else {
      print("[TX] send failed\r\n");
    }
    tx_count++;

    // Check for received frames (non-blocking)
    if (rx_done) {
      print("[RX] ID=0x");
      print_u32(rx_id);
      print(" data: ");
      for (int i = 8; i < 16; i++) print_hex_byte(rx_buf[i]);
      print("\r\n");

      // Reset for next receive
      rx_done = false;
      memset(rx_buf, 0, sizeof(rx_buf));
      allow_readwrite(CAN_DRIVER_NUM, 0, rx_buf, sizeof(rx_buf));
    }

    // Busy-wait delay (~1 second at 300 MHz)
    for (volatile int d = 0; d < 3000000; d++) {}
  }
}

int main(void) {
#if CAN_NORMAL_MODE
  print("\r\n=== CAN External Bus Test (500 kbps) ===\r\n\r\n");
#else
  print("\r\n=== CAN Loopback Test (500 kbps) ===\r\n\r\n");
#endif

  if (setup_can() != 0) return -1;

#if CAN_NORMAL_MODE
  run_external_test();
#else
  run_loopback_test();

  stopped_done = false;
  can_cmd(CMD_STOP_RECEIVE, 0, 0);
  yield_for((bool*)&stopped_done);
  can_cmd(CMD_DISABLE, 0, 0);
  yield();
  print("\r\n=== Test Complete ===\r\n");
#endif

  return 0;
}
