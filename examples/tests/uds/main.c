/* The runtime UDS server.
 *
 * A diagnostic server for the *running* kernel, answering a tester on the CAN
 * bus at the same pair of identifiers the bootloader uses:
 *
 *   request   0x18DA41F1     ISO 15765-2 normal fixed addressing,
 *   response  0x18DAF141     tester 0xF1 -> target 0x41
 *
 * Reusing the bootloader's addresses is deliberate (design document section
 * 9.3): a tester keeps talking to the same address across a reprogramming
 * cycle and tells the two servers apart by reading DID 0xF180.
 *
 * This is a separate application from the OBD-II scanner even though both use
 * CAN. They have opposite roles -- the scanner is a *tester*, transmitting
 * requests at 0x18DB33F1; this is a *server*, answering requests addressed to
 * it -- and only this one is signed and holds the reboot privilege, which
 * keeps the privileged binary small.
 *
 * The package name must stay `uds`. The kernel derives the privileged identity
 * from it, so renaming the directory or the Makefile's PACKAGE_NAME silently
 * costs the app its ability to enter the bootloader.
 */

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <libtock-sync/interface/console.h>
#include <libtock-sync/net/can.h>
#include <libtock-sync/services/alarm.h>
#include <libtock/tock.h>

#include "bl_reboot.h"
#include "isotp.h"
#include "uds.h"

// Identifiers, as above.
#define UDS_REQUEST_ID  0x18DA41F1
#define UDS_RESPONSE_ID 0x18DAF141
// One exact identifier: 29 bits, all significant.
#define UDS_REQUEST_MASK 0x1FFFFFFF

// How long the server blocks waiting for a request before looking at the S3
// session timer and going back to waiting.
#define IDLE_POLL_MS 250

static isotp_t tp;

static uint8_t request[ISOTP_MAX_MESSAGE];
static uint8_t response[ISOTP_MAX_MESSAGE];

// The synchronous console throughout, never an async write followed by a bare
// yield(). A bare yield returns on *any* upcall, and this app always has CAN
// traffic pending, so it would return before the write completed and leave a
// dead stack frame shared with the kernel.
static void print(const char* msg) {
  int written;
  libtocksync_console_write((const uint8_t*)msg, strlen(msg), &written);
}

static char hex_char(uint8_t nibble) {
  return nibble < 10 ? (char)('0' + nibble) : (char)('A' + nibble - 10);
}

static void print_hex_byte(uint8_t b) {
  char buf[2] = {hex_char((uint8_t)(b >> 4)), hex_char(b & 0x0F)};
  int written;
  libtocksync_console_write((const uint8_t*)buf, sizeof(buf), &written);
}

static int setup_can(void) {
  if (!libtocksync_can_exists()) {
    print("[CAN] driver not found\r\n");
    return -1;
  }

  // Bitrate and mode belong to the shared bus and are fixed at board setup.
  // All this app declares is which identifier it wants; nothing is delivered
  // to a process with no subscription.
  if (libtocksync_can_subscribe_id(UDS_REQUEST_ID, true, UDS_REQUEST_MASK) != RETURNCODE_SUCCESS) {
    print("[CAN] subscribe failed\r\n");
    return -1;
  }
  print("[CAN] subscribed 0x18DA41F1\r\n");

  // Refcounted in the capsule: the OBD-II scanner may already have the
  // peripheral running, in which case this returns immediately, and its later
  // disable must not take the bus away from us.
  if (libtocksync_can_enable() != RETURNCODE_SUCCESS) {
    print("[CAN] enable failed\r\n");
    return -1;
  }
  print("[CAN] enabled\r\n");

  if (isotp_init(&tp, UDS_REQUEST_ID, UDS_RESPONSE_ID) != RETURNCODE_SUCCESS) {
    print("[CAN] start receive failed\r\n");
    return -1;
  }
  print("[CAN] receiving\r\n");
  return 0;
}

int main(void) {
  print("\r\n=== UDS SERVER ===\r\n");

  uds_init();

  if (!bl_reboot_exists()) {
    print("[BL] reboot driver not found\r\n");
  } else if (bl_reboot_permitted()) {
    print("[BL] reboot privilege held (signed)\r\n");
  } else {
    print("[BL] no reboot privilege (unsigned) -- 0x10 02 and 0x11 will refuse\r\n");
  }

  if (setup_can() != 0) {
    print("[UDS] cannot serve, exiting\r\n");
    // tock_exit() rather than returning: returning runs newlib's exit()
    // cleanup, which touches the _reent struct at its linker VMA, and the MPU
    // faults that address at runtime.
    tock_exit(1);
  }

  print("[UDS] listening\r\n");

  for (;;) {
    uds_tick();

    size_t request_len = 0;
    returncode_t ret = isotp_recv(&tp, request, sizeof(request), &request_len, IDLE_POLL_MS);
    if (ret != RETURNCODE_SUCCESS) continue;

    uds_set_transport_healthy(!tp.overflowed);

    size_t response_len = uds_handle(request, request_len, response, sizeof(response));
    if (response_len > 0) {
      if (isotp_send(&tp, response, response_len) != RETURNCODE_SUCCESS) {
        print("[UDS] response failed to send: ");
        print(isotp_stage_name(tp.stage));
        print(" rc=");
        print_hex_byte((uint8_t)(-(int)tp.last_error));
        print("\r\n");
        // The action a response promised must not happen if the tester never
        // heard the promise: it would look like an unexplained reset.
        uds_cancel_pending();
        continue;
      }
    }

    // Printed after the response is on the wire so console latency cannot push
    // the answer past P2.
    print("[UDS] 0x");
    print_hex_byte(request[0]);
    print(" -> 0x");
    print_hex_byte(response_len > 0 ? response[0] : 0x00);
    print("\r\n");

    // Carries out a reset, if that is what the response promised. Does not
    // return in that case.
    uds_post_response();
  }
}
