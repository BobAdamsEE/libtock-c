#include "uds.h"

#include <string.h>

#include <libtock-sync/services/alarm.h>
#include <libtock/peripherals/syscalls/alarm_syscalls.h>
#include <libtock/tock.h>

#include "bl_reboot.h"

// Services.
#define SID_DIAGNOSTIC_SESSION_CONTROL 0x10
#define SID_ECU_RESET                  0x11
#define SID_READ_DATA_BY_IDENTIFIER    0x22
#define SID_SECURITY_ACCESS            0x27
#define SID_WRITE_DATA_BY_IDENTIFIER   0x2E
#define SID_ROUTINE_CONTROL            0x31
#define SID_TESTER_PRESENT             0x3E

#define POSITIVE_RESPONSE_OFFSET 0x40
#define NEGATIVE_RESPONSE        0x7F

// Negative response codes.
#define NRC_SERVICE_NOT_SUPPORTED     0x11
#define NRC_SUBFUNCTION_NOT_SUPPORTED 0x12
#define NRC_INCORRECT_LENGTH          0x13
#define NRC_CONDITIONS_NOT_CORRECT    0x22
#define NRC_REQUEST_SEQUENCE_ERROR    0x24
#define NRC_REQUEST_OUT_OF_RANGE      0x31
#define NRC_SECURITY_ACCESS_DENIED    0x33
#define NRC_INVALID_KEY               0x35

// Sessions.
#define SESSION_DEFAULT     0x01
#define SESSION_PROGRAMMING 0x02
#define SESSION_EXTENDED    0x03

// Data identifiers. 0xF180 and 0xF186 are the standard pair a tester uses to
// tell which server answered; the bootloader reports "TOCKBL" for 0xF180, so a
// tester polling after a reboot can see when the handover happened. The vendor
// identifiers are deliberately clear of the bootloader's 0xF200 so one DID map
// covers both.
#define DID_BOOT_SOFTWARE_IDENTIFICATION 0xF180
#define DID_ACTIVE_SESSION               0xF186
#define DID_REBOOT_PRIVILEGE             0xF210
#define DID_UPTIME_MS                    0xF211
#define DID_CONFIGURATION                0xF212

// Routines. Manufacturer range, per the design document's rule that anything
// without a natural UDS home lives in 0x0200-0xDFFF rather than being bolted
// onto a standard identifier.
#define ROUTINE_SELF_TEST 0x0300
#define ROUTINE_START     0x01
#define ROUTINE_RESULTS   0x03

// Self-test result bits.
#define SELFTEST_REBOOT_DRIVER    0x01
#define SELFTEST_REBOOT_PRIVILEGE 0x02
#define SELFTEST_TRANSPORT_OK     0x04

// XORed with the seed to form the expected key. Not a secret -- it is compiled
// into the binary an attacker is trying to reach -- and deliberately different
// from the bootloader's, so unlocking one does not unlock the other.
#define SECURITY_KEY_XOR 0x55445341u

// ISO 14229 S3: the session lapses if the tester goes quiet this long.
#define S3_MS 5000

typedef enum {
  SECURITY_LOCKED,
  SECURITY_SEED_ISSUED,
  SECURITY_UNLOCKED,
} security_t;

typedef enum {
  ACTION_NONE,
  // Restart into the bootloader so it can reprogram us.
  ACTION_ENTER_BOOTLOADER,
  // Restart into the kernel.
  ACTION_RESET,
} action_t;

static uint8_t session = SESSION_DEFAULT;
static security_t security = SECURITY_LOCKED;
static uint32_t issued_seed;
static action_t pending_action = ACTION_NONE;

static uint32_t start_ticks;
static uint32_t last_request_ticks;

static uint8_t configuration;
static uint8_t self_test_result;
static bool transport_healthy = true;

static uint32_t now_ticks(void) {
  uint32_t now = 0;
  libtock_alarm_command_read(&now);
  return now;
}

// Unsigned subtraction is correct across a counter wrap.
static uint32_t ms_since(uint32_t ticks) {
  return libtock_alarm_ticks_to_ms(now_ticks() - ticks);
}

void uds_init(void) {
  session            = SESSION_DEFAULT;
  security           = SECURITY_LOCKED;
  pending_action     = ACTION_NONE;
  configuration      = 0;
  self_test_result   = 0;
  transport_healthy  = true;
  start_ticks        = now_ticks();
  last_request_ticks = start_ticks;
}

void uds_set_transport_healthy(bool healthy) {
  transport_healthy = healthy;
}

uint8_t uds_session(void) {
  return session;
}

static void relock(void) {
  session  = SESSION_DEFAULT;
  security = SECURITY_LOCKED;
}

void uds_tick(void) {
  if (session == SESSION_DEFAULT && security == SECURITY_LOCKED) return;
  if (ms_since(last_request_ticks) >= S3_MS) relock();
}

// -- Response helpers -------------------------------------------------------

static size_t negative(uint8_t* rsp, uint8_t sid, uint8_t nrc) {
  rsp[0] = NEGATIVE_RESPONSE;
  rsp[1] = sid;
  rsp[2] = nrc;
  return 3;
}

static void put_be32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static uint32_t get_be32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

// -- Services ---------------------------------------------------------------

static size_t session_control(const uint8_t* req, size_t len, uint8_t* rsp) {
  if (len < 2) return negative(rsp, SID_DIAGNOSTIC_SESSION_CONTROL, NRC_INCORRECT_LENGTH);

  uint8_t sub = req[1] & 0x7F;
  if (sub != SESSION_DEFAULT && sub != SESSION_PROGRAMMING && sub != SESSION_EXTENDED) {
    return negative(rsp, SID_DIAGNOSTIC_SESSION_CONTROL, NRC_SUBFUNCTION_NOT_SUPPORTED);
  }

  if (sub == SESSION_PROGRAMMING) {
    // Reprogramming happens in the bootloader, so entering the programming
    // session means leaving this kernel. Refuse rather than promise something
    // we cannot deliver: an unsigned build of this app loads and runs, but the
    // kernel gives it ShortId::LocallyUnique and the driver says no.
    if (!bl_reboot_permitted()) {
      return negative(rsp, SID_DIAGNOSTIC_SESSION_CONTROL, NRC_CONDITIONS_NOT_CORRECT);
    }
    // Answer first, restart once the response is on the wire. The tester then
    // polls 0xF180 until the bootloader answers instead of us.
    pending_action = ACTION_ENTER_BOOTLOADER;
  }

  session = sub;
  if (sub == SESSION_DEFAULT) security = SECURITY_LOCKED;

  rsp[0] = SID_DIAGNOSTIC_SESSION_CONTROL + POSITIVE_RESPONSE_OFFSET;
  rsp[1] = sub;
  // P2 = 50 ms and P2* = 5000 ms, in the units ISO 14229 uses for each: 1 ms
  // and 10 ms.
  rsp[2] = 0x00;
  rsp[3] = 0x32;
  rsp[4] = 0x01;
  rsp[5] = 0xF4;
  return 6;
}

static size_t ecu_reset(const uint8_t* req, size_t len, uint8_t* rsp) {
  if (len < 2) return negative(rsp, SID_ECU_RESET, NRC_INCORRECT_LENGTH);

  uint8_t sub = req[1] & 0x7F;
  // hardReset and softReset are the same action here: the board restarts and
  // comes back up in the kernel. Note this is *not* the bootloader path --
  // that is DiagnosticSessionControl programming session.
  if (sub != 0x01 && sub != 0x03) {
    return negative(rsp, SID_ECU_RESET, NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
  if (!bl_reboot_permitted()) {
    return negative(rsp, SID_ECU_RESET, NRC_CONDITIONS_NOT_CORRECT);
  }

  pending_action = ACTION_RESET;
  rsp[0] = SID_ECU_RESET + POSITIVE_RESPONSE_OFFSET;
  rsp[1] = sub;
  return 2;
}

static size_t read_data_by_identifier(const uint8_t* req, size_t len, uint8_t* rsp) {
  if (len < 3) return negative(rsp, SID_READ_DATA_BY_IDENTIFIER, NRC_INCORRECT_LENGTH);

  uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
  rsp[0] = SID_READ_DATA_BY_IDENTIFIER + POSITIVE_RESPONSE_OFFSET;
  rsp[1] = req[1];
  rsp[2] = req[2];

  switch (did) {
    case DID_BOOT_SOFTWARE_IDENTIFICATION: {
      static const char name[] = "TOCKUDS 0.1.0";
      memcpy(&rsp[3], name, sizeof(name) - 1);
      return 3 + sizeof(name) - 1;
    }
    case DID_ACTIVE_SESSION:
      rsp[3] = session;
      return 4;
    case DID_REBOOT_PRIVILEGE:
      rsp[3] = bl_reboot_permitted() ? 1 : 0;
      return 4;
    case DID_UPTIME_MS:
      put_be32(&rsp[3], ms_since(start_ticks));
      return 7;
    case DID_CONFIGURATION:
      rsp[3] = configuration;
      return 4;
    default:
      return negative(rsp, SID_READ_DATA_BY_IDENTIFIER, NRC_REQUEST_OUT_OF_RANGE);
  }
}

static size_t security_access(const uint8_t* req, size_t len, uint8_t* rsp) {
  if (len < 2) return negative(rsp, SID_SECURITY_ACCESS, NRC_INCORRECT_LENGTH);
  if (session == SESSION_DEFAULT) {
    return negative(rsp, SID_SECURITY_ACCESS, NRC_CONDITIONS_NOT_CORRECT);
  }

  uint8_t sub = req[1];
  switch (sub) {
    case 0x01: {
      rsp[0] = SID_SECURITY_ACCESS + POSITIVE_RESPONSE_OFFSET;
      rsp[1] = sub;
      if (security == SECURITY_UNLOCKED) {
        // ISO 14229: an all-zero seed tells the tester it is already in.
        put_be32(&rsp[2], 0);
        return 6;
      }
      // Derived from the running clock so it is not the same number every
      // session. That is the most this scheme can honestly offer; see the
      // header.
      issued_seed = (now_ticks() * 2654435761u) ^ 0xA5A51234u;
      security    = SECURITY_SEED_ISSUED;
      put_be32(&rsp[2], issued_seed);
      return 6;
    }

    case 0x02: {
      if (len < 6) return negative(rsp, SID_SECURITY_ACCESS, NRC_INCORRECT_LENGTH);
      if (security != SECURITY_SEED_ISSUED) {
        return negative(rsp, SID_SECURITY_ACCESS, NRC_REQUEST_SEQUENCE_ERROR);
      }
      if (get_be32(&req[2]) != (issued_seed ^ SECURITY_KEY_XOR)) {
        // Back to locked, so each guess costs a fresh seed.
        security = SECURITY_LOCKED;
        return negative(rsp, SID_SECURITY_ACCESS, NRC_INVALID_KEY);
      }
      security = SECURITY_UNLOCKED;
      rsp[0]   = SID_SECURITY_ACCESS + POSITIVE_RESPONSE_OFFSET;
      rsp[1]   = sub;
      return 2;
    }

    default:
      return negative(rsp, SID_SECURITY_ACCESS, NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
}

static size_t write_data_by_identifier(const uint8_t* req, size_t len, uint8_t* rsp) {
  if (len < 4) return negative(rsp, SID_WRITE_DATA_BY_IDENTIFIER, NRC_INCORRECT_LENGTH);
  if (security != SECURITY_UNLOCKED) {
    return negative(rsp, SID_WRITE_DATA_BY_IDENTIFIER, NRC_SECURITY_ACCESS_DENIED);
  }

  uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
  if (did != DID_CONFIGURATION) {
    return negative(rsp, SID_WRITE_DATA_BY_IDENTIFIER, NRC_REQUEST_OUT_OF_RANGE);
  }
  if (len != 4) return negative(rsp, SID_WRITE_DATA_BY_IDENTIFIER, NRC_INCORRECT_LENGTH);

  configuration = req[3];
  rsp[0] = SID_WRITE_DATA_BY_IDENTIFIER + POSITIVE_RESPONSE_OFFSET;
  rsp[1] = req[1];
  rsp[2] = req[2];
  return 3;
}

static size_t routine_control(const uint8_t* req, size_t len, uint8_t* rsp) {
  if (len < 4) return negative(rsp, SID_ROUTINE_CONTROL, NRC_INCORRECT_LENGTH);
  // The one routine here only reads state, so a non-default session is enough.
  // The bootloader's routines erase and CRC flash and require the security
  // unlock as well.
  if (session == SESSION_DEFAULT) {
    return negative(rsp, SID_ROUTINE_CONTROL, NRC_CONDITIONS_NOT_CORRECT);
  }

  uint8_t sub      = req[1];
  uint16_t routine = (uint16_t)((req[2] << 8) | req[3]);

  if (routine != ROUTINE_SELF_TEST) {
    return negative(rsp, SID_ROUTINE_CONTROL, NRC_REQUEST_OUT_OF_RANGE);
  }

  if (sub == ROUTINE_START) {
    self_test_result = 0;
    if (bl_reboot_exists()) self_test_result |= SELFTEST_REBOOT_DRIVER;
    if (bl_reboot_permitted()) self_test_result |= SELFTEST_REBOOT_PRIVILEGE;
    if (transport_healthy) self_test_result |= SELFTEST_TRANSPORT_OK;
  } else if (sub != ROUTINE_RESULTS) {
    return negative(rsp, SID_ROUTINE_CONTROL, NRC_SUBFUNCTION_NOT_SUPPORTED);
  }

  rsp[0] = SID_ROUTINE_CONTROL + POSITIVE_RESPONSE_OFFSET;
  rsp[1] = sub;
  rsp[2] = req[2];
  rsp[3] = req[3];
  rsp[4] = self_test_result;
  return 5;
}

static size_t tester_present(const uint8_t* req, size_t len, uint8_t* rsp) {
  if (len < 2) return negative(rsp, SID_TESTER_PRESENT, NRC_INCORRECT_LENGTH);
  // The S3 timer has already been refreshed by the dispatcher, which is the
  // whole point of the service; with the suppress bit set, say nothing.
  if (req[1] & 0x80) return 0;

  rsp[0] = SID_TESTER_PRESENT + POSITIVE_RESPONSE_OFFSET;
  rsp[1] = 0x00;
  return 2;
}

// -- Dispatch ---------------------------------------------------------------

size_t uds_handle(const uint8_t* req, size_t req_len, uint8_t* rsp, size_t rsp_capacity) {
  // Every response this server produces is well under a first frame, let alone
  // the transport's limit; the check is here so a future service cannot
  // silently overrun the caller's buffer.
  if (req_len == 0 || rsp_capacity < 16) return 0;

  // Any request from the tester counts as presence, not only 0x3E.
  last_request_ticks = now_ticks();

  switch (req[0]) {
    case SID_DIAGNOSTIC_SESSION_CONTROL: return session_control(req, req_len, rsp);
    case SID_ECU_RESET:                  return ecu_reset(req, req_len, rsp);
    case SID_READ_DATA_BY_IDENTIFIER:    return read_data_by_identifier(req, req_len, rsp);
    case SID_SECURITY_ACCESS:            return security_access(req, req_len, rsp);
    case SID_WRITE_DATA_BY_IDENTIFIER:   return write_data_by_identifier(req, req_len, rsp);
    case SID_ROUTINE_CONTROL:            return routine_control(req, req_len, rsp);
    case SID_TESTER_PRESENT:             return tester_present(req, req_len, rsp);
    default:                             return negative(rsp, req[0], NRC_SERVICE_NOT_SUPPORTED);
  }
}

void uds_cancel_pending(void) {
  // The session change stands -- the tester may simply have missed the
  // response and will retry -- but the restart does not happen, because an
  // unannounced reset is indistinguishable from a crash.
  pending_action = ACTION_NONE;
}

void uds_post_response(void) {
  action_t action = pending_action;
  pending_action  = ACTION_NONE;

  switch (action) {
    case ACTION_ENTER_BOOTLOADER:
      // A short pause so the console line describing this reaches the host
      // before the reset truncates it. The CAN response has already left.
      libtocksync_alarm_delay_ms(50);
      bl_reboot_to_bootloader();
      break;
    case ACTION_RESET:
      libtocksync_alarm_delay_ms(50);
      bl_reboot_restart();
      break;
    case ACTION_NONE:
      break;
  }
}
