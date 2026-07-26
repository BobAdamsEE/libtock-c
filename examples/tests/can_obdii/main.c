/* CAN App to scan for supported OBDII PIDs and write to EEPROM.
 *
 *
 */

#include <stddef.h>
#include <string.h>
#include <libtock/tock.h>
#include <libtock/interface/console.h>
#include <libtock-sync/services/alarm.h>

// ISO 15765-4 P2_CAN_max = 50 ms; we allow 10× for slow ECUs.
#define RX_TIMEOUT_MS      500
// ISO 15765-4 P2*_CAN_max = 5000 ms; used after a 0x78 response-pending NRC.
#define RX_TIMEOUT_PEND_MS 5000
// Maximum number of consecutive 0x78 response-pending frames before giving up.
#define RX_PENDING_MAX     10

#define CAN_DRIVER_NUM  0x20007

#define CMD_SET_BITRATE      1
#define CMD_SET_OP_MODE      2
#define CMD_ENABLE           3
#define CMD_DISABLE          4
#define CMD_SEND_STD         5
#define CMD_SEND_EXT         6
#define CMD_START_RECEIVE    7
#define CMD_STOP_RECEIVE     8

#define SUB_ENABLE           0
#define SUB_DISABLE          1
#define SUB_MESSAGE_SENT     2
#define SUB_MESSAGE_RECEIVED 3
#define SUB_RECEIVED_STOPPED 4
#define SUB_TX_ERROR         5

#define OP_NORMAL     3

typedef struct { uint8_t pid; const char *desc; } pid_desc_t;

static const pid_desc_t pid_table[] = {
  {0x01, "Monitor status since DTCs cleared"},
  {0x02, "Freeze DTC"},
  {0x03, "Fuel system status"},
  {0x04, "Calculated engine load"},
  {0x05, "Engine coolant temperature"},
  {0x06, "Short term fuel trim (Bank 1)"},
  {0x07, "Long term fuel trim (Bank 1)"},
  {0x08, "Short term fuel trim (Bank 2)"},
  {0x09, "Long term fuel trim (Bank 2)"},
  {0x0A, "Fuel pressure"},
  {0x0B, "Intake manifold absolute pressure"},
  {0x0C, "Engine RPM"},
  {0x0D, "Vehicle speed"},
  {0x0E, "Timing advance"},
  {0x0F, "Intake air temperature"},
  {0x10, "Mass air flow rate"},
  {0x11, "Throttle position"},
  {0x12, "Commanded secondary air status"},
  {0x13, "Oxygen sensors present (2 banks)"},
  {0x14, "O2 Sensor 1 voltage / short term fuel trim"},
  {0x15, "O2 Sensor 2 voltage / short term fuel trim"},
  {0x16, "O2 Sensor 3 voltage / short term fuel trim"},
  {0x17, "O2 Sensor 4 voltage / short term fuel trim"},
  {0x18, "O2 Sensor 5 voltage / short term fuel trim"},
  {0x19, "O2 Sensor 6 voltage / short term fuel trim"},
  {0x1A, "O2 Sensor 7 voltage / short term fuel trim"},
  {0x1B, "O2 Sensor 8 voltage / short term fuel trim"},
  {0x1C, "OBD standards this vehicle conforms to"},
  {0x1D, "Oxygen sensors present (4 banks)"},
  {0x1E, "Auxiliary input status"},
  {0x1F, "Run time since engine start"},
  {0x20, "PIDs supported 0x21-0x40"},
  {0x21, "Distance traveled with MIL on"},
  {0x22, "Fuel rail pressure (relative to manifold vacuum)"},
  {0x23, "Fuel rail gauge pressure"},
  {0x24, "O2 Sensor 1 air-fuel equivalence ratio / voltage"},
  {0x25, "O2 Sensor 2 air-fuel equivalence ratio / voltage"},
  {0x26, "O2 Sensor 3 air-fuel equivalence ratio / voltage"},
  {0x27, "O2 Sensor 4 air-fuel equivalence ratio / voltage"},
  {0x28, "O2 Sensor 5 air-fuel equivalence ratio / voltage"},
  {0x29, "O2 Sensor 6 air-fuel equivalence ratio / voltage"},
  {0x2A, "O2 Sensor 7 air-fuel equivalence ratio / voltage"},
  {0x2B, "O2 Sensor 8 air-fuel equivalence ratio / voltage"},
  {0x2C, "Commanded EGR"},
  {0x2D, "EGR error"},
  {0x2E, "Commanded evaporative purge"},
  {0x2F, "Fuel tank level input"},
  {0x30, "Warm-ups since codes cleared"},
  {0x31, "Distance traveled since codes cleared"},
  {0x32, "Evap system vapor pressure"},
  {0x33, "Absolute barometric pressure"},
  {0x34, "O2 Sensor 1 air-fuel equivalence ratio / current"},
  {0x35, "O2 Sensor 2 air-fuel equivalence ratio / current"},
  {0x36, "O2 Sensor 3 air-fuel equivalence ratio / current"},
  {0x37, "O2 Sensor 4 air-fuel equivalence ratio / current"},
  {0x38, "O2 Sensor 5 air-fuel equivalence ratio / current"},
  {0x39, "O2 Sensor 6 air-fuel equivalence ratio / current"},
  {0x3A, "O2 Sensor 7 air-fuel equivalence ratio / current"},
  {0x3B, "O2 Sensor 8 air-fuel equivalence ratio / current"},
  {0x3C, "Catalyst temperature (Bank 1, Sensor 1)"},
  {0x3D, "Catalyst temperature (Bank 2, Sensor 1)"},
  {0x3E, "Catalyst temperature (Bank 1, Sensor 2)"},
  {0x3F, "Catalyst temperature (Bank 2, Sensor 2)"},
  {0x40, "PIDs supported 0x41-0x60"},
  {0x41, "Monitor status this drive cycle"},
  {0x42, "Control module voltage"},
  {0x43, "Absolute load value"},
  {0x44, "Commanded air-fuel equivalence ratio"},
  {0x45, "Relative throttle position"},
  {0x46, "Ambient air temperature"},
  {0x47, "Absolute throttle position B"},
  {0x48, "Absolute throttle position C"},
  {0x49, "Accelerator pedal position D"},
  {0x4A, "Accelerator pedal position E"},
  {0x4B, "Accelerator pedal position F"},
  {0x4C, "Commanded throttle actuator"},
  {0x4D, "Time run with MIL on"},
  {0x4E, "Time since trouble codes cleared"},
  {0x4F, "Maximum sensor values (fuel-air, O2 voltage, O2 current, intake pressure)"},
  {0x50, "Maximum mass air flow sensor value"},
  {0x51, "Fuel type"},
  {0x52, "Ethanol fuel percentage"},
  {0x53, "Absolute evap system vapor pressure"},
  {0x54, "Evap system vapor pressure"},
  {0x55, "Short term secondary O2 sensor trim (Bank 1, 3)"},
  {0x56, "Long term secondary O2 sensor trim (Bank 1, 3)"},
  {0x57, "Short term secondary O2 sensor trim (Bank 2, 4)"},
  {0x58, "Long term secondary O2 sensor trim (Bank 2, 4)"},
  {0x59, "Fuel rail absolute pressure"},
  {0x5A, "Relative accelerator pedal position"},
  {0x5B, "Hybrid battery pack remaining life"},
  {0x5C, "Engine oil temperature"},
  {0x5D, "Fuel injection timing"},
  {0x5E, "Engine fuel rate"},
  {0x5F, "Emission requirements this vehicle is designed to"},
  {0x60, "PIDs supported 0x61-0x80"},
  {0x61, "Driver's demand engine torque"},
  {0x62, "Actual engine torque"},
  {0x63, "Engine reference torque"},
  {0x64, "Engine percent torque data"},
  {0x65, "Auxiliary input / output supported"},
  {0x66, "Mass air flow sensor (alternate)"},
  {0x67, "Engine coolant temperature (alternate sensors)"},
  {0x68, "Intake air temperature sensor (alternate)"},
  {0x69, "Commanded EGR and EGR error (alternate)"},
  {0x6A, "Commanded diesel intake air flow control"},
  {0x6B, "Exhaust gas recirculation temperature"},
  {0x6C, "Commanded throttle actuator control (alternate)"},
  {0x6D, "Fuel pressure control system"},
  {0x6E, "Injection pressure control system"},
  {0x6F, "Turbocharger compressor inlet pressure"},
  {0x70, "Boost pressure control"},
  {0x71, "Variable geometry turbo control"},
  {0x72, "Wastegate control"},
  {0x73, "Exhaust pressure"},
  {0x74, "Turbocharger RPM"},
  {0x75, "Turbocharger temperature (Bank 1)"},
  {0x76, "Turbocharger temperature (Bank 2)"},
  {0x77, "Charge air cooler temperature"},
  {0x78, "Exhaust gas temperature (Bank 1)"},
  {0x79, "Exhaust gas temperature (Bank 2)"},
  {0x7A, "Diesel particulate filter (Bank 1)"},
  {0x7B, "Diesel particulate filter (Bank 2)"},
  {0x7C, "Diesel particulate filter temperature"},
  {0x7D, "NOx NTE control area status"},
  {0x7E, "PM NTE control area status"},
  {0x7F, "Engine run time"},
  {0x80, "PIDs supported 0x81-0xA0"},
  {0x81, "Engine run time for AECD (1-5)"},
  {0x82, "Engine run time for AECD (6-10)"},
  {0x83, "NOx sensor"},
  {0x84, "Manifold surface temperature"},
  {0x85, "NOx reagent system"},
  {0x86, "Particulate matter sensor"},
  {0x87, "Intake manifold absolute pressure (alternate)"},
  {0x88, "SCR induce system"},
  {0x89, "Engine run time for AECD (11-15)"},
  {0x8A, "Engine run time for AECD (16-20)"},
  {0x8B, "Diesel aftertreatment"},
  {0x8C, "O2 sensor (wide range)"},
  {0x8D, "Throttle position G"},
  {0x8E, "Engine friction percent torque"},
  {0x8F, "PM sensor (Bank 1 and 2)"},
  {0x90, "WWH-OBD vehicle OBD system information (Bank 1)"},
  {0x91, "WWH-OBD vehicle OBD system information (Bank 2)"},
  {0x92, "Fuel system control"},
  {0x93, "WWH-OBD vehicle OBD counters support"},
  {0x94, "NOx warning and inducement system"},
  {0x98, "Exhaust gas temperature sensor (alternate, Bank 1)"},
  {0x99, "Exhaust gas temperature sensor (alternate, Bank 2)"},
  {0x9A, "Hybrid/EV vehicle system data"},
  {0x9B, "Diesel exhaust fluid sensor data"},
  {0x9C, "O2 sensor data (alternate)"},
  {0x9D, "Engine fuel rate (alternate)"},
  {0x9E, "Engine exhaust flow rate"},
  {0x9F, "Fuel system percentage use"},
  {0xA0, "PIDs supported 0xA1-0xC0"},
  {0xA1, "NOx sensor corrected data"},
  {0xA2, "Cylinder fuel rate"},
  {0xA3, "Evap system vapor pressure (alternate)"},
  {0xA4, "Transmission actual gear"},
  {0xA5, "Diesel DEF dosing"},
  {0xA6, "Odometer"},
  {0xC0, "PIDs supported 0xC1-0xE0"},
  {0xC3, "Fuel level input (alternate)"},
  {0xC4, "Exhaust particulate control system diagnostic time/count"},
  {0xC8, "SCR catalyst storage level"},
};
#define PID_TABLE_LEN (sizeof(pid_table) / sizeof(pid_table[0]))

static const char* pid_description(uint8_t pid) {
  for (size_t i = 0; i < PID_TABLE_LEN; i++) {
    if (pid_table[i].pid == pid) return pid_table[i].desc;
  }
  return "Unknown";
}

static volatile bool     enabled_done;
static volatile bool     disabled_done;
static volatile bool     tx_done;
static volatile bool     rx_done;
static volatile bool     stopped_done;
static volatile uint32_t enable_status;
static volatile uint32_t tx_status;
static volatile uint32_t rx_id;

static uint8_t rx_buf[4 + 8 * 8] __attribute__((aligned(8)));
static uint8_t tx_buf[8] __attribute__((aligned(4)));

static void reset_rx_buf(void) {
  memset(rx_buf, 0, sizeof(rx_buf));
  allow_readwrite(CAN_DRIVER_NUM, 0, rx_buf, sizeof(rx_buf));
}

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

static void disabled_cb(int a1 __attribute__((unused)), int a2 __attribute__((unused)),
                        int a3 __attribute__((unused)), void* ud __attribute__((unused))) {
  disabled_done = true;
}

static void error_cb(int a1 __attribute__((unused)), int a2 __attribute__((unused)),
                     int a3 __attribute__((unused)), void* ud __attribute__((unused))) {
  tx_status = 1;
  tx_done   = true;
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
  subscribe(CAN_DRIVER_NUM, SUB_DISABLE, disabled_cb, NULL);
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


  ret = can_cmd(CMD_SET_OP_MODE, OP_NORMAL, 0);
  print("[CAN] Mode: Normal (external bus)\r\n");

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

// Send service 01 request for range_pid (0x00, 0x20, 0x40, ...) and parse the
// 32-bit bitmask from the ECU response. Returns false on any error.
static bool query_range(uint8_t range_pid, uint32_t *mask_out) {
  returncode_t ret;

  memset(rx_buf, 0, sizeof(rx_buf));
  allow_readwrite(CAN_DRIVER_NUM, 0, rx_buf, sizeof(rx_buf));
  rx_done = false;

  tx_buf[0] = 0x02; tx_buf[1] = 0x01; tx_buf[2] = range_pid;
  tx_buf[3] = 0; tx_buf[4] = 0; tx_buf[5] = 0; tx_buf[6] = 0; tx_buf[7] = 0;
  allow_readonly(CAN_DRIVER_NUM, 0, tx_buf, sizeof(tx_buf));

  tx_done = false;
  ret = can_cmd(CMD_SEND_EXT, 0x18DB33F1, 8);
  if (ret != RETURNCODE_SUCCESS) { print("[TX] send failed\r\n"); return false; }
  int tx_rc = libtocksync_alarm_yield_for_with_timeout((bool*)&tx_done, RX_TIMEOUT_MS);
  if (tx_rc != RETURNCODE_SUCCESS) { print("[TX] timeout (no ACK — bus empty?)\r\n"); return false; }
  if (tx_status != 0) { print("[TX] error\r\n"); return false; }

  // Wait for ECU response, retrying on 0x78 response-pending NRC.
  // Hardware filter passes 0x700–0x7FF (standard) and all extended frames.
  // We log and skip anything outside the standard OBD-II diagnostic range.
  uint8_t *msg = &rx_buf[8]; // StreamingProcessSlice header is 8 bytes
  int pending = 0;
  int skipped = 0;
  int timeout_ms = RX_TIMEOUT_MS;
#define MAX_SKIP 300
  while (1) {
    int rc = libtocksync_alarm_yield_for_with_timeout((bool*)&rx_done, timeout_ms);
    if (rc != RETURNCODE_SUCCESS) {
      print("[RX] timeout (no ECU response)\r\n");
      return false;
    }

    // Accept only 29-bit OBD-II responses: upper 24 bits must be 0x18DAF1.
    // (The "literal pool constants trigger LDRD" note that used to live here was
    // a misdiagnosis; literal pools are always word-aligned. Kept as-is only
    // because the shift form is equally clear.)
    // Also reset the rx_buf streaming slice on each skip so non-OBD extended
    // frames don't fill the 60-byte payload before the ECU response arrives.
    if (((rx_id >> 8) & 0xFFFFFFu) != 0x18DAF1u) {
      rx_done = false;
      reset_rx_buf();
      if (++skipped > MAX_SKIP) {
        print("[RX] no OBD response after many frames\r\n");
        return false;
      }
      continue;
    }

    // 0x7F = negative response; 0x78 NRC = response pending, real answer coming
    if (msg[1] == 0x7F && msg[3] == 0x78) {
      pending++;
      if (pending > RX_PENDING_MAX) {
        print("[RX] too many response-pending (0x78)\r\n");
        return false;
      }
      rx_done = false;
      timeout_ms = RX_TIMEOUT_PEND_MS;
      continue;
    }

    break;
  }

  // Normal addressing: msg[0]=len, msg[1]=0x41, msg[2]=pid, msg[3..6]=mask
  if (msg[1] != 0x41 || msg[2] != range_pid) {
    print("[RX] unexpected response\r\n");
    return false;
  }

  // 4-byte bitmask: bit 31 = range_pid+1, bit 0 = range_pid+0x20
  *mask_out = ((uint32_t)msg[3] << 24) | ((uint32_t)msg[4] << 16)
            | ((uint32_t)msg[5] << 8)  |  (uint32_t)msg[6];
  return true;
}

static void print_pid(uint8_t pid) {
  print("  PID 0x");
  print_hex_byte(pid);
  print(" - ");
  print(pid_description(pid));
  print("\r\n");
}

static void scan_obdii(void) {
  returncode_t ret;

  memset(rx_buf, 0, sizeof(rx_buf));
  allow_readwrite(CAN_DRIVER_NUM, 0, rx_buf, sizeof(rx_buf));

  ret = can_cmd(CMD_START_RECEIVE, 0, 0);
  if (ret != RETURNCODE_SUCCESS) { print("[CAN] Start RX FAILED\r\n"); return; }

  print("[OBD] Scanning supported PIDs...\r\n\r\n");

  uint8_t range_pid = 0x00;
  uint32_t mask;
  do {
    if (!query_range(range_pid, &mask)) break;

    for (int i = 1; i <= 32; i++) {
      if (mask & (1u << (32 - i))) {
        print_pid(range_pid + (uint8_t)i);
      }
    }

    // Bit 0 of the bitmask indicates the next range (range_pid+0x20) is available
    range_pid += 0x20;
  } while ((mask & 0x01) && range_pid <= 0xC0);
}

int main(void) {
  print("\r\n=== CAN OBDII PID SCAN ===\r\n\r\n");

  if (setup_can() != 0) return -1;

  scan_obdii();

  // Shutdown. Every wait here is bounded and every command's return is
  // checked: a driver that declines a command never schedules the matching
  // callback, so an unconditional yield_for() would block forever. That is
  // exactly what used to happen below -- the bare yield() after CMD_DISABLE
  // waited for an upcall on SUB_DISABLE, which this app had never subscribed
  // to, so the process slept forever and never reached tock_exit().
  stopped_done = false;
  if (can_cmd(CMD_STOP_RECEIVE, 0, 0) != RETURNCODE_SUCCESS) {
    print("[CAN] Stop RX rejected\r\n");
  } else if (libtocksync_alarm_yield_for_with_timeout((bool*)&stopped_done,
                                                     RX_TIMEOUT_MS) != RETURNCODE_SUCCESS) {
    print("[CAN] Stop RX callback timeout\r\n");
  }

  disabled_done = false;
  if (can_cmd(CMD_DISABLE, 0, 0) != RETURNCODE_SUCCESS) {
    print("[CAN] Disable rejected\r\n");
  } else if (libtocksync_alarm_yield_for_with_timeout((bool*)&disabled_done,
                                                      RX_TIMEOUT_MS) != RETURNCODE_SUCCESS) {
    print("[CAN] Disable callback timeout\r\n");
  }

  print("\r\n=== Test Complete ===\r\n");

  // Use tock_exit() instead of returning from main(). Returning would invoke
  // newlib's exit() cleanup, which tries to flush stdio by accessing the
  // _reent struct at its linker VMA (e.g. 0x00000EA4). The MPU blocks that
  // address at runtime -> DACCVIOL. tock_exit() is a direct syscall that
  // terminates the process before any C library teardown runs.
  tock_exit(0);
}
