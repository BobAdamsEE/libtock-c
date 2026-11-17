// Interactive SD card shell over the console UART (EDBG virtual COM port,
// 115200 8N1). Sends any SD command and shows the card's response, decoded,
// with the data it returned; also reads, writes and erases blocks through the
// kernel driver, and edits a 32 KiB data buffer to write from.
//
// Writes, erases and raw commands need a kernel built with the sd_write
// feature (boards/sam_v71_xult: .\build.ps1 -Features sd_write). On the
// default kernel the shell still reads, but every raw command fails with
// ENOSUPPORT.
//
// The shell starts locked: commands that can change the card's contents or
// protection -- write, erase, any raw command with a write data phase, and
// raw CMD28/29/38 -- are refused until `unlock`. The prompt shows the state.
//
// Type commands, don't paste them: input is read a character at a time, and
// characters that arrive while the kernel is busy with the card are lost.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <libtock-sync/interface/console.h>
#include <libtock-sync/services/alarm.h>

#include "sdmmc.h"

#define MAX_BLOCKS 64
#define BUF_LEN (MAX_BLOCKS * SDMMC_BLOCK_SIZE)

// Shown in full after a read; the rest stays in the buffer for `dump`.
#define SHOW_LEN 512

static uint8_t buf[BUF_LEN] __attribute__((aligned(4)));
static uint8_t tmp[8 * SDMMC_BLOCK_SIZE] __attribute__((aligned(4)));

static bool unlocked = false;
static uint16_t rca = 0;            // from init or a raw CMD3
static bool high_capacity = false;  // from init: block, not byte, addresses
static uint32_t block_count = 0;    // from init
static bool initialized = false;
static uint32_t max_blocks = MAX_BLOCKS;

// ---------------------------------------------------------------------------
// Output helpers
// ---------------------------------------------------------------------------

static void hexdump(const uint8_t* p, uint32_t len, uint32_t base) {
  for (uint32_t off = 0; off < len; off += 16) {
    printf("%05lX ", (unsigned long)(base + off));
    for (uint32_t i = 0; i < 16; i++) {
      if (off + i < len) printf(" %02X", p[off + i]);
      else printf("   ");
      if (i == 7) printf(" ");
    }
    printf("  |");
    for (uint32_t i = 0; i < 16 && off + i < len; i++) {
      uint8_t c = p[off + i];
      putchar(c >= 0x20 && c < 0x7F ? c : '.');
    }
    printf("|\r\n");
  }
  fflush(stdout);
}

static const char* const STATES[16] = {
  "idle", "ready", "ident", "stby", "tran", "data", "rcv", "prg",
  "dis", "9", "10", "11", "12", "13", "14", "15",
};

// R1 card status bits, SD Physical Layer Simplified Specification 4.10.1.
static const struct {
  uint8_t bit;
  const char* name;
} R1_BITS[] = {
  {31, "OUT_OF_RANGE"}, {30, "ADDRESS_ERROR"}, {29, "BLOCK_LEN_ERROR"},
  {28, "ERASE_SEQ_ERROR"}, {27, "ERASE_PARAM"}, {26, "WP_VIOLATION"},
  {25, "CARD_IS_LOCKED"}, {24, "LOCK_UNLOCK_FAILED"}, {23, "COM_CRC_ERROR"},
  {22, "ILLEGAL_COMMAND"}, {21, "CARD_ECC_FAILED"}, {20, "CC_ERROR"},
  {19, "ERROR"}, {16, "CSD_OVERWRITE"}, {15, "WP_ERASE_SKIP"},
  {14, "CARD_ECC_DISABLED"}, {13, "ERASE_RESET"}, {8, "READY_FOR_DATA"},
  {6, "FX_EVENT"}, {5, "APP_CMD"}, {3, "AKE_SEQ_ERROR"},
};

static void print_r1(uint32_t r) {
  printf("  R1 %08lX  state=%s", (unsigned long)r, STATES[(r >> 9) & 0xF]);
  for (unsigned i = 0; i < sizeof(R1_BITS) / sizeof(R1_BITS[0]); i++) {
    if (r & (1u << R1_BITS[i].bit)) printf(" %s", R1_BITS[i].name);
  }
  printf("\r\n");
}

static void print_sr(uint32_t sr, bool no_crc) {
  static const struct {
    uint8_t bit;
    const char* name;
  } SR_BITS[] = {
    {16, "RINDE"}, {17, "RDIRE"}, {18, "RCRCE"}, {19, "RENDE"}, {20, "RTOE"},
    {21, "DCRCE"}, {22, "DTOE"}, {23, "CSTOE"}, {30, "OVRE"}, {31, "UNRE"},
  };
  printf("  SR %08lX", (unsigned long)sr);
  for (unsigned i = 0; i < sizeof(SR_BITS) / sizeof(SR_BITS[0]); i++) {
    if (sr & (1u << SR_BITS[i].bit)) printf(" %s", SR_BITS[i].name);
  }
  if (no_crc && (sr & (1u << 18))) printf("  (RCRCE expected: R3 has no CRC)");
  if (sr & (1u << 20)) printf("  (no response: wrong state, no card, or a command the card ignores)");
  printf("\r\n");
}

static void print_ocr(uint32_t ocr) {
  printf("  OCR %08lX  %s%s%s%s window", (unsigned long)ocr, ocr & (1u << 31) ? "powered-up " : "BUSY ",
         ocr & (1u << 31) ? (ocr & (1u << 30) ? "CCS=1(SDHC/SDXC) " : "CCS=0(SDSC) ") : "",
         ocr & (1u << 29) ? "UHS-II " : "", ocr & (1u << 24) ? "S18A " : "");
  for (int b = 15; b <= 23; b++) {
    if (ocr & (1u << b)) printf(" %d.%d-%d.%d", (b - 15 + 27) / 10, (b - 15 + 27) % 10, (b - 15 + 28) / 10,
                                (b - 15 + 28) % 10);
  }
  printf(" V\r\n");
}

// R2 words to bytes, most significant first.
static void r2_bytes(const uint32_t* w, uint8_t* out) {
  for (int i = 0; i < 4; i++) {
    out[4 * i] = w[i] >> 24;
    out[4 * i + 1] = w[i] >> 16;
    out[4 * i + 2] = w[i] >> 8;
    out[4 * i + 3] = w[i];
  }
}

// Bits [msb:lsb] of a 128-bit register image, MSB first.
static uint32_t bits128(const uint8_t* r, int msb, int lsb) {
  uint32_t v = 0;
  for (int bit = msb; bit >= lsb; bit--) v = (v << 1) | ((r[15 - bit / 8] >> (bit % 8)) & 1);
  return v;
}

static void print_cid(const uint8_t* cid) {
  printf("  CID: MID %02X OID %c%c PNM %c%c%c%c%c PRV %u.%u PSN %08lX MDT %lu-%02lu\r\n", cid[0], cid[1], cid[2],
         cid[3], cid[4], cid[5], cid[6], cid[7], cid[8] >> 4, cid[8] & 0xF,
         (unsigned long)((uint32_t)cid[9] << 24 | (uint32_t)cid[10] << 16 | (uint32_t)cid[11] << 8 | cid[12]),
         (unsigned long)(2000 + bits128(cid, 19, 12)), (unsigned long)bits128(cid, 11, 8));
}

static void print_csd(const uint8_t* csd) {
  uint32_t ver = bits128(csd, 127, 126);
  uint64_t bytes = 0;
  if (ver == 0) {
    bytes = ((uint64_t)bits128(csd, 73, 62) + 1) << (bits128(csd, 49, 47) + 2) << bits128(csd, 83, 80);
  } else if (ver == 1) {
    bytes = ((uint64_t)bits128(csd, 69, 48) + 1) * 512 * 1024;
  }
  printf("  CSD: version %lu.0, %lu MiB, CCC %03lX, READ_BL_LEN %lu, WP perm %lu tmp %lu\r\n",
         (unsigned long)ver + 1, (unsigned long)(bytes >> 20), (unsigned long)bits128(csd, 95, 84),
         (unsigned long)bits128(csd, 83, 80), (unsigned long)bits128(csd, 13, 13),
         (unsigned long)bits128(csd, 12, 12));
}

static void print_error_info(void) {
  sdmmc_error_t e;
  if (sdmmc_error_info(0, &e) != RETURNCODE_SUCCESS) return;
  static const char* const STAGES[] = {"none", "response", "card status", "data", "busy", "stuck"};
  printf("  driver: CMD%u failed at %s stage, card state %s\r\n", e.cmd, e.stage < 6 ? STAGES[e.stage] : "?",
         e.card_state == 0xFF ? "unknown" : STATES[e.card_state & 0xF]);
  print_sr(e.sr, false);
  if (!(e.sr & (1u << 20))) print_r1(e.r1);
}

static void report_rc(const char* what, returncode_t rc) {
  if (rc == RETURNCODE_SUCCESS) return;
  printf("%s: %s\r\n", what, tock_strrcode(rc));
  if (rc == RETURNCODE_ENOSUPPORT) printf("  (kernel built without the sd_write feature?)\r\n");
  if (rc == RETURNCODE_EOFF) printf("  (run `init` first)\r\n");
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

#define MAX_TOKENS 12
static char* tok[MAX_TOKENS];
static int ntok;

static void tokenize(char* line) {
  ntok = 0;
  char* p = line;
  while (*p && ntok < MAX_TOKENS) {
    while (*p == ' ' || *p == '\t') p++;
    if (!*p) break;
    tok[ntok++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = 0;
  }
}

// Decimal, or hex with 0x. Returns false if `s` is not a number.
static bool parse_u32(const char* s, uint32_t* out) {
  uint32_t v = 0;
  int base = 10;
  if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    base = 16;
    s += 2;
  }
  if (!*s) return false;
  for (; *s; s++) {
    uint32_t d;
    if (*s >= '0' && *s <= '9') d = *s - '0';
    else if (base == 16 && *s >= 'a' && *s <= 'f') d = *s - 'a' + 10;
    else if (base == 16 && *s >= 'A' && *s <= 'F') d = *s - 'A' + 10;
    else return false;
    v = v * base + d;
  }
  *out = v;
  return true;
}

// Token `i` as a number, or `def` if absent. False if present but not a number.
static bool arg_u32(int i, uint32_t def, uint32_t* out) {
  if (i >= ntok) {
    *out = def;
    return true;
  }
  if (parse_u32(tok[i], out)) return true;
  printf("not a number: %s\r\n", tok[i]);
  return false;
}

// ---------------------------------------------------------------------------
// Raw commands
// ---------------------------------------------------------------------------

// What a command index normally takes, so `cmd 17 0` needs nothing more.
static void defaults(bool app, uint8_t idx, uint32_t arg, sdmmc_raw_t* c) {
  c->rsp = SDMMC_RSP_R1;
  c->data = SDMMC_DATA_NONE;
  c->block_len = 512;
  c->block_count = 1;
  c->flags = 0;
  if (app) {
    switch (idx) {
      case 13: c->data = SDMMC_DATA_READ; c->block_len = 64; break;  // SD_STATUS
      case 22: c->data = SDMMC_DATA_READ; c->block_len = 4; break;   // SEND_NUM_WR_BLOCKS
      case 41: c->rsp = SDMMC_RSP_R3; break;                         // SD_SEND_OP_COND
      case 51: c->data = SDMMC_DATA_READ; c->block_len = 8; break;   // SEND_SCR
    }
    return;
  }
  switch (idx) {
    case 0: case 4: case 15: c->rsp = SDMMC_RSP_NONE; break;
    case 2: case 9: case 10: c->rsp = SDMMC_RSP_R2; break;
    case 5: c->rsp = SDMMC_RSP_R3; break;
    case 6: c->data = SDMMC_DATA_READ; c->block_len = 64; break;  // SWITCH_FUNC status
    case 7: c->rsp = arg ? SDMMC_RSP_R1B : SDMMC_RSP_NONE; break;  // deselect never answers
    case 12: case 20: case 28: case 29: case 38: c->rsp = SDMMC_RSP_R1B; break;
    case 17: case 48: c->data = SDMMC_DATA_READ; break;
    case 18: case 58: c->data = SDMMC_DATA_READ; c->flags = SDMMC_RAW_STOP; break;
    case 19: c->data = SDMMC_DATA_READ; c->block_len = 64; break;  // SEND_TUNING_BLOCK
    case 24: case 49: c->data = SDMMC_DATA_WRITE; break;
    case 25: case 59: c->data = SDMMC_DATA_WRITE; c->flags = SDMMC_RAW_STOP; break;
    case 27: c->data = SDMMC_DATA_WRITE; c->block_len = 16; break;  // PROGRAM_CSD
    case 30: c->data = SDMMC_DATA_READ; c->block_len = 4; break;    // SEND_WRITE_PROT
    case 42: c->data = SDMMC_DATA_WRITE; break;                     // LOCK_UNLOCK: set len to CMD16's
    case 56: c->data = (arg & 1) ? SDMMC_DATA_READ : SDMMC_DATA_WRITE; break;  // GEN_CMD
  }
}

static bool modifies_card(bool app, const sdmmc_raw_t* c) {
  if (c->data == SDMMC_DATA_WRITE) return true;
  return !app && (c->index == 28 || c->index == 29 || c->index == 38);
}

static void print_response(bool app, const sdmmc_raw_t* c) {
  switch (c->rsp) {
    case SDMMC_RSP_NONE:
      printf("  (no response expected)\r\n");
      break;
    case SDMMC_RSP_R2: {
      uint8_t r[16];
      r2_bytes(c->resp, r);
      printf("  R2 ");
      for (int i = 0; i < 16; i++) printf("%02X", r[i]);
      printf("\r\n");
      if (!app && c->index == 2) print_cid(r);
      if (!app && c->index == 10) print_cid(r);
      if (!app && c->index == 9) print_csd(r);
      break;
    }
    case SDMMC_RSP_R3:
      print_ocr(c->resp[0]);
      break;
    default:
      if (!app && c->index == 3) {
        // R6: RCA, then status bits 23, 22, 19 and 12:0.
        uint32_t r = c->resp[0];
        uint32_t st = ((r & 0x8000) << 8) | ((r & 0x4000) << 8) | ((r & 0x2000) << 6) | (r & 0x1FFF);
        printf("  R6 %08lX  RCA %04lX\r\n", (unsigned long)r, (unsigned long)(r >> 16));
        print_r1(st);
      } else if (!app && c->index == 8) {
        uint32_t r = c->resp[0];
        printf("  R7 %08lX  voltage %s, check pattern %02lX\r\n", (unsigned long)r,
               ((r >> 8) & 0xF) == 1 ? "2.7-3.6 V accepted" : "NOT accepted", (unsigned long)(r & 0xFF));
      } else {
        print_r1(c->resp[0]);
      }
  }
  if (c->flags & SDMMC_RAW_STOP) {
    printf("  CMD12:");
    print_r1(c->stop_resp);
  }
}

// Send one raw command and print what happened. Returns its result.
static returncode_t run_raw(bool app, sdmmc_raw_t* c) {
  uint8_t* data = c->data == SDMMC_DATA_NONE ? NULL : buf;
  returncode_t rc = sdmmc_raw(c, data);
  printf("%sCMD%u arg %08lX: %s\r\n", app ? "A" : "", c->index, (unsigned long)c->arg,
         rc == RETURNCODE_SUCCESS ? "ok" : tock_strrcode(rc));
  if (rc == RETURNCODE_ENOSUPPORT) {
    printf("  (kernel built without the sd_write feature)\r\n");
    return rc;
  }
  if (rc == RETURNCODE_EBUSY || rc == RETURNCODE_EINVAL || rc == RETURNCODE_ESIZE) return rc;
  // On a response timeout the response register holds nothing of this
  // command's, so decoding it would only mislead.
  if (c->sr & (1u << 20)) printf("  no response\r\n");
  else print_response(app, c);
  print_sr(c->sr, c->rsp == SDMMC_RSP_R3);
  if (c->data != SDMMC_DATA_NONE) {
    printf("  data: %lu of %lu bytes %s\r\n", (unsigned long)c->bytes,
           (unsigned long)c->block_len * c->block_count, c->data == SDMMC_DATA_READ ? "read into buffer" : "written");
    if (c->data == SDMMC_DATA_READ && c->bytes) {
      uint32_t n = c->bytes < SHOW_LEN ? c->bytes : SHOW_LEN;
      hexdump(buf, n, 0);
      if (c->bytes > n) printf("  (%lu more bytes: dump %lu %lu)\r\n", (unsigned long)(c->bytes - n),
                                 (unsigned long)n, (unsigned long)(c->bytes - n));
    }
  }
  if (rc == RETURNCODE_SUCCESS && !app && c->index == 3) rca = c->resp[0] >> 16;
  // Back in idle, the card answers CMD55 only at RCA 0.
  if (rc == RETURNCODE_SUCCESS && !app && c->index == 0) rca = 0;
  return rc;
}

// cmd/acmd <index> [arg] [none|r1|r1b|r2|r3] [rd|wr <len> [blocks]] [nodata] [stop|nostop]
static void cmd_raw(bool app) {
  uint32_t idx, arg;
  if (ntok < 2) {
    printf("usage: %s <index> [arg] [none|r1|r1b|r2|r3] [rd|wr <len> [blocks]] [nodata] [stop|nostop]\r\n",
           app ? "acmd" : "cmd");
    return;
  }
  if (!parse_u32(tok[1], &idx) || idx > 63) {
    printf("index must be 0-63\r\n");
    return;
  }
  int i = 2;
  arg = 0;
  if (i < ntok && parse_u32(tok[i], &arg)) i++;

  sdmmc_raw_t c = {0};
  c.index = idx;
  c.arg = arg;
  defaults(app, idx, arg, &c);

  for (; i < ntok; i++) {
    const char* t = tok[i];
    if (!strcmp(t, "none")) c.rsp = SDMMC_RSP_NONE;
    else if (!strcmp(t, "r1") || !strcmp(t, "r6") || !strcmp(t, "r7")) c.rsp = SDMMC_RSP_R1;
    else if (!strcmp(t, "r1b")) c.rsp = SDMMC_RSP_R1B;
    else if (!strcmp(t, "r2")) c.rsp = SDMMC_RSP_R2;
    else if (!strcmp(t, "r3") || !strcmp(t, "r4")) c.rsp = SDMMC_RSP_R3;
    else if (!strcmp(t, "nodata")) c.data = SDMMC_DATA_NONE;
    else if (!strcmp(t, "stop")) c.flags |= SDMMC_RAW_STOP;
    else if (!strcmp(t, "nostop")) c.flags &= ~SDMMC_RAW_STOP;
    else if (!strcmp(t, "rd") || !strcmp(t, "wr")) {
      c.data = t[0] == 'r' ? SDMMC_DATA_READ : SDMMC_DATA_WRITE;
      uint32_t len, n = 1;
      if (i + 1 >= ntok || !parse_u32(tok[i + 1], &len)) {
        printf("%s needs a block length\r\n", t);
        return;
      }
      i++;
      if (i + 1 < ntok && parse_u32(tok[i + 1], &n)) i++;
      if (len == 0 || len > 2048 || n == 0 || len * n > BUF_LEN) {
        printf("data must be 1-2048 bytes per block and fit the %u-byte buffer\r\n", BUF_LEN);
        return;
      }
      c.block_len = len;
      c.block_count = n;
    } else {
      printf("unknown option: %s\r\n", t);
      return;
    }
  }
  if (c.data == SDMMC_DATA_NONE) c.flags &= ~SDMMC_RAW_STOP;

  if (modifies_card(app, &c) && !unlocked) {
    printf("locked: %sCMD%lu can change the card; `unlock` first\r\n", app ? "A" : "", (unsigned long)idx);
    return;
  }

  if (app) {
    sdmmc_raw_t pre = {0};
    pre.index = 55;
    pre.arg = (uint32_t)rca << 16;
    pre.rsp = SDMMC_RSP_R1;
    if (run_raw(false, &pre) != RETURNCODE_SUCCESS) return;
  }
  run_raw(app, &c);
}

// ---------------------------------------------------------------------------
// Driver-level commands
// ---------------------------------------------------------------------------

static void cmd_init(void) {
  uint32_t blocks = 0;
  returncode_t rc = sdmmc_init(&blocks);
  if (rc != RETURNCODE_SUCCESS) {
    initialized = false;
    report_rc("init", rc);
    print_error_info();
    return;
  }
  sdmmc_info_t info;
  if (sdmmc_get_info(&info) != RETURNCODE_SUCCESS) {
    printf("init ok, but card info unavailable\r\n");
    return;
  }
  initialized = true;
  rca = info.rca;
  high_capacity = info.high_capacity;
  block_count = blocks;
  printf("init ok: %lu blocks (%lu MiB), %s, RCA %04X, %u-bit at %lu Hz%s\r\n", (unsigned long)blocks,
         (unsigned long)(blocks / 2048), info.high_capacity ? "SDHC/SDXC" : "SDSC", info.rca, info.bus_width,
         (unsigned long)info.clock_hz, info.bus_fallback ? " (fell back from 4-bit)" : "");
}

static void cmd_info(void) {
  sdmmc_info_t info;
  returncode_t rc = sdmmc_get_info(&info);
  if (rc != RETURNCODE_SUCCESS) {
    report_rc("info", rc);
    return;
  }
  print_cid(info.cid);
  print_csd(info.csd);
  print_ocr(info.ocr);
  printf("  SCR:");
  for (int i = 0; i < 8; i++) printf(" %02X", info.scr[i]);
  printf("\r\n  RCA %04X, %s, physical layer %s, %u-bit at %lu Hz, %lu blocks\r\n", info.rca,
         info.high_capacity ? "block addressed" : "byte addressed", info.version2 ? "2.00+" : "1.x",
         info.bus_width, (unsigned long)info.clock_hz, (unsigned long)info.block_count);
}

static void cmd_status(void) {
  sdmmc_raw_t c = {0};
  c.index = 13;
  c.arg = (uint32_t)rca << 16;
  c.rsp = SDMMC_RSP_R1;
  run_raw(false, &c);
}

static bool range_ok(uint32_t lba, uint32_t count) {
  if (!initialized) {
    printf("run `init` first\r\n");
    return false;
  }
  if (count == 0 || count > max_blocks) {
    printf("count must be 1-%lu\r\n", (unsigned long)max_blocks);
    return false;
  }
  if (lba >= block_count || count > block_count - lba) {
    printf("past the end of the card (%lu blocks)\r\n", (unsigned long)block_count);
    return false;
  }
  return true;
}

static void cmd_read(void) {
  uint32_t lba, count;
  if (ntok < 2 || !arg_u32(1, 0, &lba) || !arg_u32(2, 1, &count)) {
    if (ntok < 2) printf("usage: read <lba> [count]\r\n");
    return;
  }
  if (!range_ok(lba, count)) return;
  returncode_t rc = sdmmc_read(lba, count, buf);
  if (rc != RETURNCODE_SUCCESS) {
    report_rc("read", rc);
    print_error_info();
    return;
  }
  uint32_t len = count * SDMMC_BLOCK_SIZE;
  printf("read %lu block(s) at LBA %lu into buffer\r\n", (unsigned long)count, (unsigned long)lba);
  hexdump(buf, len < SHOW_LEN ? len : SHOW_LEN, 0);
  if (len > SHOW_LEN) printf("  (%lu more bytes: dump %u %lu)\r\n", (unsigned long)(len - SHOW_LEN), SHOW_LEN,
                             (unsigned long)(len - SHOW_LEN));
}

static void cmd_write(void) {
  uint32_t lba, count;
  if (ntok < 2 || !arg_u32(1, 0, &lba) || !arg_u32(2, 1, &count)) {
    if (ntok < 2) printf("usage: write <lba> [count]   (writes the buffer's first count blocks)\r\n");
    return;
  }
  if (!unlocked) {
    printf("locked: `unlock` first\r\n");
    return;
  }
  if (!range_ok(lba, count)) return;
  returncode_t rc = sdmmc_write(lba, count, buf);
  if (rc != RETURNCODE_SUCCESS) {
    report_rc("write", rc);
    print_error_info();
    return;
  }
  printf("wrote %lu block(s) at LBA %lu\r\n", (unsigned long)count, (unsigned long)lba);
}

static void cmd_verify(void) {
  uint32_t lba, count;
  if (ntok < 2 || !arg_u32(1, 0, &lba) || !arg_u32(2, 1, &count)) {
    if (ntok < 2) printf("usage: verify <lba> [count]   (compares the card with the buffer)\r\n");
    return;
  }
  if (!range_ok(lba, count)) return;
  uint32_t bad = 0;
  for (uint32_t done = 0; done < count;) {
    uint32_t n = count - done < 8 ? count - done : 8;
    returncode_t rc = sdmmc_read(lba + done, n, tmp);
    if (rc != RETURNCODE_SUCCESS) {
      report_rc("verify read", rc);
      print_error_info();
      return;
    }
    const uint8_t* want = buf + done * SDMMC_BLOCK_SIZE;
    for (uint32_t i = 0; i < n * SDMMC_BLOCK_SIZE; i++) {
      if (tmp[i] != want[i]) {
        if (bad < 8) {
          uint32_t off = done * SDMMC_BLOCK_SIZE + i;
          printf("  LBA %lu +%lu: card %02X, buffer %02X\r\n", (unsigned long)(lba + off / SDMMC_BLOCK_SIZE),
                 (unsigned long)(off % SDMMC_BLOCK_SIZE), tmp[i], want[i]);
        }
        bad++;
      }
    }
    done += n;
  }
  if (bad) printf("verify: %lu byte(s) differ\r\n", (unsigned long)bad);
  else printf("verify: %lu block(s) match\r\n", (unsigned long)count);
}

// erase <first lba> <last lba> [discard|fule]: CMD32, CMD33, CMD38.
static void cmd_erase(void) {
  uint32_t first, last;
  if (ntok < 3 || !arg_u32(1, 0, &first) || !arg_u32(2, 0, &last)) {
    if (ntok < 3) printf("usage: erase <first lba> <last lba> [discard|fule]\r\n");
    return;
  }
  if (!unlocked) {
    printf("locked: `unlock` first\r\n");
    return;
  }
  if (!initialized) {
    printf("run `init` first\r\n");
    return;
  }
  if (first > last || last >= block_count) {
    printf("need first <= last < %lu\r\n", (unsigned long)block_count);
    return;
  }
  uint32_t mode = 0;
  if (ntok > 3 && !strcmp(tok[3], "discard")) mode = 1;
  else if (ntok > 3 && !strcmp(tok[3], "fule")) mode = 2;
  uint32_t scale = high_capacity ? 1 : SDMMC_BLOCK_SIZE;

  sdmmc_raw_t c = {0};
  c.rsp = SDMMC_RSP_R1;
  c.index = 32;
  c.arg = first * scale;
  if (run_raw(false, &c) != RETURNCODE_SUCCESS) return;
  c.index = 33;
  c.arg = last * scale;
  if (run_raw(false, &c) != RETURNCODE_SUCCESS) return;
  c.index = 38;
  c.arg = mode;
  c.rsp = SDMMC_RSP_R1B;
  printf("erasing %lu block(s)...\r\n", (unsigned long)(last - first + 1));
  fflush(stdout);
  run_raw(false, &c);
}

static void cmd_bus(bool host_only) {
  uint32_t width, hz, actual = 0;
  if (ntok < 3 || !arg_u32(1, 0, &width) || !arg_u32(2, 0, &hz)) {
    if (ntok < 3) printf("usage: %s <1|4> <hz>\r\n", host_only ? "host" : "bus");
    return;
  }
  returncode_t rc = host_only ? sdmmc_set_host_bus(width, hz, &actual) : sdmmc_set_bus(width, hz, &actual);
  if (rc != RETURNCODE_SUCCESS) {
    report_rc(host_only ? "host" : "bus", rc);
    if (!host_only) print_error_info();
    return;
  }
  printf("%s: %lu-bit at %lu Hz\r\n", host_only ? "host" : "card and host", (unsigned long)width,
         (unsigned long)actual);
}

static void cmd_clocks(void) {
  sdmmc_raw_t c = {0};
  c.flags = SDMMC_RAW_INIT_CLOCKS;
  returncode_t rc = sdmmc_raw(&c, NULL);
  if (rc != RETURNCODE_SUCCESS) report_rc("clocks", rc);
  else printf("sent 74 initialization clocks\r\n");
}

// ---------------------------------------------------------------------------
// Buffer
// ---------------------------------------------------------------------------

static bool buf_range(int i, uint32_t def_len, uint32_t* off, uint32_t* len) {
  if (!arg_u32(i, 0, off) || !arg_u32(i + 1, def_len, len)) return false;
  if (*off >= BUF_LEN) {
    printf("offset past the %u-byte buffer\r\n", BUF_LEN);
    return false;
  }
  if (*len > BUF_LEN - *off) *len = BUF_LEN - *off;
  return true;
}

static void cmd_dump(void) {
  uint32_t off, len;
  if (buf_range(1, 512, &off, &len)) hexdump(buf + off, len, off);
}

static void cmd_fill(void) {
  uint32_t v, off, len;
  if (ntok < 2) {
    printf("usage: fill <byte> [offset] [len]\r\n");
    return;
  }
  if (!arg_u32(1, 0, &v) || !buf_range(2, BUF_LEN, &off, &len)) return;
  memset(buf + off, (int)(v & 0xFF), len);
  printf("filled %lu bytes at %lu with %02lX\r\n", (unsigned long)len, (unsigned long)off, (unsigned long)(v & 0xFF));
}

static void cmd_set(void) {
  uint32_t off;
  if (ntok < 3) {
    printf("usage: set <offset> <byte> [byte...]   or   set <offset> \"text\r\n");
    return;
  }
  if (!arg_u32(1, 0, &off) || off >= BUF_LEN) {
    printf("offset past the %u-byte buffer\r\n", BUF_LEN);
    return;
  }
  uint32_t n = 0;
  if (tok[2][0] == '"') {
    for (const char* p = tok[2] + 1; *p && off + n < BUF_LEN; p++) buf[off + n++] = (uint8_t)*p;
  } else {
    for (int i = 2; i < ntok && off + n < BUF_LEN; i++) {
      uint32_t v;
      if (!parse_u32(tok[i], &v)) {
        printf("not a number: %s\r\n", tok[i]);
        return;
      }
      buf[off + n++] = v & 0xFF;
    }
  }
  printf("set %lu byte(s) at %lu\r\n", (unsigned long)n, (unsigned long)off);
}

// Every block gets its own index in its first 4 bytes, so a block written to
// the wrong place shows up in a verify or a read.
static void cmd_pattern(void) {
  uint32_t seed;
  if (!arg_u32(1, 0x5D, &seed)) return;
  uint32_t x = seed ? seed : 1;
  for (uint32_t i = 0; i < BUF_LEN; i++) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    buf[i] = x & 0xFF;
  }
  for (uint32_t b = 0; b < MAX_BLOCKS; b++) {
    buf[b * SDMMC_BLOCK_SIZE] = b & 0xFF;
    buf[b * SDMMC_BLOCK_SIZE + 1] = 0xB0;
    buf[b * SDMMC_BLOCK_SIZE + 2] = 0x0C;
    buf[b * SDMMC_BLOCK_SIZE + 3] = seed & 0xFF;
  }
  printf("buffer filled with pattern, seed %lu\r\n", (unsigned long)seed);
}

// ---------------------------------------------------------------------------
// Shell
// ---------------------------------------------------------------------------

static void help(void) {
  printf(
    "Numbers are decimal or 0x hex. LBAs are 512-byte blocks.\r\n"
    " Card (through the driver)\r\n"
    "  init                     full bring-up: identify, select, 4-bit, 25 MHz\r\n"
    "  info                     registers from the last init\r\n"
    "  status                   CMD13 card status\r\n"
    "  read <lba> [n]           read n blocks into the buffer and show them\r\n"
    "  write <lba> [n]          write the buffer's first n blocks       (unlock)\r\n"
    "  verify <lba> [n]         compare n blocks on the card with the buffer\r\n"
    "  erase <first> <last> [discard|fule]   CMD32/33/38             (unlock)\r\n"
    "  bus <1|4> <hz>           ACMD6 to the card, then the host\r\n"
    "  timeout <ms>             data timeout, 1-1000 (default 100)\r\n"
    "  err                      why the last driver operation failed\r\n"
    " Raw (sd_write kernel)\r\n"
    "  cmd <n> [arg] [opts]     send CMDn exactly as given\r\n"
    "  acmd <n> [arg] [opts]    send CMD55 (current RCA), then ACMDn\r\n"
    "      opts: none|r1|r1b|r2|r3  rd|wr <len> [blocks]  nodata  stop|nostop\r\n"
    "      defaults follow the SD spec per index, e.g. cmd 17 <addr> reads 512\r\n"
    "      bytes; addresses are bytes on SDSC cards, blocks on SDHC/SDXC\r\n"
    "  clocks                   74 initialization clocks (before a manual CMD0)\r\n"
    "  host <1|4> <hz>          host bus width/clock only, after a manual ACMD6/CMD6\r\n"
    "  rca [value]              RCA used by acmd and status (set by init, CMD3)\r\n"
    " Buffer (32 KiB; reads land here, writes come from here)\r\n"
    "  dump [off] [len]         fill <byte> [off] [len]\r\n"
    "  set <off> <bytes...>     set <off> \"text       pattern [seed]\r\n"
    " Safety\r\n"
    "  unlock / lock            allow / refuse commands that change the card\r\n"
    "Manual bring-up: host 1 400000; clocks; cmd 0 (sets RCA 0); cmd 8 0x1AA;\r\n"
    "acmd 41 0x40FF8000 (repeat until powered-up); cmd 2; cmd 3 (sets RCA);\r\n"
    "cmd 9 <rca<<16>; cmd 7 <rca<<16>; acmd 6 2; host 4 25000000; then init to resync\r\n");
}

// Read a line with echo and backspace. Returns its length.
static int read_line(char* line, int max) {
  int n = 0;
  for (;;) {
    char c;
    int got = 0;
    if (libtocksync_console_read((uint8_t*)&c, 1, &got) != RETURNCODE_SUCCESS || got != 1) continue;
    if (c == '\r' || c == '\n') {
      if (n == 0 && c == '\n') continue;  // the \n of a \r\n
      printf("\r\n");
      fflush(stdout);
      line[n] = 0;
      return n;
    }
    if (c == 0x08 || c == 0x7F) {
      if (n > 0) {
        n--;
        printf("\b \b");
        fflush(stdout);
      }
      continue;
    }
    if (c == 0x03) {  // ^C: abandon the line
      printf("^C\r\n");
      fflush(stdout);
      line[0] = 0;
      return 0;
    }
    if (c < 0x20 || n >= max - 1) continue;
    line[n++] = c;
    putchar(c);
    fflush(stdout);
  }
}

int main(void) {
  // Let the other applications finish their start-up output first.
  libtocksync_alarm_delay_ms(2000);
  printf("\r\n##### SD shell #####  (help for commands)\r\n");
  if (!sdmmc_exists()) {
    printf("no sdmmc driver in this kernel\r\n");
    return -1;
  }
  max_blocks = sdmmc_max_blocks();
  if (max_blocks == 0 || max_blocks > MAX_BLOCKS) max_blocks = MAX_BLOCKS;
  printf("card detect: %s. Locked: write/erase refused until `unlock`.\r\n",
         sdmmc_card_present() ? "present" : "no card (switch may be absent)");

  static char line[128];
  for (;;) {
    printf(unlocked ? "sd!> " : "sd> ");
    fflush(stdout);
    if (read_line(line, sizeof(line)) == 0) continue;
    tokenize(line);
    if (ntok == 0) continue;
    const char* c = tok[0];

    if (!strcmp(c, "help") || !strcmp(c, "?")) help();
    else if (!strcmp(c, "init")) cmd_init();
    else if (!strcmp(c, "info")) cmd_info();
    else if (!strcmp(c, "status")) cmd_status();
    else if (!strcmp(c, "read")) cmd_read();
    else if (!strcmp(c, "write")) cmd_write();
    else if (!strcmp(c, "verify")) cmd_verify();
    else if (!strcmp(c, "erase")) cmd_erase();
    else if (!strcmp(c, "bus")) cmd_bus(false);
    else if (!strcmp(c, "host")) cmd_bus(true);
    else if (!strcmp(c, "cmd")) cmd_raw(false);
    else if (!strcmp(c, "acmd")) cmd_raw(true);
    else if (!strcmp(c, "clocks")) cmd_clocks();
    else if (!strcmp(c, "err")) print_error_info();
    else if (!strcmp(c, "dump")) cmd_dump();
    else if (!strcmp(c, "fill")) cmd_fill();
    else if (!strcmp(c, "set")) cmd_set();
    else if (!strcmp(c, "pattern")) cmd_pattern();
    else if (!strcmp(c, "timeout")) {
      uint32_t ms;
      if (ntok < 2 || !arg_u32(1, 0, &ms)) printf("usage: timeout <ms>\r\n");
      else {
        returncode_t rc = sdmmc_set_timeout(ms);
        if (rc == RETURNCODE_SUCCESS) printf("data timeout %lu ms\r\n", (unsigned long)ms);
        else report_rc("timeout", rc);
      }
    } else if (!strcmp(c, "rca")) {
      uint32_t v;
      if (ntok > 1 && arg_u32(1, 0, &v)) rca = v & 0xFFFF;
      printf("RCA %04X\r\n", rca);
    } else if (!strcmp(c, "unlock")) {
      unlocked = true;
      printf("UNLOCKED: write, erase and raw writes will change the card\r\n");
    } else if (!strcmp(c, "lock")) {
      unlocked = false;
      printf("locked\r\n");
    } else {
      printf("unknown command: %s (help)\r\n", c);
    }
  }
}
