// SD card inspector.
//
// 1. Prints the card's identification registers (CID, CSD, SCR, OCR), decoded.
// 2. Finds the FAT file system and walks every directory, listing files that
//    are plainly present and directory entries that were deleted (first name
//    byte 0xE5) -- the cheap kind of recovery.
// 3. Carves the unallocated space: every sector of every free cluster, the
//    slack after each live file, and the sectors outside the partition --
//    and looks for file headers, the way photorec does. This is what finds
//    data whose directory entries are gone entirely, e.g. after a quick
//    format.
//
// Strictly read-only: the kernel driver has no write command at all.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <libtock/peripherals/syscalls/alarm_syscalls.h>
#include <libtock/services/alarm.h>
#include <libtock-sync/services/alarm.h>

#include "sdmmc.h"

#define MAX_DEPTH        6
#define MAX_HEADER_LINES 64
#define MAX_RAW_RUNS     16

// Pause before printing, so a host attaching to the console after a reset
// (an RTT logger especially: its ring is only 1 KiB) sees the whole report.
#define START_DELAY_MS   4000

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint16_t le16(const uint8_t* p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t le32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Bits [msb:lsb] of a big-endian register image `nbytes` long.
static uint32_t bits(const uint8_t* reg, int nbytes, int msb, int lsb) {
  uint32_t v = 0;
  for (int b = msb; b >= lsb; b--) {
    uint8_t byte = reg[nbytes - 1 - b / 8];
    v = (v << 1) | ((byte >> (b % 8)) & 1);
  }
  return v;
}

// CRC7 (x^7 + x^3 + 1) as used by CID and CSD.
static uint8_t crc7(const uint8_t* data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    for (int b = 7; b >= 0; b--) {
      uint8_t in  = (data[i] >> b) & 1;
      uint8_t top = (crc >> 6) & 1;
      crc = (uint8_t)((crc << 1) & 0x7F);
      if (in ^ top) crc ^= 0x09;
    }
  }
  return crc;
}

static uint32_t now_ms(void) {
  uint32_t t = 0;
  libtock_alarm_command_read(&t);
  return libtock_alarm_ticks_to_ms(t);
}

static uint32_t read_errors;

// One line explaining a failure: where it failed, the controller's error bits
// (SAMV71 HSMCI_SR) and the card's R1 error bits, and the card state after.
static void print_error(const sdmmc_error_t* e) {
  static const char* const stages[] = {"none", "response", "card status", "data", "busy", "stuck"};
  printf("CMD%u %s", e->cmd, e->stage < 6 ? stages[e->stage] : "?");
  static const struct { uint32_t bit; const char* name; } srb[] = {
    {1u << 16, "RINDE"}, {1u << 17, "RDIRE"}, {1u << 18, "RCRCE"}, {1u << 19, "RENDE"},
    {1u << 20, "RTOE"}, {1u << 21, "DCRCE"}, {1u << 22, "DTOE"}, {1u << 23, "CSTOE"},
    {1u << 30, "OVRE"}, {1u << 31, "UNRE"},
  };
  printf(" [SR");
  for (unsigned i = 0; i < sizeof(srb) / sizeof(srb[0]); i++)
    if (e->sr & srb[i].bit) printf(" %s", srb[i].name);
  printf("]");
  if (e->stage == SDMMC_STAGE_CARD_STATUS || e->stage == SDMMC_STAGE_DATA) {
    static const struct { uint32_t bit; const char* name; } r1b[] = {
      {1u << 31, "OUT_OF_RANGE"}, {1u << 30, "ADDRESS_ERROR"}, {1u << 29, "BLOCK_LEN"},
      {1u << 23, "COM_CRC"}, {1u << 22, "ILLEGAL_CMD"}, {1u << 21, "CARD_ECC_FAILED"},
      {1u << 20, "CC_ERROR"}, {1u << 19, "ERROR"},
    };
    printf(" [R1 %08lX", (unsigned long)e->r1);
    for (unsigned i = 0; i < sizeof(r1b) / sizeof(r1b[0]); i++)
      if (e->r1 & r1b[i].bit) printf(" %s", r1b[i].name);
    printf("]");
  }
  static const char* const states[] = {"idle", "ready", "ident", "stby", "tran", "data", "rcv", "prg", "dis"};
  if (e->card_state < 9) printf(" then %s", states[e->card_state]);
  else if (e->card_state == 0xFF) printf(" then no answer to CMD13");
}

// ----- Bad sectors ----------------------------------------------------------
//
// A card with failing media is read the way ddrescue reads one: a failed
// multi-block read is split down to single blocks so the good sectors around
// a bad one are still read, and after a run of failures the reader skips
// ahead, doubling the jump each time, so a large dead region costs seconds
// rather than hours (each failed read waits out a ~250 ms data timeout).
// Everything not read is accounted for: confirmed-bad sectors and skipped
// ones are listed separately.

#define MAX_RANGES     48   // ranges kept for the report; totals are exact
#define DETAIL_ERRORS  8    // failures printed in full before going quiet
#define SKIP_MIN       64
#define SKIP_MAX       65536  // 32 MiB
#define DEAD_RUN       8      // bad blocks in a row that mark a dead region

typedef struct {
  uint32_t start, len;
} range_t;

static range_t bad[MAX_RANGES], skipped[MAX_RANGES];
static uint32_t n_bad, n_skipped;              // ranges recorded (capped)
static uint32_t bad_ranges, skipped_ranges;    // ranges found (exact)
static uint64_t bad_sectors, skipped_sectors;  // sectors (exact)
// Skip window [skip_from, skip_until). It only ever lies ahead of the read
// that opened it, and each pass (and each file) starts with it closed: a
// window left open at the end of the card once swallowed every later read
// at a lower address.
static uint32_t skip_from, skip_until, skip_len = SKIP_MIN;

static void reset_skip(void) {
  skip_from = skip_until = 0;
  skip_len = SKIP_MIN;
}

static void note_range(range_t* list, uint32_t* n, uint32_t* total, uint32_t lba, uint32_t len) {
  if (*n && list[*n - 1].start + list[*n - 1].len == lba) {
    list[*n - 1].len += len;  // extends the previous range
    return;
  }
  (*total)++;
  if (*n < MAX_RANGES) list[(*n)++] = (range_t){lba, len};
}

static void note_bad(uint32_t lba, uint32_t len) {
  bad_sectors += len;
  note_range(bad, &n_bad, &bad_ranges, lba, len);
}

static void note_skipped(uint32_t lba, uint32_t len) {
  skipped_sectors += len;
  note_range(skipped, &n_skipped, &skipped_ranges, lba, len);
}

static bool read_sectors(uint32_t lba, uint32_t count, uint8_t* buf) {
  returncode_t rc = sdmmc_read(lba, count, buf);
  if (rc != RETURNCODE_SUCCESS) {
    read_errors++;
    if (read_errors <= DETAIL_ERRORS) {
      printf("  ! read of %lu block(s) at LBA %lu failed: ", (unsigned long)count, (unsigned long)lba);
      sdmmc_error_t e;
      if (sdmmc_error_info(0, &e) == RETURNCODE_SUCCESS) print_error(&e);
      else printf("%s", tock_strrcode(rc));
      printf("\r\n");
      if (read_errors == DETAIL_ERRORS) printf("  ! (further read failures are counted, not listed)\r\n");
    }
    return false;
  }
  return true;
}

// Metadata read (boot sector, FAT, directory): one targeted sector, never
// skipped, recorded as bad if it fails.
static bool read_meta(uint32_t lba, uint8_t* buf) {
  if (read_sectors(lba, 1, buf)) return true;
  note_bad(lba, 1);
  return false;
}

// The same, silently: for reads made in the middle of printing a line, which
// say for themselves that the sector was unreadable.
static bool read_meta_quiet(uint32_t lba, uint8_t* buf) {
  if (sdmmc_read(lba, 1, buf) == RETURNCODE_SUCCESS) return true;
  read_errors++;
  note_bad(lba, 1);
  return false;
}

// Read up to 64 sectors for scanning, ddrescue-style. Sets bit i of *ok for
// each sector read; returns how many were. Sectors not read are recorded as
// bad (tried and failed) or skipped (not tried).
static uint32_t read_resilient(uint32_t lba, uint32_t count, uint8_t* buf, uint64_t* ok) {
  *ok = 0;
  if (lba >= skip_from && lba < skip_until) {
    uint32_t s = skip_until - lba < count ? skip_until - lba : count;
    note_skipped(lba, s);
    if (s == count) return 0;
    uint64_t rest;
    uint32_t good = read_resilient(lba + s, count - s, buf + 512 * s, &rest);
    *ok = rest << s;
    return good;
  }
  if (sdmmc_read(lba, count, buf) == RETURNCODE_SUCCESS) {
    *ok = count == 64 ? ~0ull : ((1ull << count) - 1);
    skip_len = SKIP_MIN;
    return count;
  }
  read_errors++;
  // Narrow it down a block at a time. A good block costs well under a
  // millisecond and a bad one a full data timeout, so reading singles
  // straight away is cheaper than an intermediate split, which only adds a
  // failed read for every bad block. Only a run of DEAD_RUN bad blocks in a
  // row -- a dead region, not scattered damage -- gives up on the rest.
  // (Giving up after two once reported a photo with 1% bad blocks as 40%
  // unreadable: the skipped sectors around scattered bad ones were good.)
  uint32_t good = 0, run_bad = 0;
  for (uint32_t i = 0; i < count; i++) {
    if (run_bad >= DEAD_RUN) {
      note_skipped(lba + i, count - i);
      break;
    }
    if (read_sectors(lba + i, 1, buf + 512 * i)) {
      *ok |= 1ull << i;
      good++;
      run_bad = 0;
    } else {
      note_bad(lba + i, 1);
      run_bad++;
    }
  }
  if (good == 0) {
    // A dead extent: jump ahead, further each time it happens in a row.
    skip_from = lba + count;
    skip_until = skip_from + skip_len;
    if (skip_len < SKIP_MAX) skip_len *= 2;
  } else {
    skip_len = SKIP_MIN;
  }
  return good;
}

// ---------------------------------------------------------------------------
// Card registers
// ---------------------------------------------------------------------------

// JEDEC-style manufacturer IDs as commonly reported; the SD Association does
// not publish this table, so treat the name as a hint.
static const char* manufacturer(uint8_t mid) {
  switch (mid) {
    case 0x01: return "Panasonic";
    case 0x02: return "Toshiba/Kioxia";
    case 0x03: return "SanDisk";
    case 0x09: return "APower";
    case 0x1B: return "Samsung";
    case 0x1D: return "ADATA";
    case 0x27: return "Phison";
    case 0x28: return "Lexar";
    case 0x31: return "Silicon Power";
    case 0x41: return "Kingston";
    case 0x6F: return "STEC";
    case 0x74: return "Transcend";
    case 0x76: return "Patriot";
    case 0x82: return "Sony";
    case 0x9C: return "Angelbird/Hoodman";
    default:   return "unknown";
  }
}

static void hex(const char* label, const uint8_t* p, int n) {
  printf("  %-6s ", label);
  for (int i = 0; i < n; i++) printf("%02X", p[i]);
  printf("\r\n");
}

// SD Status register (ACMD13), SD Physical Layer Simplified Spec 4.10.2.
static void print_sd_status(const sdmmc_info_t* ci) {
  const uint8_t* s = ci->ssr;
  printf("\r\nSD Status (ACMD13)\r\n");
  if (!ci->ssr_valid) {
    sdmmc_error_t e;
    printf("  not read: ");
    if (sdmmc_error_info(1, &e) == RETURNCODE_SUCCESS) print_error(&e);
    printf("\r\n");
    return;
  }
  hex("SSR", s, 32);
  hex("", s + 32, 32);

  static const char* const speed_class[] = {"0", "2", "4", "6", "10"};
  uint32_t sc = bits(s, 64, 447, 440);
  if (sc == 0) printf("  Speed class       not rated (Class 0)\r\n");
  else printf("  Speed class       %s%s\r\n", sc < 5 ? "Class " : "reserved ", sc < 5 ? speed_class[sc] : "");
  uint32_t uhs = bits(s, 64, 399, 396);
  printf("  UHS speed grade   %s\r\n", uhs == 0 ? "none" : uhs == 1 ? "U1" : uhs == 3 ? "U3" : "reserved");
  uint32_t vsc = bits(s, 64, 391, 384);
  if (vsc) printf("  Video speed class V%lu\r\n", (unsigned long)vsc);
  else printf("  Video speed class none\r\n");
  uint32_t apc = bits(s, 64, 339, 336);
  printf("  App performance   %s\r\n", apc == 0 ? "none" : apc == 1 ? "A1" : apc == 2 ? "A2" : "reserved");

  // AU_SIZE: 1 = 16 KiB doubling to 9 = 4 MiB, then 8, 12, 16, 24, 32, 64 MiB.
  static const uint32_t au_kib[16] = {0, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096,
                                      8192, 12288, 16384, 24576, 32768, 65536};
  uint32_t au = au_kib[bits(s, 64, 431, 428)];
  if (au) printf("  Allocation unit   %lu KiB\r\n", (unsigned long)au);
  else printf("  Allocation unit   not defined\r\n");
  uint32_t perf_move = bits(s, 64, 439, 432);
  if (perf_move) printf("  Move performance  %lu MB/s\r\n", (unsigned long)perf_move);
  uint32_t erase_size = bits(s, 64, 423, 408), erase_timeout = bits(s, 64, 407, 402);
  if (erase_size)
    printf("  Erase             %lu AUs in %lu s (+%lu s offset)\r\n", (unsigned long)erase_size,
           (unsigned long)erase_timeout, (unsigned long)bits(s, 64, 401, 400));
  printf("  Card type         0x%04lX%s\r\n", (unsigned long)bits(s, 64, 495, 480),
         bits(s, 64, 495, 480) == 0 ? " (regular SD RD/WR)" : "");
  // SDHC/SDXC give bytes. SDSC gives units of MULT * BLOCK_LEN, from the CSD.
  uint64_t prot = bits(s, 64, 479, 448);
  if (bits(ci->csd, 16, 127, 126) == 0) {
    prot <<= (bits(ci->csd, 16, 49, 47) + 2) + bits(ci->csd, 16, 83, 80);
  }
  printf("  Protected area    %lu KiB\r\n", (unsigned long)(prot / 1024));
  printf("  Discard/FULE      %lu/%lu\r\n", (unsigned long)bits(s, 64, 313, 313),
         (unsigned long)bits(s, 64, 312, 312));
}

// CMD6 mode 0 status, SD Physical Layer Simplified Spec 4.3.10.
static void print_switch(const sdmmc_info_t* ci) {
  const uint8_t* s = ci->switch_status;
  printf("\r\nCMD6 check (supported functions)\r\n");
  if (!ci->switch_valid) {
    uint32_t ccc = bits(ci->csd, 16, 95, 84);
    printf("  not read%s\r\n", ccc & (1u << 10) ? " (the command failed)" : " (no command class 10)");
    return;
  }
  hex("SWITCH", s, 32);
  hex("", s + 32, 32);
  printf("  Max current       %lu mA\r\n", (unsigned long)bits(s, 64, 511, 496));

  uint32_t g1 = bits(s, 64, 415, 400);
  static const char* const modes[5] = {"default (SDR12)", "high speed (SDR25)", "SDR50", "SDR104", "DDR50"};
  printf("  Bus speed modes  ");
  for (int i = 0; i < 5; i++)
    if (g1 & (1u << i)) printf(" %s", modes[i]);
  printf("\r\n");

  // Bit 15 of each group is function 0xF ("no change"), set on every card;
  // it says nothing about capability, so it is masked off before judging.
  uint32_t g2 = bits(s, 64, 431, 416);
  printf("  Command systems   0x%04lX%s\r\n", (unsigned long)g2, (g2 & 0x7FFF) == 1 ? " (default only)" : "");
  uint32_t g3 = bits(s, 64, 447, 432);
  printf("  Driver strengths  0x%04lX%s\r\n", (unsigned long)g3, (g3 & 0x7FFF) == 1 ? " (type B only)" : "");
  uint32_t g4 = bits(s, 64, 463, 448);
  printf("  Power limits      0x%04lX%s\r\n", (unsigned long)g4, (g4 & 0x7FFF) == 1 ? " (0.72 W only)" : "");
  printf("  Status version    %lu\r\n", (unsigned long)bits(s, 64, 375, 368));
}

static void print_card(const sdmmc_info_t* ci) {
  const uint8_t* cid = ci->cid;
  const uint8_t* csd = ci->csd;
  const uint8_t* scr = ci->scr;

  printf("\r\n=== Card registers ===\r\n");
  hex("CID", cid, 16);
  hex("CSD", csd, 16);
  hex("SCR", scr, 8);
  printf("  OCR    %08lX   RCA %04X\r\n", (unsigned long)ci->ocr, ci->rca);

  printf("\r\nCID (card identification)\r\n");
  printf("  Manufacturer ID   0x%02X (%s)\r\n", cid[0], manufacturer(cid[0]));
  printf("  OEM/application   \"%c%c\"\r\n", cid[1], cid[2]);
  printf("  Product name      \"%c%c%c%c%c\"\r\n", cid[3], cid[4], cid[5], cid[6], cid[7]);
  printf("  Product revision  %u.%u\r\n", cid[8] >> 4, cid[8] & 0xF);
  printf("  Serial number     0x%02X%02X%02X%02X (%lu)\r\n", cid[9], cid[10], cid[11], cid[12],
         (unsigned long)(((uint32_t)cid[9] << 24) | ((uint32_t)cid[10] << 16) | (cid[11] << 8) | cid[12]));
  uint32_t year  = 2000 + bits(cid, 16, 19, 12);
  uint32_t month = bits(cid, 16, 11, 8);
  printf("  Manufactured      %lu-%02lu\r\n", (unsigned long)year, (unsigned long)month);
  uint8_t c = crc7(cid, 15);
  printf("  CRC7              0x%02X, computed 0x%02X: %s\r\n", cid[15] >> 1, c,
         (cid[15] >> 1) == c ? "OK" : "MISMATCH");

  printf("\r\nCSD (card specific data)\r\n");
  uint32_t csd_ver = bits(csd, 16, 127, 126);
  printf("  CSD structure     %lu (%s)\r\n", (unsigned long)csd_ver + 1,
         csd_ver == 0 ? "SDSC" : csd_ver == 1 ? "SDHC/SDXC" : "reserved");
  uint32_t tran = csd[3];
  printf("  TRAN_SPEED        0x%02lX%s\r\n", (unsigned long)tran,
         tran == 0x32 ? " (25 MHz)" : tran == 0x5A ? " (50 MHz)" : "");
  printf("  Command classes   0x%03lX\r\n", (unsigned long)bits(csd, 16, 95, 84));
  printf("  READ_BL_LEN       %lu bytes\r\n", 1UL << bits(csd, 16, 83, 80));
  uint64_t bytes = (uint64_t)ci->block_count * 512;
  printf("  Capacity          %lu blocks = %lu MiB (%lu MB)\r\n", (unsigned long)ci->block_count,
         (unsigned long)(bytes >> 20), (unsigned long)(bytes / 1000000));
  printf("  Copy flag         %lu\r\n", (unsigned long)bits(csd, 16, 14, 14));
  printf("  Write protect     permanent=%lu temporary=%lu\r\n",
         (unsigned long)bits(csd, 16, 13, 13), (unsigned long)bits(csd, 16, 12, 12));
  printf("  File format       grp=%lu fmt=%lu\r\n",
         (unsigned long)bits(csd, 16, 15, 15), (unsigned long)bits(csd, 16, 11, 10));
  c = crc7(csd, 15);
  printf("  CRC7              0x%02X, computed 0x%02X: %s\r\n", csd[15] >> 1, c,
         (csd[15] >> 1) == c ? "OK" : "MISMATCH");

  printf("\r\nSCR (SD configuration)\r\n");
  bool scr_valid = false;
  for (int i = 0; i < 8; i++) if (scr[i]) scr_valid = true;
  if (!scr_valid) {
    printf("  not read\r\n");
  } else {
    uint32_t spec  = bits(scr, 8, 59, 56);
    uint32_t spec3 = bits(scr, 8, 47, 47);
    uint32_t spec4 = bits(scr, 8, 42, 42);
    uint32_t specx = bits(scr, 8, 41, 38);
    const char* v = "unknown";
    if (spec == 0) v = "1.0x";
    else if (spec == 1) v = "1.10";
    else if (spec == 2 && !spec3) v = "2.00";
    else if (spec == 2 && spec3 && !spec4 && specx == 0) v = "3.0x";
    else if (spec == 2 && spec3 && spec4 && specx == 0) v = "4.xx";
    else if (spec == 2 && spec3 && specx >= 1) v = "5.xx or later";
    printf("  Physical layer    %s\r\n", v);
    printf("  Data after erase  all %s\r\n", bits(scr, 8, 55, 55) ? "1s (0xFF)" : "0s (0x00)");
    printf("  Security          %lu\r\n", (unsigned long)bits(scr, 8, 54, 52));
    printf("  Bus widths        0x%lX\r\n", (unsigned long)bits(scr, 8, 51, 48));
  }

  print_sd_status(ci);
  print_switch(ci);

  printf("\r\nOCR / link\r\n");
  printf("  Capacity status   %s\r\n", ci->high_capacity ? "high capacity (block addressed)" :
         "standard capacity (byte addressed)");
  printf("  Physical layer    %s\r\n", ci->version2 ? "2.00 or later (answered CMD8)" : "1.x");
  printf("  Bus               %u-bit at %lu kHz\r\n", ci->bus_width, (unsigned long)(ci->clock_hz / 1000));
  if (ci->bus_fallback)
    printf("  !! 4-bit transfers failed and 1-bit ones work: DAT1-3 are not making contact\r\n"
           "     (card pads or holder). Running at 1 bit, a quarter of the speed.\r\n");
}

// ---------------------------------------------------------------------------
// File signatures
// ---------------------------------------------------------------------------

typedef struct {
  const char* name;
  bool video;
  bool strong;   // a header that only ever appears at the start of a file
} sig_t;

static char sig_label[32];

// Identify a file header at the start of `s` (one sector). Returns NULL if
// nothing is recognised.
static const sig_t* identify(const uint8_t* s) {
  static const sig_t MP4  = {"MP4/MOV (ftyp)", true, true};
  static const sig_t QT   = {"QuickTime atom", true, false};
  static const sig_t AVI  = {"AVI (RIFF)", true, true};
  static const sig_t WAV  = {"WAV (RIFF)", false, true};
  static const sig_t RIFF = {"RIFF", false, true};
  static const sig_t MKV  = {"Matroska/WebM", true, true};
  static const sig_t ASF  = {"ASF/WMV", true, true};
  static const sig_t FLV  = {"FLV", true, true};
  static const sig_t MPS  = {"MPEG program stream", true, false};
  static const sig_t MPV  = {"MPEG video sequence", true, false};
  static const sig_t MTS  = {"MPEG transport stream", true, false};
  static const sig_t M2TS = {"M2TS (AVCHD)", true, false};
  static const sig_t OGG  = {"Ogg", false, true};
  static const sig_t JPG  = {"JPEG", false, false};
  static const sig_t PNG  = {"PNG", false, true};
  static const sig_t GIF  = {"GIF", false, true};
  static const sig_t PDF  = {"PDF", false, true};
  static const sig_t ZIP  = {"ZIP/Office", false, false};
  static const sig_t RAR  = {"RAR", false, true};
  static const sig_t SZ   = {"7-Zip", false, true};
  static const sig_t ID3  = {"MP3 (ID3)", false, true};
  static const sig_t FLAC = {"FLAC", false, true};
  static const sig_t BMP  = {"BMP", false, false};

  if (!memcmp(s + 4, "ftyp", 4)) {
    snprintf(sig_label, sizeof(sig_label), "%s '%c%c%c%c'", MP4.name, s[8], s[9], s[10], s[11]);
    return &MP4;
  }
  if (!memcmp(s + 4, "moov", 4) || !memcmp(s + 4, "mdat", 4) || !memcmp(s + 4, "wide", 4) ||
      !memcmp(s + 4, "free", 4) || !memcmp(s + 4, "skip", 4)) {
    // Plausible atom size: at least 8, big-endian.
    uint32_t size = ((uint32_t)s[0] << 24) | ((uint32_t)s[1] << 16) | (s[2] << 8) | s[3];
    if (size >= 8 || size == 1) {
      snprintf(sig_label, sizeof(sig_label), "%s '%c%c%c%c'", QT.name, s[4], s[5], s[6], s[7]);
      return &QT;
    }
  }
  if (!memcmp(s, "RIFF", 4)) {
    const sig_t* r = !memcmp(s + 8, "AVI ", 4) ? &AVI : !memcmp(s + 8, "WAVE", 4) ? &WAV : &RIFF;
    // The chunk size gives the length of the whole file, which is what lets
    // a header be tied to the data that follows it.
    uint32_t file_size = le32(s + 4) + 8;
    snprintf(sig_label, sizeof(sig_label), "%s %lu KiB", r->name, (unsigned long)(file_size / 1024));
    return r;
  }
  static const uint8_t ebml[] = {0x1A, 0x45, 0xDF, 0xA3};
  static const uint8_t asf[] = {0x30, 0x26, 0xB2, 0x75, 0x8E, 0x66, 0xCF, 0x11};
  static const uint8_t pack[] = {0x00, 0x00, 0x01, 0xBA};
  static const uint8_t seqh[] = {0x00, 0x00, 0x01, 0xB3};
  static const uint8_t png[] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  static const uint8_t sz[] = {'7', 'z', 0xBC, 0xAF, 0x27, 0x1C};
  const sig_t* r = NULL;
  if (!memcmp(s, ebml, 4)) r = &MKV;
  else if (!memcmp(s, asf, 8)) r = &ASF;
  else if (!memcmp(s, "FLV\x01", 4)) r = &FLV;
  else if (!memcmp(s, pack, 4)) r = &MPS;
  else if (!memcmp(s, seqh, 4)) r = &MPV;
  else if (s[0] == 0x47 && s[188] == 0x47 && s[376] == 0x47) r = &MTS;
  else if (s[4] == 0x47 && s[196] == 0x47 && s[388] == 0x47) r = &M2TS;
  else if (!memcmp(s, "OggS", 4)) r = &OGG;
  else if (s[0] == 0xFF && s[1] == 0xD8 && s[2] == 0xFF) r = &JPG;
  else if (!memcmp(s, png, 8)) r = &PNG;
  else if (!memcmp(s, "GIF87a", 6) || !memcmp(s, "GIF89a", 6)) r = &GIF;
  else if (!memcmp(s, "%PDF-", 5)) r = &PDF;
  else if (!memcmp(s, "PK\x03\x04", 4)) r = &ZIP;
  else if (!memcmp(s, "Rar!\x1A\x07", 6)) r = &RAR;
  else if (!memcmp(s, sz, 6)) r = &SZ;
  else if (!memcmp(s, "ID3", 3) && s[3] < 5) r = &ID3;
  else if (!memcmp(s, "fLaC", 4)) r = &FLAC;
  else if (s[0] == 'B' && s[1] == 'M' && le32(s + 2) > 54 && le32(s + 6) == 0 && le32(s + 14) == 40) r = &BMP;
  if (r) snprintf(sig_label, sizeof(sig_label), "%s", r->name);
  return r;
}

typedef enum { SEC_ZERO, SEC_FF, SEC_DATA } sec_kind_t;

// Word-wide when the buffer allows it: process RAM is uncached under Tock's
// MPU settings, so on blank sectors -- most of a freshly formatted card --
// this loop is what bounds carving speed.
static sec_kind_t sector_kind(const uint8_t* s) {
  if (((uintptr_t)s & 3) == 0) {
    const uint32_t* w = (const uint32_t*)s;
    uint32_t first = w[0];
    if (first != 0x00000000 && first != 0xFFFFFFFF) return SEC_DATA;
    for (int i = 1; i < 128; i++) {
      if (w[i] != first) return SEC_DATA;
    }
    return first ? SEC_FF : SEC_ZERO;
  }
  bool zero = true, ff = true;
  for (int i = 0; i < 512 && (zero || ff); i++) {
    if (s[i] != 0x00) zero = false;
    if (s[i] != 0xFF) ff = false;
  }
  return zero ? SEC_ZERO : ff ? SEC_FF : SEC_DATA;
}

// ---------------------------------------------------------------------------
// FAT file system
// ---------------------------------------------------------------------------

typedef struct {
  int type;                // 12, 16 or 32; 0 = none found
  uint32_t part_lba;
  uint8_t spc;             // sectors per cluster
  uint32_t fat_lba;
  uint32_t fat_sectors;
  uint32_t root_lba;       // FAT12/16 fixed root directory
  uint32_t root_sectors;
  uint32_t root_cluster;   // FAT32
  uint32_t data_lba;
  uint32_t clusters;       // data clusters, numbered 2 .. clusters + 1
  uint32_t total_sectors;  // volume size, from the boot sector
  char label[12];
  char fstype[9];
  char oem[9];
  uint8_t nfats;
  uint32_t vol_serial;
} fs_t;

// What find_fs saw, kept for the machine-readable record.
static struct {
  uint8_t type;
  bool active;
  uint32_t lba, sectors;
} parts[4];
static int n_parts;
static const char* part_table = "unreadable";  // none | superfloppy | mbr | gpt
static bool exfat_seen;

static fs_t fs;
static uint8_t fat_cache[512];
static uint32_t fat_cache_lba = 0xFFFFFFFF;

static uint32_t fat_backup_reads, fat_unreadable;

// A FAT sector, from the first copy or -- if that sector is bad -- the next.
// FAT keeps every copy identical, which is exactly what makes this safe.
static int fat_byte(uint32_t off) {
  uint32_t lba = fs.fat_lba + off / 512;
  if (lba != fat_cache_lba) {
    bool got = read_meta(lba, fat_cache);
    for (uint8_t copy = 1; !got && copy < fs.nfats; copy++) {
      got = read_meta(lba + copy * fs.fat_sectors, fat_cache);
      if (got) fat_backup_reads++;
    }
    if (!got) {
      fat_unreadable++;
      return -1;
    }
    fat_cache_lba = lba;
  }
  return fat_cache[off % 512];
}

// FAT entry for `cl`, or 0xFFFFFFFF on read error.
static uint32_t fat_get(uint32_t cl) {
  int a, b, c, d;
  switch (fs.type) {
    case 12: {
      uint32_t off = cl + cl / 2;
      a = fat_byte(off); b = fat_byte(off + 1);
      if (a < 0 || b < 0) return 0xFFFFFFFF;
      uint32_t v = (uint32_t)a | ((uint32_t)b << 8);
      return (cl & 1) ? v >> 4 : v & 0xFFF;
    }
    case 16:
      a = fat_byte(cl * 2); b = fat_byte(cl * 2 + 1);
      if (a < 0 || b < 0) return 0xFFFFFFFF;
      return (uint32_t)a | ((uint32_t)b << 8);
    default:
      a = fat_byte(cl * 4); b = fat_byte(cl * 4 + 1); c = fat_byte(cl * 4 + 2); d = fat_byte(cl * 4 + 3);
      if (a < 0 || b < 0 || c < 0 || d < 0) return 0xFFFFFFFF;
      return ((uint32_t)a | ((uint32_t)b << 8) | ((uint32_t)c << 16) | ((uint32_t)d << 24)) & 0x0FFFFFFF;
  }
}

static bool fat_is_eoc(uint32_t v) {
  return fs.type == 12 ? v >= 0xFF8 : fs.type == 16 ? v >= 0xFFF8 : v >= 0x0FFFFFF8;
}

static bool cluster_valid(uint32_t cl) {
  return cl >= 2 && cl < fs.clusters + 2;
}

static uint32_t cluster_lba(uint32_t cl) {
  return fs.data_lba + (cl - 2) * fs.spc;
}

// Parse a FAT boot sector at `lba`. Returns true if it is one.
static bool parse_bpb(const uint8_t* s, uint32_t lba) {
  if (s[510] != 0x55 || s[511] != 0xAA) return false;
  if (s[0] != 0xEB && s[0] != 0xE9) return false;
  uint16_t bps = le16(s + 11);
  uint8_t spc = s[13];
  uint16_t reserved = le16(s + 14);
  uint8_t nfats = s[16];
  uint16_t root_entries = le16(s + 17);
  uint32_t total = le16(s + 19) ? le16(s + 19) : le32(s + 32);
  uint32_t fatsz = le16(s + 22) ? le16(s + 22) : le32(s + 36);
  if (bps != 512 || spc == 0 || (spc & (spc - 1)) || reserved == 0 || nfats == 0 || fatsz == 0) return false;

  fs.part_lba = lba;
  fs.total_sectors = total;
  fs.spc = spc;
  fs.fat_lba = lba + reserved;
  fs.fat_sectors = fatsz;
  fs.root_sectors = (root_entries * 32 + 511) / 512;
  fs.root_lba = fs.fat_lba + nfats * fatsz;
  fs.data_lba = fs.root_lba + fs.root_sectors;
  fs.clusters = (total - (fs.data_lba - lba)) / spc;
  fs.type = fs.clusters < 4085 ? 12 : fs.clusters < 65525 ? 16 : 32;

  const uint8_t* ext = s + (fs.type == 32 ? 71 : 43);  // volume label in the extended BPB
  memcpy(fs.label, ext, 11); fs.label[11] = 0;
  memcpy(fs.fstype, ext + 11, 8); fs.fstype[8] = 0;
  fs.root_cluster = fs.type == 32 ? le32(s + 44) : 0;

  memcpy(fs.oem, s + 3, 8); fs.oem[8] = 0;
  fs.nfats = nfats;
  fs.vol_serial = le32(s + (fs.type == 32 ? 67 : 39));

  printf("\r\n=== FAT%d file system at LBA %lu ===\r\n", fs.type, (unsigned long)lba);
  printf("  OEM name \"%.8s\", label \"%s\", type string \"%s\", serial %08lX\r\n", s + 3, fs.label,
         fs.fstype, (unsigned long)le32(s + (fs.type == 32 ? 67 : 39)));
  printf("  %u sectors/cluster (%u KiB), %u FATs of %lu sectors, %lu clusters\r\n", spc, spc / 2, nfats,
         (unsigned long)fatsz, (unsigned long)fs.clusters);
  printf("  FAT at LBA %lu, root at %s %lu, data at LBA %lu\r\n", (unsigned long)fs.fat_lba,
         fs.type == 32 ? "cluster" : "LBA", (unsigned long)(fs.type == 32 ? fs.root_cluster : fs.root_lba),
         (unsigned long)fs.data_lba);
  return true;
}

static bool find_fs(void) {
  static uint8_t s[512];
  if (!read_meta(0, s)) {
    // No MBR to say where the partition is. Formatters put it at a few
    // conventional places, and FAT32 keeps a backup boot sector 6 sectors
    // in, so try those before giving up on the file system.
    printf("  sector 0 unreadable -- trying the usual partition starts\r\n");
    static const uint32_t starts[] = {8192, 2048, 63, 32, 16, 1};
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); i++) {
      for (uint32_t backup = 0; backup <= 6; backup += 6) {
        if (read_meta(starts[i] + backup, s) && parse_bpb(s, starts[i])) {
          part_table = "unreadable";
          printf("  found a %sboot sector for a partition at LBA %lu\r\n", backup ? "backup " : "",
                 (unsigned long)starts[i]);
          return true;
        }
      }
    }
    return false;
  }
  printf("\r\n=== Partitioning ===\r\n");
  if (s[510] != 0x55 || s[511] != 0xAA) {
    part_table = "none";
    printf("  sector 0 has no 55AA signature: no MBR and no boot sector\r\n");
    return false;
  }
  if (parse_bpb(s, 0)) {
    part_table = "superfloppy";
    printf("  (no partition table: the file system starts at sector 0)\r\n");
    return true;
  }
  if (!memcmp(s + 3, "EXFAT   ", 8)) {
    part_table = "superfloppy";
    exfat_seen = true;
    printf("  exFAT at sector 0 -- directory parsing not supported, raw carving only\r\n");
    return false;
  }

  part_table = "mbr";
  uint32_t candidate = 0;
  for (int i = 0; i < 4; i++) {
    const uint8_t* e = s + 446 + 16 * i;
    uint8_t type = e[4];
    uint32_t start = le32(e + 8), count = le32(e + 12);
    if (type == 0) continue;
    parts[n_parts].type = type;
    parts[n_parts].active = e[0] == 0x80;
    parts[n_parts].lba = start;
    parts[n_parts].sectors = count;
    n_parts++;
    if (type == 0xEE) part_table = "gpt";
    printf("  partition %d: type 0x%02X%s, LBA %lu, %lu sectors (%lu MiB)\r\n", i + 1, type,
           e[0] == 0x80 ? " (active)" : "", (unsigned long)start, (unsigned long)count,
           (unsigned long)(count / 2048));
    if (type == 0xEE) printf("    protective MBR: GPT is not supported here\r\n");
    if (!candidate && (type == 0x01 || type == 0x04 || type == 0x06 || type == 0x0B ||
                       type == 0x0C || type == 0x0E || type == 0x07)) {
      candidate = start;
    }
  }
  if (!candidate) {
    printf("  no FAT partition found\r\n");
    return false;
  }
  if (!read_meta(candidate, s) && !read_meta(candidate + 6, s)) return false;  // +6: FAT32 backup
  if (!memcmp(s + 3, "EXFAT   ", 8)) {
    exfat_seen = true;
    printf("  partition is exFAT -- directory parsing not supported, raw carving only\r\n");
    return false;
  }
  if (!parse_bpb(s, candidate)) {
    printf("  partition at LBA %lu does not hold a FAT boot sector\r\n", (unsigned long)candidate);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Directory walk
// ---------------------------------------------------------------------------

static uint32_t n_live_files, n_live_dirs, n_del_files, n_del_dirs, n_del_intact;

// ----- What an entry is: user content, an OS artifact, or in a trash -------
//
// Nearly every card has been mounted by some OS, which leaves files of its
// own (Windows' System Volume Information above all). Those say "this card was
// mounted", not "this card holds someone's data", so they are counted apart.
// A trash folder is an artifact too, but what is *in* it is user data the
// user deleted -- "present" only in the narrowest sense -- so it gets a class
// of its own.

// K_OS_SHELL: a folder an OS creates whose contents are nonetheless user
// data (Android's LOST.DIR, which holds fragments recovered after corruption).
typedef enum { K_USER, K_OS, K_OS_SHELL, K_TRASH_DIR, K_TRASHED } kind_t;
typedef enum { OS_NONE = 0, OS_WINDOWS = 1, OS_MACOS = 2, OS_LINUX = 4, OS_ANDROID = 8 } os_t;

typedef struct {
  uint8_t kind;  // kind_t
  uint8_t os;    // os_t of the artifact or trash this entry belongs to
} cls_t;

static const cls_t CLS_USER = {K_USER, OS_NONE};

static uint32_t n_user_files, n_user_dirs, n_os_files, n_os_dirs, n_trashed_files, n_trashed_dirs;
static uint32_t n_del_user, n_del_os;  // deleted directory entries (files + dirs), by class
static uint8_t os_seen;                // os_t bits of every artifact found

// Case-insensitive compare. FAT names are case-insensitive, and a deleted
// entry without a long name has lost its first character, so that one may be
// skipped.
static bool name_is(const char* name, const char* want, bool skip_first) {
  if (skip_first) {
    if (!*name || !*want) return false;
    name++;
    want++;
  }
  for (; *name && *want; name++, want++) {
    char a = *name, b = *want;
    if (a >= 'a' && a <= 'z') a -= 32;
    if (b >= 'a' && b <= 'z') b -= 32;
    if (a != b) return false;
  }
  return *name == 0 && *want == 0;
}

static bool name_starts(const char* name, const char* prefix, bool skip_first) {
  if (skip_first) {
    if (!*name || !*prefix) return false;
    name++;
    prefix++;
  }
  for (; *prefix; name++, prefix++) {
    char a = *name, b = *prefix;
    if (a >= 'a' && a <= 'z') a -= 32;
    if (b >= 'a' && b <= 'z') b -= 32;
    if (a != b) return false;
  }
  return true;
}

static bool all_digits(const char* s) {
  if (!*s) return false;
  for (; *s; s++)
    if (*s < '0' || *s > '9') return false;
  return true;
}

// Class of an entry named `name` inside a directory of class `parent`.
// `blind` = deleted with no long name, so the first character is unknown.
static cls_t classify(cls_t parent, const char* name, bool is_dir, int depth, bool blind) {
  cls_t os = {K_OS, parent.os};

  // Inside an artifact: more of it. Inside trashed data: more trashed data.
  if (parent.kind == K_OS) return parent;
  if (parent.kind == K_TRASHED) return parent;

  // Anywhere: per-folder metadata files.
  static const char* const mac_files[] = {".DS_Store", ".metadata_never_index", ".apdisk", ".VolumeIcon.icns"};
  for (unsigned i = 0; i < sizeof(mac_files) / sizeof(mac_files[0]); i++)
    if (!is_dir && name_is(name, mac_files[i], blind)) return (cls_t){K_OS, OS_MACOS};
  if (!is_dir && name_starts(name, "._", blind)) return (cls_t){K_OS, OS_MACOS};  // AppleDouble
  static const char* const win_files[] = {"Thumbs.db", "ehthumbs.db", "desktop.ini"};
  for (unsigned i = 0; i < sizeof(win_files) / sizeof(win_files[0]); i++)
    if (!is_dir && name_is(name, win_files[i], blind)) return (cls_t){K_OS, OS_WINDOWS};

  // Inside a trash folder: its own bookkeeping is an artifact, the rest is
  // what was thrown away.
  if (parent.kind == K_TRASH_DIR) {
    switch (parent.os) {
      case OS_WINDOWS:
        // $RECYCLE.BIN/<SID>/ holds $I<id> (index) and $R<id> (data) pairs;
        // XP's RECYCLER keeps INFO2 and Dc<n> files.
        if (is_dir && name_starts(name, "S-1-", blind)) return parent;
        if (!is_dir && (name_starts(name, "$I", blind) || name_is(name, "INFO2", blind))) return os;
        break;
      case OS_LINUX:
        // .Trash-<uid>/files/ holds the data, info/ the .trashinfo records.
        if (is_dir && name_is(name, "files", blind)) return parent;
        if (name_is(name, "info", blind) || name_is(name, "directorysizes", blind)) return os;
        break;
      case OS_MACOS:
        if (is_dir && all_digits(name)) return parent;  // .Trashes/<uid>/
        break;
    }
    return (cls_t){K_TRASHED, parent.os};
  }

  // Root only: whole-volume folders an OS creates on mount.
  if (depth == 0 && is_dir) {
    if (name_is(name, "System Volume Information", blind)) return (cls_t){K_OS, OS_WINDOWS};
    if (name_is(name, "$RECYCLE.BIN", blind) || name_is(name, "RECYCLER", blind) ||
        name_is(name, "RECYCLED", blind))
      return (cls_t){K_TRASH_DIR, OS_WINDOWS};
    if (name_starts(name, ".Trash-", blind)) return (cls_t){K_TRASH_DIR, OS_LINUX};
    if (name_is(name, ".Trashes", blind)) return (cls_t){K_TRASH_DIR, OS_MACOS};
    static const char* const mac_dirs[] = {".Spotlight-V100", ".fseventsd", ".TemporaryItems",
                                           ".DocumentRevisions-V100"};
    for (unsigned i = 0; i < sizeof(mac_dirs) / sizeof(mac_dirs[0]); i++)
      if (name_is(name, mac_dirs[i], blind)) return (cls_t){K_OS, OS_MACOS};
    if (name_is(name, "LOST.DIR", blind)) return (cls_t){K_OS_SHELL, OS_ANDROID};
  }
  return CLS_USER;
}

static const char* os_name(uint8_t os) {
  return os == OS_WINDOWS ? "Windows" : os == OS_MACOS ? "macOS" : os == OS_LINUX ? "Linux" :
         os == OS_ANDROID ? "Android" : "?";
}

static void print_class(cls_t c) {
  switch (c.kind) {
    case K_OS: printf("  [%s artifact]", os_name(c.os)); break;
    case K_OS_SHELL: printf("  [%s folder]", os_name(c.os)); break;
    case K_TRASH_DIR: printf("  [%s trash]", os_name(c.os)); break;
    case K_TRASHED: printf("  [in %s trash]", os_name(c.os)); break;
    default: break;
  }
}

// Live files, remembered so their slack can be carved. Slack -- the part of a
// file's last cluster beyond its end -- keeps whatever the cluster held
// before, which is often the start of a deleted file whose first clusters
// were reused.
#define MAX_SLACK_FILES 1024
static struct {
  uint32_t cluster;
  uint32_t size;
} slack_files[MAX_SLACK_FILES];
static uint32_t n_slack_files;
static uint64_t live_bytes;

// Describe what the first sector of cluster `cl` holds.
// Small files are shown in full, as hex and ASCII: they are usually metadata
// (volume GUIDs, settings) that say something about the card's history.
#define MAX_DUMP_BYTES 256

static void dump_bytes(const uint8_t* s, uint32_t size) {
  for (uint32_t off = 0; off < size; off += 16) {
    printf("           %04lX  ", (unsigned long)off);
    for (uint32_t i = 0; i < 16; i++) {
      if (off + i < size) printf("%02X ", s[off + i]);
      else printf("   ");
    }
    printf(" |");
    for (uint32_t i = 0; i < 16 && off + i < size; i++) {
      uint8_t ch = s[off + i];
      printf("%c", ch >= 0x20 && ch < 0x7F ? ch : '.');
    }
    printf("|\r\n");
  }
}

static void describe_sector(const uint8_t* s) {
  const sig_t* sig = identify(s);
  if (sig) printf("%s", sig_label);
  else {
    sec_kind_t k = sector_kind(s);
    printf("%s", k == SEC_ZERO ? "zeros" : k == SEC_FF ? "0xFF fill" : "data, no known header");
  }
}

static void describe_cluster(uint32_t cl) {
  static uint8_t s[512];
  if (!cluster_valid(cl)) {
    printf("start cluster out of range");
    return;
  }
  if (!read_meta_quiet(cluster_lba(cl), s)) {
    printf("unreadable");
    return;
  }
  describe_sector(s);
}

static void print_date(const uint8_t* d) {
  uint16_t t = le16(d + 22), dt = le16(d + 24);
  if (dt == 0) {
    printf("                ");
    return;
  }
  printf("%04u-%02u-%02u %02u:%02u", 1980 + (dt >> 9), (dt >> 5) & 15, dt & 31, t >> 11, (t >> 5) & 63);
}

// Largest read the app asks for. The card pays milliseconds of access
// latency per read command, so big reads are what make a full pass fast.
#define CARVE_MAX_BLOCKS 64
static uint8_t carve_buf[CARVE_MAX_BLOCKS * 512] __attribute__((aligned(4)));

static uint32_t carve_blocks(void) {
  uint32_t max = sdmmc_max_blocks();
  return max > CARVE_MAX_BLOCKS ? CARVE_MAX_BLOCKS : max ? max : 1;
}

// ----- Live-file check -------------------------------------------------------
//
// Every live file is read end to end through its cluster chain. That is the
// only way to know whether the user's data is intact -- the free-space pass
// never touches it -- and it says which files are damaged, which is what
// matters to whoever owns the card.

static uint32_t n_files_checked, n_files_damaged;
static uint64_t file_sectors, file_sectors_unread, file_sectors_bad;
static uint8_t first_sector[512] __attribute__((aligned(4)));
static bool first_sector_ok;
static uint32_t file_bad;  // of the last verify_file: tried and failed (the rest of its unread were skipped)

// Read a file's chain. Returns sectors that could not be read: bad, skipped
// next to bad ones, or beyond a broken chain. Leaves its first sector in
// `first_sector` (valid if `first_sector_ok`).
static uint32_t verify_file(uint32_t cl, uint32_t size) {
  uint32_t need = (size + 511) / 512, remaining = need, guard = 0;
  uint32_t max = carve_blocks();
  uint64_t lost0 = bad_sectors + skipped_sectors, bad0 = bad_sectors;
  first_sector_ok = false;
  reset_skip();
  while (remaining && cluster_valid(cl) && guard++ <= fs.clusters) {
    uint32_t n = remaining < fs.spc ? remaining : fs.spc;
    uint32_t lba = cluster_lba(cl);
    for (uint32_t off = 0; off < n; off += max) {
      uint32_t k = n - off < max ? n - off : max;
      uint64_t ok;
      read_resilient(lba + off, k, carve_buf, &ok);
      if (remaining == need && off == 0 && (ok & 1)) {
        memcpy(first_sector, carve_buf, 512);
        first_sector_ok = true;
      }
    }
    remaining -= n;
    if (!remaining) break;
    uint32_t next = fat_get(cl);
    if (next == 0xFFFFFFFF || fat_is_eoc(next) || !cluster_valid(next)) break;  // chain broken
    cl = next;
  }
  uint32_t unread = (uint32_t)(bad_sectors + skipped_sectors - lost0) + remaining;
  file_bad = (uint32_t)(bad_sectors - bad0);
  n_files_checked++;
  file_sectors += need;
  file_sectors_unread += unread;
  file_sectors_bad += file_bad;
  if (unread) n_files_damaged++;
  reset_skip();
  return unread;
}

static void walk_dir(uint32_t cluster, int depth, bool in_deleted, const char* path, cls_t cls);

static void dir_entry(const uint8_t* d, const char* lfn, int depth, bool in_deleted, const char* path,
                      cls_t parent) {
  uint8_t attr = d[11];
  bool deleted = d[0] == 0xE5;

  // 8.3 name. 0x05 stands for a real leading 0xE5. A deleted entry has lost
  // its first character; the long name, if any, still has it.
  char name[13];
  int n = 0;
  for (int i = 0; i < 8 && d[i] != ' '; i++) {
    char ch = (char)d[i];
    if (i == 0 && d[0] == 0x05) ch = (char)0xE5;
    if (i == 0 && deleted) ch = lfn[0] ? lfn[0] : '_';
    name[n++] = ch;
  }
  if (d[8] != ' ') {
    name[n++] = '.';
    for (int i = 8; i < 11 && d[i] != ' '; i++) name[n++] = (char)d[i];
  }
  name[n] = 0;
  if (!strcmp(name, ".") || !strcmp(name, "..")) return;
  const char* shown = lfn[0] ? lfn : name;

  uint32_t cl = le16(d + 26) | (fs.type == 32 ? ((uint32_t)le16(d + 20) << 16) : 0);
  uint32_t size = le32(d + 28);
  const char* state = deleted ? "DELETED" : in_deleted ? "DELETED*" : "PRESENT";

  bool is_dir = attr & 0x10;
  bool gone = deleted || in_deleted;
  cls_t cls = classify(parent, shown, is_dir, depth, deleted && !lfn[0]);
  if (cls.kind != K_USER) os_seen |= cls.os;
  // A deleted entry counts as user data unless it is an OS artifact; deleted
  // from a trash is still deleted user data.
  bool os_item = cls.kind == K_OS || cls.kind == K_OS_SHELL || cls.kind == K_TRASH_DIR;
  if (gone) {
    if (os_item) n_del_os++; else n_del_user++;
  }

  if (is_dir) {
    if (gone) {
      n_del_dirs++;
    } else {
      n_live_dirs++;
      if (cls.kind == K_TRASHED) n_trashed_dirs++;
      else if (os_item) n_os_dirs++;
      else n_user_dirs++;
    }
    printf("  %-8s <DIR>       ", state);
    print_date(d);
    printf("  %s%s/", path, shown);
    print_class(cls);
    printf("\r\n");
    if (depth + 1 < MAX_DEPTH && cluster_valid(cl)) {
      char sub[160];
      snprintf(sub, sizeof(sub), "%s%s/", path, shown);
      // Inside an Android LOST.DIR, entries are classified as ordinary ones.
      walk_dir(cl, depth + 1, gone, sub, cls.kind == K_OS_SHELL ? CLS_USER : cls);
    }
    return;
  }

  printf("  %-8s %10lu  ", state, (unsigned long)size);
  print_date(d);
  printf("  %s%s", path, shown);
  print_class(cls);
  printf("\r\n");
  if (!gone) {
    if (cls.kind == K_TRASHED) n_trashed_files++;
    else if (os_item) n_os_files++;
    else n_user_files++;
  }
  if (gone) {
    n_del_files++;
    printf("           start cluster %lu: ", (unsigned long)cl);
    if (size == 0 || cl == 0) {
      printf("no data recorded\r\n");
      return;
    }
    uint32_t next = cluster_valid(cl) ? fat_get(cl) : 0xFFFFFFFF;
    describe_cluster(cl);
    if (next == 0) {
      n_del_intact++;
      printf(" -- cluster is free, not reused: recoverable\r\n");
    } else {
      printf(" -- cluster reallocated since: likely overwritten\r\n");
    }
  } else {
    n_live_files++;
    live_bytes += size;
    if (n_slack_files < MAX_SLACK_FILES && cluster_valid(cl)) {
      slack_files[n_slack_files].cluster = cl;
      slack_files[n_slack_files].size = size;
      n_slack_files++;
    }
    if (size && cluster_valid(cl)) {
      uint32_t need = (size + 511) / 512;
      uint32_t unread = verify_file(cl, size);
      printf("           starts at cluster %lu: ", (unsigned long)cl);
      if (first_sector_ok) describe_sector(first_sector);
      else printf("unreadable");
      if (unread)
        printf("; DAMAGED: %lu of %lu sectors not read (%lu bad, %lu skipped next to bad)\r\n",
               (unsigned long)unread, (unsigned long)need, (unsigned long)file_bad,
               (unsigned long)(unread - file_bad));
      else
        printf("; all %lu sectors read\r\n", (unsigned long)need);
      if (size <= MAX_DUMP_BYTES && first_sector_ok) dump_bytes(first_sector, size);
    }
  }
}

static uint32_t dir_sectors_unreadable;

// Walk a directory. `cluster` 0 means the FAT12/16 fixed root. A deleted
// directory's chain was freed with it, so only its first cluster is read.
static void walk_dir(uint32_t cluster, int depth, bool in_deleted, const char* path, cls_t cls) {
  uint8_t s[512];
  char lfn[128];
  lfn[0] = 0;
  uint32_t guard = 0;
  bool fixed_root = cluster == 0;

  while (fixed_root || cluster_valid(cluster)) {
    uint32_t first = fixed_root ? fs.root_lba : cluster_lba(cluster);
    uint32_t count = fixed_root ? fs.root_sectors : fs.spc;
    for (uint32_t i = 0; i < count; i++) {
      if (!read_meta(first + i, s)) {
        // Lose this sector's 16 entries, not the rest of the directory.
        dir_sectors_unreadable++;
        printf("  ! %s: directory sector at LBA %lu unreadable, its 16 entries unknown\r\n", path,
               (unsigned long)(first + i));
        lfn[0] = 0;
        continue;
      }
      for (int e = 0; e < 16; e++) {
        const uint8_t* d = s + 32 * e;
        if (d[0] == 0x00) return;  // end of directory
        if (d[11] == 0x0F) {
          // Long-name fragment: 13 UTF-16 characters, stored last part first,
          // so each one is prepended. Non-ASCII is shown as '?'.
          static const uint8_t pos[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
          char part[14];
          int k = 0;
          for (int j = 0; j < 13; j++) {
            uint16_t ch = le16(d + pos[j]);
            if (ch == 0 || ch == 0xFFFF) break;
            part[k++] = ch < 0x80 ? (char)ch : '?';
          }
          part[k] = 0;
          size_t old = strlen(lfn);
          if (old + k < sizeof(lfn)) {
            memmove(lfn + k, lfn, old + 1);
            memcpy(lfn, part, k);
          }
          continue;
        }
        if ((d[11] & 0x08) && !(d[11] & 0x10)) {
          if (depth == 0 && d[0] != 0xE5) printf("  volume label \"%.11s\"\r\n", d);
          lfn[0] = 0;
          continue;
        }
        dir_entry(d, lfn, depth, in_deleted, path, cls);
        lfn[0] = 0;
      }
    }
    if (fixed_root || in_deleted) return;
    uint32_t next = fat_get(cluster);
    if (fat_is_eoc(next) || ++guard > fs.clusters) return;
    cluster = next;
  }
}

// ---------------------------------------------------------------------------
// Free-space carving
// ---------------------------------------------------------------------------

typedef struct {
  bool active;
  uint32_t start_lba;
  uint32_t sectors;
  bool labeled;
  bool video;
  char label[32];
  uint8_t head[64];   // first bytes of the run, for formats not recognised
} run_t;

static run_t run;
static uint32_t n_headers, n_video_headers, n_raw_runs, n_raw_printed, n_nested;
static uint64_t residual_sectors;

static void end_run(void) {
  if (!run.active) return;
  uint32_t kib = run.sectors / 2;
  if (run.labeled) {
    if (n_headers <= MAX_HEADER_LINES) {
      printf("  HEADER  %-26s LBA %8lu  then %lu KiB of contiguous data\r\n", run.label,
             (unsigned long)run.start_lba, (unsigned long)kib);
    }
  } else {
    n_raw_runs++;
    if (run.sectors >= 8 && n_raw_printed < MAX_RAW_RUNS) {
      n_raw_printed++;
      printf("  DATA    no recognised header       LBA %8lu  %lu KiB\r\n", (unsigned long)run.start_lba,
             (unsigned long)kib);
      for (unsigned off = 0; off < sizeof(run.head); off += 16) {
        printf("          %s ", off ? "           " : "first bytes");
        for (unsigned i = off; i < off + 16; i++) printf("%02X ", run.head[i]);
        printf(" |");
        for (unsigned i = off; i < off + 16; i++) {
          uint8_t ch = run.head[i];
          printf("%c", ch >= 0x20 && ch < 0x7F ? ch : '.');
        }
        printf("|\r\n");
      }
    }
  }
  run.active = false;
}

static void carve_sector(const uint8_t* s, uint32_t lba, bool cluster_start) {
  if (sector_kind(s) != SEC_DATA) {
    end_run();
    return;
  }
  residual_sectors++;
  const sig_t* sig = identify(s);
  // A weak signature (one that also occurs inside files) starts a new file
  // only where a file could start: at a cluster boundary, or after a gap.
  bool new_file = sig && (sig->strong || cluster_start || !run.active);
  if (sig && !new_file) n_nested++;
  if (new_file) {
    end_run();
    n_headers++;
    if (sig->video) n_video_headers++;
    run.active = true;
    run.start_lba = lba;
    run.sectors = 0;
    run.labeled = true;
    memcpy(run.head, s, sizeof(run.head));
    run.video = sig->video;
    snprintf(run.label, sizeof(run.label), "%s", sig_label);
  } else if (!run.active) {
    run.active = true;
    run.start_lba = lba;
    run.sectors = 0;
    run.labeled = false;
    memcpy(run.head, s, sizeof(run.head));
  }
  run.sectors++;
}

static void progress(uint32_t done, uint32_t total, uint32_t t0, uint64_t sectors_read) {
  uint32_t ms = now_ms() - t0;
  uint32_t kbps = ms ? (uint32_t)(sectors_read * 512 / ms) : 0;  // bytes/ms == kB/s
  printf("  .. %3lu%%  %lu s  %lu kB/s", (unsigned long)((uint64_t)done * 100 / total),
         (unsigned long)(ms / 1000), (unsigned long)kbps);
  if (bad_sectors || skipped_sectors)
    printf("  bad %lu, skipped %lu", (unsigned long)bad_sectors, (unsigned long)skipped_sectors);
  printf("\r\n");
}

static uint32_t carve_kBps;  // throughput of the carving pass, for the record
static uint32_t clusters_unknown;  // FAT entry unreadable in every copy

// Carve `count` contiguous sectors from `lba`. Returns sectors read. A sector
// that could not be read ends any run, as a gap in the data does.
static uint32_t carve_extent(uint32_t lba, uint32_t count, uint32_t max) {
  uint32_t got = 0;
  for (uint32_t off = 0; off < count; off += max) {
    uint32_t n = count - off < max ? count - off : max;
    uint64_t ok;
    got += read_resilient(lba + off, n, carve_buf, &ok);
    for (uint32_t i = 0; i < n; i++) {
      uint32_t s = lba + off + i;
      if (ok & (1ull << i)) carve_sector(carve_buf + 512 * i, s, (s - fs.data_lba) % fs.spc == 0);
      else end_run();
    }
  }
  return got;
}

// Read every sector of every free cluster, merging runs of adjacent free
// clusters into single reads.
static void carve_free_space(void) {
  reset_skip();
  uint32_t max = carve_blocks();
  uint32_t per_read = max / fs.spc ? max / fs.spc : 1;  // clusters per read
  uint32_t t0 = now_ms();
  uint64_t sectors_read = 0;
  uint32_t free_clusters = 0, step = fs.clusters / 10 + 1, next = 2 + step;
  uint32_t end = fs.clusters + 2;

  printf("\r\n=== Free-space carving (every sector of %lu clusters) ===\r\n", (unsigned long)fs.clusters);
  for (uint32_t cl = 2; cl < end;) {
    if (cl >= next) {
      progress(cl - 2, fs.clusters, t0, sectors_read);
      next += step;
    }
    uint32_t entry = fat_get(cl);
    if (entry != 0) {
      // Allocated -- or its FAT entry unreadable in every copy, in which
      // case whether it is free is unknown, and it is left alone.
      if (entry == 0xFFFFFFFF) clusters_unknown++;
      end_run();
      cl++;
      continue;
    }
    uint32_t first = cl, n = 0;
    while (cl < end && n < per_read && fat_get(cl) == 0) {
      cl++;
      n++;
    }
    free_clusters += n;
    sectors_read += carve_extent(cluster_lba(first), n * fs.spc, max);
  }
  end_run();
  uint32_t ms = now_ms() - t0;
  carve_kBps = ms ? (uint32_t)(sectors_read * 512 / ms) : 0;
  printf("  done: %lu free clusters, %lu MiB read in %lu.%lu s (%lu kB/s)\r\n", (unsigned long)free_clusters,
         (unsigned long)(sectors_read / 2048), (unsigned long)(ms / 1000), (unsigned long)(ms % 1000 / 100),
         (unsigned long)(ms ? sectors_read * 512 / ms : 0));
  if (n_headers > MAX_HEADER_LINES) printf("  (%lu more headers not listed)\r\n", (unsigned long)(n_headers - MAX_HEADER_LINES));
}

// Carve what free-space carving does not cover but photorec's whole-disk
// mode does: the slack at the end of each live file's last cluster, and any
// sectors outside the partition.
static void carve_slack_and_gaps(uint32_t card_blocks) {
  uint32_t max = carve_blocks();
  uint32_t before = n_headers;
  reset_skip();

  printf("\r\n=== File slack (%lu live files) ===\r\n", (unsigned long)n_slack_files);
  uint32_t slack_sectors = 0;
  for (uint32_t i = 0; i < n_slack_files; i++) {
    uint32_t cl = slack_files[i].cluster, size = slack_files[i].size;
    uint32_t cluster_bytes = fs.spc * 512u;
    // Follow the chain to the cluster holding the last byte.
    uint32_t hops = size ? (size - 1) / cluster_bytes : 0, guard = 0;
    while (hops-- && guard++ < fs.clusters) {
      uint32_t next = fat_get(cl);
      if (fat_is_eoc(next) || !cluster_valid(next)) break;
      cl = next;
    }
    uint32_t used = size ? ((size - 1) % cluster_bytes) / 512 + 1 : 0;  // sectors holding file data
    if (used >= fs.spc) continue;
    end_run();
    reset_skip();  // each file's slack is its own pass
    slack_sectors += carve_extent(cluster_lba(cl) + used, fs.spc - used, max);
    end_run();
  }
  printf("  %lu slack sectors carved, %lu header(s)\r\n", (unsigned long)slack_sectors,
         (unsigned long)(n_headers - before));

  before = n_headers;
  reset_skip();
  uint32_t vol_end = fs.part_lba + fs.total_sectors;
  printf("\r\n=== Outside the partition ===\r\n");
  if (fs.part_lba > 1) printf("  before: LBA 1-%lu\r\n", (unsigned long)(fs.part_lba - 1));
  if (vol_end < card_blocks) printf("  after:  LBA %lu-%lu\r\n", (unsigned long)vol_end, (unsigned long)(card_blocks - 1));
  uint32_t gap = 0;
  if (fs.part_lba > 1) gap += carve_extent(1, fs.part_lba - 1, max);
  end_run();
  if (vol_end < card_blocks) gap += carve_extent(vol_end, card_blocks - vol_end, max);
  end_run();
  printf("  %lu sectors carved, %lu header(s)\r\n", (unsigned long)gap, (unsigned long)(n_headers - before));
}

// ---------------------------------------------------------------------------
// Probe: is it the media or the bus?
// ---------------------------------------------------------------------------
//
// Single-block reads spread across the card, under several bus settings. A
// region of bad media fails at the same addresses whatever the bus does; a
// bus problem (a poor contact on DAT1-3, marginal signal at 25 MHz) fails
// everywhere at one setting and works at another. The best setting found is
// left in place for the rest of the run.

#define FAST_TIMEOUT_MS 25
static uint32_t data_timeout_ms = 100;  // in force for the scan, for the report

#define PROBE_CONFIGS 4
static const struct { uint32_t width, hz; } probe_cfg[PROBE_CONFIGS] = {
  {4, 25000000}, {4, 10000000}, {1, 25000000}, {1, 5000000},
};

static void probe(uint32_t blocks) {
  uint32_t lbas[] = {0, 1, 64, 2048, 8192, 65536, blocks / 8, blocks / 4, blocks / 2, blocks / 4 * 3, blocks - 1};
  const int n = sizeof(lbas) / sizeof(lbas[0]);
  int ok[PROBE_CONFIGS] = {0};
  sdmmc_error_t first_err[PROBE_CONFIGS];
  bool have_err[PROBE_CONFIGS] = {false};
  char cell[PROBE_CONFIGS][sizeof(lbas) / sizeof(lbas[0])][12];

  printf("\r\n=== Probe: single-block reads across the card ===\r\n");
  for (int c = 0; c < PROBE_CONFIGS; c++) {
    uint32_t actual = 0;
    if (sdmmc_set_bus(probe_cfg[c].width, probe_cfg[c].hz, &actual) != RETURNCODE_SUCCESS) {
      for (int i = 0; i < n; i++) snprintf(cell[c][i], sizeof(cell[c][i]), "no bus");
      continue;
    }
    for (int i = 0; i < n; i++) {
      uint32_t t0 = now_ms();
      returncode_t rc = sdmmc_read(lbas[i], 1, carve_buf);
      uint32_t ms = now_ms() - t0;
      if (rc == RETURNCODE_SUCCESS) {
        ok[c]++;
        snprintf(cell[c][i], sizeof(cell[c][i]), "ok %lums", (unsigned long)ms);
      } else {
        sdmmc_error_t e;
        static const char* const st[] = {"?", "resp", "status", "data", "busy", "stuck"};
        bool got = sdmmc_error_info(0, &e) == RETURNCODE_SUCCESS;
        if (got && !have_err[c]) {
          first_err[c] = e;
          have_err[c] = true;
        }
        snprintf(cell[c][i], sizeof(cell[c][i]), "FAIL %s", got && e.stage < 6 ? st[e.stage] : "?");
      }
    }
  }

  printf("  %10s", "LBA");
  for (int c = 0; c < PROBE_CONFIGS; c++)
    printf("  %lu-bit %2lu MHz", (unsigned long)probe_cfg[c].width, (unsigned long)(probe_cfg[c].hz / 1000000));
  printf("\r\n");
  for (int i = 0; i < n; i++) {
    printf("  %10lu", (unsigned long)lbas[i]);
    for (int c = 0; c < PROBE_CONFIGS; c++) printf("  %-13s", cell[c][i]);
    printf("\r\n");
  }
  int best = 0;
  for (int c = 0; c < PROBE_CONFIGS; c++) {
    printf("  %lu-bit %2lu MHz: %d/%d read", (unsigned long)probe_cfg[c].width,
           (unsigned long)(probe_cfg[c].hz / 1000000), ok[c], n);
    if (have_err[c]) {
      printf("; first failure ");
      print_error(&first_err[c]);
    }
    printf("\r\n");
    if (ok[c] > ok[best]) best = c;  // ties keep the faster, earlier setting
  }
  uint32_t actual = 0;
  sdmmc_set_bus(probe_cfg[best].width, probe_cfg[best].hz, &actual);
  printf("  continuing at %lu-bit, %lu kHz\r\n", (unsigned long)probe_cfg[best].width,
         (unsigned long)(actual / 1000));

  // Bad blocks even at the best setting: every one will cost a full data
  // timeout. On failing media they never answer at all, while good blocks
  // answer in a few ms, so waiting the specification's 100 ms is nearly all
  // waste. Shorten it, and say so -- a block slower than this but good would
  // be counted bad.
  if (ok[best] < n && sdmmc_set_timeout(FAST_TIMEOUT_MS) == RETURNCODE_SUCCESS) {
    data_timeout_ms = FAST_TIMEOUT_MS;
    printf("  bad blocks present: data timeout shortened to %u ms for the scan\r\n", FAST_TIMEOUT_MS);
  }
}

static uint32_t bench_multi_kBps, bench_single_kBps;

// Raw read speed: the same 4 MiB at the start of the card, once with the
// largest reads the driver allows and once a block at a time. No processing,
// so this is the driver plus the card, without the carving loop.
static void benchmark(uint32_t blocks) {
  const uint32_t total = 8192;  // blocks = 4 MiB
  if (blocks < 2 * total) return;
  // Mid-card rather than at LBA 0: failing cards tend to fail at the front,
  // where the file system's busiest sectors are, and a speed measured across
  // bad blocks says nothing about the card.
  uint32_t base = (blocks / 2) & ~63u;
  printf("\r\n=== Read speed (4 MiB from LBA %lu, no processing) ===\r\n", (unsigned long)base);
  uint32_t sizes[2] = {carve_blocks(), 1};
  for (int k = 0; k < 2; k++) {
    uint32_t n = sizes[k], t0 = now_ms();
    for (uint32_t lba = base; lba < base + total; lba += n) {
      if (!read_sectors(lba, n, carve_buf)) {
        printf("  read failed inside the benchmark region; speed not measured\r\n");
        return;
      }
    }
    uint32_t ms = now_ms() - t0;
    uint32_t kbps = ms ? (uint32_t)((uint64_t)total * 512 / ms) : 0;
    printf("  %2lu block(s) per read: %lu ms, %lu kB/s\r\n", (unsigned long)n, (unsigned long)ms,
           (unsigned long)kbps);
    if (k == 0) bench_multi_kBps = kbps; else bench_single_kBps = kbps;
  }
}

// No usable file system: scan the whole card.
static void carve_raw(uint32_t blocks) {
  reset_skip();
  uint32_t max = carve_blocks();
  uint32_t t0 = now_ms();
  uint32_t step = blocks / 10 + 1, next = step;
  printf("\r\n=== Raw carving (every sector of the card) ===\r\n");
  for (uint32_t lba = 0; lba < blocks; lba += max) {
    uint32_t n = blocks - lba < max ? blocks - lba : max;
    if (lba >= next) {
      progress(lba, blocks, t0, lba);
      next += step;
    }
    uint64_t ok;
    read_resilient(lba, n, carve_buf, &ok);
    for (uint32_t i = 0; i < n; i++) {
      if (ok & (1ull << i)) carve_sector(carve_buf + 512 * i, lba + i, (lba + i) % 8 == 0);
      else end_run();
    }
  }
  end_run();
  uint32_t ms = now_ms() - t0;
  carve_kBps = ms ? (uint32_t)((uint64_t)blocks * 512 / ms) : 0;
}

// ---------------------------------------------------------------------------
// Media health report
// ---------------------------------------------------------------------------

static void print_ranges(const char* what, const range_t* list, uint32_t n, uint32_t total) {
  if (!n) return;
  printf("  %s ranges (LBA, sectors):", what);
  for (uint32_t i = 0; i < n; i++) {
    if (i % 4 == 0) printf("\r\n   ");
    printf("  %lu+%lu", (unsigned long)list[i].start, (unsigned long)list[i].len);
  }
  if (total > n) printf("\r\n    ... and %lu more", (unsigned long)(total - n));
  printf("\r\n");
}

static void print_media(const sdmmc_info_t* ci) {
  printf("\r\n=== Media ===\r\n");
  if (ci && ci->bus_fallback) printf("  Bus               fell back to 1-bit: DAT1-3 not working\r\n");
  else if (ci) printf("  Bus               %u-bit, no fallback needed\r\n", ci->bus_width);
  printf("  Data timeout      %lu ms%s\r\n", (unsigned long)data_timeout_ms,
         data_timeout_ms < 100 ? " (shortened: a good block slower than this would count as bad)" : "");
  printf("  Bad sectors       %lu (%lu MiB) in %lu range%s -- tried and failed\r\n", (unsigned long)bad_sectors,
         (unsigned long)(bad_sectors / 2048), (unsigned long)bad_ranges, bad_ranges == 1 ? "" : "s");
  printf("  Skipped sectors   %lu (%lu MiB) in %lu range%s -- not tried, next to bad ones\r\n",
         (unsigned long)skipped_sectors, (unsigned long)(skipped_sectors / 2048), (unsigned long)skipped_ranges,
         skipped_ranges == 1 ? "" : "s");
  if (fat_backup_reads || fat_unreadable)
    printf("  FAT               %lu sector(s) read from a backup copy, %lu unreadable in every copy\r\n",
           (unsigned long)fat_backup_reads, (unsigned long)fat_unreadable);
  if (dir_sectors_unreadable)
    printf("  Directories       %lu sector(s) unreadable (16 entries each unknown)\r\n",
           (unsigned long)dir_sectors_unreadable);
  if (clusters_unknown)
    printf("  Clusters          %lu of unknown state (FAT entry unreadable), not scanned\r\n",
           (unsigned long)clusters_unknown);
  print_ranges("Bad", bad, n_bad, bad_ranges);
  print_ranges("Skipped", skipped, n_skipped, skipped_ranges);
  if (n_files_checked)
    printf("  Live file data    %lu file(s) read end to end, %lu damaged: %lu of %lu sectors not read\r\n"
           "                    (%lu bad, %lu skipped next to bad)\r\n",
           (unsigned long)n_files_checked, (unsigned long)n_files_damaged, (unsigned long)file_sectors_unread,
           (unsigned long)file_sectors, (unsigned long)file_sectors_bad,
           (unsigned long)(file_sectors_unread - file_sectors_bad));
  printf("  Coverage: every live file's data, free space, file slack, the sectors\r\n"
         "  outside the partition, and the file-system metadata walked. Not read:\r\n"
         "  reserved sectors and FAT sectors never consulted.\r\n");
}

// ---------------------------------------------------------------------------
// Machine-readable record
// ---------------------------------------------------------------------------
//
// One line, `@@SDREC ` followed by a JSON object, for tools/sd_record.py to
// collect. Raw register images are the record's substance; anything decoded
// can be re-derived from them later. Identifying fields (the CID, which holds
// the serial number, and the volume serial and label) are included here and
// left to the host to keep private.

#define SDREC_SCHEMA 2  // 2: entries classified (user / OS artifact / trash), tagged verdict
#define SD_INSPECT_VERSION "1.4"

static void json_hex(const char* key, const uint8_t* p, int n) {
  printf("\"%s\":\"", key);
  for (int i = 0; i < n; i++) printf("%02X", p[i]);
  printf("\"");
}

// A JSON string from a fixed-width, space-padded on-disk field.
static void json_field(const char* key, const char* s, int maxlen) {
  int n = 0;
  while (n < maxlen && s[n]) n++;
  while (n > 0 && s[n - 1] == ' ') n--;
  printf("\"%s\":\"", key);
  for (int i = 0; i < n; i++) {
    uint8_t ch = (uint8_t)s[i];
    if (ch == '"' || ch == '\\') printf("\\%c", ch);
    else if (ch >= 0x20 && ch < 0x7F) printf("%c", ch);
    else printf("\\u%04X", ch);
  }
  printf("\"");
}

// Tags, joined with '+', in a fixed order:
//   user      files of the user's own (OS artifacts excluded)
//   trashed   files sitting in an OS trash folder
//   deleted   deleted user directory entries, or file headers carved from
//             unallocated space
//   residual  unallocated data, but nothing identifiable in it
// or "clean" when there is none of these. Which OSes left artifacts is
// reported separately, because nearly every card has some.
static const char* verdict(void) {
  static char buf[40];
  bool deleted = n_del_user > 0 || n_headers > 0;
  buf[0] = 0;
  if (n_user_files) strcat(buf, "+user");
  if (n_trashed_files) strcat(buf, "+trashed");
  if (deleted) strcat(buf, "+deleted");
  if (!deleted && residual_sectors) strcat(buf, "+residual");
  return buf[0] ? buf + 1 : "clean";
}

// stdout is buffered, so without this the whole ~1.1 KB line reaches the
// console as one write -- larger than the 1 KiB SEGGER RTT ring the console
// is mirrored to, which then overwrites its own unread start. Flushing per
// section hands the console pieces of a few hundred bytes, each paced by the
// UART, so an RTT reader can keep up. Measured: unflushed, two captures in a
// row lost ~30 bytes at offset ~1017.
#define RECORD_FLUSH() fflush(stdout)

static void emit_record(const sdmmc_info_t* ci, bool have_info, bool have_fs, bool card_detect,
                        uint32_t init_ms, uint32_t blocks) {
  printf("@@SDREC {\"schema\":%d,\"tool\":\"sd_inspect %s\",", SDREC_SCHEMA, SD_INSPECT_VERSION);
  if (have_info) {
    json_hex("cid", ci->cid, 16); printf(",");
    json_hex("csd", ci->csd, 16); printf(",");
    json_hex("scr", ci->scr, 8); printf(",");
    RECORD_FLUSH();
    // Absent rather than zero when unread, so a zero register is never
    // mistaken for a real one.
    if (ci->ssr_valid) { json_hex("ssr", ci->ssr, 64); printf(","); RECORD_FLUSH(); }
    if (ci->switch_valid) { json_hex("switch", ci->switch_status, 64); printf(","); RECORD_FLUSH(); }
    printf("\"ocr\":\"%08lX\",\"rca\":\"%04X\",", (unsigned long)ci->ocr, ci->rca);
    printf("\"bus_width\":%u,\"clock_hz\":%lu,", ci->bus_width, (unsigned long)ci->clock_hz);
  }
  printf("\"blocks\":%lu,\"init_ms\":%lu,\"card_detect\":%s,", (unsigned long)blocks, (unsigned long)init_ms,
         card_detect ? "true" : "false");
  printf("\"read_kBps\":{\"multi\":%lu,\"single\":%lu,\"carve\":%lu},", (unsigned long)bench_multi_kBps,
         (unsigned long)bench_single_kBps, (unsigned long)carve_kBps);
  RECORD_FLUSH();

  printf("\"partition_table\":\"%s\",\"partitions\":[", part_table);
  for (int i = 0; i < n_parts; i++) {
    printf("%s{\"type\":\"0x%02X\",\"active\":%s,\"lba\":%lu,\"sectors\":%lu}", i ? "," : "", parts[i].type,
           parts[i].active ? "true" : "false", (unsigned long)parts[i].lba, (unsigned long)parts[i].sectors);
  }
  printf("],");
  RECORD_FLUSH();

  if (have_fs) {
    printf("\"filesystem\":{\"type\":\"FAT%d\",\"lba\":%lu,\"sectors\":%lu,\"cluster_sectors\":%u,"
           "\"clusters\":%lu,\"fats\":%u,\"fat_sectors\":%lu,",
           fs.type, (unsigned long)fs.part_lba, (unsigned long)fs.total_sectors, fs.spc,
           (unsigned long)fs.clusters, fs.nfats, (unsigned long)fs.fat_sectors);
    json_field("oem", fs.oem, 8); printf(",");
    json_field("fstype", fs.fstype, 8); printf(",");
    json_field("label", fs.label, 11); printf(",");
    printf("\"volume_serial\":\"%08lX\"},", (unsigned long)fs.vol_serial);
  } else {
    printf("\"filesystem\":%s,", exfat_seen ? "{\"type\":\"exFAT\"}" : "null");
  }
  RECORD_FLUSH();

  printf("\"content\":{\"verdict\":\"%s\",\"fs_parsed\":%s,\"os_artifacts\":[", verdict(),
         have_fs ? "true" : "false");
  static const uint8_t oses[] = {OS_WINDOWS, OS_MACOS, OS_LINUX, OS_ANDROID};
  static const char* const os_keys[] = {"windows", "macos", "linux", "android"};
  bool first = true;
  for (int i = 0; i < 4; i++) {
    if (os_seen & oses[i]) {
      printf("%s\"%s\"", first ? "" : ",", os_keys[i]);
      first = false;
    }
  }
  printf("],");
  printf("\"user_files\":%lu,\"user_dirs\":%lu,\"os_files\":%lu,\"os_dirs\":%lu,", (unsigned long)n_user_files,
         (unsigned long)n_user_dirs, (unsigned long)n_os_files, (unsigned long)n_os_dirs);
  printf("\"trashed_files\":%lu,\"trashed_dirs\":%lu,", (unsigned long)n_trashed_files,
         (unsigned long)n_trashed_dirs);
  RECORD_FLUSH();
  printf("\"present_files\":%lu,\"present_dirs\":%lu,\"present_bytes\":%lu,", (unsigned long)n_live_files,
         (unsigned long)n_live_dirs, (unsigned long)live_bytes);
  printf("\"deleted_files\":%lu,\"deleted_dirs\":%lu,\"deleted_intact\":%lu,", (unsigned long)n_del_files,
         (unsigned long)n_del_dirs, (unsigned long)n_del_intact);
  printf("\"deleted_user\":%lu,\"deleted_os\":%lu,", (unsigned long)n_del_user, (unsigned long)n_del_os);
  printf("\"carved_headers\":%lu,\"carved_video\":%lu,\"residual_kib\":%lu,", (unsigned long)n_headers,
         (unsigned long)n_video_headers, (unsigned long)(residual_sectors / 2));
  printf("\"read_errors\":%lu},", (unsigned long)read_errors);
  RECORD_FLUSH();

  // Media health, separate from what the card holds. Range lists are capped
  // to keep the line bounded; the totals are exact.
  printf("\"media\":{\"bus_fallback\":%s,\"timeout_ms\":%lu,\"bad_sectors\":%lu,\"bad_ranges\":%lu,"
         "\"skipped_sectors\":%lu,\"skipped_ranges\":%lu,",
         have_info && ci->bus_fallback ? "true" : "false", (unsigned long)data_timeout_ms,
         (unsigned long)bad_sectors, (unsigned long)bad_ranges, (unsigned long)skipped_sectors,
         (unsigned long)skipped_ranges);
  printf("\"fat_backup_reads\":%lu,\"fat_unreadable\":%lu,\"dir_sectors_unreadable\":%lu,"
         "\"clusters_unknown\":%lu,", (unsigned long)fat_backup_reads, (unsigned long)fat_unreadable,
         (unsigned long)dir_sectors_unreadable, (unsigned long)clusters_unknown);
  printf("\"files_checked\":%lu,\"files_damaged\":%lu,\"file_sectors\":%lu,\"file_sectors_unread\":%lu,"
         "\"file_sectors_bad\":%lu,",
         (unsigned long)n_files_checked, (unsigned long)n_files_damaged, (unsigned long)file_sectors,
         (unsigned long)file_sectors_unread, (unsigned long)file_sectors_bad);
  RECORD_FLUSH();
  const uint32_t list_max = 16;
  printf("\"bad\":[");
  for (uint32_t i = 0; i < n_bad && i < list_max; i++)
    printf("%s[%lu,%lu]", i ? "," : "", (unsigned long)bad[i].start, (unsigned long)bad[i].len);
  printf("],");
  RECORD_FLUSH();
  printf("\"skipped\":[");
  for (uint32_t i = 0; i < n_skipped && i < list_max; i++)
    printf("%s[%lu,%lu]", i ? "," : "", (unsigned long)skipped[i].start, (unsigned long)skipped[i].len);
  printf("]}}\r\n");
}

// ---------------------------------------------------------------------------

int main(void) {
  libtocksync_alarm_delay_ms(START_DELAY_MS);
  printf("\r\n\r\n##### SD card inspector #####\r\n");
  if (!sdmmc_exists()) {
    printf("No sdmmc driver in this kernel.\r\n");
    return -1;
  }
  bool present = sdmmc_card_present();
  printf("Card detect switch: %s\r\n", present ? "card inserted" : "no card (trying anyway)");

  uint32_t blocks = 0;
  uint32_t t = now_ms();
  returncode_t rc = sdmmc_init(&blocks);
  if (rc != RETURNCODE_SUCCESS) {
    printf("Card initialisation failed: %s\r\n", tock_strrcode(rc));
    return -1;
  }
  uint32_t init_ms = now_ms() - t;
  printf("Card initialised in %lu ms\r\n", (unsigned long)init_ms);

  sdmmc_info_t info;
  bool have_info = sdmmc_get_info(&info) == RETURNCODE_SUCCESS;
  if (have_info) print_card(&info);

  probe(blocks);
  benchmark(blocks);

  bool have_fs = find_fs();
  if (have_fs) {
    printf("\r\n=== Directory tree (DELETED* = inside a deleted directory) ===\r\n");
    printf("  state          size  modified          name\r\n");
    walk_dir(fs.type == 32 ? fs.root_cluster : 0, 0, false, "/", CLS_USER);
    if (!n_live_files && !n_live_dirs && !n_del_files && !n_del_dirs) printf("  (empty)\r\n");
    carve_free_space();
    carve_slack_and_gaps(blocks);
  } else {
    carve_raw(blocks);
  }

  print_media(have_info ? &info : NULL);

  printf("\r\n=== Summary ===\r\n");
  if (have_fs) {
    printf("  User files present:             %lu (%lu dirs)\r\n", (unsigned long)n_user_files,
           (unsigned long)n_user_dirs);
    printf("  OS artifacts:                   %lu files, %lu dirs", (unsigned long)n_os_files,
           (unsigned long)n_os_dirs);
    if (os_seen) {
      printf(" (");
      bool first = true;
      for (uint8_t bit = 1; bit <= OS_ANDROID; bit <<= 1) {
        if (os_seen & bit) {
          printf("%s%s", first ? "" : ", ", os_name(bit));
          first = false;
        }
      }
      printf(")");
    }
    printf("\r\n");
    printf("  In a trash folder:              %lu files (%lu dirs)\r\n", (unsigned long)n_trashed_files,
           (unsigned long)n_trashed_dirs);
    printf("  All present, total:             %lu files (%lu dirs), %lu bytes\r\n", (unsigned long)n_live_files,
           (unsigned long)n_live_dirs, (unsigned long)live_bytes);
    printf("  Deleted directory entries:      %lu files, %lu dirs; %lu with data not reused\r\n",
           (unsigned long)n_del_files, (unsigned long)n_del_dirs, (unsigned long)n_del_intact);
    printf("    of which user / OS artifact:  %lu / %lu\r\n", (unsigned long)n_del_user, (unsigned long)n_del_os);
  }
  printf("  Residual data in %s:   %lu KiB in %lu runs\r\n", have_fs ? "unallocated" : "the card   ",
         (unsigned long)(residual_sectors / 2), (unsigned long)(n_raw_runs + n_headers));
  printf("  File headers found there:       %lu (%lu video), %lu embedded\r\n", (unsigned long)n_headers,
         (unsigned long)n_video_headers, (unsigned long)n_nested);
  printf("  Read errors:                    %lu\r\n", (unsigned long)read_errors);
  printf("\r\n  Verdict: %s\r\n", verdict());
  if (n_user_files) printf("    user files PRESENT\r\n");
  if (n_files_damaged)
    printf("    %lu file(s) DAMAGED -- unreadable sectors in their data\r\n", (unsigned long)n_files_damaged);
  if (bad_sectors) printf("    media has bad sectors: %lu tried and failed\r\n", (unsigned long)bad_sectors);
  else if (n_os_files || n_os_dirs) printf("    no user files -- only artifacts of the OS that mounted it\r\n");
  else printf("    no files present\r\n");
  if (n_trashed_files) printf("    %lu file(s) in a trash folder: deleted by the user, still intact\r\n",
                              (unsigned long)n_trashed_files);
  if (n_del_user || n_headers) {
    printf("    DELETED user content:\r\n");
    if (n_del_user)
      printf("      %lu deleted directory entr%s, %lu with data not reused (undelete)\r\n", (unsigned long)n_del_user,
             n_del_user == 1 ? "y" : "ies", (unsigned long)n_del_intact);
    if (n_headers)
      printf("      %lu file header(s) in unallocated space, %lu of them video (carving, photorec-style)\r\n",
             (unsigned long)n_headers, (unsigned long)n_video_headers);
  } else if (residual_sectors) {
    printf("    residual data in unallocated space, but no recognisable file headers\r\n");
  } else {
    printf("    unallocated space is blank -- nothing to recover\r\n");
  }
  printf("\r\n");
  emit_record(&info, have_info, have_fs, present, init_ms, blocks);
  printf("##### done #####\r\n");
  return 0;
}
