// SD sector dumper: reads the sectors a host asks for and sends them back
// over the console, for merging into a ddrescue image on the host.
//
// Strictly read-only, like the driver beneath it. Strictly request/response,
// too: the host sends a command only after the board has said it is idle,
// because while the SD driver polls the card the kernel cannot keep up with
// incoming UART bytes, and a command arriving mid-read would be lost.
//
// Protocol (all lines end "\r\n"; anything not starting "@@" is ignored):
//   board  @@READY <blocks> <bus_width>      card initialized, waiting
//   host   R<lba:10 digits><count:4 digits>\n  exactly 16 bytes, count 1..9999
//   board  @@S <lba> <crc32 hex> <base64 of 512 bytes>   one per sector read
//   board  @@B <lba> <stage> <sr hex>        one per sector that failed
//   board  @@K <lba> <count>                 sectors skipped: a dead run (see T)
//   board  @@E <lba> <count>                 range finished, waiting again
//   host   T<ms:4><retries:1><dead:2>        then spaces to 15 bytes, then \n
//   board  @@T <ms> <retries> <dead>         settings in force, waiting again
//   board  @@F <reason>                      fatal, stopping
//
// A failed multi-block read is retried a block at a time, each block up to
// `retries` times. Defaults: the SD specification's full 100 ms data timeout,
// 3 retries, no dead-run limit -- recovering a sector matters more than time.
// A fast first pass over a badly damaged card wants the opposite (T0025108):
// 25 ms, one attempt, and after 8 bad blocks in a row the rest of the range
// is reported skipped, for the host to come back to later.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <libtock-sync/interface/console.h>
#include <libtock-sync/services/alarm.h>

#include "sdmmc.h"

#define MAX_BLOCKS 64

static uint32_t retries = 3;
static uint32_t dead_run = 0;  // 0: never give up on a range

static uint8_t buf[MAX_BLOCKS * 512] __attribute__((aligned(4)));

static uint32_t crc32(const uint8_t* p, uint32_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
  }
  return ~c;
}

static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// One sector as a single line. Built whole and written with one printf, so
// it reaches the console as one write.
static char line[16 + 12 + 10 + 688 + 4];

static void send_sector(uint32_t lba, const uint8_t* s) {
  int n = sprintf(line, "@@S %lu %08lX ", (unsigned long)lba, (unsigned long)crc32(s, 512));
  for (int i = 0; i < 512; i += 3) {
    uint32_t v = (uint32_t)s[i] << 16 | (i + 1 < 512 ? (uint32_t)s[i + 1] << 8 : 0) |
                 (i + 2 < 512 ? s[i + 2] : 0);
    line[n++] = B64[(v >> 18) & 63];
    line[n++] = B64[(v >> 12) & 63];
    line[n++] = i + 1 < 512 ? B64[(v >> 6) & 63] : '=';
    line[n++] = i + 2 < 512 ? B64[v & 63] : '=';
  }
  line[n++] = '\r';
  line[n++] = '\n';
  line[n] = 0;
  printf("%s", line);
  fflush(stdout);
}

static void send_bad(uint32_t lba) {
  sdmmc_error_t e;
  if (sdmmc_error_info(0, &e) != RETURNCODE_SUCCESS) e = (sdmmc_error_t){0};
  printf("@@B %lu %u %08lX\r\n", (unsigned long)lba, e.stage, (unsigned long)e.sr);
  fflush(stdout);
}

// Up to MAX_BLOCKS sectors. Returns how many were read.
static uint32_t dump_chunk(uint32_t lba, uint32_t count) {
  if (sdmmc_read(lba, count, buf) == RETURNCODE_SUCCESS) {
    for (uint32_t i = 0; i < count; i++) send_sector(lba + i, buf + 512 * i);
    return count;
  }
  uint32_t run_bad = 0, good = 0;
  for (uint32_t i = 0; i < count; i++) {
    if (dead_run && run_bad >= dead_run) {
      printf("@@K %lu %lu\r\n", (unsigned long)(lba + i), (unsigned long)(count - i));
      fflush(stdout);
      return good;
    }
    bool ok = false;
    for (uint32_t attempt = 0; attempt < retries && !ok; attempt++) {
      ok = sdmmc_read(lba + i, 1, buf) == RETURNCODE_SUCCESS;
    }
    if (ok) {
      send_sector(lba + i, buf);
      run_bad = 0;
      good++;
    } else {
      send_bad(lba + i);
      run_bad++;
    }
  }
  return good;
}

// A range of any length, in chunks. With a dead-run limit set, a chunk that
// yields nothing makes the rest of the way skip ahead -- 64 sectors, then
// doubling each time it happens again, up to 32 MiB -- reported with @@K
// for the host to come back to. One command per long range keeps the
// command traffic, which at high baud rates has to be paced, negligible.
#define SKIP_MIN 64
#define SKIP_MAX 65536

static void dump_range(uint32_t lba, uint32_t count) {
  uint32_t pos = lba, end = lba + count, skip = SKIP_MIN;
  while (pos < end) {
    uint32_t n = end - pos < MAX_BLOCKS ? end - pos : MAX_BLOCKS;
    uint32_t good = dump_chunk(pos, n);
    pos += n;
    if (!dead_run) continue;
    if (good == 0) {
      uint32_t s = end - pos < skip ? end - pos : skip;
      if (s) {
        printf("@@K %lu %lu\r\n", (unsigned long)pos, (unsigned long)s);
        fflush(stdout);
        pos += s;
      }
      if (skip < SKIP_MAX) skip *= 2;
    } else {
      skip = SKIP_MIN;
    }
  }
}

static uint32_t digits(const char* p, int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; i++) {
    if (p[i] < '0' || p[i] > '9') return 0xFFFFFFFFu;
    v = v * 10 + (uint32_t)(p[i] - '0');
  }
  return v;
}

// Read one 16-byte command, resynchronizing on '\n' if the framing is off.
// Returns its letter ('R' or 'T'), or 0 if there was none.
static char read_command(uint32_t* a, uint32_t* b, uint32_t* c) {
  char cmd[16];
  int got = 0;
  if (libtocksync_console_read((uint8_t*)cmd, sizeof(cmd), &got) != RETURNCODE_SUCCESS || got != 16) return 0;
  if ((cmd[0] != 'R' && cmd[0] != 'T') || cmd[15] != '\n') {
    // Out of step: discard up to and including the next newline.
    char ch = cmd[15];
    while (ch != '\n') {
      if (libtocksync_console_read((uint8_t*)&ch, 1, &got) != RETURNCODE_SUCCESS) return 0;
    }
    return 0;
  }
  if (cmd[0] == 'R') {  // R<lba:10><count:4>
    *a = digits(cmd + 1, 10);
    *b = digits(cmd + 11, 4);
    *c = 0;
    return *a != 0xFFFFFFFFu && *b != 0xFFFFFFFFu ? 'R' : 0;
  }
  *a = digits(cmd + 1, 4);  // T<ms:4><retries:1><dead:2>
  *b = digits(cmd + 5, 1);
  *c = digits(cmd + 6, 2);
  return *a != 0xFFFFFFFFu && *b != 0xFFFFFFFFu && *c != 0xFFFFFFFFu ? 'T' : 0;
}

int main(void) {
  // Let the other applications finish their start-up output first.
  libtocksync_alarm_delay_ms(3000);
  printf("\r\n##### SD sector dumper #####\r\n");
  if (!sdmmc_exists()) {
    printf("@@F no sdmmc driver\r\n");
    return -1;
  }
  uint32_t blocks = 0;
  returncode_t rc = sdmmc_init(&blocks);
  if (rc != RETURNCODE_SUCCESS) {
    // Say how: NOACK means no answer at all (no card, or no contact), BUSY
    // that it answered but never finished powering up.
    sdmmc_error_t e;
    if (sdmmc_error_info(0, &e) != RETURNCODE_SUCCESS) e = (sdmmc_error_t){0};
    printf("@@F card initialisation failed: %s (CMD%u, stage %u, SR %08lX, R1 %08lX)\r\n", tock_strrcode(rc),
           e.cmd, e.stage, (unsigned long)e.sr, (unsigned long)e.r1);
    return -1;
  }
  sdmmc_info_t info;
  uint8_t width = 0;
  if (sdmmc_get_info(&info) == RETURNCODE_SUCCESS) width = info.bus_width;

  for (;;) {
    printf("@@READY %lu %u\r\n", (unsigned long)blocks, width);
    fflush(stdout);
    uint32_t lba, count, extra;
    char kind = read_command(&lba, &count, &extra);
    if (kind == 'T') {
      uint32_t ms = lba;
      if (ms >= 1 && ms <= 1000 && sdmmc_set_timeout(ms) == RETURNCODE_SUCCESS) {
        retries = count ? count : 1;
        dead_run = extra;
      }
      printf("@@T %lu %lu %lu\r\n", (unsigned long)ms, (unsigned long)retries, (unsigned long)dead_run);
      continue;
    }
    if (kind != 'R') continue;
    if (count == 0 || count > 9999 || lba >= blocks || count > blocks - lba) {
      printf("@@E %lu 0\r\n", (unsigned long)lba);
      continue;
    }
    dump_range(lba, count);
    printf("@@E %lu %lu\r\n", (unsigned long)lba, (unsigned long)count);
    fflush(stdout);
  }
}
