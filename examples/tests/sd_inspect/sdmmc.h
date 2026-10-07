// Synchronous wrapper for the kernel's read-only `sdmmc` driver (0x50005).
//
// The driver has no write command, so nothing this application does can alter
// the card.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <libtock/tock.h>

#define DRIVER_NUM_SDMMC 0x50005
#define SDMMC_BLOCK_SIZE 512

// Mirrors the CardInfo layout documented in capsules/extra/src/sdmmc.rs.
typedef struct {
  uint8_t cid[16];
  uint8_t csd[16];
  uint8_t scr[8];
  uint32_t ocr;
  uint16_t rca;
  bool high_capacity;
  bool version2;
  uint8_t bus_width;
  uint32_t block_count;
  uint32_t clock_hz;
  uint8_t ssr[64];          // SD Status (ACMD13), MSB first
  bool ssr_valid;
  uint8_t switch_status[64]; // CMD6 check-mode status, MSB first
  bool switch_valid;
} sdmmc_info_t;

bool sdmmc_exists(void);
// Card-detect switch. Advisory: a holder may have no switch.
bool sdmmc_card_present(void);
// Bring the card up. Blocks until done.
returncode_t sdmmc_init(uint32_t* block_count);
returncode_t sdmmc_get_info(sdmmc_info_t* info);
// Largest `count` sdmmc_read accepts.
uint32_t sdmmc_max_blocks(void);
// Read `count` blocks at `lba` into `buf` (count * 512 bytes). Blocks.
returncode_t sdmmc_read(uint32_t lba, uint32_t count, uint8_t* buf);
