#include "sdmmc.h"

static uint32_t le32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool sdmmc_exists(void) {
  return driver_exists(DRIVER_NUM_SDMMC);
}

bool sdmmc_card_present(void) {
  uint32_t v = 0;
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 1, 0, 0);
  if (tock_command_return_u32_to_returncode(r, &v) != RETURNCODE_SUCCESS) return false;
  return v != 0;
}

// Wait for the driver's single upcall and unpack (kind, status, value).
static returncode_t wait_upcall(uint32_t expected_kind, uint32_t* value) {
  yield_waitfor_return_t y = yield_wait_for(DRIVER_NUM_SDMMC, 0);
  if (y.data0 != expected_kind) return RETURNCODE_FAIL;
  if (value) *value = y.data2;
  // The kernel reports an ErrorCode, whose returncode_t is its negation.
  return y.data1 == 0 ? RETURNCODE_SUCCESS : -(returncode_t)y.data1;
}

returncode_t sdmmc_init(uint32_t* block_count) {
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 2, 0, 0);
  returncode_t rc = tock_command_return_novalue_to_returncode(r);
  if (rc != RETURNCODE_SUCCESS) return rc;
  return wait_upcall(0, block_count);
}

returncode_t sdmmc_get_info(sdmmc_info_t* info) {
  uint8_t raw[184];
  allow_rw_return_t a = allow_readwrite(DRIVER_NUM_SDMMC, 1, raw, sizeof(raw));
  if (!a.success) return tock_status_to_returncode(a.status);
  uint32_t len = 0;
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 4, 0, 0);
  returncode_t rc = tock_command_return_u32_to_returncode(r, &len);
  allow_rw_return_t unallow = allow_readwrite(DRIVER_NUM_SDMMC, 1, NULL, 0);
  (void)unallow;
  if (rc != RETURNCODE_SUCCESS) return rc;
  if (len < sizeof(raw)) return RETURNCODE_FAIL;

  for (int i = 0; i < 16; i++) info->cid[i] = raw[i];
  for (int i = 0; i < 16; i++) info->csd[i] = raw[16 + i];
  for (int i = 0; i < 8; i++) info->scr[i] = raw[32 + i];
  info->ocr = le32(&raw[40]);
  info->rca = (uint16_t)(raw[44] | (raw[45] << 8));
  info->high_capacity = raw[46] & 1;
  info->version2 = (raw[46] >> 1) & 1;
  info->bus_width = raw[47];
  info->block_count = le32(&raw[48]);
  info->clock_hz = le32(&raw[52]);
  info->ssr_valid = (raw[46] >> 2) & 1;
  info->switch_valid = (raw[46] >> 3) & 1;
  info->bus_fallback = (raw[46] >> 4) & 1;
  for (int i = 0; i < 64; i++) info->ssr[i] = raw[56 + i];
  for (int i = 0; i < 64; i++) info->switch_status[i] = raw[120 + i];
  return RETURNCODE_SUCCESS;
}

uint32_t sdmmc_max_blocks(void) {
  uint32_t v = 0;
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 5, 0, 0);
  tock_command_return_u32_to_returncode(r, &v);
  return v;
}

returncode_t sdmmc_read(uint32_t lba, uint32_t count, uint8_t* buf) {
  allow_rw_return_t a = allow_readwrite(DRIVER_NUM_SDMMC, 0, buf, count * SDMMC_BLOCK_SIZE);
  if (!a.success) return tock_status_to_returncode(a.status);
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 3, lba, count);
  returncode_t rc = tock_command_return_novalue_to_returncode(r);
  uint32_t copied = 0;
  if (rc == RETURNCODE_SUCCESS) rc = wait_upcall(1, &copied);
  allow_rw_return_t unallow = allow_readwrite(DRIVER_NUM_SDMMC, 0, NULL, 0);
  (void)unallow;
  if (rc == RETURNCODE_SUCCESS && copied != count * SDMMC_BLOCK_SIZE) rc = RETURNCODE_FAIL;
  return rc;
}

returncode_t sdmmc_error_info(int which, sdmmc_error_t* err) {
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 6, which, 0);
  if (r.type != TOCK_SYSCALL_SUCCESS_U32_U32_U32) return RETURNCODE_FAIL;
  err->cmd = r.data[0] & 0xFF;
  err->stage = (r.data[0] >> 8) & 0xFF;
  err->card_state = (r.data[0] >> 16) & 0xFF;
  err->sr = r.data[1];
  err->r1 = r.data[2];
  return RETURNCODE_SUCCESS;
}

returncode_t sdmmc_set_bus(uint32_t width, uint32_t hz, uint32_t* actual_hz) {
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 7, width, hz);
  return tock_command_return_u32_to_returncode(r, actual_hz);
}

returncode_t sdmmc_set_timeout(uint32_t ms) {
  syscall_return_t r = command(DRIVER_NUM_SDMMC, 8, ms, 0);
  return tock_command_return_novalue_to_returncode(r);
}
