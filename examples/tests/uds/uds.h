/* UDS (ISO 14229-1) server for the running kernel.
 *
 * The service split is deliberate and is what keeps the runtime attack surface
 * small: this server never writes flash. The download services 0x34 / 0x36 /
 * 0x37 are absent, not stubbed. The most it can do is answer
 * DiagnosticSessionControl programming session, which restarts the board into
 * the bootloader -- and it may only do that if the kernel granted it the
 * reboot privilege, which is a property of being signed.
 *
 *   0x10  DiagnosticSessionControl   default / programming / extended
 *   0x11  ECUReset                   restart, back into the kernel
 *   0x22  ReadDataByIdentifier       identification and telemetry
 *   0x27  SecurityAccess             seed / key
 *   0x2E  WriteDataByIdentifier      configuration
 *   0x31  RoutineControl             self test
 *   0x3E  TesterPresent              refreshes the S3 session timer
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void uds_init(void);

// Build the response to one request.
//
// Returns the number of bytes to send, or 0 when the server must stay silent
// (TesterPresent with the suppress-positive-response bit).
size_t uds_handle(const uint8_t* req, size_t req_len, uint8_t* rsp, size_t rsp_capacity);

// Carry out whatever the last response promised, now that it is on the wire.
// Does not return if that was a reset.
void uds_post_response(void);

// Drop that promise instead, for when the response never reached the tester.
void uds_cancel_pending(void);

// Expire the session if the tester has gone quiet for longer than S3.
void uds_tick(void);

// Tell the server whether the transport has lost frames, for the self-test
// routine to report.
void uds_set_transport_healthy(bool healthy);

// Current session, for the caller to print.
uint8_t uds_session(void);
