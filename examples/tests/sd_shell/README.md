# sd_shell

Interactive SD card shell for the SAMV71 Xplained Ultra, over the console
UART (EDBG virtual COM port, 115200 8N1). Send any SD command and see the
card's response decoded (R1 state and error bits, R2 as CID/CSD, R3 OCR, R6
RCA, R7 voltage check) together with the host controller status and any data
the card returned. It also reads, writes, verifies and erases blocks through
the kernel driver, and edits the 32 KiB buffer that writes come from.

## Kernel

Writes and raw commands need the kernel built with the `sd_write` feature:

```
cd tock/boards/sam_v71_xult
.\build.ps1 -Features sd_write      # -> sam_v71_xult-sd_write.bin
```

The default kernel stays read-only (sd_inspect and sd_dump rely on that); on
it, the shell's reads work and every raw command reports `ENOSUPPORT`.

The `sdmmc` driver gives the card to the first process that uses it, so
install sd_shell *instead of* sd_dump or sd_inspect, not beside them.

## Build

In WSL (GCC 10):

```
make TOCK_TARGETS=cortex-m7
```

## Safety

The shell starts **locked**: `write`, `erase`, any raw command with a write
data phase, and raw CMD28/29/38 are refused until `unlock` (the prompt changes
from `sd>` to `sd!>`). Raw commands are sent exactly as typed -- nothing checks
them against the card's state -- and do not update the driver's record of the
card, so after a manual bring-up or a CMD0 run `init` before `read`/`write`.

## Example

```
sd> init
init ok: 1984000 blocks (968 MiB), SDSC, RCA 1234, 4-bit at 25000000 Hz
sd> cmd 13 0x12340000
CMD13 arg 12340000: ok
  R1 00000900  state=tran READY_FOR_DATA
  SR 0C000025
sd> acmd 51
CMD55 arg 12340000: ok
  R1 00000920  state=tran READY_FOR_DATA APP_CMD
  SR ...
ACMD51 arg 00000000: ok
  R1 00000920  state=tran READY_FOR_DATA APP_CMD
  SR ...
  data: 8 of 8 bytes read into buffer
00000  02 35 80 00 00 00 00 00 ...
sd> pattern 7
sd> unlock
sd!> write 1000 4
sd!> verify 1000 4
verify: 4 block(s) match
```

(Output above is illustrative, not captured from a card.)

Type commands rather than pasting: input is read a character at a time, and
characters that arrive while the kernel is busy with the card are lost.
