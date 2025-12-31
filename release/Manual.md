# Mini-FT8 v1.0 (M5 Cardputer / ESP32-S3)

Offline FT8 decoder/transceiver helper for M5 Cardputer with QMX USB audio (UAC) and CAT, auto-sequencing, beacon, logging, and menu-driven UI.

## Build & Flash
```
idf.py build
idf.py -p COMx flash
```
Flash size: 8MB. SPIFFS: 4MB at 0x190000. SPIFFS mount auto-formats on failure (preserves files unless erased).

## First Connection (QMX)
1) Power on Mini-FT8.  
2) Press `s` (Status) → press `2` (“Connect to /Radio”) to start UAC host.  
3) **Then power on QMX**; otherwise the host won’t enumerate.  
4) Time/Date: Status line 6 (UTC). RTC drives slot alignment and auto-sync.  
5) Band: `b` to set freq; Status line 3 cycles “ActiveBand” (from Menu page 3).  

## Modes & Keys
- `r` RX (default): waterfall + decodes. `1-6` enqueue reply. `x` toggles decode on/off. `;`/`.` page list.
- `t` TX: line1 `tx_next`, lines2-6 queue. `;`/`.` page queue. `2-6` mark/delete. `e` encode/log `tx_next`. Auto-TX at correct slot with slot-group round-robin and blank-slot guard.
- `b` Band: `1-6` edit freq (enter to confirm, ESC to cancel). `;`/`.` page.
- `m` Menu: page 1 (CQ type, SkipTx1, FreeText (long edit), Call, Grid, Offset source, Radio, Antenna); page 2 (Cursor direct edit, Radio action, RxTxLog); page 3 (ActiveBand long edit, SkipTx1, etc.). `n` jumps to page 2, `o` to page 3; `m/n/o` exit Menu.
- `s` Status: line 1 Beacon (OFF/EVEN/EVEN2/ODD/ODD2); line 2 Connect/Sync to Radio (action); line 3 Band; line 4 Tune toggle (CAT PTT); line 6 Time/Date. Beacon changes take effect when leaving Status.
- `q` QSO logs: list `.adi` newest first; `1-6` open; `;`/`.` page; `` ` `` back; entries show `hh:mm band call`.
- `c` Control/Host: USB serial file I/O. Commands: `WRITEBIN <file> <size> <crc32_hex>`, `WRITE/APPEND`, `READ/DELETE`, `LIST/INFO/HELP`, `EXIT`.
- `d` Debug: paged debug lines. `h` Host; `b/r/t/s/m/q/c` as above.

## TX/Auto-Sequence
- Replies: the first “to me” decode auto-generates a response, sets `tx_next` immediately, purges stale queue entries for that dxcall, enqueues the response, and schedules TX. Steps follow 6-1-2-3-4-5; RR73/73 handling updates active calls and ADIF.
- Beacon: when enabled, injects CQ/FreeText directly into `tx_next` (not the queue) each eligible slot (EVEN/ODD/EVEN2/ODD2), respecting CQ type (CQ/CQ SOTA/CQ POTA/CQ QRP/CQ FD/FreeText). Beacon changes are applied on exit from Status; turning Beacon OFF clears pending beacon `tx_next`.
- Slot/group: maintains slot parity; round-robin within the current parity; enforces one blank slot between transmissions; inline start allowed if <4s into slot (skips elapsed tones).
- CAT: CDC-ACM to QMX; per-tone `TAxxxx.xx;` fire-and-forget every 160ms; `MD6;` + `TX;` before tones, `RX;` after; base offset from Random/Cursor/RX offset.

## Logging
- Rx/Tx log: `/spiffs/RxTxLog.txt` (toggle in Menu page 2). TX logged at encode; RX logged on decode.
- ADIF: `/spiffs/YYYYMMDD.adi` appended on completed QSOs (TX3 or RR73 path). Comment uses expanded `/Radio` and `/Ant`.
- QSO viewer (`q`) reads `.adi` and shows `hh:mm band call`.

## Filesystem
- SPIFFS at `/spiffs`, auto-format on mount failure. `max_files=5` is concurrent open count, not total files.
- `spiffs_create_partition_image` is built but not flashed by default (preserves on-device files).

## Notes
- Connect order: Mini-FT8 on → Status `2` to start UAC → power on QMX.
- Countdown always runs; decode toggle via `x`. Auto-sync adjusts RTC by median decode offset (bounded).
- Beacon OFF clears pending beacon TX; other queued items are unaffected.
