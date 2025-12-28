#pragma once
#include <vector>
#include <string>
#include <cstdint>

struct TxEntry {
    std::string dxcall;       // target call or "FreeText"
    std::string field3;       // grid/report/RR73/73 or full FreeText message
    std::string text;         // optional prebuilt text to send/display
    int snr = 0;              // report value (unused for FreeText)
    int offset_hz = 0;        // TX offset to use
    int slot_id = 0;          // slot to transmit (0 even, 1 odd)
    int repeat_counter = 0;   // remaining TXs (1 removes after send)
    bool mark_delete = false; // UI delete flag
};

// Shared TX state
extern TxEntry tx_next;
extern std::vector<TxEntry> tx_queue;  // max size 10

void tx_init();
void tx_enqueue(const TxEntry& entry);
void tx_commit_deletions();
// Render text for UI; if for_queue is true, repeat counter is shown for normal entries.
std::string tx_entry_display(const TxEntry& e, bool for_queue);

// TX engine scheduling
void tx_engine_reset();
// Called near slot boundary with current slot/parity and ms to boundary.
void tx_engine_tick(int64_t slot_idx, int slot_parity, int ms_to_boundary);
// Fetch and clear a pending TX if scheduled
bool tx_engine_fetch_pending(TxEntry& out);
// Mark a TX as sent and update bookkeeping/removal
void tx_engine_mark_sent(const TxEntry& sent, int64_t slot_idx);
