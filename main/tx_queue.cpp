#include "tx_queue.h"
#include <algorithm>
#include "esp_log.h"

TxEntry tx_next;
std::vector<TxEntry> tx_queue;

// TX engine state
static bool s_pending_valid = false;
static TxEntry s_pending;
static int s_pending_idx = -1;          // index in tx_queue when selected
static int64_t s_last_tx_slot_idx = -1000;
static int s_last_tx_parity = -1;       // -1 unknown, else 0/1
static int64_t s_sched_slot_idx = -1;
static int s_sched_parity = -1;
static int s_sched_ms_to_boundary = 10000;

static int tx_priority(const TxEntry& e) {
    // Favor end-of-QSO messages
    if (e.field3 == "RR73" || e.field3 == "73" || e.field3 == "RRR") return 2;
    return 1;
}

static bool tx_engine_pick_candidate(int slot_parity, bool enforce_gap, TxEntry& out);

void tx_init() {
    tx_queue.clear();
    tx_next.text.clear();
    tx_next.mark_delete = false;
    s_pending_valid = false;
    s_pending_idx = -1;
    s_last_tx_slot_idx = -1000;
    s_last_tx_parity = -1;
}

void tx_enqueue(const TxEntry& entry) {
    if (tx_queue.size() >= 10) return;
    tx_queue.push_back(entry);
}

void tx_commit_deletions() {
    std::vector<TxEntry> filtered;
    filtered.reserve(tx_queue.size());
    for (auto& e : tx_queue) {
        if (!e.mark_delete) filtered.push_back(e);
    }
    tx_queue.swap(filtered);
}

std::string tx_entry_display(const TxEntry& e, bool for_queue) {
    std::string base;
    if (e.dxcall == "FreeText") {
        base = !e.text.empty() ? e.text : e.field3;
    } else {
        base = e.dxcall;
        if (!e.field3.empty()) {
            base += " ";
            base += e.field3;
        }
    }
    // Append repeat counter on queue lines for normal entries
    if (for_queue && e.dxcall != "FreeText" && e.repeat_counter > 0) {
        base += " [";
        base += std::to_string(e.repeat_counter);
        base += "]";
    }
    return base;
}

// ---------------- TX Engine ----------------
void tx_engine_reset() {
    s_pending_valid = false;
    s_pending_idx = -1;
    s_last_tx_slot_idx = -1000;
    s_last_tx_parity = -1;
    s_sched_slot_idx = -1;
    s_sched_parity = -1;
    s_sched_ms_to_boundary = 10000;
}

void tx_engine_tick(int64_t slot_idx, int slot_parity, int ms_to_boundary) {
    s_sched_slot_idx = slot_idx;
    s_sched_parity = slot_parity;
    s_sched_ms_to_boundary = ms_to_boundary;
    if (s_pending_valid) return;
    // Removed the <4s gating; schedule as soon as we know about a pending slot

    bool enforce_gap = (s_last_tx_parity != -1 && s_last_tx_parity != slot_parity &&
                        slot_idx == s_last_tx_slot_idx + 1);

    TxEntry cand;
    if (tx_engine_pick_candidate(slot_parity, enforce_gap, cand)) {
        s_pending = cand;
        s_pending_valid = true;
        ESP_LOGI("TXENG", "scheduled: dx=%s f3=%s slot=%d rep=%d", cand.dxcall.c_str(),
                 cand.field3.c_str(), cand.slot_id, cand.repeat_counter);
    }
}

void tx_engine_mark_sent(const TxEntry& sent, int64_t slot_idx) {
    // Find and update the matching entry in the queue
    int found = -1;
    if (s_pending_idx >= 0 && s_pending_idx < (int)tx_queue.size()) {
        const TxEntry& cand = tx_queue[s_pending_idx];
        if (cand.dxcall == sent.dxcall && cand.field3 == sent.field3 &&
            cand.slot_id == sent.slot_id && cand.offset_hz == sent.offset_hz) {
            found = s_pending_idx;
        }
    }
    if (found == -1) {
        for (size_t i = 0; i < tx_queue.size(); ++i) {
            const TxEntry& cand = tx_queue[i];
            if (cand.dxcall == sent.dxcall && cand.field3 == sent.field3 &&
                cand.slot_id == sent.slot_id && cand.offset_hz == sent.offset_hz) {
                found = (int)i;
                break;
            }
        }
    }
    if (found != -1) {
        auto &e = tx_queue[found];
        e.repeat_counter = std::max(0, e.repeat_counter - 1);
        if (e.repeat_counter <= 0 || e.mark_delete) {
            tx_queue.erase(tx_queue.begin() + found);
        }
    }
    s_last_tx_slot_idx = slot_idx;
    s_last_tx_parity = sent.slot_id & 1;
    s_pending_valid = false;
    s_pending_idx = -1;
}

// New scheduler (used by main)
static bool tx_engine_pick_candidate(int slot_parity, bool enforce_gap, TxEntry& out) {
    int best_idx = -1;
    int best_prio = -1;
    // LIFO within priority: iterate from back
    for (int i = (int)tx_queue.size() - 1; i >= 0; --i) {
        const auto& e = tx_queue[i];
        if (e.mark_delete) continue;
        if ((e.slot_id & 1) != slot_parity) continue;
        int pr = tx_priority(e);
        if (pr > best_prio) {
            best_prio = pr;
            best_idx = i;
        } else if (pr == best_prio && best_idx == -1) {
            best_idx = i; // fallback
        }
    }
    if (best_idx == -1) return false;
    // Enforce a blank slot when switching slot groups
    if (enforce_gap && s_last_tx_parity != -1 && s_last_tx_parity != slot_parity) {
        return false;
    }
    out = tx_queue[best_idx];
    s_pending_idx = best_idx;
    return true;
}

bool tx_engine_fetch_pending(TxEntry& out) {
    if (!s_pending_valid) {
        // Try to schedule now using last tick data
        if (s_sched_parity != -1) {
            bool enforce_gap = (s_last_tx_parity != -1 && s_last_tx_parity != s_sched_parity &&
                                s_sched_slot_idx == s_last_tx_slot_idx + 1);
            TxEntry cand;
            if (tx_engine_pick_candidate(s_sched_parity, enforce_gap, cand)) {
                s_pending = cand;
                s_pending_valid = true;
            }
        }
    }
    if (!s_pending_valid) return false;
    out = s_pending;
    s_pending_valid = false; // fetched; will be removed/decremented on mark_sent
    ESP_LOGI("TXENG", "fetch_pending: dx=%s f3=%s slot=%d rep=%d", out.dxcall.c_str(),
             out.field3.c_str(), out.slot_id, out.repeat_counter);
    return true;
}
