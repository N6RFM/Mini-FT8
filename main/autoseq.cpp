/*
 * autoseq.cpp - FT8 CQ/QSO auto-sequencing engine for Mini-FT8
 *
 * Ported from DX-FT8-MULTIBAND-TABLET-TRANSCEIVER autoseq_engine.c
 * Adapted for ESP32/C++ with integrated TX scheduling.
 */

#include "autoseq.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include "esp_log.h"

static const char* TAG = "AUTOSEQ";

// ============== Internal state ==============

static QsoContext s_queue[AUTOSEQ_MAX_QUEUE];
static int s_queue_size = 0;

// Station configuration
static std::string s_my_call;
static std::string s_my_grid;
static bool s_skip_tx1 = false;

// CQ configuration
static AutoseqCqType s_cq_type = AutoseqCqType::CQ;
static std::string s_cq_freetext;

// ADIF callback
static AdifLogCallback s_adif_callback;

// TX scheduling state
static bool s_pending_valid = false;
static AutoseqTxEntry s_pending;
static int s_pending_ctx_idx = -1;
static int64_t s_last_tx_slot_idx = -1000;
static int s_last_tx_parity = -1;

// ============== Forward declarations ==============

static void set_state(QsoContext* ctx, AutoseqState s, TxMsgType first_tx, int limit);
static void format_tx_text(const QsoContext* ctx, TxMsgType id, std::string& out);
static TxMsgType parse_rcvd_msg(QsoContext* ctx, const UiRxLine& msg);
static bool generate_response(QsoContext* ctx, const UiRxLine& msg, bool override);
static void on_decode(const UiRxLine& msg);
static int compare_ctx(const void* a, const void* b);
static void pop_front();
static QsoContext* append_ctx();
static void sort_and_clean();
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static void log_qso_if_needed(QsoContext* ctx);

// ============== Public API ==============

void autoseq_init() {
    s_queue_size = 0;
    s_pending_valid = false;
    s_pending_ctx_idx = -1;
    s_last_tx_slot_idx = -1000;
    s_last_tx_parity = -1;
}

void autoseq_clear() {
    autoseq_init();
}

bool autoseq_drop_index(int idx) {
    if (idx < 0 || idx >= s_queue_size) return false;
    for (int i = idx; i + 1 < s_queue_size; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    --s_queue_size;
    return true;
}

bool autoseq_rotate_same_parity() {
    if (s_queue_size < 2) return false;
    int parity = s_queue[0].slot_id & 1;
    int found = -1;
    for (int i = 1; i < s_queue_size; ++i) {
        if ((s_queue[i].slot_id & 1) == parity) {
            found = i;
            break;
        }
    }
    if (found == -1) return false;
    QsoContext tmp = s_queue[found];
    for (int i = found; i > 0; --i) {
        s_queue[i] = s_queue[i - 1];
    }
    s_queue[0] = tmp;
    return true;
}

void autoseq_start_cq(int slot_parity) {
    // Don't add duplicate CQ at bottom
    if (s_queue_size > 0 && s_queue_size < AUTOSEQ_MAX_QUEUE) {
        if (s_queue[s_queue_size - 1].state == AutoseqState::CALLING) {
            return;
        }
    }
    if (s_queue_size >= AUTOSEQ_MAX_QUEUE) {
        return;
    }

    QsoContext* ctx = append_ctx();
    ctx->dxcall = "CQ";
    ctx->dxgrid.clear();
    ctx->snr_tx = -99;
    ctx->snr_rx = -99;
    ctx->slot_id = slot_parity;  // Use the specified slot parity
    set_state(ctx, AutoseqState::CALLING, TxMsgType::TX6, 0);
    // No sort needed - CQ always at bottom
    ESP_LOGI(TAG, "Started CQ on slot %d", slot_parity);
}

void autoseq_on_touch(const UiRxLine& msg) {
    // If queue is full, remove the last one
    if (s_queue_size == AUTOSEQ_MAX_QUEUE) {
        --s_queue_size;
    }

    QsoContext* ctx = append_ctx();

    // Determine the DX callsign from the message (normalize to uppercase)
    std::string dxcall;
    if (!msg.field2.empty()) {
        dxcall = msg.field2;  // field2 is the sender
    } else if (!msg.field1.empty() && msg.field1 != "CQ") {
        dxcall = msg.field1;
    }
    for (auto& ch : dxcall) ch = toupper((unsigned char)ch);

    // Check if it's addressed to me
    std::string f1_upper = msg.field1;
    for (auto& ch : f1_upper) ch = toupper((unsigned char)ch);
    std::string my_upper = s_my_call;
    for (auto& ch : my_upper) ch = toupper((unsigned char)ch);

    if (!my_upper.empty() && f1_upper == my_upper) {
        generate_response(ctx, msg, true);
        sort_and_clean();
        return;
    }

    // Treat as calling CQ - we're initiating contact
    ctx->dxcall = dxcall;
    if (looks_like_grid(msg.field3)) {
        ctx->dxgrid = msg.field3;
    }
    ctx->snr_tx = msg.snr;  // Our measurement of their signal
    ctx->offset_hz = msg.offset_hz;
    ctx->slot_id = msg.slot_id ^ 1;  // TX on opposite slot

    set_state(ctx, s_skip_tx1 ? AutoseqState::REPORT : AutoseqState::REPLYING,
              s_skip_tx1 ? TxMsgType::TX2 : TxMsgType::TX1, AUTOSEQ_MAX_RETRY);
    sort_and_clean();

    ESP_LOGI(TAG, "Touch: %s grid=%s snr=%d", ctx->dxcall.c_str(),
             ctx->dxgrid.c_str(), ctx->snr_tx);
}

void autoseq_on_decodes(const std::vector<UiRxLine>& to_me_messages) {
    ESP_LOGI(TAG, "on_decodes: %d messages, queue_size=%d",
             (int)to_me_messages.size(), s_queue_size);
    for (const auto& msg : to_me_messages) {
        ESP_LOGI(TAG, "  msg: %s %s %s snr=%d",
                 msg.field1.c_str(), msg.field2.c_str(), msg.field3.c_str(), msg.snr);
        on_decode(msg);
    }
    sort_and_clean();
    if (s_queue_size > 0) {
        ESP_LOGI(TAG, "on_decodes done: queue[0] state=%d, next_tx=%d, dxcall=%s",
                 (int)s_queue[0].state, (int)s_queue[0].next_tx, s_queue[0].dxcall.c_str());
    }
}

// Called AFTER TX completes to set up retry for next attempt
// This is the reference architecture - tick is for retry management, not scheduling
void autoseq_tick(int64_t slot_idx, int slot_parity, int ms_to_boundary) {
    (void)slot_idx; (void)slot_parity; (void)ms_to_boundary;  // unused for now

    if (s_queue_size == 0) return;

    QsoContext* ctx = &s_queue[0];

    // Advance retry counter or timeout - sets up NEXT TX attempt
    switch (ctx->state) {
        case AutoseqState::REPLYING:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX1;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::REPORT:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX2;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::ROGER_REPORT:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX3;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::ROGERS:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX4;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::CALLING:  // CQ only once (controlled by beacon)
        case AutoseqState::SIGNOFF:  // QSO complete, remove from queue
            ctx->state = AutoseqState::IDLE;
            ctx->next_tx = TxMsgType::TX_UNDEF;
            break;
        default:
            break;
    }

    if (ctx->state == AutoseqState::IDLE) {
        pop_front();
    }

    ESP_LOGI(TAG, "Tick: queue_size=%d, state=%d, next_tx=%d, retry=%d/%d",
             s_queue_size, s_queue_size > 0 ? (int)s_queue[0].state : -1,
             s_queue_size > 0 ? (int)s_queue[0].next_tx : -1,
             s_queue_size > 0 ? s_queue[0].retry_counter : 0,
             s_queue_size > 0 ? s_queue[0].retry_limit : 0);
}

// Get the next TX message text based on current state (does NOT modify state)
// Returns true if there's a TX ready, false otherwise
bool autoseq_get_next_tx(std::string& out_text) {
    out_text.clear();

    if (s_queue_size == 0) return false;

    QsoContext* ctx = &s_queue[0];
    if (ctx->state == AutoseqState::IDLE || ctx->next_tx == TxMsgType::TX_UNDEF) {
        return false;
    }

    format_tx_text(ctx, ctx->next_tx, out_text);
    return !out_text.empty();
}

// Get the pending TX entry - populates from current context state
// Does NOT modify state - just reads current next_tx
bool autoseq_fetch_pending_tx(AutoseqTxEntry& out) {
    if (s_queue_size == 0) return false;

    QsoContext* ctx = &s_queue[0];
    if (ctx->state == AutoseqState::IDLE || ctx->next_tx == TxMsgType::TX_UNDEF) {
        return false;
    }

    std::string tx_text;
    format_tx_text(ctx, ctx->next_tx, tx_text);
    if (tx_text.empty()) return false;

    out.text = tx_text;
    out.dxcall = ctx->dxcall;
    out.offset_hz = ctx->offset_hz;
    out.slot_id = ctx->slot_id;
    out.repeat_counter = ctx->retry_limit - ctx->retry_counter;
    out.is_signoff = (ctx->next_tx == TxMsgType::TX4 ||
                      ctx->next_tx == TxMsgType::TX5);

    ESP_LOGI(TAG, "Fetch TX: %s (state=%d, next_tx=%d)",
             tx_text.c_str(), (int)ctx->state, (int)ctx->next_tx);
    return true;
}

void autoseq_mark_sent(int64_t slot_idx, bool sent_signoff) {
    if (s_queue_size == 0) return;

    s_last_tx_slot_idx = slot_idx;
    s_last_tx_parity = s_queue[0].slot_id & 1;

    if (sent_signoff) {
        log_qso_if_needed(&s_queue[0]);
    }

    ESP_LOGI(TAG, "TX sent on slot %lld", slot_idx);
}

void autoseq_get_qso_states(std::vector<std::string>& out) {
    out.clear();
    static const char* state_names[] = {
        "CALL", "RPLY", "RPRT", "RRPT", "RGRS", "SOFF", ""
    };

    for (int i = 0; i < s_queue_size; ++i) {
        const QsoContext* ctx = &s_queue[i];
        if (ctx->state == AutoseqState::IDLE) continue;

        char buf[32];
        snprintf(buf, sizeof(buf), "%-8.8s %.4s %d/%d",
                 ctx->dxcall.c_str(),
                 state_names[(int)ctx->state],
                 ctx->retry_counter, ctx->retry_limit);
        out.push_back(buf);
    }
}

bool autoseq_has_active_qso() {
    for (int i = 0; i < s_queue_size; ++i) {
        if (s_queue[i].state != AutoseqState::IDLE &&
            s_queue[i].state != AutoseqState::CALLING) {
            return true;
        }
    }
    return false;
}

int autoseq_queue_size() {
    return s_queue_size;
}

void autoseq_set_adif_callback(AdifLogCallback cb) {
    s_adif_callback = cb;
}

void autoseq_set_station(const std::string& call, const std::string& grid) {
    s_my_call = call;
    s_my_grid = grid;
}

void autoseq_set_skip_tx1(bool skip) {
    s_skip_tx1 = skip;
}

void autoseq_set_cq_type(AutoseqCqType type, const std::string& freetext) {
    s_cq_type = type;
    s_cq_freetext = freetext;
}

// ============== Internal helpers ==============

static void set_state(QsoContext* ctx, AutoseqState s, TxMsgType first_tx, int limit) {
    ctx->state = s;
    ctx->next_tx = first_tx;
    ctx->retry_counter = 0;
    ctx->retry_limit = limit;
}

static void format_tx_text(const QsoContext* ctx, TxMsgType id, std::string& out) {
    out.clear();
    if (!ctx || ctx->state == AutoseqState::IDLE) return;

    char buf[64];

    switch (id) {
        case TxMsgType::TX1:
            snprintf(buf, sizeof(buf), "%s %s %s",
                     ctx->dxcall.c_str(), s_my_call.c_str(), s_my_grid.c_str());
            out = buf;
            break;

        case TxMsgType::TX2:
            snprintf(buf, sizeof(buf), "%s %s %+d",
                     ctx->dxcall.c_str(), s_my_call.c_str(), ctx->snr_tx);
            out = buf;
            break;

        case TxMsgType::TX3:
            snprintf(buf, sizeof(buf), "%s %s R%+d",
                     ctx->dxcall.c_str(), s_my_call.c_str(), ctx->snr_tx);
            out = buf;
            break;

        case TxMsgType::TX4:
            snprintf(buf, sizeof(buf), "%s %s RR73",
                     ctx->dxcall.c_str(), s_my_call.c_str());
            out = buf;
            break;

        case TxMsgType::TX5:
            snprintf(buf, sizeof(buf), "%s %s 73",
                     ctx->dxcall.c_str(), s_my_call.c_str());
            out = buf;
            break;

        case TxMsgType::TX6: {
            const char* cq_prefix = "CQ";
            switch (s_cq_type) {
                case AutoseqCqType::SOTA: cq_prefix = "CQ SOTA"; break;
                case AutoseqCqType::POTA: cq_prefix = "CQ POTA"; break;
                case AutoseqCqType::QRP:  cq_prefix = "CQ QRP";  break;
                case AutoseqCqType::FD:   cq_prefix = "CQ FD";   break;
                case AutoseqCqType::FREETEXT:
                    out = s_cq_freetext;
                    return;
                default: break;
            }
            snprintf(buf, sizeof(buf), "%s %s %s",
                     cq_prefix, s_my_call.c_str(), s_my_grid.c_str());
            out = buf;
            break;
        }

        default:
            break;
    }
}

static TxMsgType parse_rcvd_msg(QsoContext* ctx, const UiRxLine& msg) {
    TxMsgType rcvd = TxMsgType::TX_UNDEF;
    // Normalize field3 to uppercase for comparison
    std::string f3 = msg.field3;
    for (auto& ch : f3) ch = toupper((unsigned char)ch);

    // Check specific keywords FIRST before grid pattern
    // (RR73 matches grid pattern AA00 but is actually TX4!)
    if (f3 == "RR73" || f3 == "RRR") {
        rcvd = TxMsgType::TX4;
    } else if (f3 == "73") {
        rcvd = TxMsgType::TX5;
    } else if (looks_like_grid(f3)) {
        rcvd = TxMsgType::TX1;
        // Only update grid if we don't have one yet (preserve from initial exchange)
        if (ctx->dxgrid.empty()) {
            ctx->dxgrid = f3;
        }
    } else if (!f3.empty() && f3[0] == 'R' && f3.size() > 1) {
        // Check if it's R+report (TX3)
        int rpt = 0;
        if (looks_like_report(f3.substr(1), rpt)) {
            rcvd = TxMsgType::TX3;
            ctx->snr_rx = rpt;  // What they reported about us
        }
    } else {
        // Check for plain report (TX2)
        int rpt = 0;
        if (looks_like_report(f3, rpt)) {
            rcvd = TxMsgType::TX2;
            ctx->snr_rx = rpt;  // What they reported about us
        }
    }

    ctx->rcvd_msg_type = rcvd;
    return rcvd;
}

static void log_qso_if_needed(QsoContext* ctx) {
    if (ctx->logged) return;
    if (!s_adif_callback) return;

    ctx->logged = true;
    s_adif_callback(ctx->dxcall, ctx->dxgrid, ctx->snr_tx, ctx->snr_rx);

    ESP_LOGI(TAG, "Logged QSO: %s grid=%s rst_sent=%d rst_rcvd=%d",
             ctx->dxcall.c_str(), ctx->dxgrid.c_str(), ctx->snr_tx, ctx->snr_rx);
}

static bool generate_response(QsoContext* ctx, const UiRxLine& msg, bool override) {
    TxMsgType rcvd = parse_rcvd_msg(ctx, msg);

    ESP_LOGI(TAG, "generate_response: override=%d, rcvd=%d, ctx->state=%d",
             override, (int)rcvd, (int)ctx->state);

    if (rcvd == TxMsgType::TX_UNDEF) {
        ESP_LOGW(TAG, "generate_response: rcvd=TX_UNDEF, returning false");
        return false;
    }

    // Update SNR we report to them on initial messages
    if (rcvd == TxMsgType::TX1 || rcvd == TxMsgType::TX2) {
        ctx->snr_tx = msg.snr;
    }

    // Get DX callsign from field2 (the sender), normalize to uppercase
    std::string dxcall = msg.field2;
    if (dxcall.empty()) dxcall = msg.field1;
    for (auto& ch : dxcall) ch = toupper((unsigned char)ch);

    if (override) {
        ctx->dxcall = dxcall;
        ctx->offset_hz = msg.offset_hz;
        ctx->slot_id = msg.slot_id ^ 1;  // TX on opposite slot

        // Reset state based on received message type
        switch (rcvd) {
            case TxMsgType::TX1:
                set_state(ctx, AutoseqState::CALLING, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX2:
                set_state(ctx, AutoseqState::REPLYING, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX3:
                set_state(ctx, AutoseqState::REPORT, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX4:
                set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX5:
                set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX_UNDEF, 0);
                break;
            default:
                break;
        }
    }

    // State machine transitions
    switch (ctx->state) {
        case AutoseqState::CALLING:  // We sent CQ
            switch (rcvd) {
                case TxMsgType::TX1:
                    set_state(ctx, AutoseqState::REPORT, TxMsgType::TX2, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX2:
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, AUTOSEQ_MAX_RETRY);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::REPLYING:  // We sent TX1
            switch (rcvd) {
                case TxMsgType::TX2:
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    // QSO complete without full exchange
                    log_qso_if_needed(ctx);
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, 0);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::REPORT:  // We sent TX2
            switch (rcvd) {
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    log_qso_if_needed(ctx);
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, 0);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::ROGER_REPORT:  // We sent TX3
            switch (rcvd) {
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    log_qso_if_needed(ctx);
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, AUTOSEQ_MAX_RETRY);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::ROGERS:  // We sent TX4
            switch (rcvd) {
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    log_qso_if_needed(ctx);
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX_UNDEF, 0);
                    break;
                default:
                    break;
            }
            return false;

        case AutoseqState::SIGNOFF:  // We sent TX5
            switch (rcvd) {
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    // They didn't get our 73; send another
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, AUTOSEQ_MAX_RETRY);
                    log_qso_if_needed(ctx);
                    return true;
                default:
                    // Ignore other traffic; keep context for a bit
                    return false;
            }
            return false;

        default:
            break;
    }

    return false;
}

static void on_decode(const UiRxLine& msg) {
    // Check if it's addressed to us
    std::string f1_upper = msg.field1;
    for (auto& ch : f1_upper) ch = toupper((unsigned char)ch);
    std::string my_upper = s_my_call;
    for (auto& ch : my_upper) ch = toupper((unsigned char)ch);

    if (my_upper.empty() || f1_upper != my_upper) {
        return;
    }

    // Get DX call from field2 (normalize to uppercase for comparison)
    std::string dxcall = msg.field2;
    for (auto& ch : dxcall) ch = toupper((unsigned char)ch);
    if (dxcall.empty()) return;

    // Check if it matches an existing context (case-insensitive)
    for (int i = 0; i < s_queue_size; ++i) {
        QsoContext* ctx = &s_queue[i];
        std::string ctx_dxcall = ctx->dxcall;
        for (auto& ch : ctx_dxcall) ch = toupper((unsigned char)ch);
        if (ctx_dxcall == dxcall) {
            ESP_LOGI(TAG, "on_decode: found ctx for %s, state=%d, next_tx=%d",
                     dxcall.c_str(), (int)ctx->state, (int)ctx->next_tx);
            generate_response(ctx, msg, false);
            ESP_LOGI(TAG, "on_decode: after response, state=%d, next_tx=%d",
                     (int)ctx->state, (int)ctx->next_tx);
            return;
        }
    }

    // Check message type before creating new context
    // Don't create context for signoff messages (RR73/73) - these are late
    // messages from completed QSOs
    std::string f3 = msg.field3;
    for (auto& ch : f3) ch = toupper((unsigned char)ch);
    if (f3 == "RR73" || f3 == "RRR" || f3 == "73") {
        ESP_LOGW(TAG, "on_decode: ignoring late signoff from %s (no active ctx)",
                 dxcall.c_str());
        return;
    }

    ESP_LOGW(TAG, "on_decode: NO ctx found for %s, creating new (queue_size=%d)",
             dxcall.c_str(), s_queue_size);

    // No matching context - create new if queue not full
    if (s_queue_size >= AUTOSEQ_MAX_QUEUE) {
        return;
    }

    QsoContext* ctx = append_ctx();
    generate_response(ctx, msg, true);
    ESP_LOGI(TAG, "on_decode: NEW ctx %s, state=%d, next_tx=%d",
             ctx->dxcall.c_str(), (int)ctx->state, (int)ctx->next_tx);
}

// Comparison for qsort: IDLE at top (to be popped), CALLING at bottom
static int compare_ctx(const void* a, const void* b) {
    const QsoContext* left = (const QsoContext*)a;
    const QsoContext* right = (const QsoContext*)b;

    // Same state? Lower retry count wins (gets priority)
    if (left->state == right->state) {
        // More retries = lower priority
        if (left->retry_counter < right->retry_counter) return -1;
        if (left->retry_counter > right->retry_counter) return 1;
        return 0;
    }

    // Higher state value wins (more advanced in QSO)
    // DESCENDING order: IDLE(6) > SIGNOFF(5) > ROGERS(4) > ROGER_REPORT(3) > ...
    // IDLE at front gets popped; more advanced QSOs processed first
    if (left->state > right->state) return -1;  // Higher state comes first
    if (left->state < right->state) return 1;
    return 0;
}

static void pop_front() {
    if (s_queue_size <= 0) return;

    // Shift array
    for (int i = 0; i < s_queue_size - 1; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    --s_queue_size;
}

static QsoContext* append_ctx() {
    if (s_queue_size >= AUTOSEQ_MAX_QUEUE) return nullptr;

    QsoContext* ctx = &s_queue[s_queue_size++];
    *ctx = QsoContext{};  // Reset to defaults
    return ctx;
}

static void sort_and_clean() {
    if (s_queue_size == 0) return;

    qsort(s_queue, s_queue_size, sizeof(QsoContext), compare_ctx);

    // Pop IDLE entries from front
    while (s_queue_size > 0 && s_queue[0].state == AutoseqState::IDLE) {
        pop_front();
    }
}

static bool looks_like_grid(const std::string& s) {
    if (s.size() != 4) return false;
    // Pattern: AA00 (letters, letters, digits, digits)
    return isalpha((unsigned char)s[0]) && isalpha((unsigned char)s[1]) &&
           isdigit((unsigned char)s[2]) && isdigit((unsigned char)s[3]);
}

static bool looks_like_report(const std::string& s, int& out) {
    if (s.empty()) return false;

    // Parse optional sign and digits
    size_t idx = 0;
    bool neg = false;

    if (s[idx] == '+') {
        idx++;
    } else if (s[idx] == '-') {
        neg = true;
        idx++;
    }

    if (idx >= s.size() || !isdigit((unsigned char)s[idx])) return false;

    int val = 0;
    while (idx < s.size() && isdigit((unsigned char)s[idx])) {
        val = val * 10 + (s[idx] - '0');
        idx++;
    }

    // Must consume entire string
    if (idx != s.size()) return false;

    // Valid FT8 report range: -30 to +30
    if (val > 30) return false;

    out = neg ? -val : val;
    return true;
}
