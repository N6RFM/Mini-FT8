# Autoseq Architecture: Reference Project Design

This document describes the single-threaded state machine architecture used in the
DX-FT8 reference project and how Mini-FT8 implements the same pattern.

## Core Principle: Three Events, Strict Ordering

The reference project is a **single-threaded state machine** with exactly 3 events:

```
Event 1: 79 symbols received → start decoding
Event 2: decoding finished → autoseq parses decodes, populates TX message
Event 3: 15s slot boundary → check parity, TX if message ready, tick if was_txing
```

**Critical: Event 2 ALWAYS happens before Event 3.** This ordering eliminates race conditions.

## Event Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         15-SECOND SLOT TIMELINE                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  SLOT N (RX)              │  SLOT N+1 (TX)           │  SLOT N+2 (RX)   │
│  ─────────────────────────┼──────────────────────────┼─────────────────  │
│                           │                          │                   │
│  [Receive 79 symbols]     │  [Transmit if ready]     │  [Receive again]  │
│           │               │         │                │                   │
│           ▼               │         │                │                   │
│  Event 1: Decode start    │         │                │                   │
│           │               │         │                │                   │
│           ▼               │         │                │                   │
│  Event 2: Decode done     │         │                │                   │
│    - autoseq_on_decodes() │         │                │                   │
│    - Check TX ready       │         │                │                   │
│    - Set QSO_xmit flag    │         │                │                   │
│           │               │         │                │                   │
│           └───────────────┼────►Event 3: Slot boundary                   │
│                           │    - if was_txing: tick()│                   │
│                           │    - if QSO_xmit &&      │                   │
│                           │      correct_parity:     │                   │
│                           │      start TX            │                   │
│                           │         │                │                   │
│                           │         ▼                │                   │
│                           │    [TX runs ~12.8s]      │                   │
│                           │         │                │                   │
│                           │         └────────────────┼──► tick() at      │
│                           │                          │    next boundary  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Reference Project Code Pattern (main.c)

```c
// EVENT 3: Slot boundary detection and TX trigger
int current_slot = ft8_time / 15000 % 2;
if (current_slot != slot_state) {
    slot_state ^= 1;

    // If we were transmitting, call tick to pop completed TX
    if (was_txing) {
        autoseq_tick();  // Pops the entry that was just transmitted
    }
    was_txing = 0;
}

// TX trigger: check if we should transmit in this slot
if (QSO_xmit && target_slot == slot_state && FT_8_counter < 29) {
    setup_to_transmit_on_next_DSP_Flag();
    QSO_xmit = 0;
    was_txing = 1;
}

// EVENT 2: After decode finishes (decode_flag set by decoder)
if (decode_flag) {
    // Only process decodes when NOT transmitting
    if (!was_txing) {
        autoseq_on_decodes(new_decoded, master_decoded);

        // Check if TX is ready from autoseq
        if (autoseq_get_next_tx(autoseq_txbuf)) {
            queue_custom_text(autoseq_txbuf);
            QSO_xmit = 1;
        }
        // ELSE IF beacon on, add CQ
        else if (Beacon_On) {
            autoseq_start_cq();
            autoseq_get_next_tx(autoseq_txbuf);
            queue_custom_text(autoseq_txbuf);
            QSO_xmit = 1;
            target_slot = slot_state ^ 1;
        }
    }
    decode_flag = 0;
}
```

## CQ/Beacon Design: Short-Lived Entries

**Key insight: CQ entries are SHORT-LIVED.** They exist in the queue for exactly ONE transmission.

```
1. Beacon is ON, no active QSO
2. Decode callback: autoseq_get_next_tx() returns false (queue empty)
3. Since Beacon_On: autoseq_start_cq() adds CQ entry
   - state = CALLING
   - next_tx = TX6 (CQ message)
   - dxcall = "CQ"
4. autoseq_get_next_tx() now returns true (CQ ready)
5. Set QSO_xmit = 1, target_slot = opposite parity
6. At slot boundary: TX starts, was_txing = 1
7. TX completes (12.8 seconds)
8. Next slot boundary: autoseq_tick() is called
9. tick() sees state=CALLING, transitions to IDLE, pops entry
10. Queue is now empty
11. Next decode: back to step 2, new CQ is added
```

**Why short-lived?**
- Makes beacon flag changes take effect immediately
- If beacon is turned OFF, the CQ in queue transmits once, then tick() removes it
- No stale CQ entries accumulating

## Queue Sorting Rules

The queue is sorted with this priority (descending):
```
IDLE (6)      → highest priority (gets popped first)
SIGNOFF (5)
ROGERS (4)
ROGER_REPORT (3)
REPORT (2)
REPLYING (1)
CALLING (0)   → lowest priority (CQ stays at bottom)
```

**Rationale:**
- IDLE entries are popped immediately by sort_and_clean()
- More advanced QSOs (closer to completion) get priority
- CQ (CALLING) stays at bottom, only transmitted when no active QSO

## autoseq_tick() Behavior

Called ONCE per TX slot, at slot boundary AFTER TX completes:

```c
void autoseq_tick(void) {
    if (queue_size == 0) return;

    ctx_t *ctx = &ctx_queue[0];  // Always operates on FRONT entry

    switch (ctx->state) {
        case AS_REPLYING:
        case AS_REPORT:
        case AS_ROGER_REPORT:
        case AS_ROGERS:
            // Increment retry counter or transition to IDLE if max retries
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TX_for_state;
                ctx->retry_counter++;
            } else {
                ctx->state = AS_IDLE;
            }
            break;

        case AS_CALLING:   // CQ - only once (controlled by beacon)
        case AS_SIGNOFF:   // 73 - only once
            ctx->state = AS_IDLE;
            ctx->next_tx = TX_UNDEF;
            break;
    }

    if (ctx->state == AS_IDLE) {
        pop();  // Remove from queue
    }
}
```

**Key points:**
- tick() always operates on queue[0] (the front)
- CALLING (CQ) immediately transitions to IDLE and gets popped
- This is why CQ is short-lived: one TX, one tick, gone

## Why This Design Prevents Race Conditions

**Problem with immediate TX scheduling:**
```
1. Decode finishes, TX is scheduled with delay
2. During delay, ANOTHER decode happens
3. New decode adds entries, queue is re-sorted
4. Original TX entry is no longer at queue[0]
5. tick() operates on wrong entry
6. Bug: CQ never gets popped, or wrong entry gets popped
```

**Reference design solution:**
```
1. Decode finishes, just SET FLAGS (QSO_xmit, target_slot)
2. Another decode happens - flags might be updated, that's fine
3. At slot boundary: check flags, start TX if conditions met
4. TX runs for entire slot
5. Next slot boundary: tick() operates on queue[0]
6. Since Event 2 always before Event 3, queue is stable when tick() runs
```

## Mini-FT8 Implementation

Mini-FT8 now follows the same pattern:

```cpp
// State machine variables
static volatile bool g_qso_xmit = false;        // TX is pending
static volatile int g_target_slot_parity = 0;   // Parity for TX
static volatile bool g_was_txing = false;       // For tick timing
static int64_t g_last_slot_idx = -1;            // Slot boundary detection

// Called from main loop every tick
static void check_slot_boundary() {
    int64_t slot_idx = now_ms / 15000;
    int slot_parity = slot_idx & 1;

    // Detect slot boundary
    if (slot_idx != g_last_slot_idx) {
        g_last_slot_idx = slot_idx;

        // If we were transmitting, call tick
        if (g_was_txing) {
            autoseq_tick(slot_idx, slot_parity, 0);
            g_was_txing = false;
        }
    }

    // TX trigger
    if (g_qso_xmit && g_target_slot_parity == slot_parity &&
        slot_ms < 4000 && s_tx_task_handle == NULL) {
        g_qso_xmit = false;
        tx_start_immediate(skip_tones);
    }
}

// In decode_monitor_results (Event 2)
if (!g_was_txing) {
    if (!to_me.empty()) {
        autoseq_on_decodes(to_me);
    }

    AutoseqTxEntry pending;
    if (autoseq_fetch_pending_tx(pending)) {
        g_qso_xmit = true;
        g_target_slot_parity = pending.slot_id & 1;
    } else if (g_beacon != BeaconMode::OFF) {
        enqueue_beacon_cq();
        if (autoseq_fetch_pending_tx(pending)) {
            g_qso_xmit = true;
            g_target_slot_parity = pending.slot_id & 1;
        }
    }
}
```

## Summary: The Golden Rules

1. **Decode sets flags, slot boundary triggers TX** - Never start TX immediately after decode
2. **tick() at slot boundary AFTER TX** - Not during TX, not after decode
3. **CQ is short-lived** - Added when needed, transmitted once, popped by tick
4. **Queue[0] is always the active entry** - tick() and fetch_pending_tx() both use queue[0]
5. **Event 2 before Event 3** - Decode processing completes before TX decision is made
