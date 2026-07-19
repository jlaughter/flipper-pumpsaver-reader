#pragma once

// Pure C port of the PumpSaver Plus IR word decoder.
//
// This is a mechanical translation of decode_burst() from
// esphome-pumpsaver/components/pumpsaver/pumpsaver_decode.h (the
// project's own field-verified C++ reference decoder) to plain C with
// no STL, so it drops into a Flipper FAP unchanged. The bit math and
// constants are copied verbatim; only container types changed
// (std::array/vector -> plain arrays/pointers).
//
// None of this file would exist without lizbit-official's pumpsaver-ir-protocol
// project: https://github.com/lizbit-official/pumpsaver-ir-protocol
// They reverse-engineered this entire undocumented IR protocol from scratch -
// bit timing, the edge-skew correction, word framing, the sync word, the full
// register map - with zero vendor documentation to work from. Every constant
// and every line of bit math below traces straight back to that work. Massive
// credit and thanks to them for it; this Flipper port is a small add-on to
// something they did the genuinely hard part of.
//
// Protocol summary (see PROTOCOL.md in lizbit-official/pumpsaver-ir-protocol
// for the full spec):
//   - Baseband IR (no 38 kHz carrier), 5,000 baud NRZ, MSB-first.
//   - Runs of identical bits appear as one pulse; edge skew makes idle-level
//     pulses read ~101us short and active-level pulses ~101us long, so
//     run length n = round((|width| +/- 101) / 202).
//   - Each burst between >8ms idle gaps is one 32-bit word:
//     0x90 | reg:8 | value:16 (big-endian), trailing zero bits omitted
//     (right-pad to 32 bits).
//   - Sync word 0x90FFAAAA (reg 0xFF, value 0xAAAA) precedes every 4 data
//     words and carries no data.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define PS_BIT_US 202          // fitted bit period (nominal 200)
#define PS_HALF_BIT_US 101     // half-bit edge skew
#define PS_SEPARATOR_US 8000   // inter-word idle threshold (real gaps are 11-16ms)
#define PS_WORD_BITS 32
#define PS_MAX_RUN 28          // a run never spans more than the 28 bits after the '1001' header prefix
#define PS_HEADER 0x90u
#define PS_DATA_REG_FIRST 0x01u
#define PS_DATA_REG_LAST 0x75u
#define PS_SYNC_REG 0xFFu
#define PS_SYNC_VALUE 0xAAAAu

// Well-known registers (see registers.json / PROTOCOL.md section 5)
#define PS_REG_FAULT_AMPS 0x04u    // newest-fault latch, A x100 (leg-sum while running)
#define PS_REG_FAULT_VOLTS 0x05u   // newest-fault latch, V x10
#define PS_REG_FAULT_WATTS 0x06u   // newest-fault latch, W
#define PS_REG_RESTART_SET 0x14u   // restart-delay setting, minutes
#define PS_REG_RESTART_HI 0x15u    // restart countdown: (hi<<16|lo)/256 = seconds remaining
#define PS_REG_RESTART_LO 0x16u
#define PS_REG_POWER 0x10u         // true watts
#define PS_REG_VOLTAGE 0x11u       // volts x10
#define PS_REG_CURRENT 0x12u       // amps x100 idle, x200 running (leg-sum while running)
#define PS_REG_DRYWELL_TRIP 0x13u  // dry-well trip point, watts
#define PS_REG_PUMP_STARTS 0x0Fu   // user-clearable device counter
#define PS_REG_RUN_MINUTES 0x17u   // user-clearable device counter
#define PS_REG_CAL_VOLTAGE 0x02u   // calibration voltage, V x10 (candidate, not yet fully confirmed)

typedef struct {
    uint8_t reg;
    uint16_t value;
} PsWord;

static inline bool ps_word_is_sync(const PsWord* w) {
    return w->reg == PS_SYNC_REG && w->value == PS_SYNC_VALUE;
}

// Decode one pulse burst (the signed pulse widths, in us, between two >8ms
// separators) into a word. `pulses[i]` alternate sign by construction;
// `idle_positive` says which sign carries the idle (logical 0) level -
// pass the sign of the separator pulse that closed this burst; it is
// always the idle level. Returns false if the burst does not parse as a
// protocol word (this is normal/expected for noise or a truncated first
// burst after the app starts capturing mid-word).
static inline bool ps_decode_burst(
    const int32_t* pulses,
    size_t n_pulses,
    bool idle_positive,
    PsWord* out) {
    if(n_pulses == 0) return false;

    uint32_t word = 0;
    int total = 0;
    bool previous_positive = false;

    for(size_t i = 0; i < n_pulses; i++) {
        int32_t t = pulses[i];
        if(t == 0) return false;

        bool positive = t > 0;
        if(i > 0 && positive == previous_positive) return false; // levels must alternate
        previous_positive = positive;

        bool is_idle = positive == idle_positive;
        if(i == 0 && is_idle) return false; // every word begins with the active '1' header

        int64_t width = t < 0 ? -(int64_t)t : (int64_t)t;
        // idle pulses read ~101us short, active pulses ~101us long
        int64_t adjusted = is_idle ? width + PS_HALF_BIT_US : width - PS_HALF_BIT_US;
        if(adjusted < 0) return false;
        int n = (int)((adjusted + PS_BIT_US / 2) / PS_BIT_US);
        if(n < 1 || n > PS_MAX_RUN) return false; // implausible run length

        total += n;
        if(total > PS_WORD_BITS) return false; // burst exceeds 32 bits

        word <<= n;
        if(!is_idle) word |= (1u << n) - 1u; // active level = logical 1
    }
    word <<= (PS_WORD_BITS - total); // right-pad the omitted trailing zero bits

    if((word >> 24) != PS_HEADER) return false;

    uint8_t reg = (uint8_t)((word >> 16) & 0xFFu);
    uint16_t value = (uint16_t)(word & 0xFFFFu);

    if(reg == PS_SYNC_REG) {
        if(value != PS_SYNC_VALUE) return false;
    } else if(reg < PS_DATA_REG_FIRST || reg > PS_DATA_REG_LAST) {
        return false;
    }

    out->reg = reg;
    out->value = value;
    return true;
}

// ---------------------------------------------------------------------------
// Register table: latest value of every data register (0x01-0x75), plus
// per-register "last updated" tick for a staleness/liveness view. Index 0
// is unused (registers start at 0x01) to keep reg-as-index simple.
// ---------------------------------------------------------------------------

#define PS_REG_TABLE_SIZE (PS_DATA_REG_LAST + 1)

typedef struct {
    uint16_t value;
    uint32_t last_seen_ms;
    bool valid;
} PsRegSlot;

typedef struct {
    PsRegSlot regs[PS_REG_TABLE_SIZE];
    uint32_t words_ok;
    uint32_t words_sync;
    uint32_t words_err;
} PsRegTable;

static inline void ps_reg_table_init(PsRegTable* t) {
    for(size_t i = 0; i < PS_REG_TABLE_SIZE; i++) {
        t->regs[i].value = 0;
        t->regs[i].last_seen_ms = 0;
        t->regs[i].valid = false;
    }
    t->words_ok = 0;
    t->words_sync = 0;
    t->words_err = 0;
}

static inline void ps_reg_table_apply(PsRegTable* t, const PsWord* w, uint32_t now_ms) {
    if(ps_word_is_sync(w)) {
        t->words_sync++;
        return;
    }
    t->words_ok++;
    if(w->reg < PS_REG_TABLE_SIZE) {
        t->regs[w->reg].value = w->value;
        t->regs[w->reg].last_seen_ms = now_ms;
        t->regs[w->reg].valid = true;
    }
}

// ---------------------------------------------------------------------------
// Fault-history ring (registers 0x19-0x75). C port of the ESPHome
// component's FaultRing class, generalized to expose all 20 stored fault
// slots (not just the newest) for a browsable history view. See
// PROTOCOL.md section 5 for the wire layout this decodes:
//   0x19-0x1D  20 x 4-bit fault codes, packed MSB-first, newest first
//   0x1E-0x56  19 x (W, V*10, A*100) snapshots, record k starts at 0x1E+3k,
//              covering faults #2..#20 - fault #1 (newest) uses the live
//              latches (0x04-0x06) instead, pushed into slot 0 of the ring
//              the *next* time a fault occurs
//   0x57-0x74  20 x 3-byte run-clock timestamps (24-bit BE minutes)
//   0x75       trailer word (unresolved; excluded from generation-match
//              equality below, since its meaning isn't established)
//
// A generation is accepted only once all 0x19..0x75 registers arrive in
// strictly ascending order, and only committed once two consecutive
// complete generations have matching event fields (0x19..0x74) - this is
// what makes ps_fault_ring_update() safe to call from a live, possibly
// interrupted stream: a ring shift mid-scan, or a missed/duplicate
// register, just discards the in-progress candidate rather than mixing an
// old fault's codes with a new one's timestamp.
// ---------------------------------------------------------------------------

#define PS_REG_FAULT_FIRST 0x19u
#define PS_REG_FAULT_LAST PS_DATA_REG_LAST // 0x75
#define PS_FAULT_REG_COUNT (PS_REG_FAULT_LAST - PS_REG_FAULT_FIRST + 1) // 93
#define PS_FAULT_HISTORY_COUNT 20 // fault slots: 0 = newest (#1) .. 19 = oldest (#20)

typedef struct {
    uint8_t code;
    uint16_t watts;
    uint16_t volts_x10;
    uint16_t amps_x100;
    uint32_t at_minutes; // run-clock minutes at the fault (same unit as PS_REG_RUN_MINUTES)
} PsFaultInfo;

// Code 1 is proven (dry-well/dead-head underload trip, confirmed by live
// inducement). 2-4 follow the documented SymCom family fault-class
// ordering (overcurrent, voltage, rapid-cycle) but are unverified - hence
// the "?" (see PROTOCOL.md section 5 / open question 2).
static inline const char* ps_fault_code_name(uint8_t code) {
    switch(code) {
    case 0: return "none";
    case 1: return "dry well / underload";
    case 2: return "overcurrent?";
    case 3: return "voltage fault?";
    case 4: return "rapid cycle?";
    default: return "unknown code";
    }
}

// Renders run-clock minutes the way the Informer displayed them, e.g. "22d 14h 52m".
static inline void ps_format_run_clock(uint32_t minutes, char* buf, size_t len) {
    unsigned d = (unsigned)(minutes / 1440), h = (unsigned)((minutes / 60) % 24),
             m = (unsigned)(minutes % 60);
    snprintf(buf, len, "%ud %uh %um", d, h, m);
}

typedef struct {
    uint16_t staging[PS_FAULT_REG_COUNT];
    uint16_t candidate[PS_FAULT_REG_COUNT];
    uint16_t committed[PS_FAULT_REG_COUNT];
    uint16_t latch[3]; // 0=amps(0x04), 1=volts(0x05), 2=watts(0x06)
    uint8_t latch_seen; // bit i set once latch[i] has been seen at least once
    size_t next_index;
    bool collecting;
    bool candidate_valid;
    bool committed_valid;
} PsFaultRing;

static inline void ps_fault_ring_init(PsFaultRing* r) {
    memset(r, 0, sizeof(*r));
}

static inline bool ps_fault_event_equal(const uint16_t* a, const uint16_t* b) {
    // Excludes the final element (unresolved trailer 0x75): it terminates a
    // complete generation but isn't known to describe a fault event.
    for(size_t i = 0; i + 1 < PS_FAULT_REG_COUNT; i++) {
        if(a[i] != b[i]) return false;
    }
    return true;
}

// Feed one register word (from the live decode stream, in arrival order).
// Returns true only when a new, stable, complete generation has just been
// committed - including the very first one. Callers should treat that
// first true as a "quiet boot baseline" rather than a real new-fault
// event (see the App-level fault_baseline_set handling in
// pumpsaver_reader.c), matching the ESPHome component's fault_sequence
// semantics: it can't distinguish an old stored fault from one that
// occurred during acquisition, so the baseline doesn't count as an event.
static inline bool ps_fault_ring_update(PsFaultRing* r, uint8_t reg, uint16_t value) {
    if(reg < PS_REG_FAULT_FIRST || reg > PS_REG_FAULT_LAST) return false;

    size_t idx = reg - PS_REG_FAULT_FIRST;
    if(reg == PS_REG_FAULT_FIRST) {
        // A new 0x19 before the previous generation reached 0x75 proves the
        // preceding candidate wasn't followed by a complete adjacent refresh.
        if(r->collecting) r->candidate_valid = false;
        r->collecting = true;
        r->next_index = 1;
        r->staging[0] = value;
        return false;
    }

    if(!r->collecting || idx != r->next_index) {
        // A missing, duplicate, or out-of-order ring word makes this
        // generation incoherent; it must not help confirm an earlier candidate.
        r->collecting = false;
        r->candidate_valid = false;
        return false;
    }

    r->staging[idx] = value;
    r->next_index++;
    if(reg != PS_REG_FAULT_LAST) return false;

    r->collecting = false;
    if(!r->candidate_valid || !ps_fault_event_equal(r->candidate, r->staging)) {
        memcpy(r->candidate, r->staging, sizeof(r->candidate));
        r->candidate_valid = true;
        return false; // require one more matching generation before committing
    }

    if(r->committed_valid && ps_fault_event_equal(r->committed, r->staging)) return false;
    memcpy(r->committed, r->staging, sizeof(r->committed));
    r->committed_valid = true;
    return true;
}

// Feed a newest-fault latch register (0x04/0x05/0x06) from the live block.
static inline void ps_fault_ring_update_latch(PsFaultRing* r, uint8_t reg, uint16_t value) {
    if(reg < PS_REG_FAULT_AMPS || reg > PS_REG_FAULT_WATTS) return;
    int i = reg - PS_REG_FAULT_AMPS;
    r->latch[i] = value;
    r->latch_seen |= (uint8_t)(1u << i);
}

// True once at least one complete, confirmed generation has been committed
// and all three condition latches have been seen at least once.
static inline bool ps_fault_ring_ready(const PsFaultRing* r) {
    return r->committed_valid && r->latch_seen == 0x7u;
}

static inline uint16_t ps_fault_ring_get(const PsFaultRing* r, uint8_t reg) {
    return r->committed[reg - PS_REG_FAULT_FIRST];
}

// Byte `byte_index` (0..59) of the 60-byte, big-endian stream formed by
// registers 0x57..0x74 read as (hi, lo) pairs - the general form of the
// "read two registers, take r0 whole plus r1's top byte" trick the
// ESPHome header uses just for the newest timestamp.
static inline uint8_t ps_fault_ring_byte(const PsFaultRing* r, int byte_index) {
    uint8_t reg_offset = (uint8_t)(byte_index / 2);
    uint16_t word = ps_fault_ring_get(r, (uint8_t)(0x57 + reg_offset));
    return (byte_index % 2 == 0) ? (uint8_t)(word >> 8) : (uint8_t)(word & 0xFFu);
}

// slot: 0 = newest (fault #1) .. PS_FAULT_HISTORY_COUNT-1 = oldest (#20).
// Only meaningful once ps_fault_ring_ready() is true. Fault #1's W/V/A
// come from the live latches; #2-#20 come from the 19-record snapshot grid.
static inline void ps_fault_ring_get_slot(const PsFaultRing* r, int slot, PsFaultInfo* out) {
    uint8_t code_reg = (uint8_t)(0x19 + slot / 4);
    int nibble_pos = slot % 4;
    out->code = (uint8_t)((ps_fault_ring_get(r, code_reg) >> (12 - 4 * nibble_pos)) & 0xFu);

    if(slot == 0) {
        out->amps_x100 = r->latch[0];
        out->volts_x10 = r->latch[1];
        out->watts = r->latch[2];
    } else {
        uint8_t base_reg = (uint8_t)(0x1E + 3 * (slot - 1));
        out->watts = ps_fault_ring_get(r, base_reg);
        out->volts_x10 = ps_fault_ring_get(r, (uint8_t)(base_reg + 1));
        out->amps_x100 = ps_fault_ring_get(r, (uint8_t)(base_reg + 2));
    }

    int b = slot * 3;
    out->at_minutes = ((uint32_t)ps_fault_ring_byte(r, b) << 16) |
                       ((uint32_t)ps_fault_ring_byte(r, b + 1) << 8) |
                       (uint32_t)ps_fault_ring_byte(r, b + 2);
}
