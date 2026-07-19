#pragma once

// GPIO edge-timestamp capture core.
//
// Installs a rise/fall interrupt on one Flipper GPIO pin and timestamps
// every edge with microsecond resolution using the Cortex-M4 DWT cycle
// counter (SysTick alone is too coarse/gets reprogrammed by the RTOS; DWT
// is the standard free-running high-res timer used for this kind of
// signal-timing capture on STM32). Edges are pushed into a small
// single-producer/single-consumer ring buffer as signed pulse widths
// (sign = level held during that interval), matching the convention the
// PumpSaver decoder expects: alternating sign per pulse, with the actual
// idle/active mapping resolved from the separator sign at decode time.
//
// This module has no GUI/App dependency - it's the same building block
// useful for (a) the raw-dump validation mode and (b) the on-device
// decoder, so there's no throwaway work between the "validate the wiring"
// step and the "build the real thing" step.
//
// NOTE: furi_hal_gpio_add_int_callback / furi_hal_gpio_read are the Furi
// HAL functions used by other community GPIO-capture FAPs (e.g. the
// logic-analyzer app's polling approach; this uses the interrupt-driven
// sibling API instead). Verify exact signatures against your SDK's
// furi_hal_gpio.h if `ufbt build` flags a mismatch - HAL surface does
// drift between firmware versions.

#include <furi.h>
#include <furi_hal.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PS_CAP_RING_SIZE 4096 // edges; see sizing note in the project README

typedef struct {
    const GpioPin* gpio;

    // Ring buffer, written only by the ISR, read only by the drain call.
    volatile int32_t ring[PS_CAP_RING_SIZE];
    volatile uint32_t head; // next write index (ISR-owned)
    volatile uint32_t tail; // next read index (consumer-owned)
    volatile uint32_t overflows;

    // ISR-local state
    volatile uint32_t last_edge_cycles;
    volatile bool last_level_high;
    volatile bool started;

    uint32_t cycles_per_us;
} PsCapture;

// Caller owns the instance (e.g. a PsCapture field on the app struct, or a
// single static in pumpsaver_reader.c) and passes its address to
// ps_capture_start(). Keeping the instance out of this header avoids
// multiple-definition issues if this header is ever included from more
// than one translation unit.

static void ps_capture_dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t ps_capture_now_cycles(void) {
    return DWT->CYCCNT;
}

static void ps_capture_isr(void* context) {
    PsCapture* cap = (PsCapture*)context;

    uint32_t now = ps_capture_now_cycles();
    bool level = furi_hal_gpio_read(cap->gpio);

    if(!cap->started) {
        // First edge just establishes the baseline; nothing to timestamp yet.
        cap->started = true;
        cap->last_edge_cycles = now;
        cap->last_level_high = level;
        return;
    }

    uint32_t delta_cycles = now - cap->last_edge_cycles; // wraps correctly (mod 2^32)
    int32_t width_us = (int32_t)(delta_cycles / cap->cycles_per_us);
    int32_t pulse = cap->last_level_high ? width_us : -width_us;

    uint32_t next_head = (cap->head + 1) % PS_CAP_RING_SIZE;
    if(next_head == cap->tail) {
        cap->overflows++; // consumer isn't keeping up; drop this edge
    } else {
        cap->ring[cap->head] = pulse;
        cap->head = next_head;
    }

    cap->last_edge_cycles = now;
    cap->last_level_high = level;
}

// gpio: e.g. &gpio_ext_pc3 (Flipper header pin 7). Must be a pin exposed by
// furi_hal_resources.h as gpio_ext_*.
static inline void ps_capture_start(PsCapture* cap, const GpioPin* gpio) {
    memset((void*)cap, 0, sizeof(*cap));
    cap->gpio = gpio;
    cap->cycles_per_us = SystemCoreClock / 1000000u;
    if(cap->cycles_per_us == 0) cap->cycles_per_us = 64; // fallback: 64MHz nominal core clock

    ps_capture_dwt_init();

    // Internal pull-up: the phototransistor's collector pulls the pin low
    // when IR is present (same wiring as the ESPHome `inverted: true`
    // config - no external resistor needed).
    furi_hal_gpio_init(cap->gpio, GpioModeInterruptRiseFall, GpioPullUp, GpioSpeedVeryHigh);
    furi_hal_gpio_add_int_callback(cap->gpio, ps_capture_isr, cap);
}

static inline void ps_capture_stop(PsCapture* cap) {
    furi_hal_gpio_remove_int_callback(cap->gpio);
    furi_hal_gpio_init(cap->gpio, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
}

// Drain up to max_out pulses into out[]. Returns count drained. Call this
// from a normal-priority worker thread, not from the ISR.
static inline size_t ps_capture_drain(PsCapture* cap, int32_t* out, size_t max_out) {
    size_t n = 0;
    while(n < max_out) {
        uint32_t tail = cap->tail;
        if(tail == cap->head) break; // empty
        out[n++] = cap->ring[tail];
        cap->tail = (tail + 1) % PS_CAP_RING_SIZE;
    }
    return n;
}

#ifdef __cplusplus
}
#endif
