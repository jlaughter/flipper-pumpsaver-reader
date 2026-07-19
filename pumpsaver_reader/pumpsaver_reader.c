// PumpSaver Reader - Flipper Zero FAP
//
// Reads the baseband IR broadcast of a SymCom/Littelfuse PumpSaver Plus
// pump-protection relay via a bare IR phototransistor wired to a Flipper
// GPIO pin (NOT the built-in IR receiver - that's a 38kHz TSOP demodulator
// and physically cannot see this unmodulated signal).
//
// Six pages, cycled with Left/Right (Back always exits):
//   - AIM: IR Link = OK/Weak/Down, words/sec and errors/sec (mirrors the
//     ESPHome component's `signal_rate`/`decode_errors` - ~38-41 words/s
//     is a clean view of the broadcast), and a live V/A/W line. The V/A/W
//     line blanks rather than showing stale numbers once the link drops.
//   - STATUS: pump idle/running (power vs PUMP_RUNNING_THRESHOLD_W),
//     V/A/W on one line, calibration voltage, dry-well trip point. Also
//     blanks below the pump state when the link is down.
//   - FAULTS: fault sequence number, restart delay setting, restart
//     countdown, and the newest fault's code/conditions/timestamp. Shows
//     "collecting..." until the fault-history ring has been read cleanly
//     twice in a row (see pumpsaver_decode.h's PsFaultRing).
//   - FAULT HISTORY: Up/Down browses all 20 stored fault slots (not just
//     the newest). Unlike the other pages this does NOT blank on link-down
//     - it's already-committed historical data, not live telemetry.
//   - RAW DUMP: OK on this page starts/stops writing every captured
//     signed pulse width (us) to /ext/pumpsaver_raw.txt on the SD card,
//     one per line, in a format directly consumable by the
//     pumpsaver-ir-protocol repo's reference Python decoder for offline
//     cross-validation. Recording continues in the background if you
//     navigate to another page while it's running.
//   - ABOUT: Up/Down browses 3 text-only screens - what the app does,
//     phototransistor wiring, and the external-pullup fallback if the
//     phototransistor alone doesn't produce a clean signal.
//
// Wiring: see the project README. Default pin is Flipper header pin 16
// (PC0) - change PUMPSAVER_GPIO below if you wire it elsewhere.
//
// Status: proof-of-concept confirmed working on real hardware (233P-1.5)
// with a bare IR phototransistor + external 1k pull-up (the Flipper's
// weak internal pull-up alone was too slow to resolve clean edges -
// symptom was two suspiciously uniform "glitch" pulses per burst in the
// raw dump; adding the external pull-up fixed it, same lesson as the
// ESP32 build's phototransistor-speed notes).
//
// Protocol credit: this app only exists because lizbit-official
// reverse-engineered the PumpSaver Plus's entire undocumented IR protocol
// from scratch - https://github.com/lizbit-official/pumpsaver-ir-protocol.
// See pumpsaver_decode.h for the full acknowledgment; that file is a
// near-verbatim port of their work.

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>
#include <storage/storage.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pumpsaver_capture.h"
#include "pumpsaver_decode.h"

#define TAG "PumpSaverReader"

// Change this to whichever header pin you wired the phototransistor to.
// Default is PC0 / header pin 16, matching "channel 0" in the
// flipper-logic-analyzer smoke test in the README so the two tools agree
// on which physical pin is which without a mental remapping step.
// Other usable pins: gpio_ext_pa7 (pin 2), gpio_ext_pa6 (pin 3),
// gpio_ext_pa4 (pin 4), gpio_ext_pb3 (pin 5), gpio_ext_pb2 (pin 6),
// gpio_ext_pc3 (pin 7), gpio_ext_pc1 (pin 15)
#define PUMPSAVER_GPIO (&gpio_ext_pc0)

#define RAW_DUMP_PATH EXT_PATH("pumpsaver_raw.txt")
#define DRAIN_BATCH 256
#define RATE_WINDOW_MS 2000
#define LINK_TIMEOUT_MS 3000 // no valid word within this long -> IR Link = Down
#define PUMP_RUNNING_THRESHOLD_W 200.0f // matches esphome-pumpsaver's example.yaml default (idle ~26W, running ~820W on the 233P-1.5)

typedef enum {
    PageAim,
    PageStatus,
    PageFaults,
    PageFaultHistory,
    PageRawDump,
    PageAbout,
    PageCount, // sentinel - not a real page, used for wraparound math
} AppPage;

typedef enum {
    EventKeyPress,
} EventType;

typedef struct {
    EventType type;
    InputEvent input;
} AppEvent;

typedef struct {
    FuriMessageQueue* event_queue;
    FuriMutex* mutex;
    ViewPort* view_port;
    Gui* gui;
    NotificationApp* notification;
    Storage* storage;
    File* raw_file;

    FuriThread* worker_thread;
    bool running;

    PsCapture capture;
    PsRegTable regs;
    PsFaultRing fault_ring;
    bool fault_baseline_set; // first committed generation is a quiet-boot baseline, not an event
    uint32_t fault_sequence; // increments once per confirmed NEW fault generation after baseline
    int fault_history_index; // 0..PS_FAULT_HISTORY_COUNT-1, browsed with Up/Down on PageFaultHistory
    int about_index; // 0..2, browsed with Up/Down on PageAbout (overview / wiring / no-signal fallback)

    AppPage page;
    bool raw_dump_active; // independent of `page` - recording continues if you navigate away
    uint32_t raw_lines_written;
    uint32_t last_word_ms; // furi_get_tick() at the last successfully decoded word (0 = never)

    // Signal-rate window (words/s, err/s), recomputed every RATE_WINDOW_MS
    uint32_t window_start_ms;
    uint32_t window_words_start;
    uint32_t window_err_start;
    float words_per_s;
    float err_per_s;
} App;

// ---------------------------------------------------------------------------
// Worker thread: drains the capture ring buffer, either decoding on-device
// (LIVE) or writing raw values to the SD card (RAW DUMP).
// ---------------------------------------------------------------------------

static int32_t worker_thread(void* context) {
    App* app = context;
    int32_t buf[DRAIN_BATCH];

    // Per-burst accumulator for the streaming decoder. A word never
    // exceeds ~32 individual level pulses (see PS_MAX_RUN); 48 gives
    // headroom without meaningfully growing stack/heap use.
    int32_t burst[48];
    size_t burst_len = 0;

    char line[24];

    while(app->running) {
        size_t n = ps_capture_drain(&app->capture, buf, DRAIN_BATCH);
        if(n == 0) {
            furi_delay_ms(5);
            continue;
        }

        uint32_t now_ms = furi_get_tick();

        furi_mutex_acquire(app->mutex, FuriWaitForever);

        for(size_t i = 0; i < n; i++) {
            int32_t p = buf[i];

            if(app->raw_dump_active && app->raw_file) {
                int len = snprintf(line, sizeof(line), "%ld\n", (long)p);
                if(len > 0) {
                    storage_file_write(app->raw_file, line, (uint16_t)len);
                    app->raw_lines_written++;
                }
            }

            // Streaming word decode: a pulse wider than the separator
            // threshold IS the idle gap between words (real gaps are
            // 11-16ms; PS_SEPARATOR_US is 8ms). Its sign tells us which
            // level is idle for the burst that just ended - no need to
            // buffer the whole capture like the offline reference does.
            int32_t abs_p = p < 0 ? -p : p;
            if(abs_p > PS_SEPARATOR_US) {
                if(burst_len > 0) {
                    bool idle_positive = p > 0;
                    PsWord w;
                    if(ps_decode_burst(burst, burst_len, idle_positive, &w)) {
                        ps_reg_table_apply(&app->regs, &w, now_ms);
                        app->last_word_ms = now_ms;

                        ps_fault_ring_update_latch(&app->fault_ring, w.reg, w.value);
                        if(ps_fault_ring_update(&app->fault_ring, w.reg, w.value)) {
                            // First commit is a quiet-boot baseline (can't tell an
                            // old stored fault from one during acquisition), so it
                            // doesn't advance the sequence counter - see PROTOCOL.md
                            // / esphome-pumpsaver's fault_sequence semantics.
                            if(!app->fault_baseline_set) {
                                app->fault_baseline_set = true;
                            } else {
                                app->fault_sequence++;
                            }
                        }
                    } else {
                        app->regs.words_err++;
                    }
                }
                burst_len = 0;
            } else {
                if(burst_len < sizeof(burst) / sizeof(burst[0])) {
                    burst[burst_len++] = p;
                } else {
                    burst_len = 0; // malformed/overlong burst - drop and resync
                }
            }
        }

        // Refresh the words/s, err/s window every RATE_WINDOW_MS.
        if(now_ms - app->window_start_ms >= RATE_WINDOW_MS) {
            float elapsed_s = (now_ms - app->window_start_ms) / 1000.0f;
            uint32_t words_delta = app->regs.words_ok - app->window_words_start;
            uint32_t err_delta = app->regs.words_err - app->window_err_start;
            app->words_per_s = elapsed_s > 0 ? words_delta / elapsed_s : 0;
            app->err_per_s = elapsed_s > 0 ? err_delta / elapsed_s : 0;
            app->window_start_ms = now_ms;
            app->window_words_start = app->regs.words_ok;
            app->window_err_start = app->regs.words_err;
        }

        furi_mutex_release(app->mutex);
    }

    return 0;
}

// ---------------------------------------------------------------------------
// GUI
// ---------------------------------------------------------------------------

static float ps_reg_scaled(App* app, uint8_t reg, float scale) {
    if(reg >= PS_REG_TABLE_SIZE || !app->regs.regs[reg].valid) return 0.0f;
    return app->regs.regs[reg].value * scale;
}

// Mirrors the ESPHome component's `link_ok` binary_sensor: true only if a
// valid word arrived within the last LINK_TIMEOUT_MS. Call under the mutex
// (render_callback already holds it for the whole page switch).
static bool ps_link_ok(App* app) {
    if(app->last_word_ms == 0) return false;
    return (furi_get_tick() - app->last_word_ms) < LINK_TIMEOUT_MS;
}

static void draw_page_dots(Canvas* canvas, AppPage page) {
    // Small dots top-right, filled dot marks the current page - cheap way
    // to show "there's more, swipe Left/Right" at a glance. Right-aligned
    // and spaced from PageCount so it doesn't need re-tuning if another
    // page gets added later.
    const int spacing = 4;
    const int start_x = 124 - (PageCount - 1) * spacing;
    for(int i = 0; i < PageCount; i++) {
        int x = start_x + i * spacing;
        if(i == (int)page) {
            canvas_draw_disc(canvas, x, 4, 2);
        } else {
            canvas_draw_circle(canvas, x, 4, 2);
        }
    }
}

static void render_aim_page(Canvas* canvas, App* app) {
    char buf[48];
    bool link_ok = ps_link_ok(app);

    canvas_set_font(canvas, FontSecondary);

    const char* link_status;
    if(!link_ok) {
        link_status = "Down";
    } else if(app->words_per_s > 32.0f) {
        link_status = "OK";
    } else {
        link_status = "Weak";
    }
    snprintf(buf, sizeof(buf), "IR Link = %s", link_status);
    canvas_draw_str(canvas, 2, 20, buf);

    snprintf(buf, sizeof(buf), "%.1f words/sec (38-41=good)", (double)app->words_per_s);
    canvas_draw_str(canvas, 2, 30, buf);

    snprintf(buf, sizeof(buf), "%.1f errors/sec (0=good)", (double)app->err_per_s);
    canvas_draw_str(canvas, 2, 40, buf);

    // Blanked (nothing drawn) rather than showing stale readings once the
    // link has dropped - "IR Link = Down" above already says why.
    if(link_ok) {
        snprintf(
            buf,
            sizeof(buf),
            "Data: V=%.1f A=%.1f W=%.0f",
            (double)ps_reg_scaled(app, PS_REG_VOLTAGE, 0.1f),
            (double)ps_reg_scaled(app, PS_REG_CURRENT, 0.01f),
            (double)ps_reg_scaled(app, PS_REG_POWER, 1.0f));
        canvas_draw_str(canvas, 2, 50, buf);
    }
}

// Register 0x10 (true watts) vs PUMP_RUNNING_THRESHOLD_W is the same
// approach ESPHome's `running` binary_sensor uses (a configurable
// threshold there; a #define here - tune it once you've seen your pump's
// actual idle-vs-running wattage). PROTOCOL.md notes idle is ~26-29W and a
// real run is in the hundreds, so 100W has good margin either way.
static void render_status_page(Canvas* canvas, App* app) {
    char buf[48];
    bool link_ok = ps_link_ok(app);

    canvas_set_font(canvas, FontSecondary);

    if(!link_ok) {
        canvas_draw_str(canvas, 2, 20, "Pump: --");
        return; // rest of the page blanked - see render_aim_page for the same rule
    }

    float watts = ps_reg_scaled(app, PS_REG_POWER, 1.0f);
    canvas_draw_str(canvas, 2, 20, watts >= PUMP_RUNNING_THRESHOLD_W ? "Pump: RUNNING" : "Pump: IDLE");

    snprintf(
        buf,
        sizeof(buf),
        "V=%.1f A=%.2f W=%.0f",
        (double)ps_reg_scaled(app, PS_REG_VOLTAGE, 0.1f),
        (double)ps_reg_scaled(app, PS_REG_CURRENT, 0.01f),
        (double)watts);
    canvas_draw_str(canvas, 2, 30, buf);

    snprintf(buf, sizeof(buf), "Cal V: %.1f", (double)ps_reg_scaled(app, PS_REG_CAL_VOLTAGE, 0.1f));
    canvas_draw_str(canvas, 2, 40, buf);

    snprintf(buf, sizeof(buf), "Dry Well Trip: %.0f W", (double)ps_reg_scaled(app, PS_REG_DRYWELL_TRIP, 1.0f));
    canvas_draw_str(canvas, 2, 50, buf);
}

static void render_faults_page(Canvas* canvas, App* app) {
    char buf[48];
    char ts[24];

    canvas_set_font(canvas, FontSecondary);

    if(!ps_fault_ring_ready(&app->fault_ring)) {
        canvas_draw_str(canvas, 2, 30, "collecting fault");
        canvas_draw_str(canvas, 2, 40, "history... (~12s)");
        return;
    }

    snprintf(
        buf,
        sizeof(buf),
        "Seq #%lu  RstDly %um",
        (unsigned long)app->fault_sequence,
        app->regs.regs[PS_REG_RESTART_SET].valid ? app->regs.regs[PS_REG_RESTART_SET].value : 0);
    canvas_draw_str(canvas, 2, 20, buf);

    uint32_t restart_hi = app->regs.regs[PS_REG_RESTART_HI].valid ? app->regs.regs[PS_REG_RESTART_HI].value : 0;
    uint32_t restart_lo = app->regs.regs[PS_REG_RESTART_LO].valid ? app->regs.regs[PS_REG_RESTART_LO].value : 0;
    uint32_t restart_remaining_s = ((restart_hi << 16) | restart_lo) / 256;
    if(restart_remaining_s == 0) {
        canvas_draw_str(canvas, 2, 30, "Countdown: none");
    } else {
        snprintf(buf, sizeof(buf), "Countdown: %lus", (unsigned long)restart_remaining_s);
        canvas_draw_str(canvas, 2, 30, buf);
    }

    PsFaultInfo newest;
    ps_fault_ring_get_slot(&app->fault_ring, 0, &newest);

    canvas_draw_str(canvas, 2, 40, ps_fault_code_name(newest.code));

    ps_format_run_clock(newest.at_minutes, ts, sizeof(ts));
    snprintf(
        buf,
        sizeof(buf),
        "%uW %.1fV %.2fA @%s",
        newest.watts,
        (double)(newest.volts_x10 / 10.0f),
        (double)(newest.amps_x100 / 100.0f),
        ts);
    canvas_draw_str(canvas, 2, 50, buf);
}

static void render_fault_history_page(Canvas* canvas, App* app) {
    char buf[48];
    char ts[24];

    canvas_set_font(canvas, FontSecondary);

    if(!ps_fault_ring_ready(&app->fault_ring)) {
        canvas_draw_str(canvas, 2, 30, "collecting fault");
        canvas_draw_str(canvas, 2, 40, "history... (~12s)");
        return;
    }

    // Not gated on link_ok: this is already-committed historical data, not
    // live telemetry, so it stays valid to show even right after a signal
    // drop (unlike the Aim/Status/Faults pages' live readings).
    PsFaultInfo f;
    ps_fault_ring_get_slot(&app->fault_ring, app->fault_history_index, &f);

    snprintf(
        buf,
        sizeof(buf),
        "Fault #%d of %d (U/D)",
        app->fault_history_index + 1,
        PS_FAULT_HISTORY_COUNT);
    canvas_draw_str(canvas, 2, 20, buf);

    canvas_draw_str(canvas, 2, 30, ps_fault_code_name(f.code));

    snprintf(
        buf,
        sizeof(buf),
        "%uW %.1fV %.2fA",
        f.watts,
        (double)(f.volts_x10 / 10.0f),
        (double)(f.amps_x100 / 100.0f));
    canvas_draw_str(canvas, 2, 40, buf);

    ps_format_run_clock(f.at_minutes, ts, sizeof(ts));
    canvas_draw_str(canvas, 2, 50, ts);
}

static void render_raw_dump_page(Canvas* canvas, App* app) {
    char buf[48];

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 22, app->raw_dump_active ? "RECORDING -> SD card" : "OK to start recording");
    canvas_draw_str(canvas, 2, 32, "/ext/pumpsaver_raw.txt");
    snprintf(buf, sizeof(buf), "lines: %lu", (unsigned long)app->raw_lines_written);
    canvas_draw_str(canvas, 2, 44, buf);
    canvas_draw_str(canvas, 2, 54, app->raw_dump_active ? "OK: stop" : "OK: start");
}

// Text-only reference screens, no wiring diagram - Up/Down cycles the 3
// states. Kept here rather than only in the README so the pin numbers are
// in front of you at install time, not back on a laptop.
static void render_about_page(Canvas* canvas, App* app) {
    canvas_set_font(canvas, FontSecondary);

    switch(app->about_index) {
    case 0:
        canvas_draw_str(canvas, 2, 20, "About (1/3): Overview");
        canvas_draw_str(canvas, 2, 30, "Reads PumpSaver Plus");
        canvas_draw_str(canvas, 2, 40, "baseband IR telemetry +");
        canvas_draw_str(canvas, 2, 50, "fault history, live.");
        break;
    case 1:
        canvas_draw_str(canvas, 2, 20, "About (2/3): Wiring");
        canvas_draw_str(canvas, 2, 30, "Phototransistor between:");
        canvas_draw_str(canvas, 2, 40, "pin 16 (C0, collector)");
        canvas_draw_str(canvas, 2, 50, "pin 18 (GND, emitter)");
        break;
    case 2:
        canvas_draw_str(canvas, 2, 20, "About (3/3): No signal?");
        canvas_draw_str(canvas, 2, 30, "Add 1k pullup between:");
        canvas_draw_str(canvas, 2, 40, "pin 16 (C0)");
        canvas_draw_str(canvas, 2, 50, "pin 9 (3V3)");
        break;
    }
}

static void render_callback(Canvas* canvas, void* ctx) {
    App* app = ctx;
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 10, "PumpSaver Reader");
    draw_page_dots(canvas, app->page);

    switch(app->page) {
    case PageAim:
        render_aim_page(canvas, app);
        break;
    case PageStatus:
        render_status_page(canvas, app);
        break;
    case PageFaults:
        render_faults_page(canvas, app);
        break;
    case PageFaultHistory:
        render_fault_history_page(canvas, app);
        break;
    case PageRawDump:
        render_raw_dump_page(canvas, app);
        break;
    case PageAbout:
        render_about_page(canvas, app);
        break;
    default:
        break;
    }

    canvas_set_font(canvas, FontSecondary);
    if(app->page == PageFaultHistory || app->page == PageAbout) {
        // This page also scrolls (Up/Down), so the shared "L/R: page   BACK:
        // exit" line doesn't say enough. Plain text, not icons - the system
        // nav-hint glyphs (I_ButtonLeft_4x7 etc.) compile fine because
        // assets_icons.h is generated locally, but they're not on the
        // firmware's exported FAP API symbol table, so they fail to link
        // on-device ("Missing Imports") even though the build succeeds.
        canvas_draw_str(canvas, 2, 63, "L/R Page  U/D Scroll  BACK");
    } else {
        canvas_draw_str(canvas, 2, 63, "L/R: page   BACK: exit");
    }

    furi_mutex_release(app->mutex);
}

static void input_callback(InputEvent* input_event, void* ctx) {
    FuriMessageQueue* queue = ctx;
    AppEvent event = {.type = EventKeyPress, .input = *input_event};
    furi_message_queue_put(queue, &event, FuriWaitForever);
}

static void open_raw_file(App* app) {
    app->raw_file = storage_file_alloc(app->storage);
    if(!storage_file_open(app->raw_file, RAW_DUMP_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        FURI_LOG_E(TAG, "failed to open %s", RAW_DUMP_PATH);
        storage_file_free(app->raw_file);
        app->raw_file = NULL;
    }
    app->raw_lines_written = 0;
}

static void close_raw_file(App* app) {
    if(app->raw_file) {
        storage_file_close(app->raw_file);
        storage_file_free(app->raw_file);
        app->raw_file = NULL;
    }
}

// ---------------------------------------------------------------------------
// App lifecycle
// ---------------------------------------------------------------------------

int32_t pumpsaver_reader_app_main(void* p) {
    UNUSED(p);

    App* app = malloc(sizeof(App));
    memset(app, 0, sizeof(App));

    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->event_queue = furi_message_queue_alloc(8, sizeof(AppEvent));
    app->notification = furi_record_open(RECORD_NOTIFICATION);
    app->gui = furi_record_open(RECORD_GUI);
    app->storage = furi_record_open(RECORD_STORAGE);

    ps_reg_table_init(&app->regs);
    ps_fault_ring_init(&app->fault_ring);
    app->page = PageAim;
    app->window_start_ms = furi_get_tick();

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, render_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app->event_queue);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    notification_message(app->notification, &sequence_display_backlight_enforce_on);

    ps_capture_start(&app->capture, PUMPSAVER_GPIO);

    app->running = true;
    app->worker_thread = furi_thread_alloc_ex("pumpsaver_worker", 2048, worker_thread, app);
    furi_thread_start(app->worker_thread);

    bool exit_app = false;
    while(!exit_app) {
        AppEvent event;
        FuriStatus status = furi_message_queue_get(app->event_queue, &event, 100);
        if(status == FuriStatusOk && event.type == EventKeyPress &&
           event.input.type == InputTypePress) {
            switch(event.input.key) {
            case InputKeyLeft:
                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->page = (AppPage)((app->page + PageCount - 1) % PageCount);
                furi_mutex_release(app->mutex);
                break;
            case InputKeyRight:
                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->page = (AppPage)((app->page + 1) % PageCount);
                furi_mutex_release(app->mutex);
                break;
            case InputKeyUp:
                // Meaningful on the two browsable pages: Fault History
                // (20 slots) and About (3 text screens).
                if(app->page == PageFaultHistory) {
                    furi_mutex_acquire(app->mutex, FuriWaitForever);
                    if(app->fault_history_index > 0) app->fault_history_index--;
                    furi_mutex_release(app->mutex);
                } else if(app->page == PageAbout) {
                    furi_mutex_acquire(app->mutex, FuriWaitForever);
                    if(app->about_index > 0) app->about_index--;
                    furi_mutex_release(app->mutex);
                }
                break;
            case InputKeyDown:
                if(app->page == PageFaultHistory) {
                    furi_mutex_acquire(app->mutex, FuriWaitForever);
                    if(app->fault_history_index < PS_FAULT_HISTORY_COUNT - 1) app->fault_history_index++;
                    furi_mutex_release(app->mutex);
                } else if(app->page == PageAbout) {
                    furi_mutex_acquire(app->mutex, FuriWaitForever);
                    if(app->about_index < 2) app->about_index++;
                    furi_mutex_release(app->mutex);
                }
                break;
            case InputKeyOk:
                // Only meaningful on the Raw Dump page; recording, once
                // started, keeps running even if you navigate elsewhere.
                if(app->page == PageRawDump) {
                    furi_mutex_acquire(app->mutex, FuriWaitForever);
                    if(!app->raw_dump_active) {
                        open_raw_file(app);
                        app->raw_dump_active = (app->raw_file != NULL);
                    } else {
                        app->raw_dump_active = false;
                        close_raw_file(app);
                    }
                    furi_mutex_release(app->mutex);
                }
                break;
            case InputKeyBack:
                exit_app = true;
                break;
            default:
                break;
            }
        }
        view_port_update(app->view_port);
    }

    app->running = false;
    furi_thread_join(app->worker_thread);
    furi_thread_free(app->worker_thread);

    close_raw_file(app);
    ps_capture_stop(&app->capture);

    notification_message(app->notification, &sequence_display_backlight_enforce_auto);

    view_port_enabled_set(app->view_port, false);
    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_message_queue_free(app->event_queue);
    furi_mutex_free(app->mutex);

    furi_record_close(RECORD_STORAGE);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);

    free(app);
    return 0;
}
