#include "PtpIpDiag.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <stdio.h>

// ============================================================================
// Full implementation — Arduino builds only
// ============================================================================

#ifndef PTPIP_NO_DIAG

// Static fallback buffer — used when PSRAM is unavailable
static DiagEntry s_fallback[PtpIpDiag::CAPACITY];

PtpIpDiag::PtpIpDiag() : _buf(nullptr), _head(0), _count(0), _psram(false) {
#ifdef BOARD_HAS_PSRAM
    _buf = (DiagEntry*)ps_malloc(CAPACITY * sizeof(DiagEntry));
    if (_buf) { _psram = true; return; }
#endif
    _buf = s_fallback;
}

void PtpIpDiag::record(uint8_t type, uint8_t d0, uint8_t d1, uint8_t d2) {
    DiagEntry& e = _buf[_head];
    e.ts   = (uint16_t)(millis() / 1000);
    e.type = type;
    e.d0   = d0;
    e.d1   = d1;
    e.d2   = d2;
    _head  = (_head + 1) % CAPACITY;
    if (_count < CAPACITY) _count++;
}

static const char* _diagTypeName(uint8_t type) {
    switch (type) {
        case DIAG_EVT_OP_SENT:     return "OP_SENT     ";
        case DIAG_EVT_RESP_RECV:   return "RESP_RECV   ";
        case DIAG_EVT_PROP_CHANGED:return "PROP_CHANGED";
        case DIAG_EVT_MODE_CHANGED:return "MODE_CHANGED";
        case DIAG_EVT_CAPTURE:     return "CAPTURE     ";
        case DIAG_EVT_ERROR:       return "ERROR       ";
        default:                   return "UNKNOWN     ";
    }
}

void PtpIpDiag::dump(Print& out) {
    if (_count == 0) {
        out.println("[DIAG] No entries recorded.");
        return;
    }
    char buf[80];
    snprintf(buf, sizeof(buf), "[DIAG] %u entries:", (unsigned)_count);
    out.println(buf);

    // Oldest entry is at (_head - _count) wrapped around CAPACITY
    size_t   start = (_head + CAPACITY - _count) % CAPACITY;
    uint16_t t0    = _buf[start].ts;

    for (size_t i = 0; i < _count; i++) {
        const DiagEntry& e  = _buf[(start + i) % CAPACITY];
        uint16_t         dt = (uint16_t)(e.ts - t0);  // wrapping delta in seconds
        snprintf(buf, sizeof(buf), "  [%3u] +%5us  %s  %02X %02X %02X",
                 (unsigned)i, (unsigned)dt,
                 _diagTypeName(e.type),
                 e.d0, e.d1, e.d2);
        out.println(buf);
    }
}

void PtpIpDiag::clear() {
    _head  = 0;
    _count = 0;
}

#else  // PTPIP_NO_DIAG

void PtpIpDiag::dump(Print& out) {
    out.println("[DIAG] Diagnostics disabled. Rebuild without -D PTPIP_NO_DIAG to enable.");
}

#endif  // PTPIP_NO_DIAG

// Singleton accessor
PtpIpDiag& PtpIpDiagInstance() {
    static PtpIpDiag instance;
    return instance;
}

#endif  // ARDUINO
