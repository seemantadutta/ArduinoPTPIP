#pragma once
#include <stdint.h>
#include <stddef.h>

// Forward declaration — implementations include <Arduino.h> which defines Print
class Print;

// ============================================================================
// PtpIpDiag — in-memory ring buffer for field diagnostics
//
// Always recording (unless -D PTPIP_NO_DIAG is defined in build_flags).
// Independent of the log callback — records events even when no callback
// is registered.
//
// Dump the buffer any time via camera.dumpDiagnostics(Serial), or directly:
//   PtpIpDiagInstance().dump(Serial);
//
// Disable entirely (zero overhead) with:  -D PTPIP_NO_DIAG
// ============================================================================

// Event type constants (stored in DiagEntry::type)
// Payload encoding per type — see DiagEntry below
static constexpr uint8_t DIAG_EVT_OP_SENT      = 0; // d0:d1=opcode,     d2=numParams
static constexpr uint8_t DIAG_EVT_RESP_RECV     = 1; // d0:d1=responseCode, d2=0
static constexpr uint8_t DIAG_EVT_PROP_CHANGED  = 2; // d0:d1=propCode,   d2=wireValue
static constexpr uint8_t DIAG_EVT_MODE_CHANGED  = 3; // d0=modeWire,      d1=d2=0
static constexpr uint8_t DIAG_EVT_CAPTURE       = 4; // d0=capturePhase,  d1=d2=0
static constexpr uint8_t DIAG_EVT_ERROR         = 5; // d0=CameraResult,  d1=d2=0

#ifndef PTPIP_NO_DIAG

struct DiagEntry {
    uint16_t ts;         // seconds since boot (millis()/1000 & 0xFFFF); wraps ~18h
    uint8_t  type;       // DIAG_EVT_* constant
    uint8_t  d0, d1, d2; // payload — encoding depends on type (see constants above)
};

class PtpIpDiag {
public:
    static constexpr size_t CAPACITY = 512;

    PtpIpDiag();

    // Record one event. Always safe; silently overwrites oldest on full buffer.
    void record(uint8_t type, uint8_t d0, uint8_t d1, uint8_t d2);

    // Dump all recorded entries to out in chronological order (oldest first).
    void dump(Print& out);

    // Clear the buffer.
    void clear();

private:
    DiagEntry* _buf;
    size_t     _head;   // index of next write slot
    size_t     _count;  // number of valid entries (0..CAPACITY)
    bool       _psram;  // true if _buf was ps_malloc'd from PSRAM
};

#else  // PTPIP_NO_DIAG

// No-op stub — zero overhead when PTPIP_NO_DIAG is defined
class PtpIpDiag {
public:
    void record(uint8_t, uint8_t, uint8_t, uint8_t) {}
    void dump(Print& out);  // prints one explanatory line
    void clear() {}
};

#endif  // PTPIP_NO_DIAG

// Global singleton — shared across CanonCamera, SimCamera, and PtpIpSession.
// Only available on Arduino builds; native unit tests never reference it.
#ifdef ARDUINO
PtpIpDiag& PtpIpDiagInstance();
#endif
