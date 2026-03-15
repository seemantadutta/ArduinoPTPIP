#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

// ============================================================================
// CanonExposure — Canon EOS APEX exposure encoding/decoding
//
// Public header. Contains the full Canon APEX wire value tables and all
// encode/decode functions as inline implementations. No Arduino dependency —
// safe to include in host-side unit tests.
//
// Wire values are single bytes derived from libgphoto2 (Canon EOS driver).
// They do NOT follow strict APEX mathematics — use these tables exclusively
// for all encode/decode operations; do not attempt to derive values by formula.
//
// Table layout:
//   APERTURE: index 0 = widest (f/1.2, most light)  → last = narrowest (f/32)
//   SHUTTER:  index 0 = longest (30s, most light)    → last = shortest (1/8000)
//   ISO:      index 0 = lowest  (ISO 50, least noise) → last = highest (ISO 3200)
//
// EV direction (brighter = +EV):
//   Aperture : +EV → LOWER index (wider aperture = more light)
//   Shutter  : +EV → LOWER index (longer exposure = more light)
//   ISO      : +EV → HIGHER index (higher ISO = more light)
// ============================================================================

// ----------------------------------------------------------------------------
// Raw APEX wire value enums
// (previously in CanonEnums.h — consolidated here)
// ----------------------------------------------------------------------------

typedef enum {
    ISO_50    = 0x40, ISO_100   = 0x48, ISO_125   = 0x4b, ISO_160   = 0x4d,
    ISO_200   = 0x50, ISO_250   = 0x53, ISO_320   = 0x55, ISO_400   = 0x58,
    ISO_500   = 0x5b, ISO_640   = 0x5d, ISO_800   = 0x60, ISO_1000  = 0x63,
    ISO_1250  = 0x65, ISO_1600  = 0x68, ISO_2000  = 0x6b, ISO_2500  = 0x6d,
    ISO_3200  = 0x70, ISO_4000  = 0x73, ISO_5000  = 0x75, ISO_6400  = 0x78,
    ISO_8000  = 0x7b, ISO_10000 = 0x7d, ISO_12800 = 0x80,
    ISO_16000 = 0x83, ISO_20000 = 0x85, ISO_25600 = 0x88, ISO_32000 = 0x8b
} canonIsoState;

typedef enum {
    APERTURE_F1_2 = 0x0d, APERTURE_F1_4 = 0x10, APERTURE_F1_6 = 0x13,
    APERTURE_F1_8 = 0x15, APERTURE_F2_0 = 0x18, APERTURE_F2_2 = 0x1b,
    APERTURE_F2_5 = 0x1d, APERTURE_F2_8 = 0x20, APERTURE_F3_2 = 0x23,
    APERTURE_F3_5 = 0x25, APERTURE_F4_0 = 0x28, APERTURE_F4_5 = 0x2b,
    APERTURE_F5_0 = 0x2d, APERTURE_F5_6 = 0x30, APERTURE_F6_3 = 0x33,
    APERTURE_F7_1 = 0x35, APERTURE_F8   = 0x38, APERTURE_F9   = 0x3b,
    APERTURE_F10  = 0x3d, APERTURE_F11  = 0x40, APERTURE_F13  = 0x43,
    APERTURE_F14  = 0x45, APERTURE_F16  = 0x48, APERTURE_F18  = 0x4b,
    APERTURE_F20  = 0x4d, APERTURE_F22  = 0x50, APERTURE_F25  = 0x53,
    APERTURE_F29  = 0x55, APERTURE_F32  = 0x58
} canonApertureState;

typedef enum {
    SHUTTER_SPEED_BULB    = 0x04,
    SHUTTER_SPEED_AUTO    = 0x0C,  // camera-controlled (P/Av auto shutter)
    SHUTTER_SPEED_30_SEC  = 0x10, SHUTTER_SPEED_25_SEC  = 0x13,
    SHUTTER_SPEED_20_SEC  = 0x15, SHUTTER_SPEED_15_SEC  = 0x18,
    SHUTTER_SPEED_13_SEC  = 0x1b, SHUTTER_SPEED_10_SEC  = 0x1d,
    SHUTTER_SPEED_8_SEC   = 0x20, SHUTTER_SPEED_6_SEC   = 0x23,
    SHUTTER_SPEED_5_SEC   = 0x25, SHUTTER_SPEED_4_SEC   = 0x28,
    SHUTTER_SPEED_3_2_SEC = 0x2b, SHUTTER_SPEED_2_5_SEC = 0x2d,
    SHUTTER_SPEED_2_SEC   = 0x30, SHUTTER_SPEED_1_6_SEC = 0x33,
    SHUTTER_SPEED_1_3_SEC = 0x35, SHUTTER_SPEED_1_SEC   = 0x38,
    SHUTTER_SPEED_0_8_SEC = 0x3b, SHUTTER_SPEED_0_6_SEC = 0x3d,
    SHUTTER_SPEED_0_5_SEC = 0x40, SHUTTER_SPEED_0_4_SEC = 0x43,
    SHUTTER_SPEED_0_3_SEC = 0x45,
    SHUTTER_SPEED_1_4     = 0x48, SHUTTER_SPEED_1_5     = 0x4b,
    SHUTTER_SPEED_1_6     = 0x4d, SHUTTER_SPEED_1_8     = 0x50,
    SHUTTER_SPEED_1_10    = 0x53, SHUTTER_SPEED_1_13    = 0x55,
    SHUTTER_SPEED_1_15    = 0x58, SHUTTER_SPEED_1_20    = 0x5b,
    SHUTTER_SPEED_1_25    = 0x5d, SHUTTER_SPEED_1_30    = 0x60,
    SHUTTER_SPEED_1_40    = 0x63, SHUTTER_SPEED_1_50    = 0x65,
    SHUTTER_SPEED_1_60    = 0x68, SHUTTER_SPEED_1_80    = 0x6b,
    SHUTTER_SPEED_1_100   = 0x6d, SHUTTER_SPEED_1_125   = 0x70,
    SHUTTER_SPEED_1_160   = 0x73, SHUTTER_SPEED_1_200   = 0x75,
    SHUTTER_SPEED_1_250   = 0x78, SHUTTER_SPEED_1_320   = 0x7b,
    SHUTTER_SPEED_1_400   = 0x7d, SHUTTER_SPEED_1_500   = 0x80,
    SHUTTER_SPEED_1_640   = 0x83, SHUTTER_SPEED_1_800   = 0x85,
    SHUTTER_SPEED_1_1000  = 0x88, SHUTTER_SPEED_1_1250  = 0x8b,
    SHUTTER_SPEED_1_1600  = 0x8d, SHUTTER_SPEED_1_2000  = 0x90,
    SHUTTER_SPEED_1_2500  = 0x93, SHUTTER_SPEED_1_3200  = 0x95,
    SHUTTER_SPEED_1_4000  = 0x98, SHUTTER_SPEED_1_5000  = 0x9a,
    SHUTTER_SPEED_1_6400  = 0x9d, SHUTTER_SPEED_1_8000  = 0xA0
} canonShutterSpeedState;

// ----------------------------------------------------------------------------
// Lookup table types
// ----------------------------------------------------------------------------

struct CanonApertureEntry { float fstop;   uint8_t wire; };
struct CanonShutterEntry  { float seconds; uint8_t wire; };
struct CanonISOEntry      { uint16_t iso;  uint8_t wire; };

// ----------------------------------------------------------------------------
// Tables — static const so each translation unit gets its own copy.
// Tables are small; linker will typically deduplicate in practice.
// ----------------------------------------------------------------------------

static const CanonApertureEntry CANON_APERTURE_TABLE[] = {
    { 1.2f,  APERTURE_F1_2 }, { 1.4f,  APERTURE_F1_4 }, { 1.6f,  APERTURE_F1_6 },
    { 1.8f,  APERTURE_F1_8 }, { 2.0f,  APERTURE_F2_0 }, { 2.2f,  APERTURE_F2_2 },
    { 2.5f,  APERTURE_F2_5 }, { 2.8f,  APERTURE_F2_8 }, { 3.2f,  APERTURE_F3_2 },
    { 3.5f,  APERTURE_F3_5 }, { 4.0f,  APERTURE_F4_0 }, { 4.5f,  APERTURE_F4_5 },
    { 5.0f,  APERTURE_F5_0 }, { 5.6f,  APERTURE_F5_6 }, { 6.3f,  APERTURE_F6_3 },
    { 7.1f,  APERTURE_F7_1 }, { 8.0f,  APERTURE_F8   }, { 9.0f,  APERTURE_F9   },
    { 10.0f, APERTURE_F10  }, { 11.0f, APERTURE_F11  }, { 13.0f, APERTURE_F13  },
    { 14.0f, APERTURE_F14  }, { 16.0f, APERTURE_F16  }, { 18.0f, APERTURE_F18  },
    { 20.0f, APERTURE_F20  }, { 22.0f, APERTURE_F22  }, { 25.0f, APERTURE_F25  },
    { 29.0f, APERTURE_F29  }, { 32.0f, APERTURE_F32  },
};

static const CanonShutterEntry CANON_SHUTTER_TABLE[] = {
    { 30.0f,      SHUTTER_SPEED_30_SEC  }, { 25.0f,      SHUTTER_SPEED_25_SEC  },
    { 20.0f,      SHUTTER_SPEED_20_SEC  }, { 15.0f,      SHUTTER_SPEED_15_SEC  },
    { 13.0f,      SHUTTER_SPEED_13_SEC  }, { 10.0f,      SHUTTER_SPEED_10_SEC  },
    { 8.0f,       SHUTTER_SPEED_8_SEC   }, { 6.0f,       SHUTTER_SPEED_6_SEC   },
    { 5.0f,       SHUTTER_SPEED_5_SEC   }, { 4.0f,       SHUTTER_SPEED_4_SEC   },
    { 3.2f,       SHUTTER_SPEED_3_2_SEC }, { 2.5f,       SHUTTER_SPEED_2_5_SEC },
    { 2.0f,       SHUTTER_SPEED_2_SEC   }, { 1.6f,       SHUTTER_SPEED_1_6_SEC },
    { 1.3f,       SHUTTER_SPEED_1_3_SEC }, { 1.0f,       SHUTTER_SPEED_1_SEC   },
    { 0.8f,       SHUTTER_SPEED_0_8_SEC }, { 0.6f,       SHUTTER_SPEED_0_6_SEC },
    { 0.5f,       SHUTTER_SPEED_0_5_SEC }, { 0.4f,       SHUTTER_SPEED_0_4_SEC },
    { 0.3f,       SHUTTER_SPEED_0_3_SEC }, { 1.0f/4,     SHUTTER_SPEED_1_4     },
    { 1.0f/5,     SHUTTER_SPEED_1_5     }, { 1.0f/6,     SHUTTER_SPEED_1_6     },
    { 1.0f/8,     SHUTTER_SPEED_1_8     }, { 1.0f/10,    SHUTTER_SPEED_1_10    },
    { 1.0f/13,    SHUTTER_SPEED_1_13    }, { 1.0f/15,    SHUTTER_SPEED_1_15    },
    { 1.0f/20,    SHUTTER_SPEED_1_20    }, { 1.0f/25,    SHUTTER_SPEED_1_25    },
    { 1.0f/30,    SHUTTER_SPEED_1_30    }, { 1.0f/40,    SHUTTER_SPEED_1_40    },
    { 1.0f/50,    SHUTTER_SPEED_1_50    }, { 1.0f/60,    SHUTTER_SPEED_1_60    },
    { 1.0f/80,    SHUTTER_SPEED_1_80    }, { 1.0f/100,   SHUTTER_SPEED_1_100   },
    { 1.0f/125,   SHUTTER_SPEED_1_125   }, { 1.0f/160,   SHUTTER_SPEED_1_160   },
    { 1.0f/200,   SHUTTER_SPEED_1_200   }, { 1.0f/250,   SHUTTER_SPEED_1_250   },
    { 1.0f/320,   SHUTTER_SPEED_1_320   }, { 1.0f/400,   SHUTTER_SPEED_1_400   },
    { 1.0f/500,   SHUTTER_SPEED_1_500   }, { 1.0f/640,   SHUTTER_SPEED_1_640   },
    { 1.0f/800,   SHUTTER_SPEED_1_800   }, { 1.0f/1000,  SHUTTER_SPEED_1_1000  },
    { 1.0f/1250,  SHUTTER_SPEED_1_1250  }, { 1.0f/1600,  SHUTTER_SPEED_1_1600  },
    { 1.0f/2000,  SHUTTER_SPEED_1_2000  }, { 1.0f/2500,  SHUTTER_SPEED_1_2500  },
    { 1.0f/3200,  SHUTTER_SPEED_1_3200  }, { 1.0f/4000,  SHUTTER_SPEED_1_4000  },
    { 1.0f/5000,  SHUTTER_SPEED_1_5000  }, { 1.0f/6400,  SHUTTER_SPEED_1_6400  },
    { 1.0f/8000,  SHUTTER_SPEED_1_8000  },
};

static const CanonISOEntry CANON_ISO_TABLE[] = {
    { 50,    ISO_50    }, { 100,   ISO_100   }, { 125,   ISO_125   }, { 160,   ISO_160   },
    { 200,   ISO_200   }, { 250,   ISO_250   }, { 320,   ISO_320   }, { 400,   ISO_400   },
    { 500,   ISO_500   }, { 640,   ISO_640   }, { 800,   ISO_800   }, { 1000,  ISO_1000  },
    { 1250,  ISO_1250  }, { 1600,  ISO_1600  }, { 2000,  ISO_2000  }, { 2500,  ISO_2500  },
    { 3200,  ISO_3200  }, { 4000,  ISO_4000  }, { 5000,  ISO_5000  }, { 6400,  ISO_6400  },
    { 8000,  ISO_8000  }, { 10000, ISO_10000 }, { 12800, ISO_12800 },
    { 16000, ISO_16000 }, { 20000, ISO_20000 }, { 25600, ISO_25600 }, { 32000, ISO_32000 },
};

static constexpr size_t CANON_APERTURE_TABLE_SIZE = sizeof(CANON_APERTURE_TABLE) / sizeof(CANON_APERTURE_TABLE[0]);
static constexpr size_t CANON_SHUTTER_TABLE_SIZE  = sizeof(CANON_SHUTTER_TABLE)  / sizeof(CANON_SHUTTER_TABLE[0]);
static constexpr size_t CANON_ISO_TABLE_SIZE      = sizeof(CANON_ISO_TABLE)      / sizeof(CANON_ISO_TABLE[0]);

// ----------------------------------------------------------------------------
// Encode: human-readable value → Canon APEX wire byte
// Uses nearest-match lookup. fstop by absolute difference, seconds by ratio,
// ISO by absolute difference.
// ----------------------------------------------------------------------------

inline uint8_t canonApertureToWire(float fstop) {
    const CanonApertureEntry* best = &CANON_APERTURE_TABLE[0];
    float bestDiff = fabsf(fstop - best->fstop);
    for (size_t i = 1; i < CANON_APERTURE_TABLE_SIZE; i++) {
        float diff = fabsf(fstop - CANON_APERTURE_TABLE[i].fstop);
        if (diff < bestDiff) { bestDiff = diff; best = &CANON_APERTURE_TABLE[i]; }
    }
    return best->wire;
}

inline uint8_t canonShutterToWire(float seconds) {
    const CanonShutterEntry* best = &CANON_SHUTTER_TABLE[0];
    float bestRatio = (seconds > best->seconds) ? (seconds / best->seconds)
                                                 : (best->seconds / seconds);
    for (size_t i = 1; i < CANON_SHUTTER_TABLE_SIZE; i++) {
        float s = CANON_SHUTTER_TABLE[i].seconds;
        float ratio = (seconds > s) ? (seconds / s) : (s / seconds);
        if (ratio < bestRatio) { bestRatio = ratio; best = &CANON_SHUTTER_TABLE[i]; }
    }
    return best->wire;
}

inline uint8_t canonISOToWire(uint16_t iso) {
    const CanonISOEntry* best = &CANON_ISO_TABLE[0];
    int bestDiff = abs((int)iso - (int)best->iso);
    for (size_t i = 1; i < CANON_ISO_TABLE_SIZE; i++) {
        int diff = abs((int)iso - (int)CANON_ISO_TABLE[i].iso);
        if (diff < bestDiff) { bestDiff = diff; best = &CANON_ISO_TABLE[i]; }
    }
    return best->wire;
}

// ----------------------------------------------------------------------------
// Decode: Canon APEX wire byte → human-readable value
// Returns 0 / 0.0f if the wire value is not in the table.
// ----------------------------------------------------------------------------

inline float canonApertureFromWire(uint8_t wire) {
    for (size_t i = 0; i < CANON_APERTURE_TABLE_SIZE; i++) {
        if (CANON_APERTURE_TABLE[i].wire == wire) return CANON_APERTURE_TABLE[i].fstop;
    }
    return 0.0f;
}

inline float canonShutterFromWire(uint8_t wire) {
    for (size_t i = 0; i < CANON_SHUTTER_TABLE_SIZE; i++) {
        if (CANON_SHUTTER_TABLE[i].wire == wire) return CANON_SHUTTER_TABLE[i].seconds;
    }
    return 0.0f;
}

inline uint16_t canonISOFromWire(uint8_t wire) {
    for (size_t i = 0; i < CANON_ISO_TABLE_SIZE; i++) {
        if (CANON_ISO_TABLE[i].wire == wire) return CANON_ISO_TABLE[i].iso;
    }
    return 0;
}

// ----------------------------------------------------------------------------
// Table index operations — used by AEB step arithmetic
// ----------------------------------------------------------------------------

// Find the exact table index for a wire value. Returns -1 if not found.
inline int canonApertureWireToIdx(uint8_t wire) {
    for (size_t i = 0; i < CANON_APERTURE_TABLE_SIZE; i++) {
        if (CANON_APERTURE_TABLE[i].wire == wire) return (int)i;
    }
    return -1;
}

inline int canonShutterWireToIdx(uint8_t wire) {
    for (size_t i = 0; i < CANON_SHUTTER_TABLE_SIZE; i++) {
        if (CANON_SHUTTER_TABLE[i].wire == wire) return (int)i;
    }
    return -1;
}

inline int canonISOWireToIdx(uint8_t wire) {
    for (size_t i = 0; i < CANON_ISO_TABLE_SIZE; i++) {
        if (CANON_ISO_TABLE[i].wire == wire) return (int)i;
    }
    return -1;
}

// Find the nearest table index for a human value.
inline int canonApertureValueToIdx(float fstop) {
    int best = 0;
    float bestDiff = fabsf(fstop - CANON_APERTURE_TABLE[0].fstop);
    for (size_t i = 1; i < CANON_APERTURE_TABLE_SIZE; i++) {
        float diff = fabsf(fstop - CANON_APERTURE_TABLE[i].fstop);
        if (diff < bestDiff) { bestDiff = diff; best = (int)i; }
    }
    return best;
}

inline int canonShutterValueToIdx(float seconds) {
    int best = 0;
    float s0 = CANON_SHUTTER_TABLE[0].seconds;
    float bestRatio = (seconds > s0) ? (seconds / s0) : (s0 / seconds);
    for (size_t i = 1; i < CANON_SHUTTER_TABLE_SIZE; i++) {
        float s = CANON_SHUTTER_TABLE[i].seconds;
        float ratio = (seconds > s) ? (seconds / s) : (s / seconds);
        if (ratio < bestRatio) { bestRatio = ratio; best = (int)i; }
    }
    return best;
}

inline int canonISOValueToIdx(uint16_t iso) {
    int best = 0;
    int bestDiff = abs((int)iso - (int)CANON_ISO_TABLE[0].iso);
    for (size_t i = 1; i < CANON_ISO_TABLE_SIZE; i++) {
        int diff = abs((int)iso - (int)CANON_ISO_TABLE[i].iso);
        if (diff < bestDiff) { bestDiff = diff; best = (int)i; }
    }
    return best;
}

// Get wire value at a table index. Index is clamped to valid range.
inline uint8_t canonApertureIdxToWire(int idx) {
    if (idx < 0) idx = 0;
    if ((size_t)idx >= CANON_APERTURE_TABLE_SIZE) idx = (int)CANON_APERTURE_TABLE_SIZE - 1;
    return CANON_APERTURE_TABLE[idx].wire;
}

inline uint8_t canonShutterIdxToWire(int idx) {
    if (idx < 0) idx = 0;
    if ((size_t)idx >= CANON_SHUTTER_TABLE_SIZE) idx = (int)CANON_SHUTTER_TABLE_SIZE - 1;
    return CANON_SHUTTER_TABLE[idx].wire;
}

inline uint8_t canonISOIdxToWire(int idx) {
    if (idx < 0) idx = 0;
    if ((size_t)idx >= CANON_ISO_TABLE_SIZE) idx = (int)CANON_ISO_TABLE_SIZE - 1;
    return CANON_ISO_TABLE[idx].wire;
}

inline float canonApertureIdxToValue(int idx) {
    if (idx < 0) idx = 0;
    if ((size_t)idx >= CANON_APERTURE_TABLE_SIZE) idx = (int)CANON_APERTURE_TABLE_SIZE - 1;
    return CANON_APERTURE_TABLE[idx].fstop;
}

inline float canonShutterIdxToValue(int idx) {
    if (idx < 0) idx = 0;
    if ((size_t)idx >= CANON_SHUTTER_TABLE_SIZE) idx = (int)CANON_SHUTTER_TABLE_SIZE - 1;
    return CANON_SHUTTER_TABLE[idx].seconds;
}

inline uint16_t canonISOIdxToValue(int idx) {
    if (idx < 0) idx = 0;
    if ((size_t)idx >= CANON_ISO_TABLE_SIZE) idx = (int)CANON_ISO_TABLE_SIZE - 1;
    return CANON_ISO_TABLE[idx].iso;
}

// ----------------------------------------------------------------------------
// Validation helpers — used by setAperture() and setShutterSpeed() to reject
// values that don't round-trip cleanly through the nearest-match encoder.
// Encode the input, decode it back, and check the error is within tolerance.
// Safe to use in host-side unit tests (no Arduino dependency).
// ----------------------------------------------------------------------------

static constexpr float CANON_APERTURE_SNAP_TOLERANCE = 0.06f;  // 6% relative error
static constexpr float CANON_SHUTTER_SNAP_TOLERANCE  = 1.06f;  // ratio ≤ 1.06

// Returns true if fstop matches a Canon aperture table entry within 6%.
// f/5.6001 (float rounding) → true.  f/27 (not a Canon step) → false.
inline bool canonApertureIsExact(float fstop) {
    if (fstop <= 0.0f) return false;
    uint8_t wire    = canonApertureToWire(fstop);
    float   decoded = canonApertureFromWire(wire);
    if (decoded == 0.0f) return false;
    return (fabsf(fstop - decoded) / decoded) <= CANON_APERTURE_SNAP_TOLERANCE;
}

// Returns true if seconds matches a Canon shutter table entry within 6% ratio.
// 1.0f/100 → true.  1.0f/90 (not a Canon step) → false.
inline bool canonShutterIsExact(float seconds) {
    if (seconds <= 0.0f) return false;
    uint8_t wire    = canonShutterToWire(seconds);
    float   decoded = canonShutterFromWire(wire);
    if (decoded == 0.0f) return false;
    float ratio = (seconds > decoded) ? (seconds / decoded) : (decoded / seconds);
    return ratio <= CANON_SHUTTER_SNAP_TOLERANCE;
}
