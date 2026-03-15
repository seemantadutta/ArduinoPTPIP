# ArduinoPTPIP — Library Context

## What this library is

A portable C++ library for wireless PTP/IP camera control on Arduino-compatible platforms (ESP32 primary target). It implements the PTP/IP transport protocol and Canon EOS vendor extensions, exposing a clean abstract interface (`ICameraControl`) that application code depends on.

The library lives in `lib/ArduinoPTPIP/src/` within the ESP32PTPCameraControl project. It is designed for eventual standalone public release.

## Layer architecture

```
ICameraControl          (abstract interface — applications depend only on this)
    ├── CanonCamera     (Canon EOS PTP/IP logic — base class for all Canon models)
    │       └── Canon5DMkIV   (5D Mark IV subclass — override only what differs)
    └── SimCamera       (full-fidelity simulated camera — no hardware required)

PtpIpSession            (PTP/IP session: OpenSession, transaction IDs, opcode wrappers)
    └── PtpIpTransport  (raw TCP: command channel + event channel, packet framing)

PtpIpLog.h/cpp          (log callback registration; LOG_DEBUG/INFO/WARNING/ERROR macros)
PtpIpDiag.h/cpp         (ring buffer recording last 512 protocol events)
CanonExposure.h         (APEX encoding tables: aperture, shutter, ISO wire values)
PtpIpConstants.h        (PTP/IP opcodes, response codes, Canon vendor codes)
```

## Key files

| File | Role |
|---|---|
| `ICameraControl.h` | Public interface + all types (CameraResult, CameraSettings, enums, callbacks). Application code includes only this. |
| `CanonCamera.h/.cpp` | Canon EOS base class. All Canon PTP/IP logic lives here. Subclass to add a new model. |
| `Canon5DMkIV.h/.cpp` | 5D Mark IV concrete class. Currently no overrides — correct place for model-specific quirks. |
| `SimCamera.h/.cpp` | Software camera for unit testing and UI development without hardware. |
| `CanonExposure.h` | APEX encoding/decoding tables (aperture, shutter, ISO). Header-only, usable in native (non-Arduino) tests. |
| `PtpIpSession.h/.cpp` | PTP/IP session management, vendor opcode wrappers (GetEventData 0x9116, SetObjectPropValue 0x9110, etc.). |
| `PtpIpTransport.h/.cpp` | TCP command + event channel management, packet framing/parsing. |
| `PtpIpLog.h/.cpp` | Logging: register a `void(uint8_t level, const char* msg)` callback. No Serial output without a callback. |
| `PtpIpDiag.h/.cpp` | 512-entry ring buffer of protocol events; dump with `dumpDiagnostics(Serial)`. |
| `PtpIpConstants.h` | PTP/IP and Canon vendor opcode/response constants. |

## Adding a new Canon model

Subclass `CanonCamera`. Override only what actually differs:

```cpp
class Canon6D : public CanonCamera {
public:
    explicit Canon6D(PtpIpSession& s) : CanonCamera(s) {}
protected:
    const char* _modeStr(uint8_t wire) const override;  // if mode dial wires differ
    bool        _extraInit() override;                  // if extra init steps needed
    float       _maxECEv() const override { return 3.0f; }  // if EC range differs
    // override initiateCapture() / releaseShutter() only if capture sequence differs
};
```

The three most common inter-model differences:

| Difference | Override |
|---|---|
| Mode dial wire values | `_modeStr()` |
| Extra vendor command during session setup | `_extraInit()` |
| Capture timing or step sequence | `initiateCapture()` / `releaseShutter()` |

Use `dumpDiagnostics()` on a new camera to see exactly where its protocol flow diverges.

## APEX encoding (CanonExposure.h)

Canon cameras encode aperture, shutter speed, and ISO as single-byte "wire" values in a fixed lookup table (Canon APEX format). The library never interpolates — it maps float → nearest table entry → byte.

**Key design decision:** `setAperture()` and `setShutterSpeed()` validate via round-trip (`canonApertureIsExact` / `canonShutterIsExact`) and return `CAM_E_INVALID_ARG` for values not in the Canon step table. The log message includes the nearest valid value. This prevents silent mismatch between requested and applied values.

Tolerance constants:
- Aperture: 6% relative (`CANON_APERTURE_SNAP_TOLERANCE = 0.06f`)
- Shutter: 6% ratio (`CANON_SHUTTER_SNAP_TOLERANCE = 1.06f`)

Index helpers (`canonApertureWireToIdx`, `canonShutterWireToIdx`, `canonISOWireToIdx`) are used internally for AEB arithmetic — lower index = brighter exposure (longer shutter, wider aperture, higher ISO).

## Connection drop detection

`CanonCamera::poll()` tracks consecutive `getCanonEventData` failures in `_pollFailCount`. After `CANON_POLL_FAIL_THRESHOLD = 3` consecutive failures:
- `_ready = false`
- `onConnectionChanged(false)` fires
- Subsequent `poll()` calls are no-ops (guarded by `isReady()`)

To recover: call `reconnect()`. It calls `end()` then `begin(_host)` using the stored host from the last successful `begin()` call. Safe to call from within the `onConnectionChanged` callback.

## CameraSettings NAN sentinels

`getSettings()` returns a `CameraSettings` struct. Float fields (`aperture`, `shutterSpeed`, `exposureCompensation`) are initialised to `NAN` and remain `NAN` until the first successful `poll()` populates them. Always test with `isnan()`, not `> 0.0f`.

## AEB design

AEB is implemented entirely in software in `CanonCamera` — it does not use Canon's built-in AEB camera mode. It:
1. Reads current exposure state from `_state` (populated by `poll()`)
2. Computes wire values for each EV offset via `_computeAEBWires()`
3. For each shot: writes wire values to camera (`_writeExposureState()`), calls `initiateCapture()`, waits for completion
4. Restores original state on completion (even on failure)

Priority order (shutter → ISO → aperture by default) controls which parameter is adjusted first when achieving an EV shift. Spills to the next parameter at limits. Per-axis soft limits (`setAEB*Limit()`) are enforced during `_computeAEBWires()`.

## EC range

`setExposureCompensation()` clamps to `±_maxECEv()`. The virtual `_maxECEv()` returns `3.0f` by default. Override in model-specific subclasses if the model supports a wider range (e.g. ±5 EV on some R-series bodies).

## Logging

Every source file defines a `LOG_TAG` and uses `LOG_DEBUG` / `LOG_INFO` / `LOG_WARNING` / `LOG_ERROR` macros. Messages are routed through the registered callback — the library never touches `Serial` directly. Callback receives `(uint8_t level, const char* msg)` where `msg` already has a `[TAG]` prefix.

No callback = silent operation.

## Diagnostics

`PtpIpDiag` records protocol events in a 512-entry ring buffer. Entries are compact structs (opcode sent, response received, property change, capture phase, error). `dumpDiagnostics(Serial)` prints a numbered chronological list with relative timestamps.

Disabled at compile time with `-D PTPIP_NO_DIAG` (saves ~3 KB PSRAM). `dumpDiagnostics()` still compiles and is a safe no-op.

Raw packet hex dumps enabled with `-D PTPIP_HEX_DUMP` — produces large output; development only.

## Test coverage

| Test suite | Environment | What it covers |
|---|---|---|
| `test_apex` | native | APEX encode/decode round-trips; index helpers; `canonApertureIsExact`/`canonShutterIsExact` |
| `test_aeb` | native | AEB EV arithmetic; priority spill-over; limit enforcement |
| `test_sim` | ESP32 device | SimCamera full lifecycle; EC validation; reconnect; callbacks |

Run native tests: `pio test -e native`
Run device tests: `pio test -e adafruit_feather_esp32s3`

## Known issues

**`releaseShutter()` returns `CAM_E_BUSY` when called alone on idle Canon bodies.** The camera must be in a shoot-ready state (mirror up, prior half-press). Use `initiateCapture()` instead. Planned fix in v1.1: `halfPress()` / `fullPress()` step API.

## Design constraints

- **No dynamic memory allocation** in the hot path. Event buffer (`_eventBuf[16384]`) is a member array. Ring buffer is statically sized.
- **No RTOS dependency.** Single-threaded: all methods called from the same Arduino `loop()` task. Not thread-safe — callers are responsible for synchronisation in RTOS environments.
- **No Serial output.** All logging gated through the callback. Library is usable in headless / production firmware.
- **Canon-agnostic transport layer.** `PtpIpTransport` and `PtpIpSession` have no Canon-specific knowledge. Adding Nikon or Sony support means implementing `ICameraControl` directly against their PTP/IP dialect.
