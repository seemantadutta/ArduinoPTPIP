# PtpIpCamera — Wireless PTP/IP Camera Control for Arduino

WiFi-based remote control of Canon EOS cameras via the PTP/IP protocol. Set aperture, shutter speed, and ISO; trigger single captures or bracketed exposure sequences; and keep your application in sync with physical dial changes via automatic background polling.

Platform: ESP32 | License: MIT | Version: 1.0.0

---

## Features

- **WiFi connection** to Canon EOS cameras in remote-shooting (EOS Utility) mode
- **Exposure control** — set aperture, shutter speed, and ISO from code
- **Capture triggering** — AF + shutter release (`initiateCapture`) or shutter-only (`releaseShutter`)
- **Automatic Exposure Bracketing (AEB)** — configurable shot count, priority order, and per-axis limits
- **State sync** — GetEventData polling detects when physical dials are turned and updates internal state
- **Event callbacks** — `onPropChanged`, `onCaptureComplete`, `onConnectionChanged`, `onBracketedShotComplete`
- **Per-model subclasses** — `Canon5DMkIV` today; adding new Canon models requires overriding only what differs
- **SimCamera** — full-fidelity simulated camera for unit testing and UI development without hardware
- **Portable API** — `ICameraControl` abstract interface; swap `SimCamera` for any camera implementation with one line
- **Field diagnostics** — silent ring buffer records the last 512 events; dump on demand without rebuilding

---

## Hardware Requirements

- **Microcontroller:** ESP32 family (developed and tested on Adafruit ESP32-S3 Feather)
- **Camera:** Canon EOS camera with built-in WiFi (tested on Canon EOS 5D Mark IV)
- **Camera mode:** Camera must be set to **Remote Shooting / EOS Utility** mode before connecting

---

## Quick Start

```cpp
#include <WiFi.h>
#include "PtpIpCamera.h"

PtpIpTransport transport;
PtpIpSession   session(transport);
Canon5DMkIV    camera(session);

void setup() {
    Serial.begin(115200);

    // Optional: receive log messages from the library
    PtpIpSetLogCallback([](uint8_t level, const char* msg) {
        if (level >= PTPIP_LOG_WARNING) Serial.println(msg);
    });

    // Connect to the camera's WiFi network
    WiFi.begin("EOS5D4-XXXXXX", "");
    while (WiFi.status() != WL_CONNECTED) { delay(500); }

    // Open a PTP/IP session
    if (camera.begin("192.168.1.2") != CAM_OK) {
        Serial.println("Connection failed");
        return;
    }

    // Set exposure and fire
    camera.setAperture(5.6f);
    camera.setShutterSpeed(1.0f / 100.0f);
    camera.setISO(400);
    camera.initiateCapture();
}

void loop() {
    // Poll at ~5 Hz to stay in sync with physical dial changes
    static uint32_t last = 0;
    if (millis() - last >= 200) { last = millis(); camera.poll(); }
}
```

---

## Installation

### PlatformIO

Add to your `platformio.ini`:

```ini
lib_deps =
    https://github.com/seemantadutta/PtpIpCamera
```

### Arduino IDE

Search for **PtpIpCamera** in Sketch > Include Library > Manage Libraries.

---

## Platform Compatibility

| Platform | Status |
|---|---|
| ESP32-S3 | Tested |
| ESP32 (all variants) | Compatible |
| ESP8266 | Compatible |
| Arduino MKR WiFi 1010 | Compatible |
| Arduino Nano 33 IoT | Compatible |
| Classic AVR (Uno, Mega) | Not supported — no WiFi |

---

## Architecture

```
ICameraControl              (abstract interface — your app depends only on this)
    ├── CanonCamera         (shared Canon EOS PTP/IP logic — base class)
    │       ├── Canon5DMkIV (Canon 5D Mark IV — tested)
    │       └── Canon6D     (add your model here — see below)
    └── SimCamera           (simulated camera for testing without hardware)

PtpIpSession                (session state, transaction IDs, OpenSession)
    └── PtpIpTransport      (raw TCP: command channel + event channel)
```

- **PtpIpTransport** manages the two TCP connections required by PTP/IP and all packet framing.
- **PtpIpSession** manages session lifecycle, transaction IDs, and all PTP operation wrappers.
- **CanonCamera** translates the high-level API into Canon vendor opcodes and APEX-encoded wire values. It is the base class for all Canon models.
- **Canon5DMkIV** is a thin subclass — currently no overrides, but the correct place to put any 5D MkIV-specific quirks found during testing.
- **ICameraControl** is the interface your application code should depend on. `SimCamera` can be substituted without any other changes.

---

## Logging

The library emits log messages but never prints to `Serial` on its own. You register a callback and decide what to do with each message:

```cpp
PtpIpSetLogCallback([](uint8_t level, const char* msg) {
    // level: PTPIP_LOG_DEBUG=0, PTPIP_LOG_INFO=1,
    //        PTPIP_LOG_WARNING=2, PTPIP_LOG_ERROR=3
    if (level >= PTPIP_LOG_WARNING) {
        Serial.println(msg);  // print warnings and errors only
    }
});
```

Call `PtpIpSetLogCallback(nullptr)` to silence all output. If no callback is registered the library runs silently.

The message string already contains a `[TAG]` prefix identifying the module (e.g. `[CANON]`, `[SESSION]`). Adding a timestamp and level prefix is the app's responsibility:

```cpp
PtpIpSetLogCallback([](uint8_t level, const char* msg) {
    if (level < myLogLevel) return;
    const char* names[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
    uint32_t ms = millis();
    Serial.printf("[%lu.%03lu][%s] %s\n", ms/1000, ms%1000, names[level], msg);
});
```

---

## Diagnostics

PtpIpCamera includes a two-tier diagnostics system designed for field deployments where you cannot attach a serial monitor.

### Tier 1 — Log callback (live tracing, development)

Register a log callback as above. Set your threshold to `PTPIP_LOG_DEBUG` to see every operation as it happens. This is the right tool during development.

### Tier 2 — Ring buffer dump (field failures, shipped devices)

The library silently records the last 512 operations in a compact ring buffer regardless of whether a log callback is registered. When something goes wrong in the field, call:

```cpp
camera.dumpDiagnostics(Serial);
```

This prints a numbered list of the last 512 events (opcodes sent, responses received, property changes, capture phases, errors) with relative timestamps. The output is ~50 lines and can be screenshot or copy-pasted by the user.

**Wiring it up in your app:**

```cpp
// Example: dump on button long-press
if (buttonHeldFor(3000)) {
    camera.dumpDiagnostics(Serial);
}
```

The `SerialShell` example includes a `diag` / `d` command that does this.

**Memory:** the ring buffer uses ~3 KB of PSRAM on ESP32-S3 (falls back to DRAM if PSRAM is unavailable). To disable it entirely and reclaim the memory, add `-D PTPIP_NO_DIAG` to your `build_flags`. `dumpDiagnostics()` still compiles and is safe to call — it prints one explanatory line and returns.

### Tier 3 — Raw packet dumps (deep protocol debugging)

For investigating protocol-level issues (new camera models, firmware differences), enable full packet hex dumps by adding `-D PTPIP_HEX_DUMP` to your `build_flags`:

```ini
build_flags =
    ...
    -D PTPIP_HEX_DUMP   ; WARNING: produces large serial output — development use only
```

This adds raw byte hex dumps of every PTP/IP packet to your log callback output. Not recommended for production builds or end-user diagnostics.

### Asking users for a diagnostic report

When a user reports a bug you can't reproduce:

1. **If they can rebuild:** ask them to add `-D PTPIP_HEX_DUMP` and set the log callback threshold to `PTPIP_LOG_DEBUG`. Reproduce the issue, copy the serial output.

2. **If the device is already deployed:** ask them to trigger `dumpDiagnostics()` (however your app exposes it), screenshot or copy the output, and send it to you. No rebuild needed.

---

## Supported Canon Models

| Model | Class | Status |
|---|---|---|
| Canon EOS 5D Mark IV | `Canon5DMkIV` | Tested |
| Canon EOS 6D | `Canon6D` | Not yet implemented |
| Other Canon EOS with WiFi | `CanonCamera` (base) | Untested — may work |

---

## Adding a New Canon Model

If you have a Canon camera that is not yet supported, here is how to add it. The base class `CanonCamera` handles all standard Canon EOS PTP/IP behaviour. You only override what actually differs.

### Step 1 — Create the subclass header

```cpp
// Canon6D.h
#pragma once
#include "CanonCamera.h"

class Canon6D : public CanonCamera {
public:
    explicit Canon6D(PtpIpSession& session);

protected:
    // Override only what differs from the Canon EOS defaults.
    // Remove any method you don't need to change.

    const char* _modeStr(uint8_t wire) const override;  // if mode dial wires differ
    bool        _extraInit() override;                  // if extra init steps needed

    // Override if capture timing or sequence differs:
    // CameraResult initiateCapture() override;
    // CameraResult releaseShutter() override;
};
```

### Step 2 — Implement it

```cpp
// Canon6D.cpp
#include "Canon6D.h"

Canon6D::Canon6D(PtpIpSession& session)
    : CanonCamera(session)
{}

// Only implement what you actually override.
// If the 6D mode wires are the same as the 5D, delete _modeStr entirely.
const char* Canon6D::_modeStr(uint8_t wire) const {
    switch (wire) {
        case 0x00: return "P";
        case 0x01: return "Tv";
        // ... add 6D-specific mappings discovered from device testing
        default: return CanonCamera::_modeStr(wire); // fall back to base for unknowns
    }
}

bool Canon6D::_extraInit() {
    // Any extra vendor commands needed by the 6D after standard init.
    // Return false to abort begin() with CAM_E_PROTOCOL.
    return true;
}
```

### Step 3 — Use it

```cpp
// main.cpp
#include "Canon6D.h"

static Canon6D camera(session);
```

### Finding model differences

The diagnostic ring buffer is your primary tool. Connect the new camera, reproduce a failure, then call `camera.dumpDiagnostics(Serial)`. The sequence of `OP_SENT` / `RESP_RECV` entries will show you exactly where the new camera diverges from the expected protocol flow.

The three most common differences between Canon EOS models are:

| Difference | Override |
|---|---|
| Mode dial wire values are different | `_modeStr()` |
| Camera needs an extra vendor command during session setup | `_extraInit()` |
| Capture timing or step sequence is different | `initiateCapture()` / `releaseShutter()` |

---

## Testing

The library includes two types of tests:

### Host-side unit tests (no hardware required)

Tests for the APEX encoding/decoding tables and AEB arithmetic run on your development machine:

```
pio test -e native
```

These tests cover:
- `test_apex` — all aperture, shutter speed, ISO, and EC wire value round-trips
- `test_aeb` — AEB arithmetic: EV shift, priority order, spill-over, limit enforcement

### Device tests (ESP32 + camera)

A `SimCamera` integration test runs on the ESP32 without a real camera:

```
pio test -e adafruit_feather_esp32s3
```

This covers the full `SimCamera` lifecycle: connect, set exposure, capture, poll, callbacks.

### Manual integration testing

Use the included `SerialShell` example to test against a real camera interactively:

```
pio run --target upload && pio device monitor
```

Available commands:

| Command | Description |
|---|---|
| `connect 5d4` | Connect to the registered Canon 5D Mark IV |
| `ap 5.6` | Set aperture to f/5.6 |
| `ss 1/100` | Set shutter speed to 1/100 s |
| `iso 400` | Set ISO 400 |
| `shoot` | AF + shutter (full capture sequence) |
| `release` | Shutter only, no AF |
| `s` | Read current settings from camera |
| `aeb 3 1` | 3-shot AEB sequence at 1-stop intervals |
| `ec +1` | Set exposure compensation to +1 EV |
| `log debug` | Set log level to DEBUG (most verbose) |
| `log warning` | Set log level to WARNING (default) |
| `d` | Dump diagnostic ring buffer |
| `status` | Connection and system status |

---

## Build Flags

Add any of these to `build_flags` in your `platformio.ini`:

| Flag | Description |
|---|---|
| `-D PTPIP_HEX_DUMP` | Enable raw PTP/IP packet hex dumps in log output. Deep protocol debugging only — produces large output. |
| `-D PTPIP_NO_DIAG` | Disable the ring buffer entirely. Saves ~3 KB PSRAM. `dumpDiagnostics()` becomes a no-op. |

---

## API Reference

All application code should depend only on `ICameraControl`. The full API is documented in `ICameraControl.h`.

### Lifecycle

```cpp
CameraResult begin(const char* host);  // connect and run PTP/IP handshake
void         end();                    // close session
bool         isReady() const;          // true after successful begin()
CameraResult reconnect();              // end() then begin() using last host; use after onConnectionChanged(false)
void         poll();                   // call at ~5 Hz from loop()
```

### Exposure

```cpp
CameraResult setAperture(float fstop);           // e.g. 5.6f
CameraResult setShutterSpeed(float seconds);     // e.g. 1.0f/100.0f
CameraResult setISO(uint16_t iso);               // e.g. 400
CameraResult setExposureCompensation(float ev);  // e.g. +1.0f, -0.5f
```

### Capture

```cpp
CameraResult initiateCapture();   // AF (half-press) + shutter release
CameraResult releaseShutter();    // shutter only, no AF
```

### AEB (Manual mode)

```cpp
CameraResult setAEBPriority(AEBPriority priority);
CameraResult setAEBStep(float ev);                          // shift from current
CameraResult takeBracketedSequence(int shotCount, float evStep);  // 3/5/7/9 shots
```

### Query

```cpp
CameraResult getSettings(CameraSettings& out);  // aperture (NAN until first poll), shutter (NAN until first poll),
                                                 // ISO (0 until first poll), mode string
void         dumpDiagnostics(Print& out);        // dump ring buffer
```

Check float fields with `isnan()` before use; integer fields with `== 0`:

```cpp
CameraSettings s;
if (camera.getSettings(s) == CAM_OK) {
    if (!isnan(s.aperture))     Serial.printf("f/%.1f\n", s.aperture);
    if (!isnan(s.shutterSpeed)) Serial.printf("1/%.0f s\n", 1.0f / s.shutterSpeed);
    if (s.iso != 0)             Serial.printf("ISO %u\n", s.iso);
}
```

### Callbacks

```cpp
void setOnPropChanged(PropChangedCb cb, void* ctx = nullptr);
void setOnCaptureComplete(CaptureCompleteCb cb, void* ctx = nullptr);
void setOnConnectionChanged(ConnectionChangedCb cb, void* ctx = nullptr);
void setOnBracketedShotComplete(BracketedShotCompleteCb cb, void* ctx = nullptr);
```

### Logging

```cpp
void PtpIpSetLogCallback(PtpLogCallback cb);
// PtpLogCallback = void (*)(uint8_t level, const char* msg)
// Levels: PTPIP_LOG_DEBUG=0, PTPIP_LOG_INFO=1, PTPIP_LOG_WARNING=2, PTPIP_LOG_ERROR=3
```

### Advanced: direct property access

For camera properties not exposed by the high-level API, use the generic escape hatch:

```cpp
CameraResult setCameraProperty(CameraProperty prop, uint32_t value);
CameraResult getCameraProperty(CameraProperty prop, uint32_t& value);
```

`CameraProperty` is an enum of known Canon property codes (see `ICameraControl.h`). These call the standard PTP `SetDevicePropValue` / `GetDevicePropValue` operations. Use this when you need a property that is not yet covered by the typed API rather than hardcoding raw opcodes.

---

## Known Issues

### `releaseShutter()` returns `CAM_E_BUSY` when used alone

On Canon EOS bodies the camera must be in a shoot-ready state before `releaseShutter()` is called. If the camera is sitting idle (mirror up on a 5D MkIV) the shutter-only command returns `0x2019 BUSY`.

**Workaround:** use `initiateCapture()` instead. This sends the full half-press + shutter sequence and handles the state transition internally. `releaseShutter()` is reliable only when the camera is already half-pressed (e.g. via back-button focus) or immediately after a prior half-press in your own code.

**Planned fix (v1.1):** a `halfPress()` / `fullPress()` step API that issues the AF command separately and exposes explicit shutter-only control once the camera confirms it is ready.

---

## Contributing

The `ICameraControl` interface is manufacturer-agnostic. Adding Nikon or Sony support means implementing the interface against their respective PTP/IP dialects. Pull requests are welcome.

For Canon models not yet listed above: if you have the camera and can test, follow the steps in **Adding a New Canon Model** above and open a pull request.

Please open an issue before starting a large feature so the approach can be discussed first.

---

## License

MIT — see [LICENSE](LICENSE) for full text.
