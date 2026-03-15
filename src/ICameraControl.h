#pragma once
#include <stdint.h>
#include <stddef.h>

// Forward declaration — keeps this header Arduino-agnostic (native tests compile fine)
class Print;

/**
 * @file ICameraControl.h
 * @brief Public interface for the ArduinoPTPIP library.
 *
 * All application code should depend only on ICameraControl and the types
 * defined in this header. Camera-manufacturer details are fully isolated in
 * concrete implementations such as CanonCamera.
 */

// ============================================================================
// CameraResult — returned by all ICameraControl methods
// ============================================================================

/**
 * @brief Return code for all ICameraControl operations.
 *
 * Use @ref cameraResultStr() to convert a value to a human-readable string.
 */
enum CameraResult {
    CAM_OK,               ///< Success.
    CAM_E_NOT_CONNECTED,  ///< begin() has not been called or the session was lost.
    CAM_E_NOT_READY,      ///< Camera not ready (e.g. state not yet synced via poll()).
    CAM_E_INVALID_ARG,    ///< Argument out of range or malformed.
    CAM_E_TIMEOUT,        ///< Network or camera response timed out.
    CAM_E_BUSY,           ///< Camera busy — maps to PTP DEVICE_BUSY (0x2019).
    CAM_E_PROTOCOL,       ///< Unexpected PTP response or packet format error.
    CAM_E_NOT_SUPPORTED,  ///< Camera does not support this operation.
    CAM_E_OUT_OF_RANGE    ///< EV shift cannot be achieved within configured AEB limits.
};

/**
 * @brief Return a short human-readable string for a @ref CameraResult.
 * @param r The result code to convert.
 * @return A non-null C string (e.g. @c "OK", @c "camera busy").
 */
inline const char* cameraResultStr(CameraResult r) {
    switch (r) {
        case CAM_OK:              return "OK";
        case CAM_E_NOT_CONNECTED: return "not connected";
        case CAM_E_NOT_READY:     return "not ready";
        case CAM_E_INVALID_ARG:   return "invalid argument";
        case CAM_E_TIMEOUT:       return "timeout";
        case CAM_E_BUSY:          return "camera busy";
        case CAM_E_PROTOCOL:      return "protocol error";
        case CAM_E_NOT_SUPPORTED: return "not supported";
        case CAM_E_OUT_OF_RANGE:  return "out of range";
        default:                  return "unknown error";
    }
}

// ============================================================================
// AEB enums
// ============================================================================

/**
 * @brief Parameter modification order for setAEBStep().
 *
 * Specifies which camera parameter is adjusted first when setAEBStep() needs
 * to achieve an EV shift. When the first parameter reaches its limit, the
 * library spills to the second, then the third.
 *
 * Default: @ref SHUTTER_ISO_APERTURE (photographic best practice — shutter
 * first avoids changing depth-of-field or noise characteristics).
 */
enum class AEBPriority {
    SHUTTER_ISO_APERTURE,  ///< Shutter → ISO → aperture (default).
    SHUTTER_APERTURE_ISO,  ///< Shutter → aperture → ISO (protect ISO).
    ISO_SHUTTER_APERTURE,  ///< ISO → shutter → aperture.
    SHUTTER_ONLY,          ///< Shutter only; returns CAM_E_OUT_OF_RANGE at limit.
    ISO_ONLY,              ///< ISO only; returns CAM_E_OUT_OF_RANGE at limit.
    APERTURE_ONLY          ///< Aperture only; returns CAM_E_OUT_OF_RANGE at limit.
};

/**
 * @brief Firing order for shots within takeBracketedSequence().
 *
 * Controls the order in which exposures are taken relative to the base
 * exposure. Does not affect setAEBStep().
 *
 * Default: @ref CENTRE_FIRST (Canon default — base exposure first).
 *
 * Example for a 3-shot sequence at ±1 EV:
 * - @ref CENTRE_FIRST:       0, −1, +1
 * - @ref ASCENDING:          −1,  0, +1
 * - @ref DESCENDING:         +1,  0, −1
 * - @ref CENTRE_BRIGHT_DARK:  0, +1, −1
 */
enum class AEBSequenceOrder {
    CENTRE_FIRST,       ///< 0, −step, +step, … (Canon default).
    ASCENDING,          ///< −n, …, 0, …, +n  (darkest to brightest).
    DESCENDING,         ///< +n, …, 0, …, −n  (brightest to darkest).
    CENTRE_BRIGHT_DARK  ///< 0, +step, −step, … (base first, bright then dark).
};

// ============================================================================
// CameraProperty — used with setCameraProperty() / getCameraProperty()
// ============================================================================

/**
 * @brief Generic camera property identifier.
 *
 * Core exposure properties (aperture, shutter speed, ISO) also have dedicated
 * typed methods (setAperture(), setShutterSpeed(), setISO()) that are
 * preferred for normal use.
 */
enum class CameraProperty {
    APERTURE,               ///< Aperture (f-stop).
    SHUTTER_SPEED,          ///< Shutter speed (exposure duration in seconds).
    ISO,                    ///< ISO sensitivity.
    EXPOSURE_COMPENSATION,  ///< Exposure compensation (bias) in 1/10-stop units.
    WHITE_BALANCE,          ///< White balance mode.
    DRIVE_MODE,             ///< Drive mode (single, continuous, etc.).
    CAPTURE_DESTINATION     ///< Capture destination (e.g. SD card vs RAM).
    // extensible — new values added here in future versions
};

// ============================================================================
// CameraSettings — snapshot of current exposure state
// ============================================================================

/** @brief Version tag embedded in every CameraSettings instance. */
static constexpr uint8_t CAMERA_SETTINGS_VERSION = 1;

/**
 * @brief Snapshot of the camera's current exposure settings.
 *
 * Zero-initialise before passing to getSettings(). Fields left at 0 after
 * the call are unknown or not yet populated by the camera.
 *
 * @note New fields will be added in future versions. They are always
 *       zero-initialised and never break existing code.
 */
struct CameraSettings {
    uint8_t     version;               ///< Always @ref CAMERA_SETTINGS_VERSION when set by the library.
    float       aperture;              ///< Aperture in f-stops (e.g. 5.6). NAN = not yet received from camera.
    float       shutterSpeed;          ///< Shutter speed in seconds (e.g. 0.01 for 1/100 s). NAN = not yet received. 0 = camera-controlled (P/Av mode).
    uint16_t    iso;                   ///< ISO sensitivity (e.g. 400). 0 = unknown.
    float       exposureCompensation;  ///< EC in EV (e.g. +1.0, -0.5). NAN = unknown.
    const char* mode;                  ///< Shooting mode string e.g. "P", "Av", "Tv", "M", "B", "A+". nullptr = unknown.
    // future fields added here — zero when not populated
};

// ============================================================================
// Callback signatures
// ============================================================================

/**
 * @brief Callback fired by poll() when a camera property changes.
 * @param prop  Which property changed.
 * @param value New raw wire value (encoding is property-specific).
 * @param ctx   User-data pointer passed to setOnPropChanged().
 */
using PropChangedCb = void (*)(CameraProperty prop, uint32_t value, void* ctx);

/**
 * @brief Callback fired by poll() when a capture sequence completes.
 *
 * Asynchronous — driven by the camera's CaptureComplete event, not by
 * initiateCapture(). The shutter fires during initiateCapture(); the card
 * write completes later and poll() picks up the completion event.
 * @param ctx User-data pointer passed to setOnCaptureComplete().
 */
using CaptureCompleteCb = void (*)(void* ctx);

/**
 * @brief Callback fired when the camera connects or disconnects.
 * @param connected @c true on successful begin(); @c false when the session drops.
 * @param ctx       User-data pointer passed to setOnConnectionChanged().
 */
using ConnectionChangedCb = void (*)(bool connected, void* ctx);

/**
 * @brief Callback fired after each individual shot within takeBracketedSequence().
 *
 * Synchronous — fires inline during the bracketed sequence loop.
 * @param ev  The EV offset that was used for the shot just taken.
 * @param ctx User-data pointer passed to setOnBracketedShotComplete().
 */
using BracketedShotCompleteCb = void (*)(float ev, void* ctx);

// ============================================================================
// ICameraControl — abstract interface for all camera implementations
// ============================================================================

/**
 * @brief Abstract interface for wireless camera control over PTP/IP.
 *
 * All application code should depend only on this interface. Concrete
 * implementations (CanonCamera, SimCamera) are swapped in without changing
 * any application logic.
 *
 * ### Typical usage
 * @code
 * // Minimal — no polling or callbacks required:
 * camera.begin("192.168.1.2");
 * camera.setAperture(5.6f);
 * camera.setShutterSpeed(1.0f / 100.0f);
 * camera.setISO(400);
 * camera.initiateCapture();
 * camera.end();
 * @endcode
 *
 * ### Thread safety
 * The library is **not thread-safe by default.** All methods on a single
 * instance must be called from the same thread or task. In multi-core RTOS
 * environments the caller is responsible for synchronisation.
 */
class ICameraControl {
public:
    virtual ~ICameraControl() = default;

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /**
     * @brief Connect to the camera and run the full PTP/IP handshake.
     *
     * Fires onConnectionChanged(true) on success.
     * @param host IP address string (e.g. "192.168.1.2") or mDNS hostname.
     * @return @ref CAM_OK on success, or an error code.
     */
    virtual CameraResult begin(const char* host) = 0;

    /**
     * @brief Close the PTP/IP session and disconnect.
     *
     * Fires onConnectionChanged(false).
     */
    virtual void end() = 0;

    /**
     * @brief Returns true if begin() succeeded and the camera is ready.
     */
    virtual bool isReady() const = 0;

    /**
     * @brief Reconnect using the host passed to the last successful begin() call.
     *
     * Equivalent to end() followed by begin(lastHost). Safe to call from within
     * the onConnectionChanged(false) callback.
     *
     * @return @ref CAM_OK on success.
     * @return @ref CAM_E_NOT_CONNECTED if begin() was never called on this instance.
     * @return Any error code that begin() itself may return.
     */
    virtual CameraResult reconnect() = 0;

    // -------------------------------------------------------------------------
    // Polling
    // -------------------------------------------------------------------------

    /**
     * @brief Poll the camera for pending events and fire registered callbacks.
     *
     * Sends GetEventData (0x9116), parses the response, updates internal state,
     * and fires onPropChanged / onCaptureComplete as appropriate.
     *
     * Call this from loop() at whatever cadence the application requires
     * (5 Hz recommended). Not thread-safe — call from the same task as all
     * other methods. No-op if the camera is not ready.
     *
     * @code
     * uint32_t lastPoll = 0;
     * void loop() {
     *     uint32_t now = millis();
     *     if (now - lastPoll >= 200) { lastPoll = now; camera.poll(); }
     * }
     * @endcode
     */
    virtual void poll() = 0;

    // -------------------------------------------------------------------------
    // Core exposure — strongly typed convenience methods
    // -------------------------------------------------------------------------

    /**
     * @brief Set the aperture.
     * @param fstop F-stop value, e.g. 5.6, 8.0, 11.0.
     * @return @ref CAM_OK on success, or an error code.
     */
    virtual CameraResult setAperture(float fstop) = 0;

    /**
     * @brief Set the shutter speed.
     * @param seconds Exposure duration in seconds, e.g. 0.01 for 1/100 s, 2.0 for 2 s.
     * @return @ref CAM_OK on success, or an error code.
     */
    virtual CameraResult setShutterSpeed(float seconds) = 0;

    /**
     * @brief Set the ISO sensitivity.
     * @param iso Direct ISO value, e.g. 100, 400, 1600.
     * @return @ref CAM_OK on success, or an error code.
     */
    virtual CameraResult setISO(uint16_t iso) = 0;

    // -------------------------------------------------------------------------
    // Generic property access
    // -------------------------------------------------------------------------

    /**
     * @brief Set any camera property by enum.
     *
     * Value encoding is property-specific. Prefer the typed methods
     * (setAperture(), etc.) for core exposure properties.
     * @param prop  The property to set.
     * @param value New value (encoding is property-specific).
     * @return @ref CAM_OK on success, @ref CAM_E_NOT_SUPPORTED if the property
     *         is not implemented for this camera.
     */
    virtual CameraResult setCameraProperty(CameraProperty prop, uint32_t value) = 0;

    /**
     * @brief Read any camera property by enum.
     *
     * Returns the cached value if available (populated by poll()); otherwise
     * queries the camera directly.
     * @param prop  The property to read.
     * @param value Output: the current raw value.
     * @return @ref CAM_OK on success, @ref CAM_E_NOT_SUPPORTED if the property
     *         is not implemented for this camera.
     */
    virtual CameraResult getCameraProperty(CameraProperty prop, uint32_t& value) = 0;

    // -------------------------------------------------------------------------
    // Exposure compensation
    // -------------------------------------------------------------------------

    /**
     * @brief Set the camera's built-in exposure compensation dial.
     *
     * Works in Av/Tv/P modes — the camera's metering applies the bias.
     * Has no effect in Manual mode; use setAEBStep() for Manual mode EV shifts.
     * @param ev Stops of bias, e.g. +1.0, -0.5, 0.0.
     * @return @ref CAM_OK on success, or an error code.
     */
    virtual CameraResult setExposureCompensation(float ev) = 0;

    // -------------------------------------------------------------------------
    // AEB — Automatic Exposure Bracketing (Manual mode)
    // -------------------------------------------------------------------------

    /**
     * @brief Set the priority order for parameter changes during AEB operations.
     *
     * Determines which parameter (shutter, ISO, aperture) is modified first
     * when setAEBStep() needs to achieve an EV shift. When the first parameter
     * reaches its limit, the library spills to the second, then the third.
     *
     * @param priority The desired priority order.
     * @return @ref CAM_OK on success.
     * @see AEBPriority
     */
    virtual CameraResult setAEBPriority(AEBPriority priority) = 0;

    /**
     * @brief Set the firing order for shots within takeBracketedSequence().
     * @param order The desired sequence order.
     * @return @ref CAM_OK on success.
     * @see AEBSequenceOrder
     */
    virtual CameraResult setAEBSequenceOrder(AEBSequenceOrder order) = 0;

    /**
     * @brief Set soft aperture limits that apply only during AEB operations.
     *
     * Direct calls to setAperture() are always unrestricted.
     * The library respects whichever limit is more restrictive: these soft
     * limits or the camera's hardware limits.
     * @param minFstop Minimum f-stop (widest allowed aperture, e.g. 2.8). 0 = unconstrained.
     * @param maxFstop Maximum f-stop (narrowest allowed aperture, e.g. 11.0). 0 = unconstrained.
     * @return @ref CAM_OK on success, @ref CAM_E_INVALID_ARG if minFstop > maxFstop.
     */
    virtual CameraResult setAEBApertureLimit(float minFstop, float maxFstop) = 0;

    /**
     * @brief Set soft shutter speed limits that apply only during AEB operations.
     *
     * Direct calls to setShutterSpeed() are always unrestricted.
     * @param minSeconds Shortest allowed shutter speed (e.g. 1.0/500). 0 = unconstrained.
     * @param maxSeconds Longest allowed shutter speed (e.g. 4.0). 0 = unconstrained.
     * @return @ref CAM_OK on success, @ref CAM_E_INVALID_ARG if minSeconds > maxSeconds.
     */
    virtual CameraResult setAEBShutterLimit(float minSeconds, float maxSeconds) = 0;

    /**
     * @brief Set soft ISO limits that apply only during AEB operations.
     *
     * Direct calls to setISO() are always unrestricted.
     * @param minISO Lowest allowed ISO (e.g. 100). 0 = unconstrained.
     * @param maxISO Highest allowed ISO (e.g. 1600). 0 = unconstrained.
     * @return @ref CAM_OK on success, @ref CAM_E_INVALID_ARG if minISO > maxISO.
     */
    virtual CameraResult setAEBISOLimit(uint16_t minISO, uint16_t maxISO) = 0;

    /**
     * @brief Shift the exposure by @p ev stops from the current setting (Manual mode).
     *
     * Stateless — always relative to the current camera state populated by poll().
     * Respects @ref AEBPriority and the AEB soft limits set by setAEB*Limit().
     * Spills to the next priority parameter if the primary parameter reaches its limit.
     *
     * @param ev Stops of shift: +1.0 = one stop brighter, -1.0 = one stop darker.
     * @return @ref CAM_OK on success.
     * @return @ref CAM_E_NOT_READY if the current state is not yet known (poll() not run).
     * @return @ref CAM_E_OUT_OF_RANGE if the full shift cannot be achieved within limits.
     */
    virtual CameraResult setAEBStep(float ev) = 0;

    /**
     * @brief Execute a full bracketed exposure sequence.
     *
     * Fires @p shotCount shots spaced @p evStep EV apart, centred on the current
     * exposure. The firing order follows the sequence order configured by
     * setAEBSequenceOrder().
     *
     * Fires onBracketedShotComplete after each individual shot.
     * Blocking — returns when all shots are complete.
     * Camera settings are restored to their pre-sequence values on completion
     * (even if an intermediate shot fails).
     *
     * @param shotCount Number of shots. Must be odd: 3, 5, 7, or 9.
     * @param evStep    EV interval between adjacent shots, e.g. 1.0 for 1-stop steps.
     * @return @ref CAM_OK when all shots complete successfully.
     * @return @ref CAM_E_INVALID_ARG if shotCount is not odd or not in [3, 9].
     * @return @ref CAM_E_NOT_READY if the camera state is not yet known.
     * @return @ref CAM_E_OUT_OF_RANGE if a shot EV offset exceeds the AEB limits.
     */
    virtual CameraResult takeBracketedSequence(int shotCount, float evStep) = 0;

    // -------------------------------------------------------------------------
    // Capture
    // -------------------------------------------------------------------------

    /**
     * @brief Full capture sequence: autofocus (half-press) then shutter release.
     *
     * Blocking — returns when the shutter has fired. The image write to the
     * memory card completes asynchronously; onCaptureComplete fires from poll()
     * when the card write is done.
     * @return @ref CAM_OK on success, or an error code.
     */
    virtual CameraResult initiateCapture() = 0;

    /**
     * @brief Shutter release only — no autofocus step.
     *
     * Use in MF mode or when autofocus is not needed.
     * @return @ref CAM_OK on success, or an error code.
     */
    virtual CameraResult releaseShutter() = 0;

    // -------------------------------------------------------------------------
    // Query
    // -------------------------------------------------------------------------

    /**
     * @brief Read the current aperture, shutter speed, and ISO.
     *
     * Returns the cached state populated by poll() if available (fast path).
     * Falls back to direct camera queries if poll() has not run yet.
     * On failure any unread field is left at 0.
     *
     * @param[out] out Zero-initialise before calling. Float fields (aperture,
     *             shutterSpeed, exposureCompensation) are set to NAN when not
     *             yet received from the camera — test with isnan(). The iso
     *             field is 0 when unknown.
     * @return @ref CAM_OK on success (partial results are acceptable).
     * @return @ref CAM_E_NOT_CONNECTED if the camera is not connected.
     */
    virtual CameraResult getSettings(CameraSettings& out) = 0;

    /**
     * @brief Dump the diagnostic ring buffer to @p out.
     *
     * Always safe to call — works before and after begin(), and after a
     * disconnect. When compiled with -D PTPIP_NO_DIAG, prints a single
     * informational line and returns.
     * @param out Any Print-compatible stream, e.g. Serial.
     */
    virtual void dumpDiagnostics(Print& out) = 0;

    // -------------------------------------------------------------------------
    // Callbacks — all optional, register before calling begin() or poll()
    // -------------------------------------------------------------------------

    /**
     * @brief Register a callback fired by poll() when a camera property changes.
     *
     * Fires when a physical dial is turned or a remote set is confirmed.
     * @param cb  Callback function pointer, or @c nullptr to deregister.
     * @param ctx User-data pointer passed back to @p cb on every invocation.
     */
    virtual void setOnPropChanged(PropChangedCb cb, void* ctx = nullptr) = 0;

    /**
     * @brief Register a callback fired by poll() when a capture completes.
     *
     * Asynchronous — fires from poll() when the card write is done, not from
     * within initiateCapture().
     * @param cb  Callback function pointer, or @c nullptr to deregister.
     * @param ctx User-data pointer passed back to @p cb on every invocation.
     */
    virtual void setOnCaptureComplete(CaptureCompleteCb cb, void* ctx = nullptr) = 0;

    /**
     * @brief Register a callback fired when the camera connects or disconnects.
     * @param cb  Callback function pointer, or @c nullptr to deregister.
     * @param ctx User-data pointer passed back to @p cb on every invocation.
     */
    virtual void setOnConnectionChanged(ConnectionChangedCb cb, void* ctx = nullptr) = 0;

    /**
     * @brief Register a callback fired after each shot within takeBracketedSequence().
     *
     * Synchronous — fires inline within the bracketed sequence loop.
     * @param cb  Callback function pointer, or @c nullptr to deregister.
     * @param ctx User-data pointer passed back to @p cb on every invocation.
     */
    virtual void setOnBracketedShotComplete(BracketedShotCompleteCb cb, void* ctx = nullptr) = 0;
};
