#pragma once
#include "ICameraControl.h"
#include "CanonExposure.h"
#include "PtpIpSession.h"
#include "PtpIpDiag.h"

// ============================================================================
// CanonCamera
// Layer 3: Canon EOS-specific PTP operations.
// Base class for all Canon EOS camera models — subclass per model to override
// model-specific behaviour (capture timing, mode dial wire values, init quirks).
// Depends only on PtpIpSession; has no knowledge of TCP sockets.
//
// Subclassing:
//   class Canon6D : public CanonCamera {
//   public:
//       explicit Canon6D(PtpIpSession& s) : CanonCamera(s) {}
//   protected:
//       const char* _modeStr(uint8_t wire) const override;  // if mode wires differ
//       bool        _extraInit() override;                  // if extra init needed
//       // override initiateCapture() / releaseShutter() if timing/sequence differs
//   };
// ============================================================================

// Cached exposure state, kept current by poll() via GetEventData (0x9116).
// Wire values use the same APEX encoding as the Canon APEX tables in CanonExposure.h.
struct CanonState {
    uint8_t apertureWire;  // APEX wire byte for aperture; 0 = not yet known
    uint8_t shutterWire;   // APEX wire byte for shutter speed; 0 = not yet known
    uint8_t isoWire;       // APEX wire byte for ISO; 0 = not yet known
    uint8_t ecWire;        // EC wire byte: (int8_t)round(ev*8); 0x80 = not yet known
    uint8_t modeWire;      // shooting mode wire value; 0xFF = not yet known
    bool    valid;         // true after at least one successful poll()
};

class CanonCamera : public ICameraControl {
public:
    explicit CanonCamera(PtpIpSession& session);

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    CameraResult begin(const char* host) override;
    void end() override;
    bool isReady() const override;
    CameraResult reconnect() override;

    // -------------------------------------------------------------------------
    // Polling
    // -------------------------------------------------------------------------

    // Send GetEventData (0x9116), parse response, update _state, fire callbacks.
    // Also checks the event channel for CaptureComplete (0x400D).
    void poll() override;

    // -------------------------------------------------------------------------
    // Core exposure
    // -------------------------------------------------------------------------

    CameraResult setAperture(float fstop) override;
    CameraResult setShutterSpeed(float seconds) override;
    CameraResult setISO(uint16_t iso) override;

    // -------------------------------------------------------------------------
    // Generic property access
    // -------------------------------------------------------------------------

    CameraResult setCameraProperty(CameraProperty prop, uint32_t value) override;
    CameraResult getCameraProperty(CameraProperty prop, uint32_t& value) override;

    // -------------------------------------------------------------------------
    // Exposure compensation
    // -------------------------------------------------------------------------

    CameraResult setExposureCompensation(float ev) override;

    // -------------------------------------------------------------------------
    // AEB
    // -------------------------------------------------------------------------

    CameraResult setAEBPriority(AEBPriority priority) override;
    CameraResult setAEBSequenceOrder(AEBSequenceOrder order) override;
    CameraResult setAEBApertureLimit(float minFstop, float maxFstop) override;
    CameraResult setAEBShutterLimit(float minSeconds, float maxSeconds) override;
    CameraResult setAEBISOLimit(uint16_t minISO, uint16_t maxISO) override;
    CameraResult setAEBStep(float ev) override;
    CameraResult takeBracketedSequence(int shotCount, float evStep) override;

    // -------------------------------------------------------------------------
    // Capture
    // -------------------------------------------------------------------------

    CameraResult initiateCapture() override;
    CameraResult releaseShutter() override;

    // -------------------------------------------------------------------------
    // Query
    // -------------------------------------------------------------------------

    CameraResult getSettings(CameraSettings& out) override;
    void dumpDiagnostics(Print& out) override;

    // -------------------------------------------------------------------------
    // Callbacks
    // -------------------------------------------------------------------------

    void setOnPropChanged(PropChangedCb cb, void* ctx = nullptr) override;
    void setOnCaptureComplete(CaptureCompleteCb cb, void* ctx = nullptr) override;
    void setOnConnectionChanged(ConnectionChangedCb cb, void* ctx = nullptr) override;
    void setOnBracketedShotComplete(BracketedShotCompleteCb cb, void* ctx = nullptr) override;

protected:
    // -------------------------------------------------------------------------
    // Model-specific override points
    // Subclasses override only what differs from the Canon EOS defaults below.
    // -------------------------------------------------------------------------

    // Returns a human-readable name for a shooting mode wire value.
    // Default implementation covers common Canon EOS mode dial codes.
    // Override if your model uses different wire values.
    virtual const char* _modeStr(uint8_t wire) const;

    // Called at the end of begin() after all standard Canon init steps succeed.
    // Override to perform model-specific additional setup (e.g. extra vendor
    // commands required by certain firmware versions).
    // Return false to abort begin() with CAM_E_PROTOCOL.
    virtual bool _extraInit() { return true; }

    // Maximum absolute EV value accepted by setExposureCompensation().
    // Override if a specific model supports a wider range (e.g. ±5 EV on some R-series bodies).
    virtual float _maxECEv() const { return 3.0f; }

    // -------------------------------------------------------------------------
    // Members — accessible to subclasses
    // -------------------------------------------------------------------------

    // Number of consecutive poll() failures — resets to 0 on success.
    // When this reaches CANON_POLL_FAIL_THRESHOLD the session is declared dead.
    static constexpr uint8_t CANON_POLL_FAIL_THRESHOLD = 3;

    PtpIpSession& _session;
    bool          _ready;
    uint8_t       _pollFailCount;
    char          _host[64];  // copy of host passed to begin(); used by reconnect()
    CanonState    _state;

    // AEB configuration
    AEBPriority      _aebPriority;
    AEBSequenceOrder _aebOrder;
    float    _aebApertureMin, _aebApertureMax;  // f-stop limits; 0 = unconstrained
    float    _aebShutterMin,  _aebShutterMax;   // seconds limits; 0 = unconstrained
    uint16_t _aebISOMin,      _aebISOMax;        // ISO limits; 0 = unconstrained

    // Callbacks
    PropChangedCb           _onPropChanged;       void* _onPropChangedCtx;
    CaptureCompleteCb       _onCaptureComplete;   void* _onCaptureCompleteCtx;
    ConnectionChangedCb     _onConnectionChanged; void* _onConnectionChangedCtx;
    BracketedShotCompleteCb _onBracketedShot;     void* _onBracketedShotCtx;

    // 16 KB buffer for GetEventData payloads (initial response is ~11 KB).
    uint8_t _eventBuf[16384];

    // Poll GetEventData until CANON_EVT_CAMERA_STATUS (0xC18B) has been seen twice.
    // Returns CAM_OK when ready, CAM_E_TIMEOUT if timeoutMs elapses first.
    CameraResult _waitCaptureComplete(uint32_t timeoutMs);

    // AEB arithmetic: compute new wire values for a given EV shift from base state.
    CameraResult _computeAEBWires(const CanonState& base, float ev, CanonState& out) const;

    // Write aperture, shutter, and ISO wire values from state to camera.
    CameraResult _writeExposureState(const CanonState& st);

    // Parse a GetEventData payload and fire callbacks.
    void _parseEventPayload(const uint8_t* buf, size_t len);

    // Apply one property-changed event to _state; fire _onPropChanged callback.
    void _handlePropChanged(uint16_t propCode, uint32_t value);

    // Canon vendor-specific property write via 0x9110
    CameraResult _setCanonProp(uint16_t propCode, uint8_t value);

    // Standard SetDevicePropValue / GetDevicePropValue helpers
    CameraResult _setPropUint8 (uint16_t propCode, uint8_t  value);
    CameraResult _setPropUint16(uint16_t propCode, uint16_t value);
    CameraResult _setPropUint32(uint16_t propCode, uint32_t value);
    CameraResult _getPropUint8 (uint16_t propCode, uint8_t&  out);
    CameraResult _getPropUint16(uint16_t propCode, uint16_t& out);
    CameraResult _getPropUint32(uint16_t propCode, uint32_t& out);
};
