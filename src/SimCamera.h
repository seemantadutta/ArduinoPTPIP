#pragma once
#include "ICameraControl.h"

// ============================================================================
// SimCamera
// Simulated camera for testing without hardware.
// Implements ICameraControl fully — all operations succeed and log their
// actions. Fires onPropChanged callbacks when settings change.
// Fires onCaptureComplete after a short simulated delay inside poll().
// Compiled in/out via ENABLE_SIM_CAMERA build flag.
// ============================================================================

#ifdef ENABLE_SIM_CAMERA

class SimCamera : public ICameraControl {
public:
    SimCamera();

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

    // Fires onCaptureComplete if a simulated capture is pending.
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

private:
    bool     _ready;
    float    _aperture;
    float    _shutterSpeed;
    uint16_t _iso;
    float    _exposureCompensation;

    // Capture pending: set by initiateCapture/releaseShutter, cleared by poll()
    bool     _capturePending;

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

    // AEB arithmetic on float values (no APEX quantisation — SimCamera is a test double).
    // ev: stops to shift, positive = brighter.
    // Returns CAM_E_OUT_OF_RANGE if the full shift cannot be achieved within limits.
    CameraResult _applyAEBStep(float ev);
};

#endif  // ENABLE_SIM_CAMERA
