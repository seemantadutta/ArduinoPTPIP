#ifdef ENABLE_SIM_CAMERA

#include "SimCamera.h"
#include "PtpIpLog.h"
#include "PtpIpDiag.h"
#include "PtpIpConstants.h"
#include <Arduino.h>
#include <math.h>

#define LOG_TAG "SIM"

SimCamera::SimCamera()
    : _ready(false)
    , _aperture(5.6f)
    , _shutterSpeed(1.0f / 100.0f)
    , _iso(400)
    , _exposureCompensation(0.0f)
    , _capturePending(false)
    , _aebPriority(AEBPriority::SHUTTER_ISO_APERTURE)
    , _aebOrder(AEBSequenceOrder::CENTRE_FIRST)
    , _aebApertureMin(0.0f), _aebApertureMax(0.0f)
    , _aebShutterMin(0.0f),  _aebShutterMax(0.0f)
    , _aebISOMin(0),         _aebISOMax(0)
    , _onPropChanged(nullptr),       _onPropChangedCtx(nullptr)
    , _onCaptureComplete(nullptr),   _onCaptureCompleteCtx(nullptr)
    , _onConnectionChanged(nullptr), _onConnectionChangedCtx(nullptr)
    , _onBracketedShot(nullptr),     _onBracketedShotCtx(nullptr)
{}

// ============================================================================
// Lifecycle
// ============================================================================

CameraResult SimCamera::begin(const char* host) {
    LOG_INFO("SimCamera connected (host ignored: %s)", host);
    _ready = true;
    if (_onConnectionChanged) _onConnectionChanged(true, _onConnectionChangedCtx);
    return CAM_OK;
}

void SimCamera::end() {
    _ready = false;
    _capturePending = false;
    if (_onConnectionChanged) _onConnectionChanged(false, _onConnectionChangedCtx);
    LOG_INFO("SimCamera disconnected");
}

CameraResult SimCamera::reconnect() {
    if (_ready) end();
    return begin("sim");
}

bool SimCamera::isReady() const {
    return _ready;
}

// ============================================================================
// Callbacks
// ============================================================================

void SimCamera::setOnPropChanged(PropChangedCb cb, void* ctx) {
    _onPropChanged    = cb;
    _onPropChangedCtx = ctx;
}

void SimCamera::setOnCaptureComplete(CaptureCompleteCb cb, void* ctx) {
    _onCaptureComplete    = cb;
    _onCaptureCompleteCtx = ctx;
}

void SimCamera::setOnConnectionChanged(ConnectionChangedCb cb, void* ctx) {
    _onConnectionChanged    = cb;
    _onConnectionChangedCtx = ctx;
}

void SimCamera::setOnBracketedShotComplete(BracketedShotCompleteCb cb, void* ctx) {
    _onBracketedShot    = cb;
    _onBracketedShotCtx = ctx;
}

// ============================================================================
// Polling
// ============================================================================

void SimCamera::poll() {
    if (!_ready) return;
    if (_capturePending) {
        _capturePending = false;
        if (_onCaptureComplete) _onCaptureComplete(_onCaptureCompleteCtx);
        LOG_DEBUG("SimCamera: onCaptureComplete fired");
    }
}

// ============================================================================
// Core exposure
// ============================================================================

CameraResult SimCamera::setAperture(float fstop) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (fstop <= 0.0f) return CAM_E_INVALID_ARG;
    _aperture = fstop;
    LOG_INFO("SimCamera: aperture f/%.1f", fstop);
    if (_onPropChanged) {
        uint32_t raw = (uint32_t)(fstop * 100.0f);
        _onPropChanged(CameraProperty::APERTURE, raw, _onPropChangedCtx);
    }
    return CAM_OK;
}

CameraResult SimCamera::setShutterSpeed(float seconds) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (seconds <= 0.0f) return CAM_E_INVALID_ARG;
    _shutterSpeed = seconds;
    if (seconds < 1.0f) {
        LOG_INFO("SimCamera: shutter 1/%.0f s", 1.0f / seconds);
    } else {
        LOG_INFO("SimCamera: shutter %.1f s", seconds);
    }
    if (_onPropChanged) {
        uint32_t raw = (uint32_t)(seconds * 1000000.0f);
        _onPropChanged(CameraProperty::SHUTTER_SPEED, raw, _onPropChangedCtx);
    }
    return CAM_OK;
}

CameraResult SimCamera::setISO(uint16_t iso) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    _iso = iso;
    LOG_INFO("SimCamera: ISO %u", (unsigned)iso);
    if (_onPropChanged) {
        _onPropChanged(CameraProperty::ISO, (uint32_t)iso, _onPropChangedCtx);
    }
    return CAM_OK;
}

// ============================================================================
// Generic property access
// ============================================================================

CameraResult SimCamera::setCameraProperty(CameraProperty prop, uint32_t value) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    switch (prop) {
        case CameraProperty::APERTURE:
            return setAperture((float)value / 100.0f);
        case CameraProperty::SHUTTER_SPEED:
            return setShutterSpeed((float)value / 1000000.0f);
        case CameraProperty::ISO:
            return setISO((uint16_t)value);
        case CameraProperty::EXPOSURE_COMPENSATION:
            return setExposureCompensation((float)(int16_t)value / 10.0f);
        default:
            return CAM_E_NOT_SUPPORTED;
    }
}

CameraResult SimCamera::getCameraProperty(CameraProperty prop, uint32_t& value) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    switch (prop) {
        case CameraProperty::APERTURE:
            value = (uint32_t)(_aperture * 100.0f);
            return CAM_OK;
        case CameraProperty::SHUTTER_SPEED:
            value = (uint32_t)(_shutterSpeed * 1000000.0f);
            return CAM_OK;
        case CameraProperty::ISO:
            value = (uint32_t)_iso;
            return CAM_OK;
        case CameraProperty::EXPOSURE_COMPENSATION:
            value = (uint32_t)(int16_t)(_exposureCompensation * 10.0f);
            return CAM_OK;
        default:
            return CAM_E_NOT_SUPPORTED;
    }
}

// ============================================================================
// Exposure compensation
// ============================================================================

CameraResult SimCamera::setExposureCompensation(float ev) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (fabsf(ev) > 3.0f) return CAM_E_INVALID_ARG;
    _exposureCompensation = ev;
    LOG_INFO("SimCamera: EC %+.1f EV", ev);
    if (_onPropChanged) {
        int16_t raw = (int16_t)roundf(ev * 10.0f);
        _onPropChanged(CameraProperty::EXPOSURE_COMPENSATION, (uint32_t)(uint16_t)raw, _onPropChangedCtx);
    }
    return CAM_OK;
}

// ============================================================================
// AEB configuration
// ============================================================================

CameraResult SimCamera::setAEBPriority(AEBPriority priority) {
    _aebPriority = priority;
    return CAM_OK;
}

CameraResult SimCamera::setAEBSequenceOrder(AEBSequenceOrder order) {
    _aebOrder = order;
    return CAM_OK;
}

CameraResult SimCamera::setAEBApertureLimit(float minFstop, float maxFstop) {
    if (minFstop < 0.0f || maxFstop < 0.0f) return CAM_E_INVALID_ARG;
    if (minFstop > 0.0f && maxFstop > 0.0f && minFstop > maxFstop) return CAM_E_INVALID_ARG;
    _aebApertureMin = minFstop;
    _aebApertureMax = maxFstop;
    return CAM_OK;
}

CameraResult SimCamera::setAEBShutterLimit(float minSeconds, float maxSeconds) {
    if (minSeconds < 0.0f || maxSeconds < 0.0f) return CAM_E_INVALID_ARG;
    if (minSeconds > 0.0f && maxSeconds > 0.0f && minSeconds > maxSeconds) return CAM_E_INVALID_ARG;
    _aebShutterMin = minSeconds;
    _aebShutterMax = maxSeconds;
    return CAM_OK;
}

CameraResult SimCamera::setAEBISOLimit(uint16_t minISO, uint16_t maxISO) {
    if (minISO > 0 && maxISO > 0 && minISO > maxISO) return CAM_E_INVALID_ARG;
    _aebISOMin = minISO;
    _aebISOMax = maxISO;
    return CAM_OK;
}

// ============================================================================
// AEB arithmetic
// ============================================================================

// SimCamera AEB operates on continuous float values — no APEX quantisation.
// Positive ev = brighter. Applies exposure shift in priority order, spilling
// to the next parameter when limits are reached.
//
// EV arithmetic:
//   Shutter  : brighter → longer → newShutter = old * 2^ev
//   ISO      : brighter → higher → newISO = old * 2^ev
//   Aperture : brighter → wider  → newFstop = old / 2^(ev/2)
//              (f-stop scales as sqrt(2) per stop, not linearly)

CameraResult SimCamera::_applyAEBStep(float ev) {
    if (!_ready) return CAM_E_NOT_CONNECTED;

    float remaining = ev;
    if (fabsf(remaining) < 0.01f) return CAM_OK;

    enum Param { SHUTTER, ISO, APERTURE };
    Param order[3] = { SHUTTER, ISO, APERTURE };
    int orderLen = 3;

    switch (_aebPriority) {
        case AEBPriority::SHUTTER_ISO_APERTURE:
            order[0]=SHUTTER; order[1]=ISO;     order[2]=APERTURE; orderLen=3; break;
        case AEBPriority::SHUTTER_APERTURE_ISO:
            order[0]=SHUTTER; order[1]=APERTURE; order[2]=ISO;    orderLen=3; break;
        case AEBPriority::ISO_SHUTTER_APERTURE:
            order[0]=ISO;     order[1]=SHUTTER; order[2]=APERTURE; orderLen=3; break;
        case AEBPriority::SHUTTER_ONLY:
            order[0]=SHUTTER; orderLen=1; break;
        case AEBPriority::ISO_ONLY:
            order[0]=ISO;     orderLen=1; break;
        case AEBPriority::APERTURE_ONLY:
            order[0]=APERTURE; orderLen=1; break;
    }

    for (int pi = 0; pi < orderLen && fabsf(remaining) > 0.01f; pi++) {
        Param p = order[pi];

        if (p == SHUTTER) {
            float newShutter = _shutterSpeed * powf(2.0f, remaining);
            // Apply limits
            float hi = (_aebShutterMax > 0.0f) ? _aebShutterMax : 30.0f;
            float lo = (_aebShutterMin > 0.0f) ? _aebShutterMin : 1.0f / 8000.0f;
            newShutter = (newShutter > hi) ? hi : (newShutter < lo) ? lo : newShutter;
            float applied = log2f(newShutter / _shutterSpeed);
            remaining -= applied;
            _shutterSpeed = newShutter;

        } else if (p == ISO) {
            float newISO = _iso * powf(2.0f, remaining);
            float hi = (_aebISOMax > 0) ? (float)_aebISOMax : 3200.0f;
            float lo = (_aebISOMin > 0) ? (float)_aebISOMin : 50.0f;
            newISO = (newISO > hi) ? hi : (newISO < lo) ? lo : newISO;
            float applied = log2f(newISO / (float)_iso);
            remaining -= applied;
            _iso = (uint16_t)(newISO + 0.5f);

        } else { // APERTURE
            // Aperture: +ev = brighter = wider = lower f-stop
            // newFstop = old * 2^(-ev/2)
            float newFstop = _aperture * powf(2.0f, -remaining / 2.0f);
            float hi = (_aebApertureMax > 0.0f) ? _aebApertureMax : 32.0f;
            float lo = (_aebApertureMin > 0.0f) ? _aebApertureMin : 1.0f;
            newFstop = (newFstop > hi) ? hi : (newFstop < lo) ? lo : newFstop;
            // applied EV = -2 * log2(newFstop / old) = 2 * log2(old / newFstop)
            float applied = 2.0f * log2f(_aperture / newFstop);
            remaining -= applied;
            _aperture = newFstop;
        }
    }

    return (fabsf(remaining) < 0.01f) ? CAM_OK : CAM_E_OUT_OF_RANGE;
}

CameraResult SimCamera::setAEBStep(float ev) {
    if (!_ready) return CAM_E_NOT_CONNECTED;

    float savedAperture = _aperture;
    float savedShutter  = _shutterSpeed;
    uint16_t savedISO   = _iso;

    CameraResult r = _applyAEBStep(ev);

    // Fire callbacks for any changed values
    if (_aperture != savedAperture && _onPropChanged) {
        _onPropChanged(CameraProperty::APERTURE,
                       (uint32_t)(_aperture * 100.0f), _onPropChangedCtx);
    }
    if (_shutterSpeed != savedShutter && _onPropChanged) {
        _onPropChanged(CameraProperty::SHUTTER_SPEED,
                       (uint32_t)(_shutterSpeed * 1000000.0f), _onPropChangedCtx);
    }
    if (_iso != savedISO && _onPropChanged) {
        _onPropChanged(CameraProperty::ISO, (uint32_t)_iso, _onPropChangedCtx);
    }

    if (r != CAM_OK) {
        LOG_WARNING("SimCamera: setAEBStep %.2f EV: %s", ev, cameraResultStr(r));
    } else {
        LOG_INFO("SimCamera: setAEBStep %+.2f EV -> f/%.1f, %.6f s, ISO %u",
                 ev, _aperture, _shutterSpeed, (unsigned)_iso);
    }
    return r;
}

CameraResult SimCamera::takeBracketedSequence(int shotCount, float evStep) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (shotCount < 3 || shotCount > 9 || (shotCount % 2) == 0) return CAM_E_INVALID_ARG;
    if (evStep <= 0.0f) return CAM_E_INVALID_ARG;

    int half = shotCount / 2;

    float offsets[9];
    switch (_aebOrder) {
        case AEBSequenceOrder::CENTRE_FIRST:
            offsets[0] = 0.0f;
            for (int i = 0; i < half; i++) {
                offsets[1 + i * 2]     = -(i + 1) * evStep;
                offsets[1 + i * 2 + 1] =  (i + 1) * evStep;
            }
            break;
        case AEBSequenceOrder::ASCENDING:
            for (int i = 0; i < shotCount; i++) offsets[i] = (-half + i) * evStep;
            break;
        case AEBSequenceOrder::DESCENDING:
            for (int i = 0; i < shotCount; i++) offsets[i] = (half - i) * evStep;
            break;
        case AEBSequenceOrder::CENTRE_BRIGHT_DARK:
            offsets[0] = 0.0f;
            for (int i = 0; i < half; i++) {
                offsets[1 + i * 2]     =  (i + 1) * evStep;
                offsets[1 + i * 2 + 1] = -(i + 1) * evStep;
            }
            break;
    }

    float    baseAperture = _aperture;
    float    baseShutter  = _shutterSpeed;
    uint16_t baseISO      = _iso;

    for (int i = 0; i < shotCount; i++) {
        // Restore to base then apply absolute offset
        _aperture     = baseAperture;
        _shutterSpeed = baseShutter;
        _iso          = baseISO;

        CameraResult r = _applyAEBStep(offsets[i]);
        if (r != CAM_OK) {
            _aperture     = baseAperture;
            _shutterSpeed = baseShutter;
            _iso          = baseISO;
            return r;
        }

        LOG_INFO("SimCamera: bracket shot %d/%d offset=%+.1f EV (f/%.1f, %.6f s, ISO %u)",
                 i + 1, shotCount, offsets[i], _aperture, _shutterSpeed, (unsigned)_iso);

        r = releaseShutter();
        if (r != CAM_OK) {
            _aperture     = baseAperture;
            _shutterSpeed = baseShutter;
            _iso          = baseISO;
            return r;
        }

        if (_onBracketedShot) {
            _onBracketedShot(offsets[i], _onBracketedShotCtx);
        }
    }

    _aperture     = baseAperture;
    _shutterSpeed = baseShutter;
    _iso          = baseISO;
    return CAM_OK;
}

// ============================================================================
// Capture
// ============================================================================

CameraResult SimCamera::initiateCapture() {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    LOG_INFO("SimCamera: capture triggered (f/%.1f, %.6f s, ISO %u)",
             _aperture, _shutterSpeed, (unsigned)_iso);
    PtpIpDiagInstance().record(DIAG_EVT_CAPTURE, (uint8_t)CANON_CAPTURE_PHASE_FOCUS, 0, 0);
    PtpIpDiagInstance().record(DIAG_EVT_CAPTURE, (uint8_t)CANON_CAPTURE_PHASE_RELEASE, 0, 0);
    PtpIpDiagInstance().record(DIAG_EVT_RESP_RECV, 0x20, 0x01, 0);  // PTP_RSP_OK = 0x2001
    delay(500);
    _capturePending = true;  // onCaptureComplete fires on next poll()
    return CAM_OK;
}

CameraResult SimCamera::releaseShutter() {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    LOG_INFO("SimCamera: shutter released (no AF)");
    PtpIpDiagInstance().record(DIAG_EVT_CAPTURE, (uint8_t)CANON_CAPTURE_PHASE_RELEASE, 0, 0);
    PtpIpDiagInstance().record(DIAG_EVT_RESP_RECV, 0x20, 0x01, 0);  // PTP_RSP_OK = 0x2001
    delay(300);
    _capturePending = true;
    return CAM_OK;
}

void SimCamera::dumpDiagnostics(Print& out) {
    PtpIpDiagInstance().dump(out);
}

// ============================================================================
// Query
// ============================================================================

CameraResult SimCamera::getSettings(CameraSettings& out) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    out = {};
    out.version      = CAMERA_SETTINGS_VERSION;
    out.aperture     = _aperture;
    out.shutterSpeed = _shutterSpeed;
    out.iso          = _iso;
    LOG_INFO("SimCamera: getSettings -> f/%.1f, %.6f s, ISO %u",
             _aperture, _shutterSpeed, (unsigned)_iso);
    return CAM_OK;
}

#endif  // ENABLE_SIM_CAMERA
