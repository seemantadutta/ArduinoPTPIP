#include "CanonCamera.h"
#include "PtpIpConstants.h"
#include "PtpIpLog.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

#define LOG_TAG "CANON"

// Forward declaration removed — _modeStr is now a protected virtual member method

// Little-endian read helper (local to this file; mirrors PtpIpSession.cpp)
static inline uint32_t evtReadU32(const uint8_t* buf, size_t off) {
    return (uint32_t)buf[off]
         | ((uint32_t)buf[off + 1] << 8)
         | ((uint32_t)buf[off + 2] << 16)
         | ((uint32_t)buf[off + 3] << 24);
}

// ============================================================================
// Constructor
// ============================================================================

CanonCamera::CanonCamera(PtpIpSession& session)
    : _session(session)
    , _ready(false)
    , _pollFailCount(0)
    , _host{}
    , _state{ 0, 0, 0, 0x80, 0xFF, false }
    , _aebPriority(AEBPriority::SHUTTER_ISO_APERTURE)
    , _aebOrder(AEBSequenceOrder::CENTRE_FIRST)
    , _aebApertureMin(0.0f), _aebApertureMax(0.0f)
    , _aebShutterMin(0.0f),  _aebShutterMax(0.0f)
    , _aebISOMin(0),         _aebISOMax(0)
    , _onPropChanged(nullptr),       _onPropChangedCtx(nullptr)
    , _onCaptureComplete(nullptr),   _onCaptureCompleteCtx(nullptr)
    , _onConnectionChanged(nullptr), _onConnectionChangedCtx(nullptr)
    , _onBracketedShot(nullptr),     _onBracketedShotCtx(nullptr)
{
    memset(_eventBuf, 0, sizeof(_eventBuf));
}

// ============================================================================
// Lifecycle
// ============================================================================

CameraResult CanonCamera::begin(const char* host) {
    _ready = false;
    _pollFailCount = 0;
    strncpy(_host, host, sizeof(_host) - 1);
    _host[sizeof(_host) - 1] = '\0';

    if (!_session.connect(host)) {
        LOG_ERROR("begin: handshake failed");
        return CAM_E_PROTOCOL;
    }

    if (!_session.openSession()) {
        LOG_ERROR("begin: OpenSession failed");
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }

    uint16_t responseCode;

    // SetRemoteMode (Canon proprietary 0x9114), param=1
    if (!_session.sendCommand(PTP_OP_CANON_SET_REMOTE, responseCode, 0x01)) {
        LOG_ERROR("begin: SetRemoteMode send failed");
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("begin: SetRemoteMode response 0x%04X", responseCode);
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }

    // SetEventMode (Canon proprietary 0x9115), param=1
    if (!_session.sendCommand(PTP_OP_CANON_SET_EVENT, responseCode, 0x01)) {
        LOG_ERROR("begin: SetEventMode send failed");
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("begin: SetEventMode response 0x%04X", responseCode);
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }

    // Set capture destination to SD card (0x02).
    // Non-fatal: some firmware revisions ignore this; image storage still works.
    if (_setCanonProp(PTP_PROP_CANON_CAPTURE_DEST, 0x02) != CAM_OK) {
        LOG_WARNING("begin: could not set capture destination — images may go to RAM only");
    }

    if (!_extraInit()) {
        LOG_ERROR("begin: model-specific init failed");
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }

    _ready = true;
    if (_onConnectionChanged) _onConnectionChanged(true, _onConnectionChangedCtx);
    LOG_INFO("Canon camera ready");
    return CAM_OK;
}

void CanonCamera::end() {
    _ready = false;
    _state = {};
    _session.disconnect();
    if (_onConnectionChanged) _onConnectionChanged(false, _onConnectionChangedCtx);
    LOG_INFO("Camera session closed");
}

CameraResult CanonCamera::reconnect() {
    if (_host[0] == '\0') {
        LOG_ERROR("reconnect: begin() was never called on this instance");
        return CAM_E_NOT_CONNECTED;
    }
    end();
    return begin(_host);
}

bool CanonCamera::isReady() const {
    return _ready;
}

// ============================================================================
// Callbacks
// ============================================================================

void CanonCamera::setOnPropChanged(PropChangedCb cb, void* ctx) {
    _onPropChanged    = cb;
    _onPropChangedCtx = ctx;
}

void CanonCamera::setOnCaptureComplete(CaptureCompleteCb cb, void* ctx) {
    _onCaptureComplete    = cb;
    _onCaptureCompleteCtx = ctx;
}

void CanonCamera::setOnConnectionChanged(ConnectionChangedCb cb, void* ctx) {
    _onConnectionChanged    = cb;
    _onConnectionChangedCtx = ctx;
}

void CanonCamera::setOnBracketedShotComplete(BracketedShotCompleteCb cb, void* ctx) {
    _onBracketedShot    = cb;
    _onBracketedShotCtx = ctx;
}

// ============================================================================
// Core exposure
// ============================================================================

CameraResult CanonCamera::setAperture(float fstop) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (fstop <= 0.0f) return CAM_E_INVALID_ARG;
    if (!canonApertureIsExact(fstop)) {
        LOG_WARNING("setAperture: f/%.2f is not a Canon step; nearest valid is f/%.2f",
                    fstop, canonApertureFromWire(canonApertureToWire(fstop)));
        return CAM_E_INVALID_ARG;
    }
    uint8_t wire = canonApertureToWire(fstop);
    LOG_DEBUG("setAperture: f/%.1f -> wire 0x%02X", fstop, wire);
    CameraResult r = _setCanonProp(PTP_PROP_CANON_APERTURE, wire);
    if (r == CAM_OK) { _state.apertureWire = wire; _state.valid = true; }
    return r;
}

CameraResult CanonCamera::setShutterSpeed(float seconds) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (seconds <= 0.0f) return CAM_E_INVALID_ARG;
    if (!canonShutterIsExact(seconds)) {
        float nearest = canonShutterFromWire(canonShutterToWire(seconds));
        if (nearest >= 1.0f)
            LOG_WARNING("setShutterSpeed: %.4f s is not a Canon step; nearest is %.1f s",
                        seconds, nearest);
        else
            LOG_WARNING("setShutterSpeed: %.6f s is not a Canon step; nearest is 1/%.0f s",
                        seconds, 1.0f / nearest);
        return CAM_E_INVALID_ARG;
    }
    uint8_t wire = canonShutterToWire(seconds);
    LOG_DEBUG("setShutterSpeed: %.6f s -> wire 0x%02X", seconds, wire);
    CameraResult r = _setCanonProp(PTP_PROP_CANON_SHUTTER_SPEED, wire);
    if (r == CAM_OK) { _state.shutterWire = wire; _state.valid = true; }
    return r;
}

CameraResult CanonCamera::setISO(uint16_t iso) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (iso < CANON_ISO_TABLE[0].iso || iso > CANON_ISO_TABLE[CANON_ISO_TABLE_SIZE - 1].iso)
        return CAM_E_INVALID_ARG;
    uint8_t wire = canonISOToWire(iso);
    LOG_DEBUG("setISO: %u -> wire 0x%02X", (unsigned)iso, wire);
    CameraResult r = _setCanonProp(PTP_PROP_CANON_ISO, wire);
    if (r == CAM_OK) { _state.isoWire = wire; _state.valid = true; }
    return r;
}

// ============================================================================
// Generic property access
// ============================================================================

CameraResult CanonCamera::setCameraProperty(CameraProperty prop, uint32_t value) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    switch (prop) {
        case CameraProperty::APERTURE:
            return _setCanonProp(PTP_PROP_CANON_APERTURE, (uint8_t)value);
        case CameraProperty::SHUTTER_SPEED:
            return _setCanonProp(PTP_PROP_CANON_SHUTTER_SPEED, (uint8_t)value);
        case CameraProperty::ISO:
            return _setCanonProp(PTP_PROP_CANON_ISO, (uint8_t)value);
        case CameraProperty::EXPOSURE_COMPENSATION:
            // value is Canon wire byte: (int8_t)round(ev * 8) stored as uint8
            return _setCanonProp(PTP_PROP_CANON_EC, (uint8_t)value);
        case CameraProperty::CAPTURE_DESTINATION:
            return _setCanonProp(PTP_PROP_CANON_CAPTURE_DEST, (uint8_t)value);
        default:
            return CAM_E_NOT_SUPPORTED;
    }
}

CameraResult CanonCamera::getCameraProperty(CameraProperty prop, uint32_t& value) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    uint8_t wire = 0;
    CameraResult r;
    switch (prop) {
        case CameraProperty::APERTURE:
            if (_state.valid && _state.apertureWire) { value = _state.apertureWire; return CAM_OK; }
            r = _getPropUint8(PTP_PROP_APERTURE, wire);
            if (r != CAM_OK) return r;
            value = wire;
            return CAM_OK;
        case CameraProperty::SHUTTER_SPEED:
            if (_state.valid && _state.shutterWire) { value = _state.shutterWire; return CAM_OK; }
            r = _getPropUint8(PTP_PROP_SHUTTER_SPEED, wire);
            if (r != CAM_OK) return r;
            value = wire;
            return CAM_OK;
        case CameraProperty::ISO:
            if (_state.valid && _state.isoWire) { value = _state.isoWire; return CAM_OK; }
            r = _getPropUint8(PTP_PROP_ISO, wire);
            if (r != CAM_OK) return r;
            value = wire;
            return CAM_OK;
        default:
            return CAM_E_NOT_SUPPORTED;
    }
}

// ============================================================================
// Exposure compensation
// ============================================================================

CameraResult CanonCamera::setExposureCompensation(float ev) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (fabsf(ev) > _maxECEv()) return CAM_E_INVALID_ARG;
    // Canon vendor encoding: wire = round(ev * 8), stored as int8 in uint8.
    // Same APEX scale as other Canon props (1 stop = 8 units).
    // Range: ±3 EV = ±24 units. Uses Canon vendor SetProp (0x9110), NOT standard 0x1016.
    int8_t wire = (int8_t)roundf(ev * 8.0f);
    LOG_DEBUG("setExposureCompensation: %.2f EV -> wire 0x%02X", ev, (uint8_t)wire);
    return _setCanonProp(PTP_PROP_CANON_EC, (uint8_t)wire);
}

// ============================================================================
// AEB configuration
// ============================================================================

CameraResult CanonCamera::setAEBPriority(AEBPriority priority) {
    _aebPriority = priority;
    return CAM_OK;
}

CameraResult CanonCamera::setAEBSequenceOrder(AEBSequenceOrder order) {
    _aebOrder = order;
    return CAM_OK;
}

CameraResult CanonCamera::setAEBApertureLimit(float minFstop, float maxFstop) {
    if (minFstop < 0.0f || maxFstop < 0.0f) return CAM_E_INVALID_ARG;
    if (minFstop > 0.0f && maxFstop > 0.0f && minFstop > maxFstop) return CAM_E_INVALID_ARG;
    _aebApertureMin = minFstop;
    _aebApertureMax = maxFstop;
    return CAM_OK;
}

CameraResult CanonCamera::setAEBShutterLimit(float minSeconds, float maxSeconds) {
    if (minSeconds < 0.0f || maxSeconds < 0.0f) return CAM_E_INVALID_ARG;
    if (minSeconds > 0.0f && maxSeconds > 0.0f && minSeconds > maxSeconds) return CAM_E_INVALID_ARG;
    _aebShutterMin = minSeconds;
    _aebShutterMax = maxSeconds;
    return CAM_OK;
}

CameraResult CanonCamera::setAEBISOLimit(uint16_t minISO, uint16_t maxISO) {
    if (minISO > 0 && maxISO > 0 && minISO > maxISO) return CAM_E_INVALID_ARG;
    _aebISOMin = minISO;
    _aebISOMax = maxISO;
    return CAM_OK;
}

// ============================================================================
// AEB arithmetic
// ============================================================================

CameraResult CanonCamera::_computeAEBWires(const CanonState& base, float ev, CanonState& out) const {
    if (!base.valid) return CAM_E_NOT_READY;

    out = base;
    if (ev == 0.0f) return CAM_OK;

    // Convert EV to 1/3-stop integer steps (Canon tables use 1/3-stop spacing)
    int remaining = (int)roundf(ev * 3.0f);
    if (remaining == 0) return CAM_OK;

    // Build priority order
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

    // Apply remaining steps through the priority chain.
    //
    // Table index direction (positive remaining = brighter):
    //   Aperture : lower index = wider = brighter  → targetIdx = currentIdx - remaining
    //   Shutter  : lower index = longer = brighter → targetIdx = currentIdx - remaining
    //   ISO      : higher index = higher ISO = brighter → targetIdx = currentIdx + remaining

    for (int pi = 0; pi < orderLen && remaining != 0; pi++) {
        Param p = order[pi];
        int curIdx, limitMin, limitMax, targetIdx, applied;

        if (p == APERTURE) {
            curIdx = canonApertureWireToIdx(base.apertureWire);
            if (curIdx < 0) return CAM_E_NOT_READY;
            // limitMin = widest allowed (lowest idx), limitMax = narrowest (highest idx)
            limitMin = (_aebApertureMin > 0.0f) ? canonApertureValueToIdx(_aebApertureMin) : 0;
            limitMax = (_aebApertureMax > 0.0f) ? canonApertureValueToIdx(_aebApertureMax)
                                                 : (int)CANON_APERTURE_TABLE_SIZE - 1;
            targetIdx = curIdx - remaining;
            if (targetIdx < limitMin) targetIdx = limitMin;
            if (targetIdx > limitMax) targetIdx = limitMax;
            applied = curIdx - targetIdx;  // positive = brighter
            remaining -= applied;
            out.apertureWire = canonApertureIdxToWire(targetIdx);

        } else if (p == SHUTTER) {
            curIdx = canonShutterWireToIdx(base.shutterWire);
            if (curIdx < 0) return CAM_E_NOT_READY;
            // limitMin = longest allowed (lowest idx), limitMax = shortest (highest idx)
            limitMin = (_aebShutterMax > 0.0f) ? canonShutterValueToIdx(_aebShutterMax) : 0;
            limitMax = (_aebShutterMin > 0.0f) ? canonShutterValueToIdx(_aebShutterMin)
                                                 : (int)CANON_SHUTTER_TABLE_SIZE - 1;
            targetIdx = curIdx - remaining;
            if (targetIdx < limitMin) targetIdx = limitMin;
            if (targetIdx > limitMax) targetIdx = limitMax;
            applied = curIdx - targetIdx;
            remaining -= applied;
            out.shutterWire = canonShutterIdxToWire(targetIdx);

        } else { // ISO
            curIdx = canonISOWireToIdx(base.isoWire);
            if (curIdx < 0) return CAM_E_NOT_READY;
            limitMin = (_aebISOMin > 0) ? canonISOValueToIdx(_aebISOMin) : 0;
            limitMax = (_aebISOMax > 0) ? canonISOValueToIdx(_aebISOMax)
                                         : (int)CANON_ISO_TABLE_SIZE - 1;
            targetIdx = curIdx + remaining;
            if (targetIdx < limitMin) targetIdx = limitMin;
            if (targetIdx > limitMax) targetIdx = limitMax;
            applied = targetIdx - curIdx;  // positive = brighter
            remaining -= applied;
            out.isoWire = canonISOIdxToWire(targetIdx);
        }
    }

    return (remaining == 0) ? CAM_OK : CAM_E_OUT_OF_RANGE;
}

CameraResult CanonCamera::setAEBStep(float ev) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (!_state.valid) return CAM_E_NOT_READY;

    CanonState newState;
    CameraResult r = _computeAEBWires(_state, ev, newState);
    if (r != CAM_OK) return r;

    r = _writeExposureState(newState);
    if (r == CAM_OK) _state = newState;
    return r;
}

CameraResult CanonCamera::_writeExposureState(const CanonState& st) {
    CameraResult r;
    if (st.apertureWire) {
        r = _setCanonProp(PTP_PROP_CANON_APERTURE, st.apertureWire);
        if (r != CAM_OK) return r;
    }
    if (st.shutterWire) {
        r = _setCanonProp(PTP_PROP_CANON_SHUTTER_SPEED, st.shutterWire);
        if (r != CAM_OK) return r;
    }
    if (st.isoWire) {
        r = _setCanonProp(PTP_PROP_CANON_ISO, st.isoWire);
        if (r != CAM_OK) return r;
    }
    return CAM_OK;
}

// Timing constants shared by takeBracketedSequence and the capture methods below
static constexpr uint32_t CANON_AF_WAIT_MS          = 500;   // StartFocus → StartRelease
static constexpr uint32_t CANON_RELEASE_WAIT_MS     = 200;   // StartRelease → StopRelease
static constexpr uint32_t CANON_AEB_SHOT_TIMEOUT_MS = 10000; // max wait for camera ready between AEB shots

// ============================================================================
// _waitCaptureComplete
// ============================================================================

CameraResult CanonCamera::_waitCaptureComplete(uint32_t timeoutMs) {
    uint32_t start = millis();
    int statusCount = 0;

    while ((millis() - start) < timeoutMs) {
        size_t dataLen = 0;
        if (_session.getCanonEventData(_eventBuf, sizeof(_eventBuf), dataLen) && dataLen > 0) {
            _parseEventPayload(_eventBuf, dataLen);

            // Count CANON_EVT_CAMERA_STATUS (0xC18B) in this payload
            size_t off = 0;
            while (off + 8 <= dataLen) {
                uint32_t segSize = evtReadU32(_eventBuf, off);
                uint32_t evtType = evtReadU32(_eventBuf, off + 4);
                if (evtType == CANON_EVT_TERMINATOR) break;
                if (segSize < 8 || off + segSize > dataLen) break;
                if (evtType == CANON_EVT_CAMERA_STATUS) statusCount++;
                off += segSize;
            }

            if (statusCount >= 2) {
                LOG_DEBUG("_waitCaptureComplete: ready after %ums", (unsigned)(millis() - start));
                return CAM_OK;
            }
        }
        delay(100);
    }

    LOG_WARNING("_waitCaptureComplete: timeout after %ums (saw %d status events)",
                (unsigned)timeoutMs, statusCount);
    return CAM_E_TIMEOUT;
}

// ============================================================================
// takeBracketedSequence
// ============================================================================

CameraResult CanonCamera::takeBracketedSequence(int shotCount, float evStep) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (shotCount < 3 || shotCount > 9 || (shotCount % 2) == 0) return CAM_E_INVALID_ARG;
    if (evStep <= 0.0f) return CAM_E_INVALID_ARG;
    if (!_state.valid) return CAM_E_NOT_READY;

    int half = shotCount / 2;

    // Build EV offset array based on sequence order
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
            for (int i = 0; i < shotCount; i++) {
                offsets[i] = (-half + i) * evStep;
            }
            break;
        case AEBSequenceOrder::DESCENDING:
            for (int i = 0; i < shotCount; i++) {
                offsets[i] = (half - i) * evStep;
            }
            break;
        case AEBSequenceOrder::CENTRE_BRIGHT_DARK:
            offsets[0] = 0.0f;
            for (int i = 0; i < half; i++) {
                offsets[1 + i * 2]     =  (i + 1) * evStep;
                offsets[1 + i * 2 + 1] = -(i + 1) * evStep;
            }
            break;
    }

    CanonState baseState = _state;

    for (int i = 0; i < shotCount; i++) {
        // Compute target state from the base (absolute offset, not cumulative)
        CanonState shotState;
        CameraResult r = _computeAEBWires(baseState, offsets[i], shotState);
        if (r != CAM_OK) {
            _writeExposureState(baseState);
            _state = baseState;
            return r;
        }

        r = _writeExposureState(shotState);
        if (r != CAM_OK) {
            _writeExposureState(baseState);
            _state = baseState;
            return r;
        }
        _state = shotState;

        r = initiateCapture();
        if (r != CAM_OK) {
            _writeExposureState(baseState);
            _state = baseState;
            return r;
        }

        if (_onBracketedShot) {
            _onBracketedShot(offsets[i], _onBracketedShotCtx);
        }

        // Wait for camera to signal ready (0xC18B seen twice) before next shot or restore
        CameraResult waitR = _waitCaptureComplete(CANON_AEB_SHOT_TIMEOUT_MS);
        if (waitR != CAM_OK) {
            LOG_WARNING("takeBracketedSequence: shot %d ready-wait timed out", i);
        }
    }

    // Restore base exposure
    _writeExposureState(baseState);
    _state = baseState;
    return CAM_OK;
}

// ============================================================================
// Capture
// ============================================================================

CameraResult CanonCamera::initiateCapture() {
    if (!_ready) return CAM_E_NOT_CONNECTED;

    uint16_t responseCode;

    // Step 1: StartImageCapture — Focus (half-press)
    PtpIpDiagInstance().record(DIAG_EVT_CAPTURE, (uint8_t)CANON_CAPTURE_PHASE_FOCUS, 0, 0);
    if (!_session.sendCommand(PTP_OP_CANON_START_CAPTURE, responseCode,
                               CANON_CAPTURE_PHASE_FOCUS, 0x00000000)) {
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_PROTOCOL, 0, 0);
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) {
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_BUSY, 0, 0);
        return CAM_E_BUSY;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("initiateCapture: StartFocus response 0x%04X", responseCode);
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_PROTOCOL, 0, 0);
        return CAM_E_PROTOCOL;
    }
    // Note: Canon returns PTP_RSP_OK for the focus command even when AF fails
    // optically (e.g. dark scene, no contrast). There is no protocol-level AF
    // confirmation — the shutter will fire regardless. Use releaseShutter() in
    // Manual focus mode to skip AF entirely.
    LOG_DEBUG("initiateCapture: AF started, waiting %ums...", (unsigned)CANON_AF_WAIT_MS);
    delay(CANON_AF_WAIT_MS);

    // Step 2: StartImageCapture — Release (full-press)
    PtpIpDiagInstance().record(DIAG_EVT_CAPTURE, (uint8_t)CANON_CAPTURE_PHASE_RELEASE, 0, 0);
    if (!_session.sendCommand(PTP_OP_CANON_START_CAPTURE, responseCode,
                               CANON_CAPTURE_PHASE_RELEASE, 0x00000000)) {
        _session.sendCommand(PTP_OP_CANON_STOP_CAPTURE, responseCode, CANON_CAPTURE_PHASE_FOCUS);
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_PROTOCOL, 0, 0);
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        _session.sendCommand(PTP_OP_CANON_STOP_CAPTURE, responseCode, CANON_CAPTURE_PHASE_FOCUS);
        if (responseCode == PTP_RSP_DEVICE_BUSY) {
            PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_BUSY, 0, 0);
            return CAM_E_BUSY;
        }
        LOG_ERROR("initiateCapture: StartRelease response 0x%04X", responseCode);
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_PROTOCOL, 0, 0);
        return CAM_E_PROTOCOL;
    }

    // Step 3: StopImageCapture — Release (release full-press)
    delay(CANON_RELEASE_WAIT_MS);
    _session.sendCommand(PTP_OP_CANON_STOP_CAPTURE, responseCode, CANON_CAPTURE_PHASE_RELEASE);

    // Step 4: StopImageCapture — Focus (release half-press)
    _session.sendCommand(PTP_OP_CANON_STOP_CAPTURE, responseCode, CANON_CAPTURE_PHASE_FOCUS);

    LOG_INFO("Shutter sequence complete");
    return CAM_OK;
}

CameraResult CanonCamera::releaseShutter() {
    if (!_ready) return CAM_E_NOT_CONNECTED;

    uint16_t responseCode;

    PtpIpDiagInstance().record(DIAG_EVT_CAPTURE, (uint8_t)CANON_CAPTURE_PHASE_RELEASE, 0, 0);
    if (!_session.sendCommand(PTP_OP_CANON_START_CAPTURE, responseCode,
                               CANON_CAPTURE_PHASE_RELEASE, 0x00000000)) {
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_PROTOCOL, 0, 0);
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) {
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_BUSY, 0, 0);
        return CAM_E_BUSY;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("releaseShutter: StartRelease response 0x%04X", responseCode);
        PtpIpDiagInstance().record(DIAG_EVT_ERROR, (uint8_t)CAM_E_PROTOCOL, 0, 0);
        return CAM_E_PROTOCOL;
    }

    delay(CANON_RELEASE_WAIT_MS);

    _session.sendCommand(PTP_OP_CANON_STOP_CAPTURE, responseCode, CANON_CAPTURE_PHASE_RELEASE);

    LOG_INFO("Shutter released");
    return CAM_OK;
}

// ============================================================================
// Query
// ============================================================================

CameraResult CanonCamera::getSettings(CameraSettings& out) {
    if (!_ready) return CAM_E_NOT_CONNECTED;

    out = {};
    out.version              = CAMERA_SETTINGS_VERSION;
    out.aperture             = NAN;  // unknown until poll() has received a prop-changed event
    out.shutterSpeed         = NAN;  // unknown until poll() has received a prop-changed event
    out.exposureCompensation = NAN;  // unknown until poll() has seen an EC event

    // Use cached state if poll() has populated it (avoids 3 round-trips)
    if (_state.valid) {
        if (_state.apertureWire) {
            out.aperture = canonApertureFromWire(_state.apertureWire);
            if (out.aperture == 0.0f)
                LOG_WARNING("getSettings: unknown aperture wire 0x%02X", _state.apertureWire);
        }
        if (_state.shutterWire) {
            if (_state.shutterWire == SHUTTER_SPEED_AUTO) {
                // camera is controlling shutter (P/Av mode) — not an error
            } else {
                out.shutterSpeed = canonShutterFromWire(_state.shutterWire);
                if (out.shutterSpeed == 0.0f)
                    LOG_WARNING("getSettings: unknown shutter wire 0x%02X", _state.shutterWire);
            }
        }
        if (_state.isoWire) {
            out.iso = canonISOFromWire(_state.isoWire);
            if (out.iso == 0)
                LOG_WARNING("getSettings: unknown ISO wire 0x%02X", _state.isoWire);
        }
        // ecWire=0x80 means not yet received; 0x00 is valid (0 EV)
        if (_state.ecWire != 0x80)
            out.exposureCompensation = (int8_t)_state.ecWire / 8.0f;
        if (_state.modeWire != 0xFF)
            out.mode = _modeStr(_state.modeWire);
        return CAM_OK;
    }

    // Fallback: poll() hasn't run yet — query camera directly
    LOG_DEBUG("getSettings: no cached state, querying camera directly");
    uint8_t wire = 0;
    CameraResult r;

    r = _getPropUint8(PTP_PROP_APERTURE, wire);
    if (r == CAM_OK) {
        out.aperture = canonApertureFromWire(wire);
        if (out.aperture == 0.0f) LOG_WARNING("getSettings: unknown aperture wire 0x%02X", wire);
    } else {
        LOG_WARNING("getSettings: failed to read aperture (%s)", cameraResultStr(r));
    }

    r = _getPropUint8(PTP_PROP_SHUTTER_SPEED, wire);
    if (r == CAM_OK) {
        if (wire == SHUTTER_SPEED_AUTO) {
            // camera is controlling shutter (P/Av mode) — not an error
        } else {
            out.shutterSpeed = canonShutterFromWire(wire);
            if (out.shutterSpeed == 0.0f) LOG_WARNING("getSettings: unknown shutter wire 0x%02X", wire);
        }
    } else {
        LOG_WARNING("getSettings: failed to read shutter speed (%s)", cameraResultStr(r));
    }

    r = _getPropUint8(PTP_PROP_ISO, wire);
    if (r == CAM_OK) {
        out.iso = canonISOFromWire(wire);
        if (out.iso == 0) LOG_WARNING("getSettings: unknown ISO wire 0x%02X", wire);
    } else {
        LOG_WARNING("getSettings: failed to read ISO (%s)", cameraResultStr(r));
    }

    return CAM_OK;  // Partial results are acceptable — missing fields are 0
}

// ============================================================================
// poll() — called at 5 Hz
// ============================================================================

void CanonCamera::poll() {
    if (!_ready) return;

    size_t dataLen = 0;
    if (!_session.getCanonEventData(_eventBuf, sizeof(_eventBuf), dataLen)) {
        _pollFailCount++;
        LOG_WARNING("poll: GetEventData failed (%u/%u)",
                    (unsigned)_pollFailCount, (unsigned)CANON_POLL_FAIL_THRESHOLD);
        if (_pollFailCount >= CANON_POLL_FAIL_THRESHOLD) {
            LOG_ERROR("poll: %u consecutive failures — session dead, disconnecting",
                      (unsigned)CANON_POLL_FAIL_THRESHOLD);
            _ready = false;
            if (_onConnectionChanged) _onConnectionChanged(false, _onConnectionChangedCtx);
        }
        return;
    }
    _pollFailCount = 0;

    if (dataLen == 0) {
        LOG_DEBUG("poll: no events");
        return;
    }

    _parseEventPayload(_eventBuf, dataLen);
}

// ----------------------------------------------------------------------------
// _parseEventPayload — walk the flat segment stream returned by GetEventData
//
// Segment layout (all fields little-endian uint32):
//   [+0]  segmentSize  — total size of this segment in bytes (incl. these 8 bytes)
//   [+4]  eventType    — 0xc189 prop-changed, 0xc18a allowed-changed, 0 terminator
//
// 0xc189 "property changed" (segmentSize == 16):
//   [+8]  propCode     — Canon vendor code (e.g. 0xd101)
//   [+12] value        — new value (APEX byte in low 8 bits for exposure props)
//
// 0xc18a "allowed values changed" — variable size, skipped.
// 0x00000000 — terminator, stop parsing.
// ----------------------------------------------------------------------------

void CanonCamera::_parseEventPayload(const uint8_t* buf, size_t len) {
    size_t off = 0;
    while (off + 8 <= len) {
        uint32_t segSize = evtReadU32(buf, off);
        uint32_t evtType = evtReadU32(buf, off + 4);

        if (evtType == CANON_EVT_TERMINATOR) break;

        if (segSize < 8 || off + segSize > len) {
            LOG_WARNING("_parseEventPayload: bad segSize %u at off=%u",
                        (unsigned)segSize, (unsigned)off);
            break;
        }

        if (evtType == CANON_EVT_PROP_CHANGED && segSize >= 16) {
            uint32_t propCode = evtReadU32(buf, off + 8);
            uint32_t value    = evtReadU32(buf, off + 12);
            _handlePropChanged((uint16_t)propCode, value);
        }

        off += segSize;
    }
}

// ----------------------------------------------------------------------------
// _handlePropChanged — update cached _state; fire onPropChanged callback
// ----------------------------------------------------------------------------

// Default mode wire → string mapping for Canon EOS cameras.
// Override _modeStr() in a subclass if your model uses different wire values.
const char* CanonCamera::_modeStr(uint8_t wire) const {
    switch (wire) {
        case 0x00: return "P";
        case 0x01: return "Tv";
        case 0x02: return "Av";
        case 0x03: return "M";
        case 0x04: return "B";
        case 0x07: return "C1M";
        case 0x10: return "C2M";
        case 0x11: return "C3M";
        case 0x16: return "A+";
        default:   return "?";
    }
}

void CanonCamera::_handlePropChanged(uint16_t propCode, uint32_t value) {
    uint8_t wire = (uint8_t)(value & 0xFF);
    PtpIpDiagInstance().record(DIAG_EVT_PROP_CHANGED,
                                (uint8_t)(propCode >> 8), (uint8_t)(propCode & 0xFF), wire);
    CameraProperty cbProp;
    bool tracked = true;

    switch (propCode) {
        case PTP_PROP_CANON_APERTURE:
            _state.apertureWire = wire;
            _state.valid = true;
            cbProp = CameraProperty::APERTURE;
            LOG_DEBUG("poll: aperture 0x%02X (f/%.1f)", wire, canonApertureFromWire(wire));
            break;
        case PTP_PROP_CANON_SHUTTER_SPEED:
            _state.shutterWire = wire;
            _state.valid = true;
            cbProp = CameraProperty::SHUTTER_SPEED;
            LOG_DEBUG("poll: shutter 0x%02X (%.3fs)", wire, canonShutterFromWire(wire));
            break;
        case PTP_PROP_CANON_ISO:
            _state.isoWire = wire;
            _state.valid = true;
            cbProp = CameraProperty::ISO;
            LOG_DEBUG("poll: ISO 0x%02X (%u)", wire, (unsigned)canonISOFromWire(wire));
            break;
        case PTP_PROP_CANON_EC: {
            _state.ecWire = (uint8_t)(value & 0xFF);
            float ecEv = (int8_t)_state.ecWire / 8.0f;
            LOG_DEBUG("poll: EC 0x%02X (%.2f EV)", _state.ecWire, ecEv);
            tracked = false;
            break;
        }
        case PTP_PROP_CANON_SHOOTING_MODE:
            _state.modeWire = (uint8_t)(value & 0xFF);
            PtpIpDiagInstance().record(DIAG_EVT_MODE_CHANGED, _state.modeWire, 0, 0);
            LOG_DEBUG("poll: mode 0x%02X (%s)", _state.modeWire, _modeStr(_state.modeWire));
            tracked = false;
            break;
        default:
            tracked = false;
            LOG_DEBUG("poll: prop 0x%04X = 0x%08X (not tracked)", propCode, (unsigned)value);
            break;
    }

    if (tracked && _onPropChanged) {
        _onPropChanged(cbProp, value, _onPropChangedCtx);
    }
}

// ============================================================================
// Canon vendor-specific property write
// ============================================================================

CameraResult CanonCamera::_setCanonProp(uint16_t propCode, uint8_t value) {
    uint16_t responseCode;
    if (!_session.sendCanonSetProp(propCode, (uint32_t)value, responseCode)) {
        LOG_ERROR("_setCanonProp: send failed (propCode=0x%04X)", propCode);
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) return CAM_E_BUSY;
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("_setCanonProp: response 0x%04X for propCode=0x%04X", responseCode, propCode);
        return CAM_E_PROTOCOL;
    }
    return CAM_OK;
}

// ============================================================================
// Standard SetDevicePropValue / GetDevicePropValue helpers
// ============================================================================

CameraResult CanonCamera::_setPropUint8(uint16_t propCode, uint8_t value) {
    uint8_t data[1] = { value };
    uint16_t responseCode;
    if (!_session.sendCommandWithData(PTP_OP_SET_DEVICE_PROP, responseCode,
                                       data, 1, propCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) return CAM_E_BUSY;
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("_setPropUint8: response 0x%04X for propCode=0x%04X", responseCode, propCode);
        return CAM_E_PROTOCOL;
    }
    return CAM_OK;
}

CameraResult CanonCamera::_setPropUint16(uint16_t propCode, uint16_t value) {
    uint8_t data[2] = { (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF) };
    uint16_t responseCode;
    if (!_session.sendCommandWithData(PTP_OP_SET_DEVICE_PROP, responseCode,
                                       data, 2, propCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) return CAM_E_BUSY;
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("_setPropUint16: response 0x%04X for propCode=0x%04X", responseCode, propCode);
        return CAM_E_PROTOCOL;
    }
    return CAM_OK;
}

CameraResult CanonCamera::_setPropUint32(uint16_t propCode, uint32_t value) {
    uint8_t data[4] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8)  & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF)
    };
    uint16_t responseCode;
    if (!_session.sendCommandWithData(PTP_OP_SET_DEVICE_PROP, responseCode,
                                       data, 4, propCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) return CAM_E_BUSY;
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("_setPropUint32: response 0x%04X for propCode=0x%04X", responseCode, propCode);
        return CAM_E_PROTOCOL;
    }
    return CAM_OK;
}

CameraResult CanonCamera::_getPropUint8(uint16_t propCode, uint8_t& out) {
    uint8_t data[1] = {};
    size_t  dataLen = 0;
    uint16_t responseCode;
    if (!_session.recvCommandWithData(PTP_OP_GET_DEVICE_PROP, responseCode,
                                       data, sizeof(data), dataLen, propCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) return CAM_E_BUSY;
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("_getPropUint8: response 0x%04X for propCode=0x%04X", responseCode, propCode);
        return CAM_E_PROTOCOL;
    }
    if (dataLen < 1) {
        LOG_ERROR("_getPropUint8: no data for propCode=0x%04X", propCode);
        return CAM_E_PROTOCOL;
    }
    out = data[0];
    return CAM_OK;
}

CameraResult CanonCamera::_getPropUint16(uint16_t propCode, uint16_t& out) {
    uint8_t data[2] = {};
    size_t  dataLen = 0;
    uint16_t responseCode;
    if (!_session.recvCommandWithData(PTP_OP_GET_DEVICE_PROP, responseCode,
                                       data, sizeof(data), dataLen, propCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) return CAM_E_BUSY;
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("_getPropUint16: response 0x%04X for propCode=0x%04X", responseCode, propCode);
        return CAM_E_PROTOCOL;
    }
    if (dataLen < 2) {
        LOG_ERROR("_getPropUint16: payload too short (%u bytes) for propCode=0x%04X",
                  (unsigned)dataLen, propCode);
        return CAM_E_PROTOCOL;
    }
    out = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return CAM_OK;
}

void CanonCamera::dumpDiagnostics(Print& out) {
    PtpIpDiagInstance().dump(out);
}

CameraResult CanonCamera::_getPropUint32(uint16_t propCode, uint32_t& out) {
    uint8_t data[4] = {};
    size_t  dataLen = 0;
    uint16_t responseCode;
    if (!_session.recvCommandWithData(PTP_OP_GET_DEVICE_PROP, responseCode,
                                       data, sizeof(data), dataLen, propCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode == PTP_RSP_DEVICE_BUSY) return CAM_E_BUSY;
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("_getPropUint32: response 0x%04X for propCode=0x%04X", responseCode, propCode);
        return CAM_E_PROTOCOL;
    }
    if (dataLen < 4) {
        LOG_ERROR("_getPropUint32: payload too short (%u bytes) for propCode=0x%04X",
                  (unsigned)dataLen, propCode);
        return CAM_E_PROTOCOL;
    }
    out = (uint32_t)data[0]
        | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 24);
    return CAM_OK;
}
