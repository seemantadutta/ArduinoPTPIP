#pragma once
#include "ICameraControl.h"
#include "PtpIpLog.h"
#include <Arduino.h>

// ============================================================================
// SerialShell
// Line-buffered serial command interpreter for testing ArduinoPTPIP.
//
// Camera registry:
//   Call registerCamera() for each camera before begin().
//   WiFi credentials are stored alongside the camera pointer — leave
//   wifiSSID null for cameras that don't need a WiFi connection (e.g. sim).
//   Use 'connect <name>' to switch to any registered camera.
// ============================================================================

class SerialShell {
public:
    struct CameraEntry {
        const char*     name;
        const char*     wifiSSID;   // null = no WiFi needed
        const char*     wifiPass;
        const char*     host;
        ICameraControl* camera;
    };

    static constexpr int MAX_CAMERAS = 4;

    explicit SerialShell();

    // Register a camera. wifiSSID may be null for cameras that don't need WiFi.
    void registerCamera(const char* name, const char* wifiSSID, const char* wifiPass,
                        const char* host, ICameraControl* camera);

    void begin();
    void update();

    // Returns the current log level threshold used by the app's log callback.
    uint8_t getLogLevel() const { return _logLevel; }

private:
    CameraEntry     _cameras[MAX_CAMERAS];
    int             _cameraCount;
    ICameraControl* _active;
    const char*     _activeName;

    char     _buf[64];
    int      _bufIdx;
    uint32_t _lastPollMs;
    uint8_t  _logLevel;  // threshold for the app's log callback; default PTPIP_LOG_WARNING

    void _processCommand();
    void _showPrompt();
    void _clearBuffer();

    void _cmdHelp();
    void _cmdStatus();
    void _cmdConnect(const char* arg);
    void _cmdDisconnect();
    void _cmdAperture(const char* arg);
    void _cmdShutter(const char* arg);
    void _cmdISO(const char* arg);
    void _cmdCapture();
    void _cmdRelease();
    void _cmdSettings();
    void _cmdEC(const char* arg);
    void _cmdAEB(const char* arg);
    void _cmdLog(const char* arg);
    void _cmdDiag();
    void _cmdReboot();

    bool        _connectCamera(const CameraEntry& entry);
    bool        _matchCmd(const char* input, const char* cmd, const char* alias = nullptr);
    const char* _getArg(const char* input);
    void        _printLine(const char* text);
    void        _printKV(const char* key, const char* value);
    void        _printKV(const char* key, int value);
};
