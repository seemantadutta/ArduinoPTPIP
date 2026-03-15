#include "SerialShell.h"
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#define LOG_TAG "SHELL"

// ============================================================================
// Construction / registration
// ============================================================================

SerialShell::SerialShell()
    : _cameraCount(0)
    , _active(nullptr)
    , _activeName("none")
    , _bufIdx(0)
    , _lastPollMs(0)
    , _logLevel(PTPIP_LOG_WARNING)
{
    _buf[0] = '\0';
    memset(_cameras, 0, sizeof(_cameras));
}

void SerialShell::registerCamera(const char* name, const char* wifiSSID,
                                  const char* wifiPass, const char* host,
                                  ICameraControl* camera) {
    if (_cameraCount >= MAX_CAMERAS) {
        LOG_ERROR("registerCamera: registry full (max %d)", MAX_CAMERAS);
        return;
    }
    _cameras[_cameraCount++] = { name, wifiSSID, wifiPass, host, camera };
    LOG_INFO("Camera registered: '%s'", name);
}

// ============================================================================
// Shell lifecycle
// ============================================================================

void SerialShell::begin() {
    _printLine("===========================================");
    _printLine(" ESP32 PTP/IP Camera Control");
    _printLine(" Type 'help' for commands");
    _printLine("===========================================");

    if (_cameraCount > 0) {
        Serial.printf("  Registered cameras: ");
        for (int i = 0; i < _cameraCount; i++) {
            Serial.printf("%s%s", _cameras[i].name,
                          i < _cameraCount - 1 ? ", " : "\r\n");
        }
        _printLine("  Type 'connect <name>' to connect");
    }

    _printLine("");
    _showPrompt();
}

void SerialShell::update() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\b' || c == 127) {
            if (_bufIdx > 0) {
                _bufIdx--;
                _buf[_bufIdx] = '\0';
                Serial.print("\b \b");
            }
            continue;
        }

        if (c == '\n' || c == '\r') {
            Serial.print("\r\n");
            if (_bufIdx > 0) {
                _processCommand();
                _clearBuffer();
            }
            _showPrompt();
            continue;
        }

        if (c >= 32 && c < 127 && _bufIdx < 63) {
            _buf[_bufIdx++] = c;
            _buf[_bufIdx]   = '\0';
            Serial.print(c);
        }
    }

    // 5 Hz keep-alive: send GetEventData to maintain session and track
    // any settings changed via the camera's physical dials.
    if (_active && _active->isReady()) {
        uint32_t now = millis();
        if (now - _lastPollMs >= 200) {
            _lastPollMs = now;
            _active->poll();
        }
    }
}

void SerialShell::_showPrompt() {
    if (_active && !_active->isReady()) {
        Serial.printf("CAM[%s!]> ", _activeName);
    } else {
        Serial.printf("CAM[%s]> ", _activeName);
    }
}

void SerialShell::_clearBuffer() {
    _bufIdx = 0;
    _buf[0] = '\0';
}

// ============================================================================
// Command routing
// ============================================================================

void SerialShell::_processCommand() {
    char lower[64];
    int  len = _bufIdx < 63 ? _bufIdx : 63;
    for (int i = 0; i < len; i++) lower[i] = (char)tolower(_buf[i]);
    lower[len] = '\0';

    char*       firstWord = strtok(lower, " ");
    if (!firstWord) return;
    const char* arg = _getArg(_buf);

    if      (_matchCmd(firstWord, "help", "?"))         _cmdHelp();
    else if (_matchCmd(firstWord, "status", "st"))      _cmdStatus();
    else if (_matchCmd(firstWord, "connect", "conn"))   _cmdConnect(arg);
    else if (_matchCmd(firstWord, "disconnect", "disc"))_cmdDisconnect();
    else if (_matchCmd(firstWord, "ap", "aperture"))    _cmdAperture(arg);
    else if (_matchCmd(firstWord, "ss", "shutter"))     _cmdShutter(arg);
    else if (_matchCmd(firstWord, "iso"))               _cmdISO(arg);
    else if (_matchCmd(firstWord, "ec"))                _cmdEC(arg);
    else if (_matchCmd(firstWord, "aeb"))               _cmdAEB(arg);
    else if (_matchCmd(firstWord, "shoot", "capture"))  _cmdCapture();
    else if (_matchCmd(firstWord, "release", "rel"))    _cmdRelease();
    else if (_matchCmd(firstWord, "settings", "s"))     _cmdSettings();
    else if (_matchCmd(firstWord, "diag", "d"))         _cmdDiag();
    else if (_matchCmd(firstWord, "log"))               _cmdLog(arg);
    else if (_matchCmd(firstWord, "reboot"))            _cmdReboot();
    else Serial.printf("Unknown command: '%s'. Type 'help'.\r\n", firstWord);
}

// ============================================================================
// Command handlers
// ============================================================================

void SerialShell::_cmdHelp() {
    _printLine("Commands:");
    _printLine("");
    _printLine("  help, ?              This help");
    _printLine("  status, st           Connection and camera state");
    _printLine("");
    _printLine("Connection:");
    _printLine("  connect <name>       Connect to a registered camera");
    if (_cameraCount > 0) {
        Serial.printf("                       Available: ");
        for (int i = 0; i < _cameraCount; i++) {
            Serial.printf("%s%s", _cameras[i].name,
                          i < _cameraCount - 1 ? ", " : "\r\n");
        }
    }
    _printLine("  disconnect, disc     Disconnect current camera");
    _printLine("");
    _printLine("Exposure (camera must be connected first):");
    _printLine("  ap <f-stop>          Set aperture  e.g. 'ap 5.6', 'ap 8'");
    _printLine("  ss <speed>           Set shutter speed");
    _printLine("                       Fraction: 'ss 1/100', 'ss 1/50'");
    _printLine("                       Seconds:  'ss 0.5', 'ss 2'");
    _printLine("  iso <value>          Set ISO  e.g. 'iso 400', 'iso 1600'");
    _printLine("  ec <ev>              Set exposure compensation (Av/Tv/P modes)");
    _printLine("                       e.g. 'ec +1', 'ec -0.5', 'ec 0'");
    _printLine("  aeb <n> <step>       Bracketed sequence (Manual mode)");
    _printLine("                       n = shot count (3/5/7/9), step = EV interval");
    _printLine("                       e.g. 'aeb 3 1' = 3 shots at 1-stop intervals");
    _printLine("");
    _printLine("Capture:");
    _printLine("  shoot, capture       AF + shutter (full sequence)");
    _printLine("  release, rel         Shutter only, no AF steps");
    _printLine("");
    _printLine("Query:");
    _printLine("  settings, s          Read current aperture, shutter, ISO from camera");
    _printLine("  diag, d              Dump diagnostic ring buffer to serial");
    _printLine("");
    _printLine("System:");
    _printLine("  log                  Show current log level");
    _printLine("  log <level>          Set log level: debug/info/warning/error/off");
    _printLine("  reboot               Restart ESP32");
}

void SerialShell::_cmdStatus() {
    _printLine("=== Status ===");
    _printKV("Camera",   _activeName);
    _printKV("Ready",    (_active && _active->isReady()) ? "Yes" : "No");
    _printKV("WiFi",     WiFi.status() == WL_CONNECTED
                         ? WiFi.SSID().c_str() : "Not connected");
    if (WiFi.status() == WL_CONNECTED) {
        _printKV("Local IP", WiFi.localIP().toString().c_str());
    }
    const char* levelNames[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
    _printKV("Log level", _logLevel < 4 ? levelNames[_logLevel] : "OFF");
    Serial.printf("  %-20s %d KB\r\n", "Free heap:", (int)(ESP.getFreeHeap() / 1024));
}

void SerialShell::_cmdConnect(const char* arg) {
    if (!arg) {
        _printLine("Usage: connect <name>");
        if (_cameraCount > 0) {
            Serial.printf("  Available: ");
            for (int i = 0; i < _cameraCount; i++) {
                Serial.printf("%s%s", _cameras[i].name,
                              i < _cameraCount - 1 ? ", " : "\r\n");
            }
        }
        return;
    }

    const CameraEntry* entry = nullptr;
    for (int i = 0; i < _cameraCount; i++) {
        if (strcasecmp(_cameras[i].name, arg) == 0) {
            entry = &_cameras[i];
            break;
        }
    }

    if (!entry) {
        Serial.printf("Unknown camera: '%s'\r\n", arg);
        return;
    }

    if (_active && _active->isReady()) {
        _active->end();
    }

    if (_connectCamera(*entry)) {
        _active     = entry->camera;
        _activeName = entry->name;
        Serial.printf("Connected to '%s'\r\n", entry->name);
    } else {
        _active     = nullptr;
        _activeName = "none";
        _printLine("Connection failed");
    }
}

void SerialShell::_cmdDisconnect() {
    if (!_active || !_active->isReady()) {
        _printLine("No camera connected");
        return;
    }
    _active->end();
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(true);
    }
    _active     = nullptr;
    _activeName = "none";
    _printLine("Disconnected");
}

bool SerialShell::_connectCamera(const CameraEntry& entry) {
    bool needsWifi = entry.wifiSSID && entry.wifiSSID[0] != '\0';

    if (needsWifi) {
        if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == entry.wifiSSID) {
            LOG_INFO("Already on WiFi '%s'", entry.wifiSSID);
        } else {
            if (WiFi.status() == WL_CONNECTED) {
                LOG_INFO("Switching WiFi to '%s'", entry.wifiSSID);
                WiFi.disconnect(true);
                delay(200);
            }
            Serial.printf("Connecting to WiFi '%s' (Ctrl+C to abort)", entry.wifiSSID);
            WiFi.begin(entry.wifiSSID, entry.wifiPass);

            uint32_t start = millis();
            while (WiFi.status() != WL_CONNECTED) {
                if (millis() - start > 15000) {
                    Serial.println();
                    LOG_ERROR("WiFi timeout");
                    WiFi.disconnect(true);
                    return false;
                }
                if (Serial.available()) {
                    char c = (char)Serial.peek();
                    if (c == 0x03 || c == 0x1B) {
                        Serial.read();
                        Serial.println();
                        _printLine("Aborted");
                        WiFi.disconnect(true);
                        return false;
                    }
                }
                Serial.print(".");
                delay(500);
            }
            Serial.println();
            LOG_INFO("WiFi connected. IP: %s", WiFi.localIP().toString().c_str());
        }
    }

    Serial.printf("Connecting to %s at %s...\r\n", entry.name, entry.host);
    CameraResult r = entry.camera->begin(entry.host);
    if (r != CAM_OK) {
        LOG_ERROR("Failed to connect to '%s': %s", entry.name, cameraResultStr(r));
        return false;
    }
    return true;
}

void SerialShell::_cmdAperture(const char* arg) {
    if (!_active || !_active->isReady()) {
        _printLine("No camera ready — use 'connect <name>' first");
        return;
    }
    if (!arg) { _printLine("Usage: ap <f-stop>  e.g. 'ap 5.6'"); return; }
    float fstop = atof(arg);
    if (fstop <= 0.0f) { Serial.printf("Invalid f-stop: %s\r\n", arg); return; }
    CameraResult r = _active->setAperture(fstop);
    if (r == CAM_OK) {
        Serial.printf("Aperture set to f/%.1f\r\n", fstop);
    } else {
        Serial.printf("Failed to set aperture: %s\r\n", cameraResultStr(r));
    }
}

void SerialShell::_cmdShutter(const char* arg) {
    if (!_active || !_active->isReady()) {
        _printLine("No camera ready — use 'connect <name>' first");
        return;
    }
    if (!arg) { _printLine("Usage: ss <speed>  e.g. 'ss 1/100', 'ss 2'"); return; }

    float seconds = 0.0f;
    const char* slash = strchr(arg, '/');
    if (slash) {
        float denom = atof(slash + 1);
        if (denom == 0.0f) { _printLine("Division by zero"); return; }
        seconds = atof(arg) / denom;
    } else {
        seconds = atof(arg);
    }
    if (seconds <= 0.0f) { Serial.printf("Invalid speed: %s\r\n", arg); return; }

    CameraResult r = _active->setShutterSpeed(seconds);
    if (r == CAM_OK) {
        if (seconds < 0.3f) Serial.printf("Shutter speed set to 1/%.0f s\r\n", 1.0f / seconds);
        else                Serial.printf("Shutter speed set to %.1f s\r\n", seconds);
    } else {
        Serial.printf("Failed to set shutter speed: %s\r\n", cameraResultStr(r));
    }
}

void SerialShell::_cmdISO(const char* arg) {
    if (!_active || !_active->isReady()) {
        _printLine("No camera ready — use 'connect <name>' first");
        return;
    }
    if (!arg) { _printLine("Usage: iso <value>  e.g. 'iso 400'"); return; }
    int val = atoi(arg);
    if (val <= 0) { Serial.printf("Invalid ISO: %s\r\n", arg); return; }
    CameraResult r = _active->setISO((uint16_t)val);
    if (r == CAM_OK) {
        Serial.printf("ISO set to %d\r\n", val);
    } else {
        Serial.printf("Failed to set ISO: %s\r\n", cameraResultStr(r));
    }
}

void SerialShell::_cmdCapture() {
    if (!_active || !_active->isReady()) {
        _printLine("No camera ready — use 'connect <name>' first");
        return;
    }
    _printLine("Triggering capture...");
    CameraResult r = _active->initiateCapture();
    if (r == CAM_OK) {
        _printLine("Capture complete");
    } else {
        Serial.printf("Capture failed: %s\r\n", cameraResultStr(r));
    }
}

void SerialShell::_cmdRelease() {
    if (!_active || !_active->isReady()) {
        _printLine("No camera ready — use 'connect <name>' first");
        return;
    }
    _printLine("Releasing shutter (no AF)...");
    CameraResult r = _active->releaseShutter();
    if (r == CAM_OK) {
        _printLine("Done");
    } else {
        Serial.printf("Failed: %s\r\n", cameraResultStr(r));
    }
}

void SerialShell::_cmdSettings() {
    if (!_active) {
        _printLine("No camera selected — use 'connect <name>' first");
        return;
    }

    bool ready = _active->isReady();
    _printKV("Status:", ready ? "[OK]" : "[DISCONNECTED]");
    if (!ready) return;

    CameraSettings s = {};
    CameraResult r = _active->getSettings(s);
    if (r != CAM_OK) {
        Serial.printf("Failed to read settings: %s\r\n", cameraResultStr(r));
        return;
    }

    if (s.mode) {
        Serial.printf("  %-20s %s\r\n", "Mode:", s.mode);
    } else {
        Serial.printf("  %-20s unknown\r\n", "Mode:");
    }

    if (!isnan(s.aperture)) {
        Serial.printf("  %-20s f/%.1f\r\n", "Aperture:", s.aperture);
    } else {
        Serial.printf("  %-20s unknown\r\n", "Aperture:");
    }

    if (!isnan(s.shutterSpeed) && s.shutterSpeed > 0.0f) {
        if (s.shutterSpeed < 0.3f) {
            Serial.printf("  %-20s 1/%.0f s\r\n", "Shutter speed:", 1.0f / s.shutterSpeed);
        } else {
            Serial.printf("  %-20s %.1f s\r\n", "Shutter speed:", s.shutterSpeed);
        }
    } else if (!isnan(s.shutterSpeed) && s.shutterSpeed == 0.0f) {
        Serial.printf("  %-20s auto\r\n", "Shutter speed:");
    } else {
        Serial.printf("  %-20s unknown\r\n", "Shutter speed:");
    }

    if (s.iso > 0) {
        Serial.printf("  %-20s %u\r\n", "ISO:", (unsigned)s.iso);
    } else {
        Serial.printf("  %-20s unknown\r\n", "ISO:");
    }

    bool ecMode = s.mode && (strcmp(s.mode, "P")  == 0 ||
                             strcmp(s.mode, "Av") == 0 ||
                             strcmp(s.mode, "Tv") == 0);
    if (ecMode) {
        if (!isnan(s.exposureCompensation)) {
            Serial.printf("  %-20s %+.1f EV\r\n", "EC:", s.exposureCompensation);
        } else {
            Serial.printf("  %-20s unknown\r\n", "EC:");
        }
    }
}

void SerialShell::_cmdEC(const char* arg) {
    if (!_active || !_active->isReady()) {
        _printLine("No camera ready — use 'connect <name>' first");
        return;
    }
    if (!arg) { _printLine("Usage: ec <ev>  e.g. 'ec +1', 'ec -0.5', 'ec 0'"); return; }
    float ev = atof(arg);
    CameraResult r = _active->setExposureCompensation(ev);
    if (r == CAM_OK) {
        Serial.printf("Exposure compensation set to %+.1f EV\r\n", ev);
    } else {
        Serial.printf("Failed to set EC: %s\r\n", cameraResultStr(r));
    }
}

void SerialShell::_cmdAEB(const char* arg) {
    if (!_active || !_active->isReady()) {
        _printLine("No camera ready — use 'connect <name>' first");
        return;
    }
    if (!arg) {
        _printLine("Usage: aeb <count> <step>  e.g. 'aeb 3 1' or 'aeb 5 0.5'");
        return;
    }

    int   count = 0;
    float step  = 0.0f;
    if (sscanf(arg, "%d %f", &count, &step) != 2 || step <= 0.0f) {
        _printLine("Usage: aeb <count> <step>  e.g. 'aeb 3 1' or 'aeb 5 0.5'");
        return;
    }
    if (count < 3 || count > 9 || (count % 2) == 0) {
        _printLine("Shot count must be odd: 3, 5, 7, or 9");
        return;
    }

    Serial.printf("Starting %d-shot AEB sequence at %.1f-stop intervals...\r\n", count, step);
    CameraResult r = _active->takeBracketedSequence(count, step);
    if (r == CAM_OK) {
        Serial.printf("AEB sequence complete (%d shots)\r\n", count);
    } else {
        Serial.printf("AEB failed: %s\r\n", cameraResultStr(r));
    }
}

void SerialShell::_cmdLog(const char* arg) {
    if (!arg) {
        const char* names[] = { "DEBUG (0)", "INFO (1)", "WARNING (2)", "ERROR (3)", "OFF (4)" };
        Serial.printf("Current log level: %s\r\n",
                      _logLevel < 4 ? names[_logLevel] : names[4]);
        return;
    }
    uint8_t newLevel;
    bool    valid = true;
    if      (strcmp(arg, "debug")   == 0 || strcmp(arg, "0") == 0) newLevel = PTPIP_LOG_DEBUG;
    else if (strcmp(arg, "info")    == 0 || strcmp(arg, "1") == 0) newLevel = PTPIP_LOG_INFO;
    else if (strcmp(arg, "warning") == 0 || strcmp(arg, "2") == 0) newLevel = PTPIP_LOG_WARNING;
    else if (strcmp(arg, "error")   == 0 || strcmp(arg, "3") == 0) newLevel = PTPIP_LOG_ERROR;
    else if (strcmp(arg, "off")     == 0 || strcmp(arg, "none") == 0 ||
             strcmp(arg, "4")       == 0)                           newLevel = 4;
    else valid = false;

    if (valid) {
        _logLevel = newLevel;
        const char* names[] = { "DEBUG", "INFO", "WARNING", "ERROR", "OFF" };
        Serial.printf("Log level set to %s\r\n", newLevel < 5 ? names[newLevel] : "?");
    } else {
        _printLine("Invalid level. Use: debug/info/warning/error/off  or  0-4");
    }
}

void SerialShell::_cmdDiag() {
    if (!_active) {
        _printLine("No camera selected — use 'connect <name>' first");
        return;
    }
    _active->dumpDiagnostics(Serial);
}

void SerialShell::_cmdReboot() {
    _printLine("Rebooting...");
    delay(300);
    ESP.restart();
}

// ============================================================================
// Utilities
// ============================================================================

bool SerialShell::_matchCmd(const char* input, const char* cmd, const char* alias) {
    if (strcmp(input, cmd) == 0) return true;
    if (alias && strcmp(input, alias) == 0) return true;
    return false;
}

const char* SerialShell::_getArg(const char* input) {
    const char* space = strchr(input, ' ');
    if (!space) return nullptr;
    space++;
    while (*space == ' ') space++;
    return (*space != '\0') ? space : nullptr;
}

void SerialShell::_printLine(const char* text) {
    Serial.printf("%s\r\n", text);
}

void SerialShell::_printKV(const char* key, const char* value) {
    Serial.printf("  %-20s %s\r\n", key, value);
}

void SerialShell::_printKV(const char* key, int value) {
    Serial.printf("  %-20s %d\r\n", key, value);
}
