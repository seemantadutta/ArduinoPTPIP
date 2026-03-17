/**
 * SerialShell.ino — PtpIpCamera interactive test harness
 *
 * Provides a line-based serial command interface for testing all library
 * features against a real Canon EOS camera over WiFi.
 *
 * Open a serial monitor at 115200 baud and type 'help' for a full command list.
 *
 * Key commands:
 *   connect 5d4      — join camera WiFi and open PTP/IP session
 *   ap 5.6           — set aperture to f/5.6
 *   ss 1/100         — set shutter speed to 1/100 s
 *   iso 400          — set ISO 400
 *   shoot            — AF + shutter release
 *   s                — read current settings from camera
 *   aeb 3 1          — 3-shot AEB sequence at 1-stop intervals
 *   ec +1            — set exposure compensation to +1 EV
 *   d                — dump diagnostic ring buffer
 *   log debug        — set log verbosity to DEBUG
 *
 * Hardware: ESP32 (tested on Adafruit ESP32-S3 Feather)
 * Camera:   Canon EOS 5D Mark IV (or any Canon EOS with built-in WiFi)
 *           Camera must be in Remote Shooting / EOS Utility mode.
 */

#include <Arduino.h>
#include <WiFi.h>

#include "PtpIpCamera.h"
#define LOG_TAG "MAIN"

#include "SerialShell.h"

// ---------------------------------------------------------------------------
// Configuration — fill in your values before uploading
// ---------------------------------------------------------------------------
static const char* CANON_5D4_SSID = "EOS5D4-XXXXXX";  // camera WiFi SSID
static const char* CANON_5D4_PASS = "";                 // leave empty for open network
static const char* CANON_5D4_IP   = "192.168.1.2";      // IP shown on camera WiFi screen
// ---------------------------------------------------------------------------

static PtpIpTransport transport;
static PtpIpSession   session(transport);
static Canon5DMkIV    camera(session);
static SerialShell    shell;

// Log callback: filters by shell log level, formats with timestamp and level prefix.
static void logCallback(uint8_t level, const char* msg) {
    if (level < shell.getLogLevel()) return;
    const char* names[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
    uint32_t ms = millis();
    Serial.printf("[%lu.%03lu][%s] %s\r\n",
                  ms / 1000, ms % 1000,
                  level < 4 ? names[level] : "???",
                  msg);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    PtpIpSetLogCallback(logCallback);
    LOG_INFO("PtpIpCamera SerialShell starting");

    // Register cameras. 'connect <name>' handles WiFi switching and PTP
    // session setup for whichever camera is selected.
    shell.registerCamera("5d4", CANON_5D4_SSID, CANON_5D4_PASS, CANON_5D4_IP, &camera);

    shell.begin();
}

void loop() {
    shell.update();
}
