/**
 * ConnectAndShoot.ino — PtpIpCamera minimal example
 *
 * Demonstrates the simplest possible usage of PtpIpCamera:
 *   1. Connect to the camera's WiFi network
 *   2. Open a PTP/IP session with a Canon EOS camera
 *   3. Set aperture, shutter speed, and ISO
 *   4. Trigger a single AF + shutter capture
 *
 * Hardware: ESP32 (tested on Adafruit ESP32-S3 Feather)
 * Camera:   Canon EOS 5D Mark IV (or any Canon EOS with built-in WiFi)
 *           Camera must be in Remote Shooting / EOS Utility mode.
 */

#include <WiFi.h>
#include "PtpIpCamera.h"

// ---------------------------------------------------------------------------
// Configuration — fill in your values before uploading
// ---------------------------------------------------------------------------
static const char* WIFI_SSID  = "EOS5D4-XXXXXX";  // camera WiFi SSID
static const char* WIFI_PASS  = "";                 // leave empty for open network
static const char* CAMERA_IP  = "192.168.1.2";      // IP shown on camera WiFi screen
// ---------------------------------------------------------------------------

// PTP/IP stack: transport owns the TCP sockets, session owns transaction IDs,
// Canon5DMkIV translates the high-level API into Canon vendor opcodes.
static PtpIpTransport transport;
static PtpIpSession   session(transport);
static Canon5DMkIV    camera(session);

// Logs a result and returns true if it was CAM_OK.
static bool check(const char* label, CameraResult r) {
    Serial.print(label);
    Serial.print(": ");
    Serial.println(cameraResultStr(r));
    return r == CAM_OK;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== PtpIpCamera ConnectAndShoot ===");

    // ------------------------------------------------------------------
    // Step 1: Connect to WiFi
    // The camera creates its own WiFi access point when set to Remote
    // Shooting mode — connect the ESP32 directly to that network.
    // ------------------------------------------------------------------
    Serial.print("Connecting to WiFi: ");
    Serial.print(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" connected.");

    // ------------------------------------------------------------------
    // Step 2: Open a PTP/IP session with the camera
    // begin() opens the command TCP channel (port 15740), performs the
    // Init_Command handshake, then opens the event channel.
    // ------------------------------------------------------------------
    Serial.print("Opening PTP/IP session at ");
    Serial.println(CAMERA_IP);
    if (!check("camera.begin", camera.begin(CAMERA_IP))) {
        Serial.println("Cannot continue without a camera connection.");
        return;
    }

    // ------------------------------------------------------------------
    // Step 3: Set exposure parameters
    // Values use natural units: f-number, seconds, ISO integer.
    // The library converts these to Canon APEX wire encoding internally.
    // Only exact Canon step values are accepted — e.g. f/5.6, not f/6.
    // ------------------------------------------------------------------
    check("setAperture(f/5.6)",     camera.setAperture(5.6f));
    check("setShutterSpeed(1/100)", camera.setShutterSpeed(1.0f / 100.0f));
    check("setISO(400)",            camera.setISO(400));

    // ------------------------------------------------------------------
    // Step 4: Trigger capture
    // initiateCapture() runs autofocus then fires the shutter.
    // The shutter fires synchronously; the image write to the memory card
    // completes asynchronously — poll() picks up the completion event.
    // ------------------------------------------------------------------
    Serial.println("Triggering capture...");
    if (check("initiateCapture", camera.initiateCapture())) {
        Serial.println("Shot fired successfully.");
    } else {
        Serial.println("Capture failed — is the camera in playback mode?");
    }

    // ------------------------------------------------------------------
    // Step 5: Close the session
    // ------------------------------------------------------------------
    camera.end();
    Serial.println("Session closed. Done.");
}

void loop() {
    // Nothing to do — this is a single-shot example.
    // In a real application, call camera.poll() here at ~5 Hz to process
    // GetEventData responses, detect dial changes, and fire callbacks.
}
