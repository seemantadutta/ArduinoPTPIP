#pragma once
#include <WiFiClient.h>
#include <stdint.h>
#include <stddef.h>

// ============================================================================
// PtpIpTransport
// Layer 1: Raw TCP socket management.
// Owns two WiFiClient sockets: command channel and event channel.
// Provides raw send/receive with no protocol awareness.
// All callers pass flat byte buffers — no structs cross this boundary.
// ============================================================================

class PtpIpTransport {
public:
    // Open only the command channel. Call first.
    bool connectCommand(const char* host, uint16_t port = 15740);

    // Open only the event channel. Call after Init_Command_Ack is received.
    bool connectEvent(const char* host, uint16_t port = 15740);

    void disconnect();
    bool isConnected();

    // Write exactly len bytes to the command channel.
    bool sendCommand(const uint8_t* buf, size_t len);

    // Write exactly len bytes to the event channel.
    bool sendEvent(const uint8_t* buf, size_t len);

    // Block until exactly len bytes arrive on the command channel,
    // or until timeoutMs elapses. Returns actual bytes read.
    size_t recvCommand(uint8_t* buf, size_t len, uint32_t timeoutMs = 5000);

    // Block until exactly len bytes arrive on the event channel,
    // or until timeoutMs elapses. Returns actual bytes read.
    size_t recvEvent(uint8_t* buf, size_t len, uint32_t timeoutMs = 5000);

    // Non-blocking: returns number of bytes available on the command channel.
    int availableCommand();

private:
    WiFiClient _cmdSocket;
    WiFiClient _evtSocket;

    // Loop-read until exactly len bytes are received or timeout fires.
    size_t _recvFull(WiFiClient& sock, uint8_t* buf, size_t len, uint32_t timeoutMs);
};
