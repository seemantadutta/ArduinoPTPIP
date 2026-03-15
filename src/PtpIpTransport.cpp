#include "PtpIpTransport.h"
#include "PtpIpLog.h"
#include <Arduino.h>

#define LOG_TAG "TRANS"

bool PtpIpTransport::connectCommand(const char* host, uint16_t port) {
    LOG_DEBUG("Connecting command channel to %s:%d", host, port);
    if (!_cmdSocket.connect(host, port)) {
        LOG_ERROR("Command channel connect failed");
        return false;
    }
    LOG_DEBUG("Command channel connected");
    return true;
}

bool PtpIpTransport::connectEvent(const char* host, uint16_t port) {
    LOG_DEBUG("Connecting event channel to %s:%d", host, port);
    if (!_evtSocket.connect(host, port)) {
        LOG_ERROR("Event channel connect failed");
        return false;
    }
    LOG_DEBUG("Event channel connected");
    return true;
}

void PtpIpTransport::disconnect() {
    _cmdSocket.stop();
    _evtSocket.stop();
    LOG_INFO("Disconnected");
}

bool PtpIpTransport::isConnected() {
    return _cmdSocket.connected() && _evtSocket.connected();
}

bool PtpIpTransport::sendCommand(const uint8_t* buf, size_t len) {
    size_t written = _cmdSocket.write(buf, len);
    if (written != len) {
        LOG_ERROR("sendCommand: wrote %d of %d bytes", (int)written, (int)len);
        return false;
    }
    return true;
}

bool PtpIpTransport::sendEvent(const uint8_t* buf, size_t len) {
    size_t written = _evtSocket.write(buf, len);
    if (written != len) {
        LOG_ERROR("sendEvent: wrote %d of %d bytes", (int)written, (int)len);
        return false;
    }
    return true;
}

size_t PtpIpTransport::recvCommand(uint8_t* buf, size_t len, uint32_t timeoutMs) {
    return _recvFull(_cmdSocket, buf, len, timeoutMs);
}

size_t PtpIpTransport::recvEvent(uint8_t* buf, size_t len, uint32_t timeoutMs) {
    return _recvFull(_evtSocket, buf, len, timeoutMs);
}

int PtpIpTransport::availableCommand() {
    return _cmdSocket.available();
}

size_t PtpIpTransport::_recvFull(WiFiClient& sock, uint8_t* buf, size_t len, uint32_t timeoutMs) {
    size_t received = 0;
    uint32_t start = millis();

    while (received < len) {
        if (millis() - start > timeoutMs) {
            LOG_ERROR("recvFull timeout after %ums (got %d of %d bytes)",
                      (unsigned)timeoutMs, (int)received, (int)len);
            break;
        }
        if (sock.available()) {
            int n = sock.read(buf + received, len - received);
            if (n > 0) received += (size_t)n;
        } else {
            delay(1);
        }
    }

    return received;
}
