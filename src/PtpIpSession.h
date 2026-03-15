#pragma once
#include "PtpIpTransport.h"
#include "PtpIpConstants.h"
#include <stdint.h>
#include <stddef.h>

// ============================================================================
// PtpIpSession
// Layer 2: PTP/IP handshake, session management, and transaction engine.
// Manages the TransactionID counter — callers never touch it directly.
// Knows about PTP/IP packet structure; does NOT know about Canon opcodes.
// ============================================================================

class PtpIpSession {
public:
    explicit PtpIpSession(PtpIpTransport& transport);

    // Execute the 4-phase PTP/IP handshake on both channels.
    // Must be called before openSession().
    bool connect(const char* host);

    void disconnect();
    bool isConnected();

    // Send OpenSession (0x1002) with TransactionID=0, SessionID=1.
    // TransactionID counter is reset so the next sendCommand uses ID=1.
    bool openSession();

    // Send a PTP command with 0, 1, or 2 parameters (no data phase).
    bool sendCommand(uint16_t opCode, uint16_t& responseCode);
    bool sendCommand(uint16_t opCode, uint16_t& responseCode, uint32_t p1);
    bool sendCommand(uint16_t opCode, uint16_t& responseCode, uint32_t p1, uint32_t p2);

    // Send a PTP command that includes a data phase (e.g. SetDevicePropValue).
    // param1 is sent as Parameter 1 of the Command Request (e.g. PropCode).
    // data/dataLen is the payload sent in the End Data packet.
    bool sendCommandWithData(uint16_t opCode, uint16_t& responseCode,
                              const uint8_t* data, size_t dataLen,
                              uint32_t param1);

    // Send a PTP command that receives a data phase from the camera (e.g. GetDevicePropValue).
    // param1 is sent as Parameter 1 of the Command Request (e.g. PropCode).
    // data/maxDataLen is the buffer for the incoming payload; dataLen is set to actual bytes received.
    bool recvCommandWithData(uint16_t opCode, uint16_t& responseCode,
                              uint8_t* data, size_t maxDataLen, size_t& dataLen,
                              uint32_t param1);

    // Canon vendor-specific SetDevicePropValue (opcode 0x9110).
    // Differs from standard sendCommandWithData in three ways:
    //   1. No Parameter 1 in the Operation Request (propCode lives in the data payload).
    //   2. Start Data uses packet type 0x09 instead of 0x0A.
    //   3. Data payload is always 12 bytes: [payloadLen=12][propCode as uint32][value as uint32].
    // propCode: Canon vendor property code (e.g. 0xd101 aperture, 0xd102 shutter, 0xd103 ISO).
    // value:    APEX-encoded byte value zero-extended to uint32.
    bool sendCanonSetProp(uint16_t propCode, uint32_t value, uint16_t& responseCode);

    // Canon GetEventData (opcode 0x9116): poll the camera's internal event queue.
    // Sends an Op Request with DataPhaseInfo=1 (no params).
    // Camera responds with Canon Start Data (type 0x09) + End Data carrying a
    // concatenated list of event segments, followed by the Command Response.
    // buf/maxLen: caller-supplied buffer (allocate 16384 bytes to be safe).
    // On success returns true and sets dataLen to the number of payload bytes received.
    bool getCanonEventData(uint8_t* buf, size_t maxLen, size_t& dataLen);

    // Non-blocking poll for an event packet on the command channel.
    // Canon sends capture events on the command channel, not the event channel.
    // Returns true and fills eventCode + param1 if a full event packet is ready.
    bool pollEvent(uint16_t& eventCode, uint32_t& param1);

private:
    PtpIpTransport& _transport;
    uint32_t        _transactionId;
    bool            _connected;
    bool            _sessionOpen;

    // Handshake steps
    bool _sendInitCommandRequest();
    bool _recvInitCommandAck(uint32_t& connectionNumber);
    bool _sendInitEventRequest(uint32_t connectionNumber);
    bool _recvInitEventAck();

    // Internal command sender used by all public sendCommand overloads.
    bool _sendCmd(uint16_t opCode, uint16_t& responseCode,
                  const uint32_t* params, uint8_t numParams);

    // Read a complete PTP/IP packet from the command or event channel.
    // Reads the 8-byte header first, then the remaining payload.
    // Returns total bytes read, or 0 on error.
    size_t _recvPacketCommand(uint8_t* buf, size_t maxLen, uint32_t timeoutMs = 5000);
    size_t _recvPacketEvent(uint8_t* buf, size_t maxLen, uint32_t timeoutMs = 5000);

    // Parse a Command Response packet and extract the response code.
    bool _parseResponsePkt(const uint8_t* buf, size_t len, uint16_t& responseCode);

    // Increment and return the transaction ID.
    // Returns 1 on the first call after openSession().
    uint32_t _nextTransactionId();
};
