#include "PtpIpSession.h"
#include "PtpIpLog.h"
#include "PtpIpDiag.h"
#include <Arduino.h>
#include <string.h>

#define LOG_TAG "SESSION"

// ============================================================================
// Little-Endian wire helpers (no alignment assumptions)
// ============================================================================

static inline void writeU8(uint8_t* buf, size_t& off, uint8_t val) {
    buf[off++] = val;
}

static inline void writeU16LE(uint8_t* buf, size_t& off, uint16_t val) {
    buf[off++] = (uint8_t)(val & 0xFF);
    buf[off++] = (uint8_t)((val >> 8) & 0xFF);
}

static inline void writeU32LE(uint8_t* buf, size_t& off, uint32_t val) {
    buf[off++] = (uint8_t)(val & 0xFF);
    buf[off++] = (uint8_t)((val >> 8) & 0xFF);
    buf[off++] = (uint8_t)((val >> 16) & 0xFF);
    buf[off++] = (uint8_t)((val >> 24) & 0xFF);
}

static inline void writeU64LE(uint8_t* buf, size_t& off, uint64_t val) {
    for (int i = 0; i < 8; i++) {
        buf[off++] = (uint8_t)((val >> (i * 8)) & 0xFF);
    }
}

static inline uint16_t readU16LE(const uint8_t* buf, size_t off) {
    return (uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8);
}

static inline uint32_t readU32LE(const uint8_t* buf, size_t off) {
    return (uint32_t)buf[off]
         | ((uint32_t)buf[off + 1] << 8)
         | ((uint32_t)buf[off + 2] << 16)
         | ((uint32_t)buf[off + 3] << 24);
}

// Hex dump — compiled in only when PTPIP_HEX_DUMP is defined.
// Add -D PTPIP_HEX_DUMP to platformio.ini build_flags to enable.
#ifdef PTPIP_HEX_DUMP
static void logHexDump(const char* label, const uint8_t* buf, size_t len) {
    Serial.printf("[%s] %u bytes:\r\n", label, (unsigned)len);
    for (size_t i = 0; i < len; i += 16) {
        Serial.printf("  %04X: ", (unsigned)i);
        for (size_t j = 0; j < 16; j++) {
            if (i + j < len) Serial.printf("%02X ", buf[i + j]);
            else             Serial.printf("   ");
            if (j == 7)      Serial.printf(" ");
        }
        Serial.printf("\r\n");
    }
}
#else
#define logHexDump(label, buf, len)  // compiled out
#endif

// ============================================================================
// PtpIpSession
// ============================================================================

PtpIpSession::PtpIpSession(PtpIpTransport& transport)
    : _transport(transport)
    , _transactionId(0)
    , _connected(false)
    , _sessionOpen(false)
{}

bool PtpIpSession::connect(const char* host) {
    // Step 1: open command channel only
    if (!_transport.connectCommand(host, PTPIP_PORT)) {
        return false;
    }

    // Step 2: send Init_Command_Request on command channel
    if (!_sendInitCommandRequest()) {
        _transport.disconnect();
        return false;
    }

    // Step 3: wait for Init_Command_Ack and extract connection number
    uint32_t connectionNumber = 0;
    if (!_recvInitCommandAck(connectionNumber)) {
        _transport.disconnect();
        return false;
    }

    // Step 4: now open event channel (camera can now link it via connection number)
    if (!_transport.connectEvent(host, PTPIP_PORT)) {
        _transport.disconnect();
        return false;
    }

    // Step 5: send Init_Event_Request with the connection number
    if (!_sendInitEventRequest(connectionNumber)) {
        _transport.disconnect();
        return false;
    }

    // Step 6: wait for Init_Event_Ack
    if (!_recvInitEventAck()) {
        _transport.disconnect();
        return false;
    }

    _connected = true;
    LOG_INFO("PTP/IP handshake complete");
    return true;
}

void PtpIpSession::disconnect() {
    _transport.disconnect();
    _connected    = false;
    _sessionOpen  = false;
    _transactionId = 0;
    LOG_INFO("Session disconnected");
}

bool PtpIpSession::isConnected() {
    return _connected && _transport.isConnected();
}

// ----------------------------------------------------------------------------
// Handshake: Phase 1 — Init_Command_Request (type 0x01)
// Packet layout:
//   [0..3]   length (uint32 LE) — total packet size
//   [4..7]   type   (uint32 LE) = 0x00000001
//   [8..23]  GUID   (16 bytes)
//   [24..]   Client name in UTF-16LE, null-terminated (no length prefix)
//   [..]     PTP version (uint32 LE) = 0x00010000
// ----------------------------------------------------------------------------
bool PtpIpSession::_sendInitCommandRequest() {
    static const uint8_t guid[16] = {
        0x4d, 0xc4, 0x79, 0xe1, 0xc1, 0x0e, 0x46, 0x27,
        0x9e, 0xe1, 0x1b, 0x2a, 0xbc, 0xe2, 0xda, 0x29
    };

    // 0x0a, 0x39, 0xff, 0x54, 0x80, 0x3b, 0xb1, 0x45,
    // 0x9a, 0xe6, 0x76, 0x6b, 0xf1, 0x60, 0x31, 0x44

    const char* clientName = "ESP-32";
    uint8_t numChars = (uint8_t)(strlen(clientName) + 1);  // +1 for null terminator

    // Total: 8 (header) + 16 (GUID) + numChars*2 (UTF-16LE incl. null) + 4 (version)
    uint32_t totalLen = 8 + 16 + ((uint32_t)numChars * 2) + 4;

    uint8_t buf[128];
    size_t off = 0;

    writeU32LE(buf, off, totalLen);
    writeU32LE(buf, off, PTPIP_PKT_INIT_CMD_REQ);
    memcpy(buf + off, guid, 16);
    off += 16;

    // Client name as UTF-16LE, null-terminated
    for (int i = 0; i < numChars - 1; i++) {
        writeU16LE(buf, off, (uint16_t)clientName[i]);
    }
    writeU16LE(buf, off, 0x0000);  // null terminator

    writeU32LE(buf, off, PTP_VERSION);

    logHexDump("SEND Init_Command_Request", buf, off);
    return _transport.sendCommand(buf, off);
}

// ----------------------------------------------------------------------------
// Handshake: Phase 2 — Init_Command_Ack (type 0x02)
// Connection number is at offset 8 of the packet.
// ----------------------------------------------------------------------------
bool PtpIpSession::_recvInitCommandAck(uint32_t& connectionNumber) {
    uint8_t buf[64];
    size_t n = _recvPacketCommand(buf, sizeof(buf));
    if (n < 8) {
        LOG_ERROR("Init_Command_Ack too short: %u bytes", (unsigned)n);
        return false;
    }

    uint32_t type = readU32LE(buf, 4);

    // Init_Fail (0x05): camera rejected the connection
    if (type == 0x00000005) {
        uint32_t errorCode = (n >= 12) ? readU32LE(buf, 8) : 0;
        const char* reason = "unknown";
        if      (errorCode == 0x00000001) reason = "Rejected — camera may need to be in EOS Utility waiting mode";
        else if (errorCode == 0x00000002) reason = "Busy — camera is already connected to another client";
        else if (errorCode == 0x00000003) reason = "Unspecified error";
        LOG_ERROR("Camera sent Init_Fail (errorCode=0x%08X): %s", (unsigned)errorCode, reason);
        return false;
    }

    if (type != PTPIP_PKT_INIT_CMD_ACK) {
        LOG_ERROR("Expected Init_Command_Ack (0x02), got 0x%08X", (unsigned)type);
        return false;
    }

    if (n < 12) {
        LOG_ERROR("Init_Command_Ack too short for connection number: %u bytes", (unsigned)n);
        return false;
    }

    logHexDump("RECV Init_Command_Ack", buf, n);
    connectionNumber = readU32LE(buf, 8);
    LOG_DEBUG("Init_Command_Ack received, connection number: %u", (unsigned)connectionNumber);
    return true;
}

// ----------------------------------------------------------------------------
// Handshake: Phase 3 — Init_Event_Request (type 0x03)
// Packet: 8 (header) + 4 (connection number) = 12 bytes
// ----------------------------------------------------------------------------
bool PtpIpSession::_sendInitEventRequest(uint32_t connectionNumber) {
    uint8_t buf[12];
    size_t off = 0;
    writeU32LE(buf, off, 12);
    writeU32LE(buf, off, PTPIP_PKT_INIT_EVT_REQ);
    writeU32LE(buf, off, connectionNumber);

    logHexDump("SEND Init_Event_Request", buf, off);
    return _transport.sendEvent(buf, off);
}

// ----------------------------------------------------------------------------
// Handshake: Phase 4 — Init_Event_Ack (type 0x04)
// ----------------------------------------------------------------------------
bool PtpIpSession::_recvInitEventAck() {
    uint8_t buf[32];
    size_t n = _recvPacketEvent(buf, sizeof(buf));
    if (n < 8) {
        LOG_ERROR("Init_Event_Ack too short: %u bytes", (unsigned)n);
        return false;
    }

    uint32_t type = readU32LE(buf, 4);
    if (type != PTPIP_PKT_INIT_EVT_ACK) {
        LOG_ERROR("Expected Init_Event_Ack (0x04), got 0x%08X", (unsigned)type);
        return false;
    }

    logHexDump("RECV Init_Event_Ack", buf, n);
    LOG_DEBUG("Init_Event_Ack received — handshake complete");
    return true;
}

// ----------------------------------------------------------------------------
// OpenSession (0x1002)
// Special: TransactionID MUST be 0. After this, counter resets to 0 so
// the first _nextTransactionId() call returns 1.
// Packet layout (Command Request with 1 param):
//   [0..3]   length (uint32 LE)
//   [4..7]   type   (uint32 LE) = 0x06
//   [8..11]  DataPhaseInfo (uint32 LE) = 1 (no data)
//   [12..13] OperationCode (uint16 LE) = 0x1002
//   [14..17] TransactionID (uint32 LE) = 0x00000000
//   [18..21] Parameter1 = SessionID = 0x00000001
// ----------------------------------------------------------------------------
bool PtpIpSession::openSession() {
    if (!_connected) {
        LOG_ERROR("openSession: not connected");
        return false;
    }

    uint8_t buf[32];
    size_t off = 0;

    writeU32LE(buf, off, 22);                    // length = 22 bytes
    writeU32LE(buf, off, PTPIP_PKT_CMD_REQUEST);
    writeU32LE(buf, off, 1);                     // DataPhaseInfo = no data
    writeU16LE(buf, off, PTP_OP_OPEN_SESSION);
    writeU32LE(buf, off, PTP_TXID_OPEN_SESSION); // TransactionID = 0
    writeU32LE(buf, off, PTP_SESSION_ID);        // Parameter1 = SessionID

    LOG_DEBUG("Sending OpenSession (txId=0)");
    logHexDump("SEND OpenSession", buf, off);

    if (!_transport.sendCommand(buf, off)) {
        LOG_ERROR("Failed to send OpenSession");
        return false;
    }

    uint8_t resp[32];
    size_t n = _recvPacketCommand(resp, sizeof(resp));
    if (n == 0) return false;

    logHexDump("RECV OpenSession_Response", resp, n);
    uint16_t responseCode;
    if (!_parseResponsePkt(resp, n, responseCode)) return false;

    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("OpenSession failed: 0x%04X", responseCode);
        return false;
    }

    _transactionId = 0;   // _nextTransactionId() will return 1 on next call
    _sessionOpen   = true;
    LOG_INFO("Session opened (SessionID=%u)", (unsigned)PTP_SESSION_ID);
    return true;
}

// ----------------------------------------------------------------------------
// Public sendCommand overloads — delegate to _sendCmd
// ----------------------------------------------------------------------------
bool PtpIpSession::sendCommand(uint16_t opCode, uint16_t& responseCode) {
    return _sendCmd(opCode, responseCode, nullptr, 0);
}

bool PtpIpSession::sendCommand(uint16_t opCode, uint16_t& responseCode, uint32_t p1) {
    uint32_t params[1] = { p1 };
    return _sendCmd(opCode, responseCode, params, 1);
}

bool PtpIpSession::sendCommand(uint16_t opCode, uint16_t& responseCode,
                                uint32_t p1, uint32_t p2) {
    uint32_t params[2] = { p1, p2 };
    return _sendCmd(opCode, responseCode, params, 2);
}

// ----------------------------------------------------------------------------
// _sendCmd — internal implementation
// Command Request packet layout:
//   [0..3]   length
//   [4..7]   type = 0x06
//   [8..11]  DataPhaseInfo = 1 (no data)
//   [12..13] OperationCode
//   [14..17] TransactionID
//   [18..]   Parameters (4 bytes each)
// ----------------------------------------------------------------------------
bool PtpIpSession::_sendCmd(uint16_t opCode, uint16_t& responseCode,
                              const uint32_t* params, uint8_t numParams) {
    if (!_sessionOpen) {
        LOG_ERROR("sendCommand: session not open");
        return false;
    }

    uint32_t txId = _nextTransactionId();
    // Base: 8 (header) + 4 (dataPhase) + 2 (opCode) + 4 (transID) = 18
    uint32_t totalLen = 18 + (uint32_t)numParams * 4;

    uint8_t buf[64];
    size_t off = 0;

    writeU32LE(buf, off, totalLen);
    writeU32LE(buf, off, PTPIP_PKT_CMD_REQUEST);
    writeU32LE(buf, off, 1);        // DataPhaseInfo = no data
    writeU16LE(buf, off, opCode);
    writeU32LE(buf, off, txId);
    for (int i = 0; i < numParams; i++) {
        writeU32LE(buf, off, params[i]);
    }

    LOG_DEBUG("sendCommand: opCode=0x%04X txId=%u params=%d", opCode, (unsigned)txId, numParams);
    PtpIpDiagInstance().record(DIAG_EVT_OP_SENT, (uint8_t)(opCode >> 8), (uint8_t)(opCode & 0xFF), numParams);
    logHexDump("SEND Command_Request", buf, off);

    if (!_transport.sendCommand(buf, off)) {
        LOG_ERROR("sendCommand: failed to send packet");
        return false;
    }

    uint8_t resp[64];
    size_t n = _recvPacketCommand(resp, sizeof(resp));
    if (n == 0) return false;

    logHexDump("RECV Command_Response", resp, n);
    if (!_parseResponsePkt(resp, n, responseCode)) return false;

    PtpIpDiagInstance().record(DIAG_EVT_RESP_RECV, (uint8_t)(responseCode >> 8), (uint8_t)(responseCode & 0xFF), 0);
    LOG_DEBUG("sendCommand: response=0x%04X", responseCode);
    return true;
}

// ----------------------------------------------------------------------------
// sendCommandWithData — Command Request + Start Data + End Data, then Response
// Used for SetDevicePropValue (0x1016) and similar.
//
// Command Request packet layout (dataPhase=2, 1 param):
//   [0..3]   length = 22
//   [4..7]   type = 0x06
//   [8..11]  DataPhaseInfo = 2 (data out: initiator -> responder)
//   [12..13] OperationCode
//   [14..17] TransactionID
//   [18..21] param1 (e.g. PropCode)
//
// Start Data packet (type 0x0A):
//   [0..3]   length = 20
//   [4..7]   type = 0x0A
//   [8..11]  TransactionID
//   [12..19] TotalDataLength (uint64 LE)
//
// End Data packet (type 0x0C):
//   [0..3]   length = 12 + dataLen
//   [4..7]   type = 0x0C
//   [8..11]  TransactionID
//   [12..]   data payload
// ----------------------------------------------------------------------------
bool PtpIpSession::sendCommandWithData(uint16_t opCode, uint16_t& responseCode,
                                        const uint8_t* data, size_t dataLen,
                                        uint32_t param1) {
    if (!_sessionOpen) {
        LOG_ERROR("sendCommandWithData: session not open");
        return false;
    }

    uint32_t txId = _nextTransactionId();

    // --- Command Request ---
    {
        uint8_t buf[32];
        size_t off = 0;
        writeU32LE(buf, off, 22);
        writeU32LE(buf, off, PTPIP_PKT_CMD_REQUEST);
        writeU32LE(buf, off, 2);        // DataPhaseInfo = data out
        writeU16LE(buf, off, opCode);
        writeU32LE(buf, off, txId);
        writeU32LE(buf, off, param1);

        logHexDump("SEND Command_Request(data)", buf, off);
        if (!_transport.sendCommand(buf, off)) {
            LOG_ERROR("sendCommandWithData: failed to send command request");
            return false;
        }
    }

    // --- Start Data ---
    {
        uint8_t buf[20];
        size_t off = 0;
        writeU32LE(buf, off, 20);
        writeU32LE(buf, off, PTPIP_PKT_DATA_START);
        writeU32LE(buf, off, txId);
        writeU64LE(buf, off, (uint64_t)dataLen);

        logHexDump("SEND Data_Start", buf, off);
        if (!_transport.sendCommand(buf, off)) {
            LOG_ERROR("sendCommandWithData: failed to send data start");
            return false;
        }
    }

    // --- End Data ---
    {
        size_t totalLen = 12 + dataLen;
        uint8_t buf[32];   // dataLen is at most 4 bytes for our use cases
        size_t off = 0;
        writeU32LE(buf, off, (uint32_t)totalLen);
        writeU32LE(buf, off, PTPIP_PKT_DATA_END);
        writeU32LE(buf, off, txId);
        memcpy(buf + off, data, dataLen);
        off += dataLen;

        logHexDump("SEND Data_End", buf, off);
        if (!_transport.sendCommand(buf, off)) {
            LOG_ERROR("sendCommandWithData: failed to send data end");
            return false;
        }
    }

    // --- Response ---
    uint8_t resp[32];
    size_t n = _recvPacketCommand(resp, sizeof(resp));
    if (n == 0) return false;

    logHexDump("RECV Command_Response(data)", resp, n);
    if (!_parseResponsePkt(resp, n, responseCode)) return false;

    PtpIpDiagInstance().record(DIAG_EVT_OP_SENT, (uint8_t)(opCode >> 8), (uint8_t)(opCode & 0xFF), 1);
    PtpIpDiagInstance().record(DIAG_EVT_RESP_RECV, (uint8_t)(responseCode >> 8), (uint8_t)(responseCode & 0xFF), 0);
    LOG_DEBUG("sendCommandWithData: opCode=0x%04X txId=%u response=0x%04X",
              opCode, (unsigned)txId, responseCode);
    return true;
}

// ----------------------------------------------------------------------------
// sendCanonSetProp — Canon vendor-specific property write (opcode 0x9110)
//
// Packet 1: Operation Request — 18 bytes, NO parameters
//   [0..3]   length = 18
//   [4..7]   type = 0x06
//   [8..11]  DataPhaseInfo = 2 (data out)
//   [12..13] OperationCode = 0x9110
//   [14..17] TransactionID
//
// Packet 2: Start Data — 20 bytes, Canon uses type 0x09 (not 0x0A)
//   [0..3]   length = 20
//   [4..7]   type = 0x09
//   [8..11]  TransactionID
//   [12..19] TotalDataLength = 12 (uint64 LE)
//
// Packet 3: End Data — 24 bytes (12 header + 12 payload)
//   [0..3]   length = 24
//   [4..7]   type = 0x0C
//   [8..11]  TransactionID
//   [12..15] Internal payload length = 12
//   [16..19] Property Code (uint32 LE, e.g. 0xd102 -> 02 d1 00 00)
//   [20..23] Property Value (uint32 LE)
// ----------------------------------------------------------------------------
bool PtpIpSession::sendCanonSetProp(uint16_t propCode, uint32_t value,
                                     uint16_t& responseCode) {
    if (!_sessionOpen) {
        LOG_ERROR("sendCanonSetProp: session not open");
        return false;
    }

    uint32_t txId = _nextTransactionId();

    // --- Operation Request (no parameters) ---
    {
        uint8_t buf[18];
        size_t off = 0;
        writeU32LE(buf, off, 18);
        writeU32LE(buf, off, PTPIP_PKT_CMD_REQUEST);
        writeU32LE(buf, off, 2);                    // DataPhaseInfo = data out
        writeU16LE(buf, off, PTP_OP_CANON_SET_PROP);
        writeU32LE(buf, off, txId);

        logHexDump("SEND Canon_SetProp_Request", buf, off);
        if (!_transport.sendCommand(buf, off)) {
            LOG_ERROR("sendCanonSetProp: failed to send operation request");
            return false;
        }
    }

    // --- Start Data (type 0x09 — Canon vendor quirk) ---
    {
        uint8_t buf[20];
        size_t off = 0;
        writeU32LE(buf, off, 20);
        writeU32LE(buf, off, PTPIP_PKT_CANON_DATA_START);  // 0x09
        writeU32LE(buf, off, txId);
        writeU64LE(buf, off, 12);                           // payload is always 12 bytes

        logHexDump("SEND Canon_SetProp_DataStart", buf, off);
        if (!_transport.sendCommand(buf, off)) {
            LOG_ERROR("sendCanonSetProp: failed to send data start");
            return false;
        }
    }

    // --- End Data (12-byte Canon payload) ---
    {
        uint8_t buf[24];
        size_t off = 0;
        writeU32LE(buf, off, 24);               // 12 (ptp/ip header) + 12 (payload)
        writeU32LE(buf, off, PTPIP_PKT_DATA_END);
        writeU32LE(buf, off, txId);
        writeU32LE(buf, off, 12);               // internal payload length field
        writeU32LE(buf, off, (uint32_t)propCode); // property code zero-extended to 4 bytes
        writeU32LE(buf, off, value);            // APEX value zero-extended to 4 bytes

        logHexDump("SEND Canon_SetProp_DataEnd", buf, off);
        if (!_transport.sendCommand(buf, off)) {
            LOG_ERROR("sendCanonSetProp: failed to send data end");
            return false;
        }
    }

    // --- Response ---
    {
        uint8_t resp[32];
        size_t n = _recvPacketCommand(resp, sizeof(resp));
        if (n == 0) return false;

        logHexDump("RECV Canon_SetProp_Response", resp, n);
        if (!_parseResponsePkt(resp, n, responseCode)) return false;

        PtpIpDiagInstance().record(DIAG_EVT_OP_SENT, (uint8_t)(PTP_OP_CANON_SET_PROP >> 8), (uint8_t)(PTP_OP_CANON_SET_PROP & 0xFF), 0);
        PtpIpDiagInstance().record(DIAG_EVT_RESP_RECV, (uint8_t)(responseCode >> 8), (uint8_t)(responseCode & 0xFF), 0);
        LOG_DEBUG("sendCanonSetProp: propCode=0x%04X value=0x%02X response=0x%04X",
                  propCode, (unsigned)value, responseCode);
    }

    return true;
}

// ----------------------------------------------------------------------------
// recvCommandWithData — Command Request (data in) + recv Start Data + End Data + Response
// Used for GetDevicePropValue (0x1015) and similar.
//
// Command Request packet layout (dataPhase=3 = data in: responder -> initiator):
//   [0..3]   length = 22
//   [4..7]   type = 0x06
//   [8..11]  DataPhaseInfo = 3
//   [12..13] OperationCode
//   [14..17] TransactionID
//   [18..21] param1 (e.g. PropCode)
//
// Start Data packet (type 0x0A):
//   [0..3]   length = 20
//   [4..7]   type = 0x0A
//   [8..11]  TransactionID
//   [12..19] TotalDataLength (uint64 LE)
//
// End Data packet (type 0x0C):
//   [0..3]   length = 12 + dataLen
//   [4..7]   type = 0x0C
//   [8..11]  TransactionID
//   [12..]   data payload
// ----------------------------------------------------------------------------
bool PtpIpSession::recvCommandWithData(uint16_t opCode, uint16_t& responseCode,
                                        uint8_t* data, size_t maxDataLen, size_t& dataLen,
                                        uint32_t param1) {
    if (!_sessionOpen) {
        LOG_ERROR("recvCommandWithData: session not open");
        return false;
    }

    dataLen = 0;
    uint32_t txId = _nextTransactionId();

    // --- Command Request (DataPhaseInfo=3: data flows from camera to us) ---
    {
        uint8_t buf[32];
        size_t off = 0;
        writeU32LE(buf, off, 22);
        writeU32LE(buf, off, PTPIP_PKT_CMD_REQUEST);
        writeU32LE(buf, off, 3);        // DataPhaseInfo = data in
        writeU16LE(buf, off, opCode);
        writeU32LE(buf, off, txId);
        writeU32LE(buf, off, param1);

        logHexDump("SEND Command_Request(recv)", buf, off);
        if (!_transport.sendCommand(buf, off)) {
            LOG_ERROR("recvCommandWithData: failed to send command request");
            return false;
        }
    }

    // --- Start Data ---
    {
        uint8_t buf[32];
        size_t n = _recvPacketCommand(buf, sizeof(buf));
        if (n < 20) {
            LOG_ERROR("recvCommandWithData: Start_Data too short (%u bytes)", (unsigned)n);
            return false;
        }
        logHexDump("RECV Data_Start", buf, n);

        uint32_t type = readU32LE(buf, 4);
        if (type != PTPIP_PKT_DATA_START) {
            LOG_ERROR("recvCommandWithData: expected Start_Data (0x0A), got 0x%08X", (unsigned)type);
            return false;
        }
        // totalDataLength is at [12..19] (uint64 LE) — only low 32 bits needed for small props
        uint32_t totalDataLen = readU32LE(buf, 12);
        LOG_DEBUG("recvCommandWithData: totalDataLen=%u", (unsigned)totalDataLen);
    }

    // --- End Data ---
    {
        uint8_t buf[64];
        size_t n = _recvPacketCommand(buf, sizeof(buf));
        if (n < 12) {
            LOG_ERROR("recvCommandWithData: End_Data too short (%u bytes)", (unsigned)n);
            return false;
        }
        logHexDump("RECV Data_End", buf, n);

        uint32_t type = readU32LE(buf, 4);
        if (type != PTPIP_PKT_DATA_END) {
            LOG_ERROR("recvCommandWithData: expected End_Data (0x0C), got 0x%08X", (unsigned)type);
            return false;
        }

        size_t payloadLen = n - 12;
        dataLen = payloadLen < maxDataLen ? payloadLen : maxDataLen;
        if (dataLen > 0) {
            memcpy(data, buf + 12, dataLen);
        }
    }

    // --- Response ---
    {
        uint8_t resp[32];
        size_t n = _recvPacketCommand(resp, sizeof(resp));
        if (n == 0) return false;

        logHexDump("RECV Command_Response(recv)", resp, n);
        if (!_parseResponsePkt(resp, n, responseCode)) return false;

        LOG_DEBUG("recvCommandWithData: opCode=0x%04X txId=%u response=0x%04X dataLen=%u",
                  opCode, (unsigned)txId, responseCode, (unsigned)dataLen);
    }

    return true;
}

// ----------------------------------------------------------------------------
// getCanonEventData — Canon GetEventData (0x9116)
//
// Canon's approach:
//   Host sends Op Request with DataPhaseInfo=1 (no params) — an 18-byte packet.
//   Camera responds with Start Data (type 0x09, Canon quirk) then End Data
//   carrying all queued events as a flat byte stream, followed by a Response.
//
// On the initial call after SetEventMode the camera sends ~11KB of events
// (full camera state dump).  Subsequent calls return 16-32 bytes.
// The caller must provide a buffer of at least 16384 bytes.
// ----------------------------------------------------------------------------
bool PtpIpSession::getCanonEventData(uint8_t* buf, size_t maxLen, size_t& dataLen) {
    if (!_sessionOpen) {
        LOG_ERROR("getCanonEventData: session not open");
        return false;
    }

    dataLen = 0;
    uint32_t txId = _nextTransactionId();

    // --- Operation Request: 18 bytes, no params, DataPhaseInfo=1 ---
    {
        uint8_t pkt[18];
        size_t off = 0;
        writeU32LE(pkt, off, 18);
        writeU32LE(pkt, off, PTPIP_PKT_CMD_REQUEST);
        writeU32LE(pkt, off, 1);   // DataPhaseInfo=1 (no data) — Canon still sends data back
        writeU16LE(pkt, off, PTP_OP_CANON_GET_EVENT);
        writeU32LE(pkt, off, txId);

        logHexDump("SEND Canon_GetEvent_Request", pkt, off);
        if (!_transport.sendCommand(pkt, off)) {
            LOG_ERROR("getCanonEventData: failed to send Op Request");
            return false;
        }
    }

    // --- Start Data (Canon uses type 0x09) ---
    {
        uint8_t hdr[32];
        size_t n = _recvPacketCommand(hdr, sizeof(hdr));
        if (n < 20) {
            LOG_ERROR("getCanonEventData: Start_Data too short (%u bytes)", (unsigned)n);
            return false;
        }
        uint32_t type = readU32LE(hdr, 4);
        if (type != PTPIP_PKT_CANON_DATA_START && type != PTPIP_PKT_DATA_START) {
            LOG_ERROR("getCanonEventData: expected Start_Data, got 0x%08X", (unsigned)type);
            return false;
        }
        uint32_t announced = readU32LE(hdr, 12);  // low 32 bits of TotalDataLength
        LOG_DEBUG("getCanonEventData: Start_Data received, announced %u bytes", (unsigned)announced);
    }

    // --- End Data — read header manually then payload straight into caller's buf ---
    // Packet layout: [length(4)][type=0x0C(4)][txId(4)][payload...]
    {
        uint8_t hdr[12];
        if (_transport.recvCommand(hdr, 12, 10000) != 12) {
            LOG_ERROR("getCanonEventData: failed to read End_Data header (12 bytes)");
            return false;
        }

        uint32_t pktLen = readU32LE(hdr, 0);
        uint32_t type   = readU32LE(hdr, 4);
        // hdr[8..11] = txId (ignored)

        if (type != PTPIP_PKT_DATA_END) {
            LOG_ERROR("getCanonEventData: expected End_Data (0x0C), got 0x%08X", (unsigned)type);
            return false;
        }
        if (pktLen < 12) {
            LOG_ERROR("getCanonEventData: End_Data pktLen too small: %u", (unsigned)pktLen);
            return false;
        }

        size_t payloadLen = pktLen - 12;

        if (payloadLen > maxLen) {
            LOG_WARNING("getCanonEventData: payload %u exceeds buffer %u — truncating",
                        (unsigned)payloadLen, (unsigned)maxLen);
            if (_transport.recvCommand(buf, maxLen, 10000) != maxLen) {
                LOG_ERROR("getCanonEventData: failed to read payload (truncated)");
                return false;
            }
            dataLen = maxLen;
            // Drain remaining bytes so the stream stays in sync
            size_t remaining = payloadLen - maxLen;
            uint8_t drain[64];
            while (remaining > 0) {
                size_t chunk = remaining < sizeof(drain) ? remaining : sizeof(drain);
                if (_transport.recvCommand(drain, chunk, 5000) != chunk) break;
                remaining -= chunk;
            }
        } else {
            if (payloadLen > 0) {
                if (_transport.recvCommand(buf, payloadLen, 10000) != payloadLen) {
                    LOG_ERROR("getCanonEventData: failed to read payload (%u bytes)", (unsigned)payloadLen);
                    return false;
                }
            }
            dataLen = payloadLen;
        }

#ifdef PTPIP_HEX_DUMP
        // Dump at most the first 128 bytes of potentially large payloads
        logHexDump("RECV Canon_GetEvent_Payload (first 128 bytes)",
                   buf, dataLen < 128 ? dataLen : 128);
#endif
    }

    // --- Response ---
    {
        uint8_t resp[32];
        size_t n = _recvPacketCommand(resp, sizeof(resp));
        if (n == 0) return false;

        logHexDump("RECV Canon_GetEvent_Response", resp, n);
        uint16_t responseCode;
        if (!_parseResponsePkt(resp, n, responseCode)) return false;

        if (responseCode != PTP_RSP_OK) {
            LOG_ERROR("getCanonEventData: response 0x%04X", responseCode);
            return false;
        }
        LOG_DEBUG("getCanonEventData: txId=%u dataLen=%u OK", (unsigned)txId, (unsigned)dataLen);
    }

    return true;
}

// ----------------------------------------------------------------------------
// pollEvent — non-blocking check for event packet on command channel
// Canon sends capture events (ObjectAdded, CaptureComplete) on command channel.
// Event packet layout:
//   [0..3]   length
//   [4..7]   type = 0x09
//   [8..9]   EventCode (uint16 LE)
//   [10..13] TransactionID (uint32 LE) — ignored
//   [14..17] Parameter1 (uint32 LE) — optional
// ----------------------------------------------------------------------------
bool PtpIpSession::pollEvent(uint16_t& eventCode, uint32_t& param1) {
    if (_transport.availableCommand() < (int)PTPIP_HEADER_SIZE) {
        return false;
    }

    uint8_t buf[32];
    size_t n = _recvPacketCommand(buf, sizeof(buf), 1000);
    if (n < 14) {
        LOG_WARNING("pollEvent: packet too short (%u bytes)", (unsigned)n);
        return false;
    }

    uint32_t type = readU32LE(buf, 4);
    if (type != PTPIP_PKT_EVENT) {
        LOG_WARNING("pollEvent: unexpected packet type 0x%08X", (unsigned)type);
        return false;
    }

    eventCode = readU16LE(buf, 8);
    param1    = (n >= 18) ? readU32LE(buf, 14) : 0;

    LOG_DEBUG("pollEvent: eventCode=0x%04X param1=0x%08X", eventCode, (unsigned)param1);
    return true;
}

// ----------------------------------------------------------------------------
// _recvPacketCommand / _recvPacketEvent
// Reads a full PTP/IP packet: header (8 bytes) + payload (length - 8 bytes).
// ----------------------------------------------------------------------------
size_t PtpIpSession::_recvPacketCommand(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
    if (_transport.recvCommand(buf, 8, timeoutMs) != 8) {
        LOG_ERROR("_recvPacketCommand: failed to read 8-byte header");
        return 0;
    }

    uint32_t totalLen = readU32LE(buf, 0);
    if (totalLen < 8 || totalLen > maxLen) {
        LOG_ERROR("_recvPacketCommand: invalid packet length %u", (unsigned)totalLen);
        return 0;
    }

    size_t remaining = totalLen - 8;
    if (remaining > 0) {
        if (_transport.recvCommand(buf + 8, remaining, timeoutMs) != remaining) {
            LOG_ERROR("_recvPacketCommand: failed to read %u-byte payload", (unsigned)remaining);
            return 0;
        }
    }

    return totalLen;
}

size_t PtpIpSession::_recvPacketEvent(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
    if (_transport.recvEvent(buf, 8, timeoutMs) != 8) {
        LOG_ERROR("_recvPacketEvent: failed to read 8-byte header");
        return 0;
    }

    uint32_t totalLen = readU32LE(buf, 0);
    if (totalLen < 8 || totalLen > maxLen) {
        LOG_ERROR("_recvPacketEvent: invalid packet length %u", (unsigned)totalLen);
        return 0;
    }

    size_t remaining = totalLen - 8;
    if (remaining > 0) {
        if (_transport.recvEvent(buf + 8, remaining, timeoutMs) != remaining) {
            LOG_ERROR("_recvPacketEvent: failed to read %u-byte payload", (unsigned)remaining);
            return 0;
        }
    }

    return totalLen;
}

// ----------------------------------------------------------------------------
// _parseResponsePkt
// Response packet layout:
//   [0..3]   length
//   [4..7]   type = 0x07
//   [8..9]   ResponseCode (uint16 LE)
//   [10..13] TransactionID (uint32 LE)
//   [14..]   Parameters (optional)
// ----------------------------------------------------------------------------
bool PtpIpSession::_parseResponsePkt(const uint8_t* buf, size_t len, uint16_t& responseCode) {
    if (len < 14) {
        LOG_ERROR("_parseResponsePkt: packet too short (%u bytes)", (unsigned)len);
        return false;
    }

    uint32_t type = readU32LE(buf, 4);
    if (type != PTPIP_PKT_CMD_RESPONSE) {
        LOG_ERROR("_parseResponsePkt: expected 0x07, got 0x%08X", (unsigned)type);
        return false;
    }

    responseCode = readU16LE(buf, 8);
    return true;
}

uint32_t PtpIpSession::_nextTransactionId() {
    return ++_transactionId;
}
