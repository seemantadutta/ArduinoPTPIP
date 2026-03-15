#pragma once
#include <stdint.h>

// ============================================================================
// PTP/IP Protocol Constants
// Reference: PIMA 15740:2000 + Canon proprietary extensions
// All multi-byte values are Little-Endian on the wire
// ============================================================================

// Network
static constexpr uint16_t PTPIP_PORT        = 15740;
static constexpr uint8_t  PTPIP_HEADER_SIZE = 8;  // length (4) + type (4)

// ----------------------------------------------------------------------------
// PTP/IP Packet Types
// ----------------------------------------------------------------------------
static constexpr uint32_t PTPIP_PKT_INIT_CMD_REQ        = 0x00000001;
static constexpr uint32_t PTPIP_PKT_INIT_CMD_ACK        = 0x00000002;
static constexpr uint32_t PTPIP_PKT_INIT_EVT_REQ        = 0x00000003;
static constexpr uint32_t PTPIP_PKT_INIT_EVT_ACK        = 0x00000004;
static constexpr uint32_t PTPIP_PKT_CMD_REQUEST         = 0x00000006;
static constexpr uint32_t PTPIP_PKT_CMD_RESPONSE        = 0x00000007;
static constexpr uint32_t PTPIP_PKT_EVENT               = 0x00000009;
static constexpr uint32_t PTPIP_PKT_DATA_START          = 0x0000000A;
static constexpr uint32_t PTPIP_PKT_DATA_END            = 0x0000000C;
// Canon vendor extension: Start_Data uses type 0x09 instead of 0x0A
static constexpr uint32_t PTPIP_PKT_CANON_DATA_START    = 0x00000009;

// ----------------------------------------------------------------------------
// PTP Operation Codes
// ----------------------------------------------------------------------------
static constexpr uint16_t PTP_OP_OPEN_SESSION     = 0x1002;
static constexpr uint16_t PTP_OP_GET_DEVICE_PROP  = 0x1015;
static constexpr uint16_t PTP_OP_SET_DEVICE_PROP  = 0x1016;
static constexpr uint16_t PTP_OP_INITIATE_CAPTURE = 0x100E;

// Canon proprietary
static constexpr uint16_t PTP_OP_CANON_SET_PROP      = 0x9110;  // vendor SetDevicePropValue
static constexpr uint16_t PTP_OP_CANON_SET_REMOTE    = 0x9114;
static constexpr uint16_t PTP_OP_CANON_SET_EVENT     = 0x9115;
static constexpr uint16_t PTP_OP_CANON_GET_EVENT     = 0x9116;  // poll camera event queue
static constexpr uint16_t PTP_OP_CANON_START_CAPTURE = 0x9128;  // half-press(focus) or full-press(release)
static constexpr uint16_t PTP_OP_CANON_STOP_CAPTURE  = 0x9129;  // release full-press or half-press

// Canon GetEventData segment event types (uint32 in the payload)
static constexpr uint32_t CANON_EVT_PROP_CHANGED     = 0x0000c189; // property value changed
static constexpr uint32_t CANON_EVT_ALLOWED_CHANGED  = 0x0000c18a; // allowed values changed
static constexpr uint32_t CANON_EVT_CAMERA_STATUS    = 0x0000c18b; // camera busy/ready state transition (fires twice per shot)
static constexpr uint32_t CANON_EVT_OBJECT_ADDED     = 0x0000c1a5; // file written to storage (once per output file)
static constexpr uint32_t CANON_EVT_TERMINATOR       = 0x00000000; // end of event list

// Canon capture phase parameter values
static constexpr uint32_t CANON_CAPTURE_PHASE_FOCUS   = 0x00000001; // AF / half-press
static constexpr uint32_t CANON_CAPTURE_PHASE_RELEASE = 0x00000002; // shutter / full-press

// ----------------------------------------------------------------------------
// PTP Response Codes
// ----------------------------------------------------------------------------
static constexpr uint16_t PTP_RSP_OK             = 0x2001;
static constexpr uint16_t PTP_RSP_DEVICE_BUSY    = 0x2019;
static constexpr uint16_t PTP_RSP_INCOMPLETE_TX  = 0x2007;

// ----------------------------------------------------------------------------
// PTP Event Codes
// ----------------------------------------------------------------------------
static constexpr uint16_t PTP_EVT_OBJECT_ADDED       = 0x4002;
static constexpr uint16_t PTP_EVT_CAPTURE_COMPLETE   = 0x400D;

// ----------------------------------------------------------------------------
// PTP Device Property Codes
// ----------------------------------------------------------------------------

// Standard PTP property codes (used for GetDevicePropValue reads)
static constexpr uint16_t PTP_PROP_APERTURE      = 0x5007;
static constexpr uint16_t PTP_PROP_SHUTTER_SPEED = 0x500D;
static constexpr uint16_t PTP_PROP_ISO           = 0x500F;

// Canon vendor property codes (used with PTP_OP_CANON_SET_PROP 0x9110)
// Values are APEX-encoded single bytes; see CanonExposure.h for the full tables.
//   Aperture   0xd101  e.g. f/5.6  -> 0x30
//   Shutter    0xd102  e.g. 1/100s -> 0x6D
//   ISO        0xd103  e.g. ISO400 -> 0x58
//   CaptureDst 0xd11c  0x02 = SD card, 0x01 = RAM only
static constexpr uint16_t PTP_PROP_CANON_APERTURE      = 0xd101;
static constexpr uint16_t PTP_PROP_CANON_SHUTTER_SPEED = 0xd102;
static constexpr uint16_t PTP_PROP_CANON_ISO           = 0xd103;
// EC wire encoding: wire = (int8_t)round(ev * 8), stored as uint8 (two's complement for negative).
// 1 stop = 8 units. Range ±3 EV = ±24 units (0x18 / 0xE8). Uses Canon vendor SetProp (0x9110).
static constexpr uint16_t PTP_PROP_CANON_EC            = 0xd104;
static constexpr uint16_t PTP_PROP_CANON_SHOOTING_MODE = 0xd138; // physical mode dial
static constexpr uint16_t PTP_PROP_CANON_CAPTURE_DEST  = 0xd11c;


// ----------------------------------------------------------------------------
// Session / Handshake Constants
// ----------------------------------------------------------------------------
static constexpr uint32_t PTP_VERSION            = 0x00010000;  // PTP v1.0
static constexpr uint32_t PTP_SESSION_ID         = 0x00000001;
static constexpr uint32_t PTP_TXID_OPEN_SESSION  = 0x00000000;  // TransactionID for OpenSession only
static constexpr uint32_t PTP_FIRST_TXID         = 0x00000001;  // TransactionID for all subsequent ops
