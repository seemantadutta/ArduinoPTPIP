#pragma once
#include <stdint.h>

// ============================================================================
// PtpIpLog — lightweight log callback system
//
// The library never filters or buffers log output. Register a callback once
// in setup() to receive all messages. The callback decides whether and how
// to print each one.
//
// Example (in setup()):
//   PtpIpSetLogCallback([](uint8_t level, const char* msg) {
//       if (level >= PTPIP_LOG_WARNING) Serial.println(msg);
//   });
//
// Each library .cpp file defines LOG_TAG before including this header.
// ============================================================================

// Log level constants — passed to callback; filtering is the app's responsibility
static constexpr uint8_t PTPIP_LOG_DEBUG   = 0;
static constexpr uint8_t PTPIP_LOG_INFO    = 1;
static constexpr uint8_t PTPIP_LOG_WARNING = 2;
static constexpr uint8_t PTPIP_LOG_ERROR   = 3;

// Callback type. level: one of PTPIP_LOG_* above.
// msg: null-terminated formatted string, no trailing newline.
typedef void (*PtpLogCallback)(uint8_t level, const char* msg);

// Register (or clear) the log callback. Pass nullptr to silence all output.
void PtpIpSetLogCallback(PtpLogCallback cb);

// Internal dispatcher — not for app use
void _ptpLog(uint8_t level, const char* tag, const char* fmt, ...);

// Convenience macros — each .cpp file must define LOG_TAG before using these
#define LOG_DEBUG(fmt, ...)   _ptpLog(PTPIP_LOG_DEBUG,   LOG_TAG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    _ptpLog(PTPIP_LOG_INFO,    LOG_TAG, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) _ptpLog(PTPIP_LOG_WARNING, LOG_TAG, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   _ptpLog(PTPIP_LOG_ERROR,   LOG_TAG, fmt, ##__VA_ARGS__)
