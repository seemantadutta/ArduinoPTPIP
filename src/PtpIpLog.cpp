#include "PtpIpLog.h"
#include <stdio.h>
#include <stdarg.h>

static PtpLogCallback s_logCb = nullptr;

void PtpIpSetLogCallback(PtpLogCallback cb) {
    s_logCb = cb;
}

void _ptpLog(uint8_t level, const char* tag, const char* fmt, ...) {
    if (!s_logCb) return;
    char buf[256];
    // Prefix with [TAG] then the formatted message
    int n = snprintf(buf, sizeof(buf), "[%s] ", tag);
    if (n > 0 && n < (int)sizeof(buf)) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, args);
        va_end(args);
    }
    buf[sizeof(buf) - 1] = '\0';
    s_logCb(level, buf);
}
