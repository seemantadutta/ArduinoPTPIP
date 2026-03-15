#pragma once
#include "CanonCamera.h"

// ============================================================================
// Canon5DMkIV
// Concrete implementation for the Canon EOS 5D Mark IV.
// Currently uses all CanonCamera base defaults — this class exists to name
// the model explicitly and to serve as the override point if 5D MkIV-specific
// behaviour is discovered during testing.
//
// To add a new Canon model:
//   1. Copy this file, rename the class (e.g. Canon6D)
//   2. Override only the methods that differ (_modeStr, _extraInit,
//      initiateCapture, releaseShutter — see CanonCamera.h for details)
//   3. Register it in main.cpp the same way
// ============================================================================

class Canon5DMkIV : public CanonCamera {
public:
    explicit Canon5DMkIV(PtpIpSession& session);
};
