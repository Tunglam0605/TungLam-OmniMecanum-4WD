#ifndef TUNGLAM_MECANUM_L298N_COMPAT_H
#define TUNGLAM_MECANUM_L298N_COMPAT_H

// Compatibility header from the 0.5.x packaging phase.
// It intentionally preserves the legacy V5 API.
// New projects should include <TungLam_OmniMecanum_4WD.h>.

#include "TungLam_Control_MotorV5.h"

using TungLamMecanumL298N = TungLam_Control_MotorV5;

#endif
