/**
 * @file TungLam_Control_MotorV5.h
 * @brief Compatibility include for projects written against the original V5 library.
 *
 * @details
 * This file is intentionally tiny. The full legacy class declaration now lives
 * in TungLam_OmniMecanum_4WD.h so the library has only one public declaration
 * source of truth.
 *
 * Existing sketches may remain unchanged:
 *
 * @code
 * #include <TungLam_Control_MotorV5.h>
 *
 * TungLam_Control_MotorV5 motor;
 *
 * void setup() {
 *   motor.Mode1();
 * }
 * @endcode
 *
 * All original V5 movement names, ABS(duty), setTimABS(...), Mode0()/Mode1(),
 * and the per-wheel-duty movement functions remain available through this
 * compatibility include.
 *
 * @note
 * Do not duplicate the class declaration here. Keeping this as a forwarding
 * header prevents the legacy and modern APIs from drifting apart over time.
 */

#ifndef TUNGLAM_CONTROL_MOTORV5_COMPAT_H
#define TUNGLAM_CONTROL_MOTORV5_COMPAT_H

// Import the single canonical public header containing both modern and V5 APIs.
#include "TungLam_OmniMecanum_4WD.h"

#endif  // TUNGLAM_CONTROL_MOTORV5_COMPAT_H
