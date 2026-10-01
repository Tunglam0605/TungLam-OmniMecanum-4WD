/*==============================================================================
  TUNGLAM_CONTROL_MOTORV5 - LEGACY COMPATIBILITY HEADER
  ==============================================================================
  T├íc giß║ú : Nguyß╗àn Khß║»c T├╣ng L├óm
  Lß╗¢p     : DHTD16A2CL
  MSSV     : 2210430016
  Tung L├óm Automation

  TH╞» VIß╗åN ─ÉIß╗ÇU KHIß╗éN XE 4 B├üNH ─ÉA H╞»ß╗ÜNG - V5 COMPATIBILITY

  S╞á ─Éß╗Æ ─Éß║ñU Nß╗ÉI V5 - ─É├â ─Éß╗ÉI CHIß║╛U Vß╗ÜI CODE
  ------------------------------------------------------------------------------
  PWM:
    EN B├üNH 1 -> D5
    EN B├üNH 2 -> D6
    EN B├üNH 3 -> D7
    EN B├üNH 4 -> D8

  DIR:
    M1: tiß║┐n D30 / PC7, l├╣i D31 / PC6
    M2: tiß║┐n D32 / PC5, l├╣i D33 / PC4
    M3: tiß║┐n D34 / PC3, l├╣i D35 / PC2
    M4: tiß║┐n D37 / PC0, l├╣i D36 / PC1

  Header n├áy chß╗ë forward sang TungLam_OmniMecanum_4WD.h; to├án bß╗Ö API V5 vß║½n
  ─æ╞░ß╗úc giß╗» nguy├¬n ─æß╗â project c┼⌐ chß╗ë cß║ºn update library m├á kh├┤ng sß╗¡a h├ám.
  ==============================================================================
*/

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
