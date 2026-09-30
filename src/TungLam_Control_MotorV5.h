/*==============================================================================
  TUNGLAM_CONTROL_MOTORV5 - LEGACY COMPATIBILITY HEADER
  ==============================================================================
  Tác giả : Nguyễn Khắc Tùng Lâm
  Lớp     : DHTD16A2CL
  MSSV     : 2210430016
  Tung Lâm Automation

  THƯ VIỆN ĐIỀU KHIỂN XE 4 BÁNH ĐA HƯỚNG - V5 COMPATIBILITY

  SƠ ĐỒ ĐẤU NỐI V5 GỐC
  ------------------------------------------------------------------------------
  PWM:
    EN BÁNH 1 -> D5
    EN BÁNH 2 -> D6
    EN BÁNH 3 -> D7
    EN BÁNH 4 -> D8

  DIR theo comment V5 gốc:
    DIR1 -> D30 / D31
    DIR2 -> D32 / D33
    DIR3 -> D34 / D35
    DIR4 -> D36 / D37

  Mapping thực tế theo implementation AVR:
    M1: tiến D30 / PC7, lùi D31 / PC6
    M2: tiến D32 / PC5, lùi D33 / PC4
    M3: tiến D34 / PC3, lùi D35 / PC2
    M4: tiến D37 / PC0, lùi D36 / PC1

  Lưu ý: DIR4 trong comment lịch sử V5 bị đảo so với code thực tế.
  Header này chỉ forward sang TungLam_OmniMecanum_4WD.h; toàn bộ API V5 vẫn
  được giữ nguyên để project cũ chỉ cần update library mà không sửa hàm.
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
