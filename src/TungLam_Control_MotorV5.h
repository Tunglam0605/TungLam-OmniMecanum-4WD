/*==============================================================================
  TUNGLAM_CONTROL_MOTORV5 - HEADER TƯƠNG THÍCH LEGACY
  ==============================================================================
  Tác giả : Nguyễn Khắc Tùng Lâm
  Lớp     : DHTD16A2CL
  MSSV    : 2210430016
  Tung Lâm Automation

  THƯ VIỆN ĐIỀU KHIỂN XE 4 BÁNH ĐA HƯỚNG - V5 COMPATIBILITY

  PWM:
    M1 -> D5
    M2 -> D6
    M3 -> D7
    M4 -> D8

  DIR:
    M1: tiến D30 / PC7, lùi D31 / PC6
    M2: tiến D32 / PC5, lùi D33 / PC4
    M3: tiến D34 / PC3, lùi D35 / PC2
    M4: tiến D37 / PC0, lùi D36 / PC1

  File này chỉ forward sang TungLam_OmniMecanum_4WD.h. Toàn bộ class/API V5
  được khai báo tại một nơi duy nhất để tránh hai phiên bản bị lệch nhau.
==============================================================================*/

/**
 * @file TungLam_Control_MotorV5.h
 * @brief Header tương thích cho các project viết bằng thư viện V5 cũ.
 *
 * Project cũ có thể tiếp tục dùng nguyên:
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
 * Các hàm chuyển động V5, ABS(duty), setTimABS(...), Mode0()/Mode1() và API
 * PWM độc lập từng bánh vẫn được cung cấp thông qua header này.
 *
 * @note Không khai báo lại class tại đây. Header này chỉ include header chính
 * để Legacy và Modern luôn dùng cùng một source of truth.
 */

#ifndef TUNGLAM_CONTROL_MOTORV5_COMPAT_H
#define TUNGLAM_CONTROL_MOTORV5_COMPAT_H

// Nạp header public duy nhất chứa cả Modern API và Legacy V5 API.
#include "TungLam_OmniMecanum_4WD.h"

#endif  // TUNGLAM_CONTROL_MOTORV5_COMPAT_H
