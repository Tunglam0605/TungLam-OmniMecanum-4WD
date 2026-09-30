/*
    Nguyễn Khắc Tùng Lâm 
    DHTD16A2CL
    2210430016
    Thư viện điều khiển xe 4 bánh đa hướng 

    SƠ ĐỒ ĐẤU NỐI

    CHÂN PWM ------CHÂN SỐ  
      EN BÁNH 1 ----- 5
      EN BÁNH 2 ----- 6
      EN BÁNH 3 ----- 7
      EN BÁNH 4 ----- 8

    CHÂN CHIỀU DIR ( BÁNH ) ------ CHÂN CHIỀU TIẾN ----- CHÂN CHIỀU LÙI
                    DIR1                 30                    31
                    DIR2                 32                    33
                    DIR3                 34                    35
                    DIR4                 36                    37
*/

#ifndef _TungLam_Control_MotorV5_H_
#define _TungLam_Control_MotorV5_H_

#include <Arduino.h>

class TungLam_Control_MotorV5 {
  public:
    TungLam_Control_MotorV5();

    void Mode0();                                                                        // Khởi tạo các chân và Timer với tần số thấp 976,56 Hz
    void Mode1();                                                                       // Khởi tạo các chân và Timer với tần số cao   7,81 kHz
    void Init_Timer1(uint8_t duty11, uint8_t duty12);                                  // Cấu hình tần số thấp cho Timer1 chân 11, 12
    void Init_Timer2(uint8_t duty9,  uint8_t duty10);                                 // Cấu hình tần số thấp cho Timer2 chân  9, 10
    void STOP();                                                                     // Dừng động cơ
    void moveForward   (uint8_t duty);                                              // Di chuyển về phía trước
    void moveBackward  (uint8_t duty);                                             // Di chuyển lùi
    void Forward_Right (uint8_t duty);                                            // Di chuyển tiến phải
    void Backward_Right(uint8_t duty);                                           // Di chuyển lùi phải
    void moveRight     (uint8_t duty);                                          // Di chuyển quay phải
    void moveLeft      (uint8_t duty);                                         // Di chuyển quay trái
    void moveLeftSide  (uint8_t duty);                                        // Di chuyển ngang trái
    void moveRightSide (uint8_t duty);                                       // Di chuyển ngang phải
    void Forward_Left  (uint8_t duty);                                      // Di chuyển tiến trái
    void Backward_Left (uint8_t duty);                                     // Di chuyển lùi trái
    void ABS           (uint8_t duty);                                    // Hàm ABS di chuyển động cơ
    void Dir(uint8_t BanhNumber, bool Set);                              // Đặt lại chiều bánh 
    void Tien  (uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4); 
    void Lui   (uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void Trai  (uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void Phai  (uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void T_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void T_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void L_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void L_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void N_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void N_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);
    void setTimABS(uint8_t t500, uint8_t t1000, uint8_t t1500, uint8_t t2000, uint8_t t3000, uint8_t tAbove3000);
  private:
    void Reset_45 (bool Off);                                                                       // Đặt lại chiều quay các bánh khi  đi chéo
    void Reset_Timer(uint8_t timerNumber);                                                         // Đặt lại các bộ Timer
    void setSTOP(uint8_t pattern);                                                                // Đặt chiều quay của động cơ
    void setPWM(uint8_t duty);                                                                   // Đặt PWM cho các động cơ
    void PWM(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4);                          // Đặt tốc độ riêng từng bánh
    void Tim();                                                                                // Tính toán thời gian di chuyển
    void Timer();                                                                             // Cập nhật thời gian ABS
    bool isMoving = false;                                                                   // Trạng thái di chuyển
    unsigned long startTime = 0;                                                            // Thời gian bắt đầu
    uint8_t TIM = 0;                                                                       // Biến thời gian ABS
    uint8_t pre = 0;                                                                      // Biến trạng thái di chuyển trước đó
    uint8_t tim500=45, tim1000=65, tim1500=70, tim2000=75, tim3000=80, timAbove3000=85;  // Thời gian các giai đoạn
    bool Set = false;                                                                   // Chiều thuận đúng
};

#endif // _TungLam_Control_MotorV5_H_
