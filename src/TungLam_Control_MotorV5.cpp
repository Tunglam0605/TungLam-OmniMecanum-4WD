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

#include "TungLam_Control_MotorV5.h"

TungLam_Control_MotorV5::TungLam_Control_MotorV5() {
  // Constructor để khởi tạo các giá trị mặc định nếu cần
}
void TungLam_Control_MotorV5:: Reset_45 (bool Off)
{
  if     (Off == true ) PORTB &= ~ ((1<<PC0)|(1<<PC1)|(1<<PC4)|(1<<PC5));
  else if(Off == false) PORTB &= ~ ((1<<PC2)|(1<<PC3)|(1<<PC6)|(1<<PC7));
}
void TungLam_Control_MotorV5::Dir(uint8_t BanhNumber, bool Set)
{
  switch (BanhNumber)
  {
  case 1: 
    if (Set == false)
    {
      PORTC |=  (1 << PC7);
      PORTC &= ~(1 << PC6);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC6);
      PORTC &= ~(1 << PC7);
    }
    break;  // Add break here

  case 2:
    if (Set == false)
    {
      PORTC |=  (1 << PC5);
      PORTC &= ~(1 << PC4);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC4);
      PORTC &= ~(1 << PC5);
    }
    break;  // Add break here

  case 3:
    if (Set == false)
    {
      PORTC |=  (1 << PC3);
      PORTC &= ~(1 << PC2);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC2);
      PORTC &= ~(1 << PC3);
    }
    break;  // Add break here

  case 4:
    if (Set == false)
    {
      PORTC |=  (1 << PC0);
      PORTC &= ~(1 << PC1);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC1);
      PORTC &= ~(1 << PC0);
    }
    break;  // Add break here
  }
}

void TungLam_Control_MotorV5::Reset_Timer(uint8_t timerNumber)
{
    switch (timerNumber)
    {
    case 1:
        // Reset Timer 1
        TCCR1A = 0;
        TCCR1B = 0;
        TIMSK1 = 0;
        break;

    case 2:
        // Reset Timer 2
        TCCR2A = 0;
        TCCR2B = 0;
        TIMSK2 = 0;
        break;

    case 3:
        // Reset Timer 3
        TCCR3A = 0;
        TCCR3B = 0;
        TIMSK3 = 0;
        break;

    case 4:
        // Reset Timer 4
        TCCR4A = 0;
        TCCR4B = 0;
        TIMSK4 = 0;
        break;

    default:
        // Không thực hiện gì nếu tham số không hợp lệ
        break;
    }
}


void TungLam_Control_MotorV5::Mode1() // Cài đặt tần số cao cho 4 chân 5,6,7,8 Mode riêng để điều khiển xe 4 bánh đa hướng với cài đặt các chân chiều và chân PWM
 {
    // Cài đặt các chân điều khiển chiều (Dir) từ PC0 đến PC7 là OUTPUT
    DDRC |= 0xFF;

    // Khai báo chân 5, 6, 7, 8 là OUTPUT PWM
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5); // PORT H: PH3 = Chân 6, PH4 = Chân 7, PH5 = Chân 8
    DDRE |= (1 << PE3);                           // PORT E: PE3 = Chân 5

    // Đặt lại các thanh ghi Timer 3, Timer 4
      Reset_Timer(3);
      Reset_Timer(4);

    // Cấu hình Timer 3 cho chân 5 (OC3A)
    TCCR3A = (1 << WGM30) | (1 << COM3A1);        // Fast PWM, Clear on Compare Match
    TCCR3B = (1 << WGM32) | (1 << CS31);          // Prescaler 8

    // Cấu hình Timer 4 cho chân 6, 7, 8 (OC4A, OC4B, OC4C)
    TCCR4A = (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1); // Fast PWM, Clear on Compare Match
    TCCR4B = (1 << WGM42) | (1 << CS41);                                  // Prescaler 8

    // Đặt giá trị ban đầu cho các chân PWM
    OCR3A = 0; // Chân 5
    OCR4A = 0; // Chân 6
    OCR4B = 0; // Chân 7
    OCR4C = 0; // Chân 8
}
void TungLam_Control_MotorV5::Mode0() // Cài đặt tần số thấp cho 4 chân 5,6,7,8 Mode riêng để điều khiển xe 4 bánh đa hướng với cài đặt các chân chiều và chân PWM
  {
    // Cài đặt các chân điều khiển chiều (Dir) từ PC0 đến PC7 là OUTPUT
    DDRC |= 0xFF;

    // Khai báo chân 5, 6, 7, 8 là OUTPUT PWM
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5); // PORT H: PH3 = Chân 6, PH4 = Chân 7, PH5 = Chân 8
    DDRE |= (1 << PE3);                           // PORT E: PE3 = Chân 5

    // Đặt lại các thanh ghi Timer 3, Timer 4
      Reset_Timer(3);
      Reset_Timer(4);

    // Cấu hình Timer 3 cho chân 5 (OC3A)
      // Fast PWM với TOP = ICR3
      TCCR3A |= (1 << WGM31);
      TCCR3B |= (1 << WGM32) | (1 << WGM33);

      // Clear OC3A/OC3B on Compare Match
      TCCR3A |= (1 << COM3A1);

      // Prescaler = 64
      TCCR3B |= (1 << CS31) | (1 << CS30);

      // Đặt TOP và giá trị PWM ban đầu
      ICR3 = 255;  // TOP

    // Cấu hình Timer 4 cho chân 6, 7, 8 (OC4A, OC4B, OC4C)
      // Fast PWM với TOP = ICR4
      TCCR4A |= (1 << WGM41);
      TCCR4B |= (1 << WGM42) | (1 << WGM43);

      // Clear OC4A/OC4B/OC4C on Compare Match
      TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);

      // Prescaler = 64
      TCCR4B |= (1 << CS41) | (1 << CS40);

      // Đặt TOP và giá trị PWM ban đầu
      ICR4 = 255;  // TOP
    // Đặt giá trị ban đầu cho các chân PWM
      OCR3A = 0; // Chân 5
      OCR4A = 0; // Chân 6
      OCR4B = 0; // Chân 7
      OCR4C = 0; // Chân 8
}
void TungLam_Control_MotorV5:: Init_Timer1(uint8_t duty11, uint8_t duty12) // Cài đặt Timer1 tần số thấp cho chân 11 12 chuyên dùng cho Planet có PID hộp
{
    // Động cơ chân 11 12 
      Reset_Timer(1);

      DDRB |= (1 << PB6);
      DDRB |= (1 << PB5);
      TCCR1A |= (1 << WGM11);
      TCCR1B |= (1 << WGM12) | (1 << WGM13);
      TCCR1A |= (1 << COM1B1);   // Chân số 12
      TCCR1A |= (1 << COM1A1);  // Chan số 11
      TCCR1B |= (1 << CS10) | (1 << CS11);
      ICR1 = 255;

      OCR1B = duty12;  // PWM chân 12
      OCR1A = duty11; // PWM chân 11
}
void TungLam_Control_MotorV5:: Init_Timer2(uint8_t duty9, uint8_t duty10) // Cài đặt Timer 2 tần số thấp cho chân 9 10 chuyên dùng cho Planet có PID hộp
{
    // Reset cấu hình Timer2
      Reset_Timer(2);

    // Cấu hình tần số Timer2                                         
      TCCR2A |= (1 << WGM21) | (1 << WGM20);                                  // Chế độ Fast PWM (TOP = 0xFF)
      TCCR2B |= (1 << CS22);                                                 // Chọn bộ chia xung 64

    // Động cơ Planet chân 9 PH6
      DDRH |= (1 << PH6);     
      TCCR2A |= (1 << COM2B1);                                            
      OCR2B = duty9;                                                     

    // Động cơ Planet chân 10 PB4
      DDRB |= (1 << PB4);
      TCCR2A |= (1 << COM2A1);
      OCR2A = duty10; 
}
void TungLam_Control_MotorV5::STOP() {
    setSTOP(0); // Tất cả chân điều khiển về LOW
    setPWM(0);       // Dừng PWM
    pre = 0;
}

void TungLam_Control_MotorV5::moveForward(uint8_t duty) {
    Tim();
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, Set);
    Dir(4, Set);
    setPWM(duty);
    pre = 1; // Tiến
}

void TungLam_Control_MotorV5::moveBackward(uint8_t duty) {
    Tim();
    Dir(1,!Set);
    Dir(2,!Set);
    Dir(3,!Set);
    Dir(4,!Set);
    setPWM(duty);
    pre = 2; // Lùi
}

void TungLam_Control_MotorV5::Forward_Right(uint8_t duty) {
    Tim();
    Dir(1,Set);
    Dir(3,Set);
    Reset_45(false);
    // Tiến Phải
    setPWM(duty);
    pre = 3; // Tiến Phải
}

void TungLam_Control_MotorV5::Backward_Right(uint8_t duty) {
    Tim();
    Dir(2,!Set);
    Dir(4,!Set);
    Reset_45(true);    
    // Lùi Phải
    setPWM(duty);
    pre = 4; // Lùi Phải
}

void TungLam_Control_MotorV5::moveRight(uint8_t duty) {
    Tim();
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, !Set);
  // Quay phải
    setPWM(duty);
    pre = 5; // Quay phải
}

void TungLam_Control_MotorV5::moveLeft(uint8_t duty) {
    Tim();
    Dir(1, !Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, Set);
  // Quay trái
    setPWM(duty);
    pre = 6; // Quay trái
}

void TungLam_Control_MotorV5::moveLeftSide(uint8_t duty) {
    Tim();
    Dir(1, !Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, Set);
  // Ngang trái
    setPWM(duty);
    pre = 7;
}

void TungLam_Control_MotorV5::moveRightSide(uint8_t duty) {
    Tim();
    Dir(1, Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, !Set);
  // Ngang phải
    setPWM(duty);
    pre = 8; // Ngang trái
}

void TungLam_Control_MotorV5::Forward_Left(uint8_t duty) {
    Tim();    
    Dir(2, Set);
    Dir(4, Set);
    Reset_45(true);
  // Tiến Trái 
    setPWM(duty);
    pre = 9; // Tiến Trái
}

void TungLam_Control_MotorV5::Backward_Left(uint8_t duty) {
    Tim();      
    Dir(1, !Set);
    Dir(3, !Set);
    Reset_45(false);
  // Lùi Trái
    setPWM(duty);
    pre = 10; // Lùi Trái
}

void TungLam_Control_MotorV5::ABS(uint8_t duty) {
    if(pre == 0) {STOP(); pre = 0; isMoving = false; return;}
    if(pre == 1) {isMoving = true; Timer(); moveBackward(duty);  delay(TIM); pre = 0;}
    if(pre == 2) {isMoving = true; Timer(); moveForward(duty);   delay(TIM); pre = 0;}
    if(pre == 3) {isMoving = true; Timer(); Backward_Left(duty); delay(TIM); pre = 0;} 
    if(pre == 4) {isMoving = true; Timer(); Forward_Left(duty);  delay(TIM); pre = 0;}   
    if(pre == 5) {isMoving = true; Timer(); moveLeft(duty);      delay(TIM); pre = 0;} 
    if(pre == 6) {isMoving = true; Timer(); moveRight(duty);     delay(TIM); pre = 0;}
    if(pre == 7) {isMoving = true; Timer(); moveRightSide(duty); delay(TIM); pre = 0;} 
    if(pre == 8) {isMoving = true; Timer(); moveLeftSide(duty);  delay(TIM); pre = 0;} 
    if(pre == 9) {isMoving = true; Timer(); Backward_Right(duty);delay(TIM); pre = 0;} 
    if(pre ==10) {isMoving = true; Timer(); Forward_Right(duty); delay(TIM); pre = 0;} 
    // Reset trạng thái di chuyển sau khi hoàn thành
    isMoving = false;
}

void TungLam_Control_MotorV5::Tim() {
    if (!isMoving) {
        startTime = millis();  // Ghi lại thời điểm bắt đầu di chuyển
        isMoving = true;       // Bật cờ di chuyển
    }
}

void TungLam_Control_MotorV5::setTimABS(uint8_t t500, uint8_t t1000, uint8_t t1500, uint8_t t2000, uint8_t t3000, uint8_t tAbove3000) {
    tim500  = t500 ;
    tim1000 = t1000;
    tim1500 = t1500;
    tim2000 = t2000;
    tim3000 = t3000;
    timAbove3000 = tAbove3000;
}

void TungLam_Control_MotorV5::Timer() {
    unsigned long duration = millis() - startTime;  // Tính thời gian đã di chuyển
    if (duration < 500) TIM = tim500;
    else if (duration < 1000) TIM = tim1000;
    else if (duration < 1500) TIM = tim1500;
    else if (duration < 2000) TIM = tim2000;
    else if (duration < 3000) TIM = tim3000;
    else if (duration > 3000) TIM = timAbove3000;
    isMoving = false; // Tắt cờ
    startTime = 0;    // Reset startTime để chuẩn bị cho lần di chuyển tiếp theo
}

void TungLam_Control_MotorV5::setSTOP(uint8_t pattern) {
    // Cài đặt chiều động cơ từ pattern
    PORTC = pattern; // Giả sử PORTC được sử dụng để điều khiển các chân
}

void TungLam_Control_MotorV5::setPWM(uint8_t duty) {
    // Cài đặt giá trị PWM cho các chân
    OCR3A = duty;  // Chân 5
    OCR4A = duty;  // Chân 6
    OCR4B = duty;  // Chân 7
    OCR4C = duty;  // Chân 8
}

void TungLam_Control_MotorV5::PWM(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    // Cài đặt giá trị PWM cho các chân
    OCR3A = duty1;  // Chân 5
    OCR4A = duty2;  // Chân 6
    OCR4B = duty3;  // Chân 7
    OCR4C = duty4;  // Chân 8
}

void TungLam_Control_MotorV5::Tien(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, Set);
    Dir(4, Set); 
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::Lui(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1,!Set);
    Dir(2,!Set);
    Dir(3,!Set);
    Dir(4,!Set); 
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::T_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1, Set);
    Dir(3,Set);
    Reset_45(false);
  // Tiến Phải
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::L_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(2, !Set);
    Dir(4,!Set);
    Reset_45(false);
  // Lùi Phải
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, !Set);
  // Quay phải
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1, !Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, Set);
  // Quay trái
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::N_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1, !Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, Set);
  // Ngang trái
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::N_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1, Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, !Set);
  // Ngang phải
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::T_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) { 
    Dir(2, Set);
    Dir(4, Set);
    Reset_45(true);
  // Tiến Trái 
    PWM(duty1,duty2,duty3,duty4);
}

void TungLam_Control_MotorV5::L_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Dir(1, !Set);
    Dir(3, !Set);
    Reset_45(true);
  // Lùi Trái
    PWM(duty1,duty2,duty3,duty4);
}