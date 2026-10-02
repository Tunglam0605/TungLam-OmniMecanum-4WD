[🇻🇳 Tiếng Việt](WIRING.md) • [🌐 English](WIRING.en.md)

# Hướng dẫn đấu nối và kiểm tra phần cứng

Tài liệu này giải thích chi tiết cách đấu **Arduino Mega 2560 + 2× L298N + 4 động cơ DC** cho thư viện `TungLam_OmniMecanum_4WD`.

> Nếu bạn mới làm robot, nên đọc tài liệu này trước khi chạy Mecanum/Omni.

---

# 1. An toàn trước khi cấp nguồn

Trước khi cấp nguồn cho động cơ:

- kê robot lên để cả 4 bánh không chạm sàn;
- bắt đầu với PWM thấp khoảng 60–80;
- kiểm tra đúng cực nguồn;
- bắt buộc nối chung GND;
- tháo jumper ENA/ENB nếu Arduino đang xuất PWM vào EN;
- không cấp nguồn động cơ từ chân 5V Arduino;
- dừng ngay nếu L298N nóng bất thường.

Thư viện chỉ điều khiển phần logic. Dòng motor, công suất nguồn, tiết diện dây, tải cơ khí và giới hạn nhiệt của L298N vẫn phải do người thiết kế đảm bảo.

---

# 2. Quy ước thứ tự bánh

Nhìn từ trên xuống:

```text
                     ĐẦU XE / FRONT
                           +X
                            ↑

              M1                         M3
         TRƯỚC-TRÁI                  TRƯỚC-PHẢI
            PWM D5                     PWM D7

              M2                         M4
          SAU-TRÁI                    SAU-PHẢI
            PWM D6                     PWM D8

                            ↓
                     ĐUÔI XE / REAR
```

Luôn giữ đúng quy ước M1..M4 trong đấu dây, code và tài liệu.

---

# 3. Phân chia hai module L298N

## L298N số 1

| Kênh | Motor | EN/PWM | DIR 1 | DIR 2 |
|---|---|---:|---:|---:|
| A | M1 | D5 | D30 | D31 |
| B | M2 | D6 | D32 | D33 |

## L298N số 2

| Kênh | Motor | EN/PWM | DIR 1 | DIR 2 |
|---|---|---:|---:|---:|
| A | M3 | D7 | D34 | D35 |
| B | M4 | D8 | D37 | D36 |

> Với M4: **D37 là chiều tiến logic**, **D36 là chiều lùi logic**.

---

# 4. Bảng chân Arduino Mega

## PWM / EN

| Motor | Chân Mega | AVR | Timer |
|---|---:|---|---|
| M1 | D5 | PE3 / OC3A | Timer3 |
| M2 | D6 | PH3 / OC4A | Timer4 |
| M3 | D7 | PH4 / OC4B | Timer4 |
| M4 | D8 | PH5 / OC4C | Timer4 |

## Chân chiều

| Motor | Tiến (+) | Lùi (-) |
|---|---:|---:|
| M1 | D30 / PC7 | D31 / PC6 |
| M2 | D32 / PC5 | D33 / PC4 |
| M3 | D34 / PC3 | D35 / PC2 |
| M4 | D37 / PC0 | D36 / PC1 |

Thư viện sử dụng PORTC D30..D37 cho 4 cặp chân chiều.

---

# 5. Nguồn và mass chung

Sơ đồ nguyên tắc:

```text
Nguồn motor (+)
      ├──────────────> L298N #1 motor supply
      └──────────────> L298N #2 motor supply

Nguồn motor (-)
      ├──────────────> GND L298N #1
      ├──────────────> GND L298N #2
      └──────────────> GND Arduino Mega
```

Điều bắt buộc:

```text
GND Arduino = GND L298N #1 = GND L298N #2 = âm nguồn motor
```

Không nên nối 5V giữa Arduino và module L298N một cách máy móc nếu chưa hiểu rõ jumper/regulator trên đúng phiên bản module đang dùng.

---

# 6. Jumper ENA / ENB

Nhiều module L298N có jumper ENA/ENB mặc định, khiến EN luôn ở mức HIGH.

Nếu vẫn cắm jumper, PWM từ Arduino có thể không điều khiển tốc độ như mong muốn.

Với thư viện này:

```text
L298N #1 ENA <- D5
L298N #1 ENB <- D6

L298N #2 ENA <- D7
L298N #2 ENB <- D8
```

Hãy tháo jumper EN tương ứng trước khi nối D5..D8.

---

# 7. Kiểm tra lần đầu bằng FirstMotorTest

Mở Arduino IDE:

```text
File
→ Examples
→ TungLam_OmniMecanum_4WD
→ FirstMotorTest
```

Trình tự test:

```text
M1 tiến
M1 lùi

M2 tiến
M2 lùi

M3 tiến
M3 lùi

M4 tiến
M4 lùi
```

Đúng vị trí phải là:

| Motor | Vị trí |
|---|---|
| M1 | Trước-trái |
| M2 | Sau-trái |
| M3 | Trước-phải |
| M4 | Sau-phải |

Nếu chỉ một bánh bị ngược chiều:

```cpp
robot.setMotorInverted(3, true);
```

Không nên sửa phương trình động học để che lỗi đấu dây/polarity.

---

# 8. Hệ tọa độ robot

Modern API dùng hệ tay phải:

```text
+vx = tiến
-vx = lùi

+vy = trái
-vy = phải

+wz = quay trái / CCW
-wz = quay phải / CW
```

Nhìn từ trên xuống:

```text
             +X / +vx
                ↑
                |
      +Y / +vy ← ROBOT

+wz = quay ngược chiều kim đồng hồ
```

---

# 9. Vector Mecanum-X

Các vector vật lý chính:

```text
Tiến            + + + +
Lùi             - - - -

Trái            - + + -
Phải            + - - +

Quay trái CCW   - - + +
Quay phải CW    + + - -
```

Đường chéo:

```text
Tiến-phải       + 0 0 +
Tiến-trái       0 + + 0
Lùi-phải        0 - - 0
Lùi-trái        - 0 0 -
```

Nếu robot không đi ngang đúng sau khi đã xác nhận polarity từng motor, hãy kiểm tra **hướng lắp bánh Mecanum** trước khi sửa code.

---

# 10. Omni X-drive

Chọn:

```cpp
robot.setChassis(TungLamChassis::OmniX);
```

Hệ tọa độ vẫn giữ:

```text
+vx = tiến
+vy = trái
+wz = CCW
```

Omni có nhiều kiểu bố trí cơ khí khác nhau. Cần test từng bánh, sau đó test riêng pure vx, pure vy, pure wz trước khi dùng chuyển động tổng hợp.

---

# 11. Dead-time khi đảo chiều

Để giảm việc đảo chiều H-bridge khi PWM vẫn đang hoạt động, common HAL thực hiện:

```text
PWM = 0
   ↓
dead-time
   ↓
đổi DIR
   ↓
PWM mới
```

Mặc định:

```text
100 µs
```

Có thể thay đổi:

```cpp
robot.setDirectionDeadTimeUs(150);
```

---

# 12. ABS / hãm ngược chủ động

```cpp
robot.ABS(180);
```

ABS áp mô-men ngược trong thời gian ngắn rồi Timer3 ISR tự cắt.

Bảng mặc định:

| Thời gian đã chạy | Thời gian hãm |
|---:|---:|
| < 500 ms | 45 ms |
| < 1000 ms | 65 ms |
| < 1500 ms | 70 ms |
| < 2000 ms | 75 ms |
| < 3000 ms | 80 ms |
| ≥ 3000 ms | 85 ms |

Điều chỉnh:

```cpp
robot.setTimABS(45, 65, 70, 75, 80, 85);
```

> ABS mạnh có thể gây dòng lớn, nóng driver, sụt áp pin, trượt bánh hoặc sốc hộp số. Tune từ mức thấp.

---

# 13. Tài nguyên Timer

Drive system sử dụng:

```text
Timer3  → PWM M1 + thời gian ABS
Timer4  → PWM M2/M3/M4
PORTC   → D30..D37 DIR
```

Nếu thư viện khác cấu hình lại Timer3/Timer4 thì có thể xung đột.

Legacy V5 vẫn giữ các hàm phụ `Init_Timer1()`, `Init_Timer2()`, nên khi dùng chúng cũng phải chú ý quyền sở hữu timer.

---

# 14. Không dùng đồng thời Legacy và Modern cho cùng phần cứng

Trên một Mega, nên chọn một trong hai:

```text
TungLam_Control_MotorV5
```

hoặc:

```text
TungLamDrive4WD
```

Không nên để cả hai object cùng điều khiển D5..D8 và D30..D37.

---

# 15. Checklist xử lý lỗi

Nếu robot chạy sai, kiểm tra theo thứ tự:

1. GND đã chung chưa.
2. Jumper ENA/ENB đã tháo chưa.
3. M1..M4 có đúng vị trí không.
4. D5..D8 có đúng EN không.
5. D30..D37 có đúng DIR không.
6. Chạy FirstMotorTest để kiểm tra polarity.
7. Kiểm tra hướng lắp bánh Mecanum/Omni.
8. Kiểm tra điện áp pin khi có tải.
9. Kiểm tra nhiệt độ L298N.
10. Sau cùng mới chỉnh software.

Làm theo thứ tự này giúp tránh việc “sửa code để che lỗi phần cứng”.
