# V5 Baseline — chuẩn phần cứng đã chạy thực tế

Tài liệu này là **nguồn sự thật (ground truth)** cho mapping motor và vector chuyển động của đế Mecanum đã được xác nhận bằng thư viện `TungLam_Control_MotorV5` trên robot thật.

## 1. Mapping channel motor

```text
                         ĐẦU XE
                            ↑

                M1                     M4
           TRƯỚC-TRÁI              TRƯỚC-PHẢI
             PWM D5                  PWM D8

                M2                     M3
             SAU-TRÁI               SAU-PHẢI
             PWM D6                  PWM D7

                            ↓
                         ĐUÔI XE
```

| Channel | Vị trí vật lý | PWM |
|---|---|---:|
| M1 | trước-trái | D5 |
| M2 | sau-trái | D6 |
| M3 | sau-phải | D7 |
| M4 | trước-phải | D8 |

Thứ tự vector trong code luôn là `[M1, M2, M3, M4]`.

Thứ tự vật lý theo chiều kim đồng hồ từ góc trước-trái là `M1 -> M4 -> M3 -> M2`.

Hai thứ tự này **không được đánh đồng**.

## 2. DIR và V5

| Motor | nhánh tiến V5 / Set=false | nhánh đảo |
|---|---:|---:|
| M1 | D30 / PC7 | D31 / PC6 |
| M2 | D32 / PC5 | D33 / PC4 |
| M3 | D34 / PC3 | D35 / PC2 |
| M4 | D37 / PC0 | D36 / PC1 |

V5 gốc khai báo `bool Set = false;` và `moveForward()` gọi `Dir(1..4, Set)`.

## 3. Vector lệnh V5 đã chạy đúng

Dấu dưới đây là **polarity lệnh theo channel M1..M4**, không phải mô tả trực tiếp chiều quay trục cơ khí nhìn bằng mắt.

| Chuyển động | M1 | M2 | M3 | M4 |
|---|---:|---:|---:|---:|
| tiến | + | + | + | + |
| lùi | - | - | - | - |
| ngang trái | - | + | - | + |
| ngang phải | + | - | + | - |
| quay trái / CCW | - | - | + | + |
| quay phải / CW | + | + | - | - |
| tiến-phải | + | 0 | + | 0 |
| tiến-trái | 0 | + | 0 | + |
| lùi-phải | 0 | - | 0 | - |
| lùi-trái | - | 0 | - | 0 |

## 4. Modern Mecanum phải giữ parity với V5

```text
+vx = tiến
+vy = trái
+wz = quay trái / CCW

M1 = vx - vy - wz
M2 = vx + vy - wz
M3 = vx - vy + wz
M4 = vx + vy + wz
```

Với SI kinematics, thành phần quay dùng thêm lever arm `K`.

## 5. Kiểu điều khiển PS2 đã dùng trong RoboBall

Các project `Mecanum_STEP_Roboball`, `RoboBall_STEP`, `Thumon_Meganum` dùng pattern chính:

```text
joystick trái:
  UP/DOWN    -> tiến/lùi
  LEFT/RIGHT -> ngang trái/phải

joystick phải:
  LEFT/RIGHT -> quay trái/phải
```

Trong các project Mecanum cũ, lệnh joystick phải thường được xử lý sau joystick trái. Vì lệnh motor sau ghi đè lệnh trước, joystick phải có priority cao hơn khi cả hai cùng tác động.

Example modern `PS2RobotControl` biểu diễn priority này rõ ràng thay vì phụ thuộc thứ tự câu lệnh.

## 6. Quy tắc regression

Trước khi thay đổi Mecanum mixer hoặc wheel mapping:

1. Đối chiếu tài liệu này trước.
2. Không suy luận lại M1..M4 từ hình textbook.
3. Giữ mapping M1/M2/M3/M4 như V5 đã chạy thật.
4. Giữ compile-time basis assertions.
5. Chạy Legacy RoboBall Regression.
6. Chỉ thay baseline sau khi test phần cứng thật xác nhận.
