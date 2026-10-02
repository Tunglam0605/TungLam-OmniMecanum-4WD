[🇻🇳 Tiếng Việt](KINEMATICS.md) • [🌐 English](KINEMATICS.en.md)

# Động học, vận tốc SI và mô hình vòng hở

Tài liệu này giải thích phần toán và mô hình vật lý của modern API trong `TungLam_OmniMecanum_4WD`.

Mục tiêu là để:

- học sinh có thể khai báo thông số robot rồi dùng ngay;
- sinh viên có thể đọc phương trình động học và hiểu vì sao thư viện tính như vậy;
- về sau dễ nối encoder, PID, IMU và ROS2.

---

# 1. Hệ tọa độ chuẩn

Thư viện dùng body frame tay phải:

```text
                 +X / +vx
                  tiến
                   ↑
                   |
                   |
        +Y / +vy ← ROBOT
             trái

+Z hướng lên khỏi mặt robot
```

Nhìn từ trên xuống:

```text
+wz = quay trái / CCW
-wz = quay phải / CW
```

Tóm tắt:

```text
+vx = tiến
-vx = lùi

+vy = trái
-vy = phải

+wz = ngược chiều kim đồng hồ
-wz = cùng chiều kim đồng hồ
```

Legacy `TungLam_Control_MotorV5` vẫn giữ hành vi lịch sử.

---

# 2. Thứ tự bánh và kích thước đế

> Mapping M1..M4 và vector Mecanum chuẩn đã chạy thật được khóa tại **[V5_BASELINE.md](V5_BASELINE.md)**.

```text
                     ĐẦU XE

              M1             M4
          trước-trái     trước-phải

              M2             M3
           sau-trái       sau-phải

                      ĐUÔI XE
```

Quy ước rất quan trọng:

```text
Thứ tự vector/code       : M1, M2, M3, M4
Vị trí tương ứng         : trước-trái, sau-trái, sau-phải, trước-phải
Theo chiều kim đồng hồ   : M1 -> M4 -> M3 -> M2
```

Mọi công thức `v1..v4` trong tài liệu đều theo **thứ tự channel M1..M4**, không theo thứ tự vòng quanh chassis.

Các kích thước:

```text
wheelRadiusM
= bán kính lăn hiệu dụng của bánh [m]

wheelbaseM
= khoảng cách từ đường tâm bánh trước
  đến đường tâm bánh sau [m]

trackWidthM
= khoảng cách từ đường tâm bánh trái
  đến đường tâm bánh phải [m]
```

Đặt:

```text
L = wheelbase / 2
W = trackWidth / 2

K = L + W
  = (wheelbase + trackWidth) / 2
```

---

# 3. Khai báo nhanh

Ví dụ:

```cpp
TungLamDriveConfig model(
    12.0f,   // điện áp danh định motor [V]
    300.0f,  // RPM đầu ra hộp số tại điện áp danh định
    12.0f,   // điện áp nguồn cấp driver [V]
    0.050f,  // bán kính bánh [m]
    0.320f,  // wheelbase [m]
    0.280f,  // track width [m]
    0.85f    // hệ số hiệu chỉnh vòng hở
);

robot.begin();
robot.setChassis(TungLamChassis::MecanumX);
robot.setDriveConfig(model);
```

Sau đó điều khiển bằng đơn vị vật lý:

```cpp
robot.driveVelocity(
    0.40f,  // vx = 0.40 m/s tiến
    0.10f,  // vy = 0.10 m/s sang trái
    0.50f   // wz = 0.50 rad/s CCW
);
```

---

# 4. Ý nghĩa từng thông số

## motorNominalVoltageV

Điện áp danh định gắn với thông số RPM của motor.

Ví dụ:

```text
motor 12V → 12.0f
motor 24V → 24.0f
```

## motorNoLoadRpm

Tốc độ không tải tại **đầu ra cuối cùng kéo bánh**.

Nếu dùng motor giảm tốc, hãy nhập RPM đầu ra hộp số, không nhập RPM rotor bên trong motor.

Ví dụ motor ghi:

```text
12V
300 RPM
```

thì:

```text
motorNominalVoltageV = 12.0
motorNoLoadRpm       = 300.0
```

Nếu còn dây đai/hộp số phụ giữa motor và bánh thì phải quy đổi về RPM trục bánh trước.

## supplyVoltageV

Điện áp cấp vào H-bridge/motor driver.

Thông số này dùng để ước lượng tốc độ theo điện áp.

Không được hiểu là có thể cấp quá áp định mức tùy ý.

## speedScale

Hệ số thực nghiệm để bù sai số do:

- sụt áp L298N;
- pin sụt áp khi tải;
- tải robot;
- ma sát;
- hộp số;
- sai số motor;
- biến dạng bánh.

Ban đầu có thể dùng:

```cpp
1.0f
```

Sau khi đo robot thật, ví dụ thấy tốc độ chỉ khoảng 85% lý thuyết:

```cpp
0.85f
```

---

# 5. Ước lượng RPM theo điện áp

Với DC motor, tốc độ không tải thường xấp xỉ tỷ lệ với điện áp.

Thư viện ước lượng:

```text
RPM_est =
    motorNoLoadRpm
    × supplyVoltageV / motorNominalVoltageV
    × speedScale
```

Đây là **mô hình feed-forward**, không phải cảm biến đo RPM thật.

---

# 6. RPM → rad/s

```text
omega [rad/s]
    = RPM × 2π / 60
```

---

# 7. rad/s → m/s tại vành bánh

Với bán kính bánh `r`:

```text
v_wheel = r × omega
```

Suy ra:

```text
v_wheel_max =
    wheelRadiusM
    × RPM_est
    × 2π / 60
```

Có thể đọc bằng:

```cpp
robot.maxWheelLinearSpeedMps();
```

---

# 8. Động học nghịch Mecanum-X

Đầu vào:

```text
vx = vận tốc tiến [m/s]
vy = vận tốc trái [m/s]
wz = tốc độ quay CCW [rad/s]
```

Với:

```text
K = (wheelbase + trackWidth) / 2
```

thì:

```text
v1 = vx - vy - K*wz
v2 = vx + vy - K*wz
v3 = vx - vy + K*wz
v4 = vx + vy + K*wz
```

Trong đó:

```text
v1 → M1 trước-trái
v2 → M2 sau-trái
v3 → M3 sau-phải
v4 → M4 trước-phải
```

API:

```cpp
TungLamWheelVelocity wheels =
    robot.inverseKinematics(vx, vy, wz);
```

---

# 9. Kiểm tra các vector cơ bản

## Chỉ tiến

```text
vx > 0
vy = 0
wz = 0

→ + + + +
```

## Chỉ sang trái

```text
vx = 0
vy > 0
wz = 0

→ - + - +
```

## Chỉ quay trái / CCW

```text
vx = 0
vy = 0
wz > 0

→ - - + +
```

Đây vẫn là các vector vật lý đã được chứng minh từ V5, chỉ chuẩn hóa dấu Cartesian của modern API.

---

# 10. Động học thuận Mecanum-X

Nếu biết vận tốc 4 bánh, ví dụ lấy từ encoder:

```text
vx =
    (v1 + v2 + v3 + v4) / 4

vy =
    (-v1 + v2 - v3 + v4) / 4

wz =
    (-v1 - v2 + v3 + v4) / (4*K)
```

API:

```cpp
TungLamBodyVelocity body =
    robot.forwardKinematics(wheels);
```

Kết quả:

```cpp
body.vxMps;
body.vyMps;
body.wzRadps;
```

Hàm này chỉ tính toán toán học, không tự đọc encoder.

---

# 11. Omni X-drive

Mô hình Omni-X mặc định giả sử 4 bánh rolling-axis góc 45°.

Đặt:

```text
S = 1/sqrt(2)
K = (wheelbase + trackWidth) / 2
```

Ta có:

```text
v1 = S × ( vx - vy - K*wz)
v2 = S × ( vx + vy - K*wz)
v3 = S × (-vx - vy - K*wz)
v4 = S × (-vx + vy - K*wz)
```

Các vector normalized:

```text
+vx → + + - -
+vy → - + - +
+wz → - - - -
```

Vì Omni có nhiều kiểu bố trí thực tế, cần kiểm tra phần cứng trước khi tin hoàn toàn vào model canonical này.

---

# 12. Xử lý khi yêu cầu vượt tốc độ

Sau inverse kinematics, thư viện kiểm tra tốc độ từng bánh.

Ví dụ yêu cầu:

```text
M1 = 1.20 m/s
M2 = 0.80 m/s
M3 = 1.50 m/s
M4 = 1.10 m/s
```

nhưng khả năng bánh là:

```text
1.00 m/s
```

thì:

```text
scale = 1.00 / 1.50
```

và cả bốn bánh cùng nhân hệ số đó.

Không clamp riêng bánh M3 vì làm như vậy sẽ làm méo vector chuyển động.

---

# 13. m/s bánh → PWM vòng hở

Sau khi giới hạn:

```text
PWM_i =
    255 × v_i / v_wheel_max
```

Giữ nguyên dấu:

```text
+0.5 m/s → PWM dương
-0.5 m/s → PWM âm
```

Dấu cuối cùng được đưa vào common HAL để chọn chiều H-bridge.

---

# 14. Vì sao đây vẫn là vòng hở?

Không có encoder thì controller biết:

```text
vận tốc yêu cầu
thông số RPM
điện áp
bán kính bánh
kích thước đế
PWM đã cấp
```

nhưng không biết vận tốc bánh thực tế.

Sai số có thể đến từ:

- tải robot;
- ma sát sàn;
- pin;
- L298N;
- trượt bánh;
- gearbox;
- nhiệt độ;
- sai lệch giữa các motor.

Vì vậy:

```cpp
robot.driveVelocity(0.40f, 0.0f, 0.0f);
```

nghĩa là:

> tính PWM ước lượng để robot đạt khoảng 0.40 m/s theo model đã khai báo.

Không có nghĩa là robot chắc chắn đo được đúng 0.400 m/s.

---

# 15. Nền cho encoder PID

Về sau có thể thay tầng feed-forward bằng closed-loop:

```text
vx, vy, wz mục tiêu
      ↓
inverse kinematics
      ↓
target speed M1..M4
      ↓
PID M1 ← encoder M1
PID M2 ← encoder M2
PID M3 ← encoder M3
PID M4 ← encoder M4
      ↓
PWM
```

Phần động học phía trên không cần viết lại.

---

# 16. Nền cho IMU giữ hướng

IMU có thể đo yaw, sau đó PID sinh correction theo rad/s:

```cpp
float yawErrorRad =
    targetYawRad - measuredYawRad;

float wzCorrectionRadps =
    headingPid(yawErrorRad);

robot.driveVelocity(
    vxCommandMps,
    vyCommandMps,
    wzManualRadps + wzCorrectionRadps
);
```

Kiến trúc:

```text
Joystick / planner
      │
      ├── vx [m/s]
      ├── vy [m/s]
      └── wz manual [rad/s]
                    │
IMU yaw → PID → wz correction [rad/s]
                    │
                    ▼
             wz final [rad/s]
                    │
              driveVelocity()
```

Như vậy PID làm việc bằng đơn vị vật lý thay vì PWM tùy ý.

---

# 17. Nền cho ROS2

Có thể map trực tiếp từ `cmd_vel`:

```text
linear.x  → vx [m/s]
linear.y  → vy [m/s]
angular.z → wz [rad/s]
```

Về mặt ý tưởng:

```cpp
robot.driveVelocity(
    cmdVelLinearX,
    cmdVelLinearY,
    cmdVelAngularZ
);
```

Nếu cả hai bên cùng dùng body frame chuẩn thì không cần đảo dấu thủ công.

---

# 18. Lộ trình học gợi ý

Cho người mới:

```text
forward()/backward()
→ setWheels()
→ drive(vx,vy,wz)
→ driveVelocity()
```

Cho sinh viên:

```text
hệ tọa độ
→ kích thước đế
→ RPM
→ rad/s
→ m/s
→ inverse kinematics
→ forward kinematics
→ feed-forward
→ encoder PID
→ IMU heading PID
→ odometry
→ ROS2
```

Mục tiêu của thư viện là để cùng một nền tảng có thể dùng từ robot học sinh đến bài thực hành robotics ở bậc đại học.


---

# 19. Tối ưu control loop từ v0.10

ATmega2560 không có FPU, nên phép chia số thực tốn nhiều chu kỳ hơn phép cộng/nhân.

Từ v0.10, khi gọi:

```cpp
robot.setDriveConfig(model);
```

thư viện tính trước và cache:

```text
K = (wheelbase + trackWidth) / 2
RPM ước lượng tại điện áp nguồn
max wheel speed [m/s]
PWM trên mỗi m/s
max body speed
max yaw rate
```

Vì vậy `driveVelocity()` không cần tính lại chuỗi chia số thực cho từng bánh ở mỗi vòng lặp.

---

# 20. Smart Safety

Bật nhanh:

```cpp
robot.enableSmartSafety(500, 1.0f, 2.0f);
```

Trong đó:

```text
500 ms      = watchdog mất lệnh
1.0 m/s²    = giới hạn thay đổi vx/vy
2.0 rad/s²  = giới hạn thay đổi wz
```

Sau đó bắt buộc gọi:

```cpp
robot.update();
```

liên tục trong `loop()`.

Nếu mất lệnh mới lâu hơn timeout trong khi robot đang chạy, watchdog đưa PWM về 0.

Bộ ramp chỉ áp dụng cho `driveVelocity()`; API PWM trực tiếp vẫn phản hồi trực tiếp như trước.

---

# 21. Theo dõi saturation

```cpp
bool limited = robot.wasVelocityLimited();
float scale = robot.lastVelocityScale();
```

`scale = 1.0` nghĩa là lệnh nằm trong khả năng model.

`scale = 0.5` nghĩa là thư viện đã giảm đồng đều vector còn 50%.

Có thể đọc thêm:

```cpp
TungLamBodyVelocity requested = robot.requestedBodyVelocity();
TungLamBodyVelocity applied = robot.appliedBodyVelocity();
```

để tầng PID/ROS2 biết lệnh yêu cầu và lệnh ước lượng thực sự được áp.
