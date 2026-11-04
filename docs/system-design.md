# Raspberry Pi 3 LEGO 兼容机械臂：XYZ 自动抓取系统设计与开发计划

## 1. 项目范围与假设

目标是利用 AliExpress 1029PCS LEGO Technic 兼容机械臂、现有 Raspberry Pi 3、Python 软件及必要的驱动/传感器，实现可重复的 XYZ 自动抓取和放置。

**当前假设，待实物验证：**
- 机械臂具有底座旋转 J1、肩关节 J2、肘关节 J3，以及独立夹爪电机。
- 原配遥控盒以开关控制 DC 电机，未确认编码器或可编程通信接口。
- 保留机械结构，优先保留电机；如电机/减速器不适合闭环则更换。

**边界：** 3 个定位自由度可控制末端位置 XYZ，但不能独立控制完整的末端姿态。桌面垂直抓取可能需要专用夹爪安装角度或额外腕关节。

## 2. 系统架构

```text
Raspberry Pi 3 / Python
  Application / optional Web UI
    Pick-and-Place FSM
      Motion Planner (joint / Cartesian trajectory)
        Kinematics (FK / IK)
          Joint Controller (position feedback / PID / limits)
            Motor HAL
              PWM + direction -> H-bridges -> J1 J2 J3 gripper
              Joint sensors -> actual joint angles
```

Pi 3 负责高级运动规划与自动化。Linux 不是硬实时系统，若闭环周期稳定性不足，将电机控制与传感器采样下沉至 RP2040 或 STM32；Pi 与底层控制器通过 UART/USB 通信。

## 3. 硬件方案与验证

| 部件 | 初步选择 | 必须验证 |
| --- | --- | --- |
| 主控 | Raspberry Pi 3 | GPIO、电源与系统版本 |
| 电机 | 原装四个 DC 电机 | 电压、堵转电流、极性、接头、齿比 |
| 驱动 | 两块 TB6612FNG 双路 H-bridge（候选） | 电机堵转电流和驱动额定值是否匹配 |
| 关节反馈 | 三个 AS5600（候选） | 输出轴安装空间、磁铁对心、角度行程 |
| 传感器总线 | I²C 多路复用器（如需要） | AS5600 固定 I²C 地址冲突 |
| 原点与限位 | 限位开关或绝对角度基准 | 安全行程与重复性 |
| 电源 | 电机独立供电 | 额定电压、电流裕量、共地、保险保护 |

AS5600 是单圈角度传感器；必须处理机械零点、角度跨圈和磁铁安装偏差。优先直接测量输出关节，避免齿轮回差造成的电机轴位置与真实关节角度不一致。

先检查原控制板是否已有可复用 H-bridge；不要在不了解原电路时直接将 Pi GPIO 接到电机或遥控板功率端。

## 4. 软件模块

```text
robot_arm/
├── hal/
│   ├── motor.py
│   ├── encoder.py
│   └── gpio_driver.py
├── control/
│   ├── joint_controller.py
│   └── homing.py
├── kinematics/
│   ├── forward.py
│   └── inverse.py
├── motion/
│   ├── trajectory.py
│   └── planner.py
├── automation/
│   └── pick_and_place.py
├── config/
│   └── robot.yaml
└── main.py
```

**HAL：** `set_motor_pwm(joint, duty)`、`read_joint_angle(joint)`、`disable_all()`。

**Joint Controller：** 编码器校准、位置误差、PID/限幅、到位保持、超时和故障状态。位置控制以实际关节角度为闭环变量。

**Kinematics：** 正运动学 FK（角度 → XYZ）与逆运动学 IK（XYZ → 候选关节角度）。考虑实际连杆长度、关节偏置、工具中心点 TCP、机械限位、肘上/肘下分支及奇异位置。

**Motion Planner：** 首先实现关节空间插值及速度/加速度限制；再实现 Cartesian 直线插值、每个采样点 IK、可达性和限位检查。同步多轴运动，避免只依赖并发电机命令。

**Automation FSM：** 每一步依据位置误差和稳定时间完成，而不是固定 sleep。处理超时、急停、传感器故障、夹取失败。

## 5. 简化 3-DOF IK

假设底座绕 Z 轴旋转，肩肘在一个竖直平面内，连杆长度为 L1、L2，肩关节高度为 h，目标为 (x,y,z)：

```text
theta1 = atan2(y, x)
r      = sqrt(x² + y²)
z2     = z - h
D      = (r² + z2² - L1² - L2²) / (2 L1 L2)
theta3 = atan2(±sqrt(1-D²), D)
theta2 = atan2(z2, r) - atan2(L2 sin(theta3), L1 + L2 cos(theta3))
```

若 |D| > 1，则该简化模型下目标不可达。实际应用还必须检查关节限位、碰撞、工具偏置、奇异位置及不同 IK 分支；公式不是原装机械臂的已校准模型。

## 6. Pick-and-Place 状态机

```text
IDLE -> HOME -> MOVE_ABOVE_PICK -> LOWER_TO_PICK
     -> CLOSE_GRIPPER -> LIFT -> MOVE_ABOVE_PLACE
     -> LOWER_TO_PLACE -> OPEN_GRIPPER -> RETRACT -> DONE
                                      \-> FAULT / SAFE_STOP
```

运动前校验目标工作空间、速度、关节限制及预定路径；运行中监测编码器、超时与电机状态。发生故障时停止运动，必要时切断电机供电。

目标 API（示意）：

```python
robot.home()
robot.move_xyz(150, 80, 120)
robot.move_xyz(150, 80, 40)
robot.gripper.close()
robot.move_xyz(150, 80, 120)
robot.move_xyz(200, -50, 120)
robot.move_xyz(200, -50, 40)
robot.gripper.open()
```

## 7. 分阶段实施与验收

| 阶段 | 内容 | 初步工期 | 验收 |
| --- | --- | --- | --- |
| P1 | 机械、电机与原控制板调查 | 1–2 天 | 参数与接线记录 |
| P2 | Pi GPIO + H-bridge 开环控制 | 2–3 天 | 四路正反转、PWM、停止 |
| P3 | 关节反馈、标定、闭环控制 | 3–5 天 | 可重复执行关节目标角度 |
| P4 | FK/IK、可达空间、轨迹规划 | 3–4 天 | XYZ 目标转换并执行 |
| P5 | Pick-and-Place FSM | 2–3 天 | 自动抓取、搬运、释放 |

总计约 11–17 个工作日，为粗略开发估计，不含采购和复杂机械改造。

**第一版目标：**
- 三个定位关节闭环控制，夹爪可独立开闭；
- 能根据 XYZ 指令执行可达点位移动；
- 具备关节限位、超时、故障停止和硬件电机断电；
- 能重复完成 pick-and-place；
- 以末端重复定位 ±5 mm 为实验目标，实测后决定是否需要加强机械结构或更换电机。

## 8. 首要风险与下一步

1. **电机驱动能力：** 原装电机堵转电流未知，驱动选型不能仅凭电机数量。
2. **传感器机械安装：** 输出关节轴角度测量是 XYZ 精度的关键。
3. **齿轮间隙与结构挠曲：** 可能主导末端重复定位误差。
4. **末端姿态限制：** 3-DOF 无法任意保持夹爪竖直。
5. **实时性：** Pi Linux 适合 IK 和规划，精密电机闭环可独立部署到 MCU。

**建议先执行 P1：** 拍摄原控制板、接插件、电机及三个关节输出轴，记录电压、堵转电流、连杆尺寸和角度行程，再决定是否采购驱动板与 AS5600。

## 9. 参考 GitHub 项目

- https://github.com/Aidan-Gildea/robot-arm-3dof
- https://github.com/Tonssov88/3dof-robot-arm-ik
- https://github.com/rookidroid/movens

仅作 IK、运动控制与软件架构参考；未验证这些仓库可直接驱动本套件。
