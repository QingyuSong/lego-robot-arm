# Raspberry Pi 3 LEGO 兼容机械臂：XYZ 自动抓取系统设计与开发计划

## 1. 项目范围与假设

目标是利用 AliExpress 1029PCS LEGO Technic 兼容机械臂、现有 Raspberry Pi 3、Python 软件及必要的驱动/传感器，实现可重复的 XYZ 自动抓取和放置。

**当前假设，待实物验证：**
- 用户确认共有四个 Power Functions M 型有刷直流电机：底座旋转、夹爪、两路臂部运动。两路臂部电机是否分别一对一驱动肩/肘关节，尚需验证。
- 原配遥控盒疑似开关式直流电机控制，未确认编码器或可编程通信接口；M 电机本身不具备位置反馈。
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
              PWM + direction -> 4 H-bridge channels -> base / arm A / arm B / gripper
              Joint sensors -> actual joint angles
```

Pi 3 负责高级运动规划与自动化。Linux 不是硬实时系统，若闭环周期稳定性不足，将电机控制与传感器采样下沉至 RP2040 或 STM32；Pi 与底层控制器通过 UART/USB 通信。

## 3. 硬件方案与验证

| 部件 | 初步选择 | 必须验证 |
| --- | --- | --- |
| 主控 | Raspberry Pi 3 | GPIO、电源与系统版本 |
| 电机 | 四个 Power Functions M 型有刷直流电机（用户确认） | 第三方兼容电机电压、堵转电流、插头实际触点、齿比 |
| 驱动 | 两块 TB6612FNG 双路 H-bridge（候选） | 电机堵转电流和驱动额定值是否匹配 |
| 关节反馈 | 三个 AS5600（候选）；先采购一个试装 | 底座/肩/肘实际关节轴、磁铁对心、相对固定支架、角度行程 |
| 传感器总线 | I²C 多路复用器（如需要） | AS5600 固定 I²C 地址冲突 |
| 原点与限位 | 限位开关或绝对角度基准 | 安全行程与重复性 |
| 电源 | 电机独立供电 | 额定电压、电流裕量、共地、保险保护 |

AS5600 是单圈角度传感器；必须处理机械零点、角度跨圈和磁铁安装偏差。优先直接测量输出关节，避免齿轮回差造成的电机轴位置与真实关节角度不一致。

先检查原控制板是否已有可复用 H-bridge；不要在不了解原电路时直接将 Pi GPIO 接到电机或遥控板功率端。

## 3.1 结合产品照片的机械结构分析（待实物验证）

用户提供的产品照片显示：黑色底座旋转平台、下臂双侧结构梁、肘部附近外露的大/小齿轮，以及较复杂的多连杆/齿轮传动。照片只能支持候选位置判断，不能直接证明所有传动轴的运动关系。

| 编码器 | 目标测量 | 候选安装原则 | 必须验证 |
| --- | --- | --- | --- |
| E1 底座 | 上部转台相对固定底座的偏航角 θ1 | 磁铁与转台同步旋转，PCB 固定在底座，尽量同轴 | 底座实际旋转中心、线缆走线与行程 |
| E2 肩部 | 下臂相对底座的实际肩关节角 θ2 | 磁铁/PCB 分别固定在肩关节的两个相对旋转构件 | 找到真正肩关节枢轴，不能只选电机输出轴 |
| E3 肘部 | 前臂相对上臂的实际肘关节角 θ3 | 在实际肘关节轴两侧分别固定磁铁和 PCB | 照片中大齿轮是否与关节输出轴刚性连接 |

**AS5600 安装要点：** 使用与传感器匹配的径向/直径方向磁化磁铁（diametrically magnetized magnet），磁铁与关节轴同心；PCB 固定在相对不旋转的另一构件上。磁铁间距、偏心与倾斜需依器件规格检查。可使用 3D 打印的 Technic 十字轴磁铁座与传感器支架。不要将电机轴/中间齿轮的角度未经换算就当作关节角度。

**电机控制：** 标准 Power Functions M 型电机通过两个有效电机端子施加正反极性电压；4 触点 Power Functions 插头并不表示四条独立方向控制线。两块双通道 TB6612FNG 可作为 4 路 PWM H-bridge 候选，选型前需实测兼容电机堵转电流，并核实转接线触点。Pi GPIO 不得直接驱动电机。

**传感器总线：** 三个 AS5600 I²C 模块通常共用固定地址 0x36，可采用 TCA9548A 多路复用器。Pi 侧使用 3.3 V 兼容的逻辑电平和上拉电压。

## 3.2 运动耦合与标定风险

虽然从功能上可以将三路定位自由度描述为底座、肩、肘，但照片中的齿轮和连杆机构意味着：电机 A/B 的转角与肩/肘实际角度可能并非一一对应，甚至可能耦合。

```text
q_motor = [motor_base, motor_arm_A, motor_arm_B]
q_joint = [base_yaw, shoulder_pitch, elbow_relative_pitch]

q_joint = F(q_motor, linkage_geometry, gear_ratios, backlash)
```

不能在尚未验证机构前假设 F 为对角矩阵。若有闭环关节输出角度反馈，可以优先基于 q_joint 控制，同时通过单独驱动每路电机的实验辨识作用关系。必要时使用数值运动学或更完整的连杆约束模型，而非直接套用标准二连杆 IK。

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

**仅在机构可等效为底座偏航 + 两个独立平面转动关节时适用。** 假设底座绕 Z 轴旋转，肩肘在一个竖直平面内，连杆长度为 L1、L2，肩关节高度为 h，目标为 (x,y,z)：

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

## 6.1 编码器试装与机械辨识实验

1. **先买一个 AS5600：** 在易于接近的肩部或肘部真正枢轴上制作磁铁座与固定 PCB 支架，手动转动关节，验证角度单调、重复且无磁场失效。
2. **识别传动：** 每次只短时驱动一个臂部电机，记录肩/肘两角的变化；换另一个电机重复，判断是否存在耦合。测试前设置机械限位与断电措施。
3. **量取几何：** 测量肩部高度 h、连杆有效长度 L1/L2、工具中心点 TCP、每个关节的机械行程和零位。
4. **扩展反馈：** 安装 E1/E2/E3，验证三个输出角度和电机运动的对应关系；必要时加入限位开关和线缆应力释放。
5. **标定与验证：** 多点测量实际 XYZ，拟合或修正 FK/IK 模型；绘制真实可达工作空间。检查抓取姿态、碰撞和运动路径。

## 7. 分阶段实施与验收

| 阶段 | 内容 | 初步工期 | 验收 |
| --- | --- | --- | --- |
| P1 | 机械、电机、原控制板及双臂电机耦合辨识 | 1–2 天起 | 传动关系、关节轴、行程、接线记录 |
| P2 | Pi GPIO + H-bridge 开环控制 | 2–3 天 | 四路正反转、PWM、停止 |
| P3 | 单个 AS5600 试装、三个输出关节反馈、耦合标定与闭环控制 | 3–5 天起 | 输出关节角度可测、可重复定位 |
| P4 | FK/IK、可达空间、轨迹规划 | 3–4 天 | XYZ 目标转换并执行 |
| P5 | Pick-and-Place FSM | 2–3 天 | 自动抓取、搬运、释放 |

原先约 11–17 个工作日仅为简单独立关节结构下的粗略估计；如实际存在复杂耦合机构、编码器支架需反复迭代，工期会明显增加。不含采购时间。

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

**建议先执行 P1：** 已有产品整体照片和齿轮特写。下一步获取组装说明书中电机传动、肩/肘关节的页面，实物确认两路臂电机的运动耦合；先试装一个 AS5600，再决定其余传感器与支架的设计。

## 9. 参考 GitHub 项目

- https://github.com/Aidan-Gildea/robot-arm-3dof
- https://github.com/Tonssov88/3dof-robot-arm-ik
- https://github.com/rookidroid/movens

仅作 IK、运动控制与软件架构参考；未验证这些仓库可直接驱动本套件。
