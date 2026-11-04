# LEGO-Compatible Robotic Arm — Raspberry Pi 3

A low-cost experimental **3-DOF XYZ pick-and-place robot** using a 1029-piece LEGO Technic-compatible mechanical arm, Raspberry Pi 3, and Python.

> **Status: planning / hardware verification.** The arm's motor electrical ratings, gearing, joint limits, controller interface, and encoder availability have not yet been verified. The 3-DOF configuration is an assumption to validate against the actual kit.

## Goals
- Reuse the kit's mechanical structure and, if practical, its four DC motors.
- Control base, shoulder, elbow, and gripper.
- Measure actual joint angles and perform closed-loop positioning.
- Convert XYZ targets to joint targets using forward/inverse kinematics.
- Plan speed/acceleration-limited movements and execute automated pick-and-place sequences.
- Provide homing, travel limits, timeout, fault handling, and emergency motor disable.

## Proposed architecture

```text
Python application / optional Web UI
  └── Pick-and-place finite-state machine
        └── XYZ / joint trajectory planner
              └── Forward & inverse kinematics
                    └── Joint position controller
                          └── Motor HAL
                                ├── H-bridge drivers → DC motors
                                └── Joint angle sensors → feedback
```

**Host:** Raspberry Pi 3 running Raspberry Pi OS and Python 3. The Pi handles planning and high-level automation. For reliable, deterministic motor control, a future RP2040/STM32 can host the fast control loops.

## Hardware (provisional)

| Item | Purpose | Notes |
| --- | --- | --- |
| Raspberry Pi 3 | High-level controller | Already available |
| 1029-piece arm | Base, shoulder, elbow, gripper | Confirm actual kinematics |
| 4 existing DC motors | Joint and gripper actuation | Confirm voltage, stall current, connector pinout |
| 2 dual H-bridge drivers (e.g. TB6612FNG) | Four motor channels | Only if verified electrical ratings are suitable |
| 3 absolute joint-angle sensors (e.g. AS5600) | Position feedback | Mount on output joints; resolve I²C address collisions |
| Limit switches / absolute references | Homing and safe travel | As needed |
| Separate motor power supply | Motor power | Common ground with Pi; never power motors from GPIO |

**Safety:** Verify supply voltage and stall current before energizing motors. Provide an accessible hardware motor-power disconnect, guard moving joints, limit motion, and stop on sensor faults/timeouts. Never rely on open-loop timed commands for accurate XYZ placement.

## Proposed Python API (illustrative, not implemented)

```python
robot.home()
robot.move_xyz(150, 80, 120)
robot.move_xyz(150, 80, 40)
robot.gripper.close()
robot.move_xyz(150, 80, 120)
robot.move_xyz(200, -50, 120)
robot.move_xyz(200, -50, 40)
robot.gripper.open()
robot.home()
```

Coordinates are in millimeters relative to a calibrated robot base frame. Reachability, joint limits, gripper pose, and collision checks must be implemented before hardware motion.

## Milestones
1. **Characterize hardware:** verify motor type, driver, gear ratios, arm dimensions, and joint limits.
2. **Manual motor control:** PWM/direction via suitable H-bridge drivers.
3. **Closed-loop joints:** sensors, calibration, position control, limits, homing.
4. **XYZ IK and motion planning:** FK/IK validation, reachable workspace, joint-space and Cartesian motion.
5. **Pick-and-place FSM:** approach, descend, grasp, lift, transfer, release, return; error recovery.

**Initial acceptance target:** Demonstrate repeated pick-and-place moves; evaluate whether ±5 mm endpoint repeatability is achievable. This is an experimental target, not a guaranteed specification.

## Reference projects
- [Aidan-Gildea/robot-arm-3dof](https://github.com/Aidan-Gildea/robot-arm-3dof)
- [Tonssov88/3dof-robot-arm-ik](https://github.com/Tonssov88/3dof-robot-arm-ik)
- [rookidroid/movens](https://github.com/rookidroid/movens)

These are possible algorithm/architecture references, not verified drop-in drivers for this kit.

## Documentation
- [中文系统设计与开发计划](docs/system-design.md)

## License
No license has been selected yet.