# LEGO-Compatible Robotic Arm — Raspberry Pi 3

A low-cost experimental **3-DOF XYZ pick-and-place robot** using a 1029-piece LEGO Technic-compatible mechanical arm, Raspberry Pi 3, and Python.

> **Status: planning / hardware verification.** Product photos and user description indicate four Power Functions M-type brushed DC motors: base rotation, gripper, and two arm-motion actuators. Electrical ratings, exact linkage topology, joint limits, and output-angle sensing still require hands-on verification. Three independent XYZ positioning DOFs are a working hypothesis; coupling must be characterized.

## Goals
- Reuse the kit's mechanical structure and, if practical, its four DC motors.
- Control base rotation, two arm-motion actuators, and gripper; identify the true shoulder/elbow kinematic mapping.
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
| 4 Power Functions M-type brushed DC motors | Base, two arm actuators, gripper | User-identified motor type; verify clone voltage/current and connector wiring |
| 2 dual H-bridge drivers (e.g. TB6612FNG) | Four motor channels | Only if verified electrical ratings are suitable |
| 3 AS5600 absolute magnetic angle sensors (candidate) | Base, shoulder, elbow output-angle feedback | Verify moving/fixed reference parts and mounting space; use I²C multiplexer |
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

## Kit-specific mechanical findings (from product photos)
- The photographed TuoMu-style Technic-compatible arm has exposed gears and multi-link structures. **A visible gear is not automatically the output joint axle.**
- Proposed sensor sites: E1 on the rotating base turntable relative to the fixed base; E2 across the actual shoulder pivot; E3 across the actual elbow pivot.
- Mount each diametrically magnetized magnet to one joint member and its AS5600 PCB to the other. Measure relative joint rotation, not a motor/gear intermediate angle.
- Two arm motors may be coupled by linkages or gearing. Validate actuator-to-joint mapping experimentally before assuming independent shoulder/elbow motor angles.
- Simplified 3-DOF IK remains a candidate model, **not yet verified for this kit**. First record joint axes, linkage lengths, hard stops, gear ratios, and encoder mounting geometry.
- Start with **one AS5600 prototype**, test on a reachable pivot, then expand to three sensors.

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