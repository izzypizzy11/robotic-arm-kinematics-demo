# Planar Manipulator Kinematics & Workspace Engine

A modular Python framework computing analytical Forward (FK) and Inverse Kinematics (IK) for robotic manipulators with workspace reachability validation.

## Technical Highlights
- Analytical IK solving avoiding numerical instability near workspace boundaries.
- Workspace boundary checking to avoid motor stall and unreachable states.
- Lightweight verification test suite included.

## Execution
```bash
git clone [https://github.com/izzypizzy11/robotic-arm-kinematics-demo.git](https://github.com/izzypizzy11/robotic-arm-kinematics-demo.git)
cd robotic-arm-kinematics-demo
python3 kinematics_engine.py
