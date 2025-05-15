# 6D Impedance Controller with Pinocchio

This repository demonstrates a **6-degree-of-freedom Cartesian impedance controller** that simultaneously regulates end-effector **position** and **orientation** for a *bimanual* UFactory **xArm 7** setup, simulated in [MuJoCo](https://mujoco.org/). Kinematics, dynamics, and Jacobians computed using the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library.



## Demo
<p align="center">
  <img src="fig/bimanual_ic_demo.gif" alt="6-DoF impedance controller demo" width="720">
</p>

---

## Quick Start

### 1. Clone and enter the repository
```bash
git clone https://github.com/YOUR_USERNAME/xarm_bimanual_ic.git
cd xarm_bimanual_ic