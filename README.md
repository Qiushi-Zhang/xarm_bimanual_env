# 6D Impedance Controller with Pinocchio

This repository showcases a **6-DoF Cartesian impedance controller** for a *bimanual* UFactory **xArm 7** pair, simulated in [MuJoCo](https://mujoco.org/).  
Rigid-body kinematics, dynamics, and Jacobians are computed with the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library.

<p align="center">
  <img src="fig/bimanual_ic_demo.gif" width="720" alt="Impedance-control demo GIF">
</p>

---

## Table of Contents
1. [Quick Start](#quick-start)  
2. [Environment Setup](#environment-setup)  
3. [Running the Demo](#running-the-demo)  
4. [Repository Layout](#repository-layout)  
5. [Troubleshooting](#troubleshooting)  
6. [License](#license)  

---

## Quick Start

```bash
# 1 Clone the repo
git clone https://github.com/YOUR_USERNAME/xarm_bimanual_ic.git
cd xarm_bimanual_ic

# 2 Create a conda env (Python 3.10 recommended)
conda create -n xarm_bimanual python=3.10 -y
conda activate xarm_bimanual

# 3 Install dependencies
conda install -c conda-forge pinocchio            # Pinocchio ≥ 3.4
pip install mujoco==3.2.7 numpy scipy            # MuJoCo + helpers

