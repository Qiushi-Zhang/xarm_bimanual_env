# 6D Impedance Controller with Pinocchio

This repository showcases a **6-DoF Cartesian impedance controller** for a *bimanual* UFactory **xArm 7** pair, simulated in [MuJoCo](https://mujoco.org/).  
Rigid-body kinematics, dynamics, and Jacobians are computed with the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library.



## Demo

Run example/xarm_bimanual_ic to see how to use impedance controller to move EE

<p align="center">
  <img src="fig/bimanual_ic_demo.gif" width="720" alt="Impedance-control demo GIF">
</p>




---



## Set Up

### 1. Create and activate the Conda environment
```bash
# 2 Create a conda env (Python 3.10 recommended)
conda create -n xarm_bimanual python=3.10 -y
conda activate xarm_bimanual

```   

### 2. Install dependencies
```bash
# 3 Install dependencies
conda install -c conda-forge pinocchio            # Pinocchio ≥ 3.4
pip install mujoco==3.2.7 numpy scipy            # MuJoCo + helpers


```  


