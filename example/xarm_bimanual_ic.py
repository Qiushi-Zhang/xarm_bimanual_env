import mujoco 
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import pinocchio as pin 
import numpy as np 
import mujoco_viewer
# from utils import pin_utils
from utils import visualizer, robot_utils


# build mujoco model and data (containing 2 arms)
mj_model = mujoco.MjModel.from_xml_path("../xarm_description/xarm_mj/xarm7_bimanual.xml")
mj_data = mujoco.MjData(mj_model)

print(mj_data.qpos.shape, mj_data.ctrl.shape, "qpos and ctrl shapes")

# build pinocchio model and data (containing only one arm, model origin is at the base of a single xarm) 
pin_model = pin.buildModelFromUrdf("../xarm_description/robots/xarm7.urdf")
pin_data = pin_model.createData()

# get mujoco and pinocchio end-effector id 
eeid_pin = pin_model.getFrameId("link7")
eeid_mj_left = mj_data.body("link7_A").id 
eeid_mj_right = mj_data.body("link7_B").id
print("eeid_pin", eeid_pin, "eeid_mj_left", eeid_mj_left, "eeid_mj_right", eeid_mj_right)



viewer = mujoco_viewer.MujocoViewer(mj_model,mj_data)


# get offset from robot base to robot 
T_left = np.array([[0,1,0,0],[-1,0,0,0.5],[0,0,1,0.12],[0,0,0,1]])
T_right = np.array([[0,-1,0,0], [1,0,0,-0.5], [0,0,1,0.12], [0,0,0,1]])
q_left_init = mj_data.qpos[:7]
q_right_init = mj_data.qpos[13:20]

T_ee_left_init =  T_left @ robot_utils.forward_kinematics(pin_model, pin_data, eeid_pin, q_left_init)
T_ee_right_init = T_right @ robot_utils.forward_kinematics(pin_model, pin_data, eeid_pin, q_right_init)


# gains 
KP = np.diag([7000,10000,10000,800,800,900])
KD = np.diag([200,300,300, 2.1,2.1,3])

mode = "PD_test"
# mode = "hard_set_q"
print(mj_data.ctrl.shape)
# run a PD controller with torque as action 
if mode == "PD_test" : 
    P = 1000
    D = 50 
    while mj_data.time<10:
        t = mj_data.time
        q, v =  mj_data.qpos, mj_data.qvel 
        
        mj_data.ctrl[7] = 255
        mj_data.ctrl[15] = 255
        
        # get observation
        q_left, q_right = q[:7], q[13:20]
        v_left, v_right = v[:7], v[13:20]

        
        T_ee_left = T_left @ robot_utils.forward_kinematics(pin_model, pin_data, eeid_pin, q_left)
        T_ee_right = T_right @ robot_utils.forward_kinematics(pin_model, pin_data, eeid_pin, q_right)

        # here we simply set a sine wave motion for the EE while keeping the head down 
        R_ee_left_des, X_ee_left_des = T_ee_left_init[:3,:3], T_ee_left_init[:3,3]+ np.array([1,0,0])*0.2*np.sin(2*np.pi*t)
        R_ee_right_des, X_ee_right_des = T_ee_right_init[:3,:3], T_ee_right_init[:3,3]+ np.array([1,0,0])*0.2*np.sin(2*np.pi*t)

        # set desired position and orientation 
        T_des_left, T_des_right = np.eye(4), np.eye(4)
        T_des_left[:3,:3], T_des_right[:3,:3] = R_ee_left_des, R_ee_right_des
        T_des_left[:3,3], T_des_right[:3,3] = X_ee_left_des, X_ee_right_des

        # compute torque to realize end-effector impedance control 
        tau_left = robot_utils.compute_impedance_control(pin_model, pin_data, eeid_pin, q_left, v_left, KP, KD, np.linalg.inv(T_left)@T_des_left, np.zeros(6))
        tau_right = robot_utils.compute_impedance_control(pin_model, pin_data, eeid_pin, q_right, v_right, KP, KD, np.linalg.inv(T_right)@T_des_right, np.zeros(6))

        
        # send torque to corresponding motors 
        mj_data.ctrl[:7], mj_data.ctrl[8:15] = tau_left, tau_right 

        # add a torque to close gripper 
        mj_data.ctrl[7] = 255
        mj_data.ctrl[15] = 255
        
        # visualize EE frame 
        visualizer.visualize_frame(viewer, T_ee_left)
        visualizer.visualize_frame(viewer, T_ee_right)

        # step simulation 
        mujoco.mj_step(mj_model,mj_data)
        viewer.render()