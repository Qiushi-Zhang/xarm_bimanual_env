import mujoco 
import pinocchio as pin
import numpy as np

def forward_kinematics(model,data,frame_id,q):
    pin.forwardKinematics(model,data,q)
    pin.updateFramePlacements(model,data)
    T = data.oMf[frame_id].homogeneous
    return T 

def compute_frame_err(T1,T2):
    T1 = pin.SE3(T1)
    T2 = pin.SE3(T2)
    err = pin.log(T1.actInv(T2)).vector

    
    return err

def compute_jacobian(model,data,frame_id,q):
    #Computes the Jacobian for a frame placed at a specified origin and aligned with the world frame.
    J = pin.computeFrameJacobian(model,data, q,frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    return J 



def compute_impedance_control(pin_model, pin_data, eeid, q, v, kp, kd, T_des, V_des):
    T_ee = forward_kinematics(pin_model, pin_data, eeid, q)
    J_ee = compute_jacobian(pin_model, pin_data, eeid, q)
    V_ee = J_ee@v
    x_ee = T_ee[:3,3]
    R_ee = T_ee[:3,:3]

    x_des = T_des[:3,3]
    Ree_des = T_des[:3,:3]
    x_err = np.concatenate([x_des - x_ee, pin.rpy.matrixToRpy(Ree_des @ R_ee.T)])
    # print(x_err)
    v_err = V_des - V_ee 

    tau = J_ee.T @ (kp @(x_err) + kd @(v_err))
    return tau 
    

        
