from controller import Supervisor,Connector
from ikpy.chain import Chain
import pandas as pd
import numpy as np
import time
import random
import cv2
import math
import random
import csv

import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="ikpy.chain")


supervisor = Supervisor()
# get the time step of the current world.
robot_chain = Chain.from_urdf_file("LRMate-200iD.urdf",base_elements=['Base'])

# print(robot_chain.links)
timestep = int(supervisor.getBasicTimeStep())

#利用順項運動學計算出末端軸位置
def get_endpoint_position(angles):
    endpoint_position=robot_chain.forward_kinematics(angles)
    return endpoint_position

#利用逆向運動學，根據末端軸位置以及角度推算手臂個軸角度
def get_IK_angle(target_position, target_orientation=[1, 0, 0], orientation_axis="X",starting_nodes_angles=[0,0,0,0,0,0,0]): 
    # 初始化機器人鏈條
    
    # for i, link in enumerate(robot_chain.links):
    #     print(f"Joint {i}: {link.name}")
    # 計算逆向運動學
    # ikAnglesD = robot_chain.inverse_kinematics(target_position,target_orientation=[0, 1, 0])#不限制角度
    ikAnglesD= robot_chain.inverse_kinematics(
    target_position,
    target_orientation=target_orientation,
    orientation_mode=orientation_axis,
    initial_position=starting_nodes_angles
    )#限制角度以及末端軸位置
    
    return ikAnglesD

#將旋轉矩陣轉換為歐拉角
def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles (in degrees) with ZYX order.
    
    :param R: np.ndarray, a 3x3 rotation matrix.
    :return: tuple of Euler angles (rx, ry, rz) in degrees.
    """
    # Check for valid rotation matrix
    if not np.allclose(np.dot(R, R.T), np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(R), 1.0):
        raise ValueError("Invalid rotation matrix")
    
    # Extract angles
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    
    singular = sy < 1e-6
    
    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0
    
    # Convert radians to degrees
    rx = math.degrees(rx)
    ry = math.degrees(ry)
    rz = math.degrees(rz)
    
    return rx, ry, rz

#輸入工件路徑採樣點研磨時的座標平移與歐拉角，輸出工件座標研磨時的平移與歐拉角
def calculate_A_prime(R_BA, t_BA, R_B_prime, t_B_prime):
    """
    根据 B' 座标系在世界座标系下的表示，计算 A' 在世界座标系下的表示。

    :param R_BA: np.ndarray, B 座标系在 A 座标系下的旋转矩阵 (3x3).
    :param t_BA: np.ndarray, B 座标系在 A 座标系下的平移向量 (3x1).
    :param R_B_prime: np.ndarray, B' 座标系在世界座标系下的旋转矩阵 (3x3).
    :param t_B_prime: np.ndarray, B' 座标系在世界座标系下的平移向量 (3x1).
    :return: (R_A_prime, t_A_prime), A' 座标系在世界座标系下的旋转矩阵和平移向量.
    """
    # 计算 A' 的旋转矩阵
    R_A_prime = R_B_prime @ np.linalg.inv(R_BA)
    
    # 计算 A' 的平移向量
    t_A_prime = t_B_prime - R_A_prime @ t_BA
    R_A_prime=rotation_matrix_to_euler_angles(R_A_prime)
    return R_A_prime, t_A_prime

#將歐拉角轉換為旋轉矩陣
def Rotation_matrix(rx,ry,rz):
    # Convert degrees to radians
    rx = math.radians(rx)
    ry = math.radians(ry)
    rz = math.radians(rz)
    
    # Calculate the rotation matrix
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(rx), -math.sin(rx)],
                    [0, math.sin(rx), math.cos(rx)]])
    
    R_y = np.array([[math.cos(ry), 0, math.sin(ry)],
                    [0, 1, 0],
                    [-math.sin(ry), 0, math.cos(ry)]])
    
    R_z = np.array([[math.cos(rz), -math.sin(rz), 0],
                    [math.sin(rz), math.cos(rz), 0],
                    [0, 0, 1]])
    
    # Combine the rotation matrices
    R = np.dot(np.dot(R_z, R_y), R_x)
    return R

#欧拉角轉换为轴-角表示(webots內的rotation以轴-角表示)
def euler_to_axis_angle(rx, ry, rz):
    """
    Converts Euler angles (in degrees) to axis-angle representation.
    
    Args:
    rx: Rotation around x-axis in degrees.
    ry: Rotation around y-axis in degrees.
    rz: Rotation around z-axis in degrees.
    
    Returns:
    A tuple of four values representing the axis-angle (axis_x, axis_y, axis_z, angle).
    """
    # Convert degrees to radians
    rx = math.radians(rx)
    ry = math.radians(ry)
    rz = math.radians(rz)
    
    # Calculate the rotation matrix
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(rx), -math.sin(rx)],
                    [0, math.sin(rx), math.cos(rx)]])
    
    R_y = np.array([[math.cos(ry), 0, math.sin(ry)],
                    [0, 1, 0],
                    [-math.sin(ry), 0, math.cos(ry)]])
    
    R_z = np.array([[math.cos(rz), -math.sin(rz), 0],
                    [math.sin(rz), math.cos(rz), 0],
                    [0, 0, 1]])
    
    # Combine the rotation matrices
    R = np.dot(np.dot(R_z, R_y), R_x)
    
    # Calculate the axis-angle representation
    angle = math.acos((np.trace(R) - 1) / 2)
    sin_angle = math.sin(angle)
    
    if sin_angle > 1e-6:  # Avoid division by zero
        axis_x = (R[2, 1] - R[1, 2]) / (2 * sin_angle)
        axis_y = (R[0, 2] - R[2, 0]) / (2 * sin_angle)
        axis_z = (R[1, 0] - R[0, 1]) / (2 * sin_angle)
    else:
        # If the angle is very small, the axis is not well-defined, return the default axis
        axis_x = 1
        axis_y = 0
        axis_z = 0

    return axis_x, axis_y, axis_z, angle
#------------------------------------------------------工件姿態反推手臂末端軸姿態

def get_transformation_matrix(rotation, translation):
    """
    根據旋轉矩陣和平移向量生成4x4的齊次轉換矩陣
    """
    T = np.eye(4)
    T[:3, :3] = rotation  # 設置旋轉部分
    T[:3, 3] = translation  # 設置平移部分
    return T

def invert_transformation_matrix(T):
    """
    反轉4x4齊次轉換矩陣
    """
    R_inv = T[:3, :3].T  # 旋轉矩陣的轉置
    t_inv = -R_inv @ T[:3, 3]  # 平移向量的反轉
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

def calculate_B_prime(A_rot, A_trans, B_rot, B_trans, A_prime_rot, A_prime_trans):#從
    """
    計算B'的旋轉與平移矩陣
    給予一個A坐標系與B坐標系相對世界座標的轉移矩陣,接著給出A座標系移動後的新座標系A',假設B'相對A'位置與B相對A相同,輸入A,B,A'的旋轉以及平移矩陣,求出B'的平移與旋轉矩陣

    """
    # Step 1: 計算齊次轉換矩陣
    T_A_W = get_transformation_matrix(A_rot, A_trans)
    T_B_W = get_transformation_matrix(B_rot, B_trans)
    T_A_prime_W = get_transformation_matrix(A_prime_rot, A_prime_trans)

    # Step 2: 計算T_B_A
    T_A_W_inv = invert_transformation_matrix(T_A_W)
    T_B_A = T_A_W_inv @ T_B_W

    # Step 3: 計算T_B_prime_W
    T_B_prime_W = T_A_prime_W @ T_B_A

    # Step 4: 提取B'的旋轉和平移矩陣
    B_prime_rot = T_B_prime_W[:3, :3]
    B_prime_trans = T_B_prime_W[:3, 3]

    return B_prime_rot, B_prime_trans

# def check_robot_box_errors(box_size):
#     sensors = []
#     for motor in motors:
#         sensor = motor.getPositionSensor()
#         sensor.enable(timestep)
#         sensors.append(sensor.getValue())
    
#     box_position=box_translation_field.getSFVec3f()
#     print("errors=[{:.6f}, {:.6f}, {:.6f}]".format(
#         box_position[0]-robot_pt[0][3],box_position[1]-robot_pt[1][3],box_position[2]-robot_pt[2][3]))
#-----------------------------------------------------------------------

# 取得力感測器
# force_sensor = supervisor.getDevice("touch sensor")
# if force_sensor is None:
#     print(" ERROR: 'force_sensor' not found!")
#     exit(1)
    
# force_sensor.enable(timestep)  # 啟用感測器

# 準備儲存清單
data = []    # 每筆： [time(s), fx, fy, fz]
t = 0.0

#控制手臂各軸馬達
motors = []
motors.append(supervisor.getDevice('J1'))
motors.append(supervisor.getDevice('J2'))
motors.append(supervisor.getDevice('J3'))
motors.append(supervisor.getDevice('J4'))
motors.append(supervisor.getDevice('J5'))
motors.append(supervisor.getDevice('J6'))


paths=np.linspace([0.5,0,0.7],[0.5,0,0.6],int((0.8-0.72)/0.01))
# paths=np.linspace([0,0,0],[0,0,0],int((0.8-0.72)/0.01))
feeds=np.array([[0,0,0] for _ in range(len(paths))])
# feeds=np.array([[0.001*int(random.random()*10),0,0] for _ in range(len(paths))])
box_size=0.05

each_step=0.0001 #1次走0.0001m=>0.1mm

pre_robot_pt=None
error_limit=0.001

in_robot_paths=[]
#最長0.911m
origin=get_endpoint_position([0 for _ in range(7)])
origin_pt=np.array([origin[0][3],origin[1][3],origin[2][3]])
origin_pt_stpt_near=np.linspace(origin_pt,paths[0],int(np.linalg.norm(paths[0]-origin_pt)/0.01))[-3]
origin_pt_edpt_near=np.linspace(paths[-1],origin_pt,int(np.linalg.norm(paths[0]-origin_pt)/0.01))[2]
start_end_paths = np.vstack([origin_pt_stpt_near, paths, origin_pt_edpt_near])
print("start_end_paths=",start_end_paths)
for _ in range(len(paths)-1):
    st=start_end_paths[_]+feeds[_]
    ed=start_end_paths[_+1]+feeds[_+1]
    in_robot_paths.append(np.linspace(st, ed, 
                                      int(np.linalg.norm(ed-st)/each_step)))
in_robot_paths = np.vstack(in_robot_paths)
sensors = []
for motor in motors:
    sensor = motor.getPositionSensor()
    sensor.enable(timestep)
    sensors.append(sensor)
i=0


while supervisor.step(timestep) != -1:
    ## 三軸
    # force_values = force_sensor.getValues()  # 取得感測數據
    # # print(f"End-Effector Force: X={force_values[0]:.3f}, Y={force_values[1]:.3f}, Z={force_values[2]:.3f}")
    # fx, fy, fz = force_sensor.getValues()[:3]         # 讀取三軸力
    # print("fx={:.3f},fy={:.3f},fz={:.3f}".format(fx, fy, fz))
    # data.append([32/1000., fx, fy, fz])

    ## 單軸
    # fz = force_sensor.getValue()  # 取得感測數據
    # # data.append([t, fz])
    # print(f'Fz = {fz:.3f} N')


    # t += timestep / 1000.0              # basicTimeStep 單位是 ms
    # t += timestep/240

    if i==0:
        for _ in range(6): 
            motors[_].setPosition(0)
        for _ in range(10):
            ball_node = supervisor.getFromDef('ball'+str(_+1))
            # ball_translation_field = ball_node.getField('translation')
            # ball_translation_field.setSFVec3f(start_end_paths[_].tolist())
    if i==1:
        print("----------start path----------")       
    if i == len(in_robot_paths):
        print("----------end path----------")
        robot_to_pt=origin_pt
        sensor_values = [s.getValue() for s in sensors]
        angles=get_IK_angle(robot_to_pt,starting_nodes_angles=[0]+sensor_values)
        for n, motor in enumerate(motors):
            motor.setPosition(angles[n+1])
        print("i={},error={}".format(i,error))
        break

    sensor_values = [s.getValue() for s in sensors]
    robot_pt=get_endpoint_position([0]+sensor_values)
        
    robot_now_pt=np.array([robot_pt[0][3],robot_pt[1][3],robot_pt[2][3]])
    error=np.array([0,0,0])
    if pre_robot_pt is not None:
        error=robot_now_pt-pre_robot_pt
        if np.any(np.abs(error) > error_limit):
            print("i={},error={}".format(i,error))
    
    robot_to_pt=in_robot_paths[i]
    angles=get_IK_angle(robot_to_pt,starting_nodes_angles=[0]+sensor_values)
    for n, motor in enumerate(motors):
        motor.setPosition(angles[n+1])
    
    pre_robot_pt=robot_to_pt.copy()

    i=i+1
    
# 結束時寫成 CSV
# with open('force_data.csv', 'w', newline='') as f:
#     writer = csv.writer(f)
#     writer.writerow(['time','fx','fy','fz'])
#     writer.writerows(data)
# print("已將感測資料儲存到 force_data.csv")

# with open('force_data.csv', 'w', newline='') as f:
#     writer = csv.writer(f)
#     writer.writerow(['time','fz'])
#     writer.writerows(data)
# print("已將感測資料儲存到 force_data.csv")

 #--------------------------------------------------
    