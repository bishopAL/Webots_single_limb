import numpy as np
from controller import Robot
import math
from math import asin, atan2
import matplotlib.pyplot as plt

robot = Robot()
timestep = 10

robot_motor = []
robot_motor.append(robot.createMotor("RF0"))
robot_motor.append(robot.createMotor("RF1"))
robot_motor.append(robot.createMotor("RF2"))
for i in range(3):
    robot_motor[i].enableForceFeedback(1)

robot_pos_sensor = []
robot_pos_sensor.append(robot.createPositionSensor("RF0 sensor"))
robot_pos_sensor.append(robot.createPositionSensor("RF1 sensor"))
robot_pos_sensor.append(robot.createPositionSensor("RF2 sensor"))

L1 = 0.07
L2 = 0.05
L3 = 0.024
timeFlag = 0

T = 300 # swing phase period
v_yd = 3.0 # target forward speed
dy0 = 2.0 # init forward speed
y0 = 0.0 # cordinate y
z0 = 0.0 #cordinate z
zh = 20.0 #cordinate z max
yt = v_yd * (T / 100) / 2 # land pos

M = 1.0
B = 0.5
K = 0.5
C = 2.0 # stiffness
t_p = np.array([0.0, 0.0, 0.0])

Xd_f = np.array([0.0, 0.0, 0.0])
dXd_f = np.array([0.0, 0.0, 0.0])
d2Xd_f = np.array([0.0, 0.0, 0.0])

Xd = np.array([0.0, 0.0, 0.0])
d2Xd = np.array([0.0, 0.0, 0.0])
dXd = np.array([0.0, 0.0, 0.0])

Xc = np.array([0.0, 0.0, 0.0])
d2Xc = np.array([0.0, 0.0, 0.0])
dXc = np.array([0.0, 0.0, 0.0])

Xc_f = np.array([0.0, 0.0, 0.0])
dXc_f = np.array([0.0, 0.0, 0.0])
d2Xc_f = np.array([0.0, 0.0, 0.0])

px_list = []
py_list = []
pz_list = []
p_alpha_list = []
p_gamma_list = []
p_beta_list = []
i = 0
Xd_list = []
Xc_list = []
while robot.step(timestep) != -1:
    px = 70.0
    if i < 0.25 * T:
        t = float(i) / T
        T1 = T / T
        py = (-4 * dy0 / T1) * (t ** 2) + dy0 * t + y0
        pz = (16 * z0 - 16 * zh) * (t ** 3) / (T1 ** 3) + (12 * zh - 12 * z0) * (t ** 2) / (T1 ** 3) + z0
    elif i < 0.5 * T:
        t = float(i) / T
        T1 = T / T
        py = (-4 * T1 * dy0 - 16 * yt + 16 * y0) / (T1 ** 3) * (t ** 3) + (7 * T1 * dy0 + 24 * yt - 24 * y0) / (T1 ** 2) * (t ** 2) + (-15 * T1 * dy0 - 36 * yt + 36 * y0) / (T1 * 4) * t + (9 * T1 * dy0 + 16 * yt) / 16
        pz = (16 * z0 - 16 * zh) * (t ** 3) / (T1 ** 3) + (12 * zh - 12 * z0) * (t ** 2) / (T1 ** 3) + z0
    elif i < 0.75 * T:
        t = float(i) / T
        T1 = T / T
        py = (-4 * T1 * dy0 - 16 * yt + 16 * y0) / (T1 ** 3) * (t ** 3) + (7 * T1 * dy0 + 24 * yt - 24 * y0) / (T1 ** 2) * (t ** 2) + (-15 * T1 * dy0 - 36 * yt + 36 * y0) / (T1 * 4) * t + (9 * T1 * dy0 + 16 * yt) / 16
        pz = (4 * z0 - 4 * zh) * (t ** 2) / (T1 ** 2) - (4 * z0 - 4 * zh) * t / T1 + z0
    else:
        t = float(i) / T
        T1 = T / T
        py = yt
        pz = (4 * z0 - 4 * zh) * (t ** 2) / (T1 ** 2) - (4 * z0 - 4 * zh) * t / T1 + z0
    py = py + 50.0
    pz = pz - 24.0
    p_alpha = asin(-24.0 / (px ** 2 + pz ** 2) ** 0.5) - atan2(pz, px)
    p_gamma = asin((70.0 ** 2 + 50.0 ** 2 + 24.0 ** 2 - px ** 2 - py ** 2 - pz ** 2) / (2 * 70.0 * 50.0))
    p_beta = asin((px ** 2 + py ** 2 + pz ** 2 + 70.0 ** 2 - 50.0 ** 2 - 24.0 ** 2) / (2 * 70.0 * (px ** 2 + py ** 2 + pz ** 2 - 24.0 ** 2) ** 0.5)) - atan2((px ** 2 + pz ** 2 - 24.0 ** 2) ** 0.5, py)
    Xd = np.array([p_alpha, p_beta, p_gamma])
    dXd = (Xd - Xd_f) / 0.01
    d2Xd = (dXd - dXd_f) / 0.01
    Xd_list.append(Xd)
    for times in range(3):
        t_p[times] = robot_motor[times].getTorqueFeedback()
        #print(robot_motor[times].getTorqueFeedback())
    
    d2Xc = d2Xd - (t_p - B * (dXd - dXc_f) - K * (Xd - Xc_f)) / M
    dXc = dXc_f + d2Xc * 0.01
    Xc = Xc_f + dXc * 0.01
    Xc_list.append(Xc)
    
    Xd_f = Xd
    dXd_f = dXd
    d2Xd_f = d2Xd
    
    Xc_f = Xc
    dXc_f = dXc
    d2Xc_f = d2Xc
    
    Xcc = C * Xc
    for times in range(3):
        robot_motor[times].setPosition(Xc[times])
    i += 1
    if i==300:
        i = 0
        np.savetxt('Xd_list.csv', np.array(Xd_list), delimiter=',')
        np.savetxt('Xc_list.csv', np.array(Xc_list), delimiter=',')
        