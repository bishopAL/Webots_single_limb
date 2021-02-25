import numpy as np
from controller import Robot
import math

robot = Robot()
timestep = 1

robot_motor = []
robot_motor.append(robot.createMotor("RF0"))
robot_motor.append(robot.createMotor("RF1"))
robot_motor.append(robot.createMotor("RF2"))

robot_pos_sensor = []
robot_pos_sensor.append(robot.createPositionSensor("RF0 sensor"))
robot_pos_sensor.append(robot.createPositionSensor("RF1 sensor"))
robot_pos_sensor.append(robot.createPositionSensor("RF2 sensor"))

L1 = 0.07
L2 = 0.05
L3 = 0.024
timeFlag = 0

while robot.step(timestep) != -1:
    for i in range(3):
        robot_motor.setPosition