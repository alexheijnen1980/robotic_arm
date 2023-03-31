import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# puma = rtb.models.DH.Puma560()
# print(puma)
# print(puma.qr)
# puma.plot([0, 0, 0, 0, 0, 0])

# ## Create robotic arm using Denavit-Hartenberg parameters
link_1 = rtb.RevoluteDH(d = 0.100, a = 0, alpha = 0)
link_2 = rtb.RevoluteDH(d = 0.028, a = 0, alpha = 0.5*np.pi )
link_3 = rtb.RevoluteDH(d = 0.040, a = -0.08004, alpha = 1.0*np.pi)
link_4 = rtb.RevoluteDH(d = 0.006, a = -0.11003, alpha = 0)
robot = rtb.DHRobot([link_1, link_2, link_3, link_4])
print(robot)

# Forward kinematics: define joint positions and determine end effector position
q1 = np.radians(0)
q2 = -0.44
q3 = -2.94
q4 = -0.78
T = robot.fkine([q1, q2, q3, q4])
print("Transformation Matrix :\n", T)
(robot.plot([q1, q2, q3, q4]))
print('hello')

# # # Inverse kinematics
# # point = SE3(-0.1901, -0.1644, 0.175)
# # point_solution = robot.ikine_LM(T)
# # print(point_solution)
