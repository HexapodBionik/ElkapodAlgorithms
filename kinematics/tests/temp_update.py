from kinematics.kinematics_solvers import KinematicsSolver
import numpy as np

m1 = np.array([0, 0, 1])
a1 = np.array([1, 0, 0])
a2 = np.array([3, 0, 0])
a3 = np.array([2, 0, 0])
q1 = 0
q2 = 25
q3 = -45
q3_ma = -30

ma_angles = np.array((0, 0, 15))
test_kinematics = KinematicsSolver(m1, a1, a2, a3, ma_angles)

a = test_kinematics.forward(np.array([q1, q2, q3_ma]))
print(a)
print(test_kinematics.inverse(a))