import numpy as np
import pybullet as pb
import pybullet_data
from Solver.ik_solver import IkSolver

# Make an instance of a physic client
phyClient = pb.connect(pb.GUI)

# Set/unset real time simulation
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Charge objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
floor = pb.loadURDF('plane.urdf')
robot = pb.loadURDF('URDF/scara.urdf', useFixedBase=1)

# Inverse kinematics
my_scara = IkSolver(0.5, 0.5, 1.0, 0.2)
th1, th2, d3, th4 = my_scara.ikSolve(0.7, 0.3, 0.35, np.pi/5)

# Move the robot
joint_positions = [th1, th2, d3, th4]
pb.setJointMotorControlArray(robot, range(4), pb.POSITION_CONTROL, targetPositions=joint_positions)
print('Joint positions: ', joint_positions)

# Execute the simulation
input('Press Enter to continue...')
for _ in range(1000):
    pb.stepSimulation()

#Get the position of the joint states
q = pb.getJointStates(robot, range(4))
print(q)

# End program
input('Press Enter to stop...')
pb.disconnect()

