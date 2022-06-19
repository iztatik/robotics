import pybullet as pb
from solver.ik_solver import Ik_solver
import pybullet_data
import numpy as np

# Make an instance of a physic client
phy_client = pb.connect(pb.GUI)

# Simulation parameters
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
ground = pb.loadURDF('plane.urdf')
table = pb.loadURDF('table/table.urdf') 
cube = pb.loadURDF('cube.urdf', basePosition = [0.15, 0.0, 0.775], globalScaling = 0.3)
elbow = pb.loadURDF('urdf/elbow.urdf.xml', basePosition=[-0.4, 0.0, 0.625], useFixedBase=1)

# Inverse kinematics
o = (0.4, -0.15, 0.3)
rpy = (0.0, np.pi/2, np.pi/4)
my_solver = Ik_solver(0.17, 0.25, 0.23, 0.16)
q = my_solver.solve(o, rpy, -1)
print('q: ', q)


# Execute simulation
pb.setJointMotorControlArray(elbow, range(6), pb.POSITION_CONTROL, targetPositions = q)

input('Press any key to continue')
for _ in range(4000):
    pb.stepSimulation()

# Disconnect the physic client
input('Press any key to stop...')
pb.disconnect()


