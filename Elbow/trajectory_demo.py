import numpy as np
import pybullet as pb
import pybullet_data
from solver.ik_solver import Ik_solver
from planner.planner import Planner
import time

# Make an instance of a physic client
client = pb.connect(pb.GUI)

# Set/unset real time simulation
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Charge objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
ground = pb.loadURDF('plane.urdf')
table = pb.loadURDF('table/table.urdf')
cube = pb.loadURDF('cube.urdf', basePosition = [0.15, 0.0, 0.775], globalScaling = 0.3)
elbow = pb.loadURDF('urdf/elbow.urdf.xml', basePosition=[-0.4, 0.0, 0.625], useFixedBase=1)

# Simulation parameters
dt = 0.01

# Desired poses
o = [(0.4, -0.15, 0.0), 
     (0.4, -0.15, 0.3),
     (0.4,  0.15, 0.3), 
     (0.4,  0.15, 0.0)]
rpy = [(-np.pi/4, 3*np.pi/4, 0.0),
       (-np.pi/4, 3*np.pi/4, 0.0), 
       ( np.pi/4, 3*np.pi/4, 0.0), 
       ( np.pi/4, 3*np.pi/4, 0.0)]
duration = [4.0, 5.0, 4.0, 5.0]

input('Press Enter to continue...')
# Trajectory tracer
solver = Ik_solver(0.17, 0.25, 0.23, 0.16)
planner = Planner()
home = np.zeros(6)
q0 = home

for i in range(len(o)):
    qf = solver.solve(o[i], rpy[i], -1)
    _, _, q = planner.jTraj3(duration[i], dt, q0, qf, np.zeros(6), np.zeros(6))
    q0 = qf
    for j in range(q.shape[0]):
        joint_positions = q[j, :]
        pb.setJointMotorControlArray(elbow, range(6), pb.POSITION_CONTROL, targetPositions=joint_positions)
        time.sleep(dt)
        pb.stepSimulation()
    time.sleep(1)

# End program
input('Press Enter to stop...')
pb.disconnect()

