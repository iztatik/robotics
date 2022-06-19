import numpy as np
import pybullet as pb
import pybullet_data
from Planner.planner import Planner
import time

# Make an instance of a physic client
phyClient = pb.connect(pb.GUI)

# Set/unset real time simulation
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Charge objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
floor = pb.loadURDF('plane.urdf')
robot = pb.loadURDF('URDF/scara.urdf', useFixedBase=1)

# Simulation parameters
simTime = 3 
dt = 0.01

q0 = np.array([0, 0, 0, 0])
qf = np.array([np.pi/4, np.pi/4, 0.3, 0])

v0 = np.array([0, 0, 0, 0])
vf = np.array([0, 0, 0, 0])

# Trajectory tracer
my_planner = Planner()
_, _, q = my_planner.jTraj3(simTime, dt, q0, qf, v0, vf)
print(q)


# Execute the simulation
input('Press Enter to continue...')
for i in range(q.shape[0]):
    # Move the robot
    joint_positions = q[i, :] 
    pb.setJointMotorControlArray(robot, range(4), pb.POSITION_CONTROL, targetPositions=joint_positions)

    time.sleep(dt)
    pb.stepSimulation()

# End program
input('Press Enter to stop...')
pb.disconnect()

