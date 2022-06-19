import pybullet as pb
import pybullet_data

# Make an instance of a physical client
client = pb.connect(pb.GUI)

# Set the simulation parameters
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Open the objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
ground = pb.loadURDF('plane.urdf')
table = pb.loadURDF('table/table.urdf')
cube = pb.loadURDF('cube.urdf', basePosition = [0.2, 0.0, 0.8], globalScaling = 0.4)
robot = pb.loadURDF('urdf/elbow.urdf.xml', basePosition=[-0.5, 0.0, 0.6], useFixedBase=1)

# End the program
input('Press ENTER to stop...')
pb.disconnect()

