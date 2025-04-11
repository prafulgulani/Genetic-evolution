import pybullet as p
import pybullet_data as pd
import cw_envt as envt

import creature

p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# plane_shape = p.createCollisionShape(p.GEOM_PLANE)
# floor = p.createMultiBody(plane_shape, plane_shape)
p.setGravity(0, 0, -10)
# envt.make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5)
p.setAdditionalSearchPath(pd.getDataPath())
# p.setAdditionalSearchPath('shapes/')
# mountain = p.loadURDF("mountain.urdf", mountain_position, mountain_orientation, useFixedBase=1)
# mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)
mountain_position = (0, 0, -1)  # Adjust as needed
mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)
envt.make_arena(arena_size=40, wall_height=2)


# plane = p.loadURDF("plane.urdf")
dummy_object = p.loadURDF("plane.urdf", [0, 0, -1])
cube = p.loadURDF("r2d2.urdf", [0, 0, 10])
for _ in range(10):
    print("i am runnning")
    p.stepSimulation()
p.setRealTimeSimulation(1)
print("i am runnning")
# p.disconnect()

