import mujoco
import mujoco.viewer
import numpy as np

# Visualize the route create by A* algorithm
def visualize(route,viewer):
    viewer.user_scn.ngeom   = 0
    tile_count              = 0 
    for point in range(len(route)):   
        mujoco.mjv_initGeom(
                      viewer.user_scn.geoms[tile_count],
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size = [0.1, 0.,0.],
                      pos =np.array([route[point,0], route[point,1], 0]),
                      mat =np.eye(3).flatten(),
                      rgba=np.array([0.89, 0.051, 0.051,0.4])
        )            
        tile_count += 1
    viewer.user_scn.ngeom = tile_count
    viewer.sync()
    return tile_count
# color the robot's current position
def addpoint(point,viewer,tile_count):
    tile_count = tile_count + 1 
    mujoco.mjv_initGeom(
                      viewer.user_scn.geoms[tile_count],
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size = [0.05, 0.,0.],
                      pos =np.array([point[0], point[1], 0]),
                      mat =np.eye(3).flatten(),
                      rgba=np.array([0.016, 0.224, 0.451,1.])
    )            
    viewer.user_scn.ngeom = tile_count
    viewer.sync()
    return tile_count

# Initialize the dynamic trajectory
def initalize_route(route_length,viewer,start_count):
    counter = start_count
    for point in range(route_length): 
        mujoco.mjv_initGeom(
                      viewer.user_scn.geoms[start_count+point],
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size = [0.025, 0.,0.],
                      pos =np.array([0, 0, 0]),
                      mat =np.eye(3).flatten(),
                      rgba=np.array([0.40000, 0.00000, 0.40000, 0.9])
        )
        counter +=1     
    viewer.user_scn.ngeom = counter       
    viewer.sync()
    return counter

# Update and modify the instantiated objects and use them to represent the selected trajectory
def vizualize_route(route,viewer,start_count):
    for counter in range(len(route)): 
        marker                     = viewer.user_scn.geoms[start_count+counter]
        marker.pos                 = [route[counter,0],route[counter,1],0.05]     
    viewer.sync()
