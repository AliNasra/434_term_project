import mujoco
import mujoco.viewer
import numpy as np

def visualize(route,viewer):
    viewer.user_scn.ngeom   = 0
    tile_count              = 0
    for point in range(len(route)):   
        mujoco.mjv_initGeom(
                      viewer.user_scn.geoms[tile_count],
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size = [0.05, 0.,0.],
                      pos =np.array([route[point,0], route[point,1], 0.05]),
                      mat =np.eye(3).flatten(),
                      rgba=np.array([0.89, 0.051, 0.051,0.4])
        )            
        tile_count += 1