import os
import trimesh
import numpy as np

def main():
    scene = trimesh.Scene()
    repo_dir = os.path.dirname(__file__)
    with open(os.path.join(repo_dir, "path", "track_data.csv"), "r") as f:
        lines = f.readlines()
    ref_coords = np.array([list(map(float, line.replace(","," ").split())) for line in lines])
    pc = trimesh.PointCloud(ref_coords, (255, 0, 0))
    scene.add_geometry(pc)
    path_geo = trimesh.load(os.path.join(repo_dir, "path", "mobilesystemcontrol.stl"))
    scene.add_geometry(path_geo)
    scene.show()

if __name__ == "__main__":
    main()