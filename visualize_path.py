#!/usr/bin/python3

import os
import trimesh
import numpy as np

def main():
    scene = trimesh.Scene()
    repo_dir = os.path.dirname(__file__)
    with open(os.path.join(repo_dir, "path", "racing_line_midpoints.csv"), "r") as f:
        lines = f.readlines()
    ref_coords = np.array([list(map(float, line.replace(","," ").split())) for line in lines])[:, :3]
    pc = trimesh.PointCloud(ref_coords, (255, 0, 0))
    coor = [[-5, -24, 6]]
    pc2 = trimesh.PointCloud(coor, (255, 255, 0))
    scene.add_geometry(pc)
    scene.add_geometry(pc2)
    # path_geo = trimesh.load(os.path.join(repo_dir, "path", "mobilesystemcontrol.stl"))
    # scene.add_geometry(path_geo)
    scene.show()

if __name__ == "__main__":
    main()