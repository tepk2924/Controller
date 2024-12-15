#!/usr/bin/python3

import os
import trimesh
import numpy as np
from typing import List, Tuple

def pc_set(csv_path: str,
           color: list) -> Tuple[trimesh.PointCloud, np.ndarray]:
    data = []
    with open(csv_path, "r") as f:
        lines = f.readlines()
    
    for line in lines:
        data.append(list(map(float, line.replace("\t", " ").replace("\n", " ").replace(",", " ").split())))
    
    return trimesh.PointCloud(np.array(data)[:, :3], colors=color), np.array(data)[:, :3]

def cyl(pt1: np.ndarray,
        pt2: np.ndarray) -> trimesh.Trimesh:
    diff = pt2 - pt1
    cylinder = trimesh.creation.cylinder(radius=0.1, height=np.linalg.norm(diff), sections=3)
    cylinder.apply_translation([0, 0, np.linalg.norm(diff)/2])
    z_ = diff/np.linalg.norm(diff)
    x_ = np.cross([0, 0, 1], z_)
    x_ = x_ / np.linalg.norm(x_)
    y_ = np.cross(z_, x_)
    cylinder.apply_transform(np.array([[x_[0], y_[0], z_[0], pt1[0]],
                                       [x_[1], y_[1], z_[1], pt1[1]],
                                       [x_[2], y_[2], z_[2], pt1[2]],
                                       [0, 0, 0, 1]]))
    return cylinder

def main():
    scene = trimesh.Scene()
    pc_1, data_1 = pc_set(input("csv 파일 1 위치 : "), [255, 0, 0])
    scene.add_geometry(pc_1)
    [scene.add_geometry(cyl(data_1[idx], data_1[(idx + 1)%len(data_1)])) for idx in range(len(data_1))]
    # scene.add_geometry(pc(input("csv 파일 2 위치 : "), [0, 255, 0]))
    # scene.add_geometry(pc(input("csv 파일 3 위치 : "), [0, 0, 255]))
    scene.show()

if __name__ == "__main__":
    main()