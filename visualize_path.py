#!/usr/bin/python3

import os
import trimesh
import numpy as np
from typing import List

def pc(csv_path: str,
       color: list) -> trimesh.PointCloud:
    data = []
    with open(csv_path, "r") as f:
        lines = f.readlines()
    
    for line in lines:
        data.append(list(map(float, line.replace("\t", " ").replace("\n", " ").replace(",", " ").split())))
    
    return trimesh.PointCloud(np.array(data)[:, :3], colors=color)

def main():
    scene = trimesh.Scene()
    scene.add_geometry(pc(input("csv 파일 1 위치 : "), [255, 0, 0]))
    scene.add_geometry(pc(input("csv 파일 2 위치 : "), [0, 255, 0]))
    scene.add_geometry(pc(input("csv 파일 3 위치 : "), [0, 0, 255]))
    scene.show()

if __name__ == "__main__":
    main()