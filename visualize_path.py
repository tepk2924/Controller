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
        data.append(list(map(str, line.replace("\t", " ").replace("\n", " ").replace(",", " ").split())))
    
    return trimesh.PointCloud(np.array(data)[:, :3], colors=color)

def main():
    scene = trimesh.Scene()
    scene.add_geometry(pc(input("원본 csv 파일 위치 : "), [255, 0, 0]))
    scene.add_geometry(pc(input("PSO 최적화 csv 파일 위치 : "), [0, 255, 0]))
    scene.add_geometry(pc(input("경사하강 최적화 csv 파일 위치 : "), [0, 0, 255]))
    scene.show()

if __name__ == "__main__":
    main()