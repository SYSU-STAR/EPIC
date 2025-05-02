<!--
 * @Author: ning-zelin zl.ning@qq.com
 * @Date: 2025-03-12 21:29:31
 * @LastEditors: ning-zelin zl.ning@qq.com
 * @LastEditTime: 2025-04-03 16:54:30
 * @Description: 
 * 
 * Copyright (c) 2025 by ning-zelin zl.ning@qq.com, All Rights Reserved. 
-->
<div align = "center">
  <h1>
    EPIC: Exploring on Point Clouds 
  </h1>
</div>
<div align = "center">
  <h2>
    A Lightweight LiDAR-Based AAV Exploration Framework for Large-Scale Scenarios
  </h2>
</div>
<div align="center">
  <strong>
        Shuang Geng<sup>*</sup>,
        Zelin Ning<sup>*</sup>,
        Fu Zhang, and
        Boyu Zhou<sup>†</sup>
  </strong>
  <p>
    <sup>*</sup>Equal Contribution;&nbsp;
    <sup>†</sup>Corresponding Author
  </p>
  <a href="https://ieeexplore.ieee.org/document/10945408"><img src="https://img.shields.io/badge/Paper-IEEE RAL-004088.svg"/></a>
  <a href='https://arxiv.org/pdf/2410.14203.pdf'><img src='https://img.shields.io/badge/arXiv-2410.14203-red' alt='arxiv'></a>
  <a href='https://www.bilibili.com/video/BV1nrx5eaESY/?spm_id_from=333.1387.homepage.video_card.click&vd_source=07945b0b56417e213633c9332f4f4716'><img alt="Video" src="https://img.shields.io/badge/BiliBili-Video-purple"/></a>
</div>

## 💡 News
* **[2025.03.28]** **EPIC** is officially published by RAL ! 
* **[2025.03.12]** The source code of **EPIC** is released !
* **[2025.03.08]** **EPIC** is accepted by RAL 2025 🚀 !

## 📜 Introduction

**EPIC** (**E**xploring on **P**o**I**nt **C**louds) is a lightweight LiDAR-based AAV (Autonomous Aerial Vehicle) exploration framework that directly exploits point cloud data to explore large-scale environments. Experimental results demonstrate that our framework achieves faster exploration while significantly reducing memory consumption. (Click the image to view the video)

[![video](misc/overview.png)](https://www.bilibili.com/video/BV1nrx5eaESY/?spm_id_from=333.1387.homepage.video_card.click&vd_source=07945b0b56417e213633c9332f4f4716)

Please cite our paper if you use this project in your research:

```
@ARTICLE{10945408,
  author={Geng, Shuang and Ning, Zelin and Zhang, Fu and Zhou, Boyu},
  journal={IEEE Robotics and Automation Letters}, 
  title={EPIC: A Lightweight LiDAR-Based AAV Exploration Framework for Large-Scale Scenarios}, 
  year={2025},
  volume={10},
  number={5},
  pages={5090-5097},
  keywords={Point cloud compression;Autonomous aerial vehicles;Memory management;Laser radar;Trajectory;Surface treatment;Real-time systems;Planning;Navigation;Faces;Aerial systems: perception and autonomy;motion and path planning;aerial systems: applications},
  doi={10.1109/LRA.2025.3555878}}

```
Please kindly star ⭐️ this project if it helps you. We take great efforts to develop and maintain it 😁.

## 🛠️ Installation

### Test Environment
* Ubuntu 20.04
* ROS Noetic
* C++17

### 🚀 Quick Start

#### Clone our repository and build
```bash
git clone https://github.com/SYSU-STAR/EPIC.git
cd EPIC 
catkin build
```
#### Download dataset 
Download simulation maps from my [Google cloud](https://drive.google.com/drive/folders/1tuoVo8PL1m2cmmufkHpu4e7hK36WhJs3?usp=drive_link), create the folder `MARSIM/map_generator/resource` if it doesn't exist, and move the downloaded maps to this folder.

```bash
mkdir -p MARSIM/map_generator/resource
mv /path/to/downloaded/maps/*.pcd MARSIM/map_generator/resource/
```

#### Run program 
```bash
source ./devel/setup.zsh && roslaunch epic_planner garage.launch
```
Trigger the quadrotor by the `2D Nav Goal` in Rviz.

You can replace `garage` with other maps. We provide three test scenarios: `garage`, `cave` and `factory`.

Our simulation environment is developed based on the GPU version of MARSIM. So if you don't have a GPU, you may need to make some necessary modifications to the simulator.
## ⚠️ Known Issues

* After launching the program, if the terminal keeps displaying a `no odom` warning, this is likely due to graphics card compatibility issues. Please refer to [this issue](https://github.com/SYSU-STAR/EPIC/issues/6) for solutions.


## 🤓 Acknowledgments

We would like to express our gratitude to the following projects, which have provided significant support and inspiration for our work:
- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER): A general-purpose trajectory optimizer for multicopters, our local planner is based on it.
- [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL): An efficient framework for fast UAV exploration from which our global planner draws inspiration.
- [MARSIM](https://github.com/hku-mars/MARSIM): A lightweight point-realistic simulator for LiDAR-based UAVs upon which our simulator is built.
- [FALCON](https://github.com/HKUST-Aerial-Robotics/FALCON): An efficient framework for fast UAV exploration, from which our method for constructing topological maps draws inspiration.
