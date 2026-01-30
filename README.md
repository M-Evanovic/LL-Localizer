<div align="center">
  <h1>LL-Localizer</h1>
  <h2>A Lifelong Localization System based on Dynamic i-Octree</h2>
  <p><strong>Xinyi Li, Shenghai Yuan, Haoxin Cai, Shunan Lu, Wenhua Wang, and Jianqi Liu</strong></p>
  <br>

  [![arXiv](https://img.shields.io/badge/arXiv-b31b1b.svg)](http://arxiv.org/abs/2504.01583)
  [![TIM](https://img.shields.io/badge/IEEE%20TIM-2025-005BBB?logo=ieee&logoColor=white)](https://ieeexplore.ieee.org/document/11123901)
  [![YouTube](https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white)](https://youtu.be/UWn7RCb9kA8)
  [![Bilibili](https://img.shields.io/badge/Bilibili-00A1D6?logo=bilibili&logoColor=white)](https://www.bilibili.com/video/BV1faZHYCEkZ)

  <strong>This work has been accpeted at <i>IEEE Transactions on Instrumentation and Measurement (TIM).  </i></strong>

  <img src="pic/introduction.jpg" />
</div>


Relative experiments videos on real AGV: [indoor](https://www.bilibili.com/video/BV16KCKBREHe/) and [outdoor](https://www.bilibili.com/video/BV1nKCKBREwY)

## Statement
All experiments, including both simulation and real-world experiments in paper and on real AGV, were conducted on Ubuntu 22.04 with ROS2 Humble. Due to commercial applications and ongoing academic research, the complete code deployed on the real vehicle is not fully open-sourced. Instead, we provide a simplified demo implementation based on [Faster-LIO](https://github.com/gaoxiang12/faster-lio).

## Requirements
- Ubuntu 20.04  
- ROS Noetic
- Eigen  
- PCL 

## Build
```
mkdir -p /workspace/src
cd /workspace/src
git clone https://github.com/M-Evanovic/LL-Localizer.git
cd /workspace
catkin build
source devel/setup.bash
```

## Run SLAM Mode
- Set the path ```/localization_param/map_file_name``` to ```" "``` in ```/config/config.yaml```. When no prior map is available, the system automatically switches to SLAM mode.
- Derectly launch ll_localizer:
```
roslaunch ll_localizer localizer_velodyne.launch
```
- And play your rosbags:
```
rosbag play yourbag.bag
```

## Run Localization Mode
- If you want to use a prior map to run the localization mode, please put the map (.pcd file) in ```/map```, and modify the path ```/localization_param/map_file_name``` in ```/config/config.yaml```.  
- Then launch ll_localizer:
```
roslaunch ll_localizer localizer_velodyne.launch
```
- After loading the prior map, provide a rough ```2D Pose Estimate``` in ```rviz```. (Notice: Since ```2D Pose Estimation``` can only provide the initial x and y coordinates, users can specify the initial z coordinate by adjusting the ```/localization_param/init_height``` in ```/config/config.yaml```.)  
- Finally play your rosbags:
```
rosbag play yourbag.bag
```

## Datasets
- [NCLT](https://robots.engin.umich.edu/nclt/): The University of Michigan North Campus Long-Term Vision and LIDAR Dataset
- [M2DGR](https://github.com/SJTU-ViSYS/M2DGR): A Multi-modal and Multi-scenario SLAM Dataset for Ground Robots
- [BotanicGarden](https://github.com/robot-pesg/BotanicGarden): A high-quality dataset for robot navigation in unstructured natural environments

## Acknowledgements
- We thank the authors of [Faster-LIO](https://github.com/gaoxiang12/faster-lio), [fast-gicp](https://github.com/koide3/fast_gicp) and [robin-map](https://github.com/Tessil/robin-map) for their great contributions.  
- And please cite us and support us with a star if you find this work useful:
```
@ARTICLE{11123901,
  author={Li, Xinyi and Yuan, Shenghai and Cai, Haoxin and Lu, Shunan and Wang, Wenhua and Liu, Jianqi},
  journal={IEEE Transactions on Instrumentation and Measurement}, 
  title={LL-Localizer: A Lifelong Localization System Based on Dynamic i-Octree}, 
  year={2025},
  volume={74},
  number={},
  pages={1-11},
  doi={10.1109/TIM.2025.3598401}}

```
