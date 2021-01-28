# linefit_ground_segmentation_details

快速3D点云分割论文代码（带注解）：Fast segmentation of 3d point clouds for ground vehicles


Implementation of the ground segmentation algorithm proposed in

```
@inproceedings{himmelsbach2010fast,
  title={Fast segmentation of 3d point clouds for ground vehicles},
  author={Himmelsbach, Michael and Hundelshausen, Felix V and Wuensche, H-J},
  booktitle={Intelligent Vehicles Symposium (IV), 2010 IEEE},
  pages={560--565},
  year={2010},
  organization={IEEE}
}
```


参考：https://github.com/lorenwel/linefit_ground_segmentation

@[双愚](https://github.com/HuangCongQing/) , 若fork或star请注明来源


### Intro

个人笔记：

### Setup

##### Frameworks and Packages

Make sure you have the following is installed:

- [ROS Kinetic/Melodic](http://wiki.ros.org/melodic)
- [PCL 1.7.2](http://pointclouds.org/downloads/)

##### Dataset bag

数据集已处理好，放在百度网盘上，需要自己下载

* kitti_2011_09_26_drive_0005_synced.bag
* 链接: https://pan.baidu.com/s/1sYWHzF11RpyEW25cQ_iNGA  密码: b6pd

### Start

主要代码路径：

linefit_ground_segmentation/linefit_ground_segmentation/src

* [linefit_ground_segmentation](linefit_ground_segmentation/linefit_ground_segmentation/src)


##### Terminal 1

```
roscore
```

##### Terminal 2

`--loop`循环

```
# kitti官方  注意修改路径path
rosbag play path/kitti_2011_09_26_drive_0005_synced.bag --loop


```

##### Terminal 3

```
rviz
```

![arch](https://cdn.nlark.com/yuque/0/2021/png/232596/1611807439212-954eb16d-fdc6-49db-8647-b4df4d4115f7.png)

##### Terminal 4

```

#  运行launch
roslaunch linefit_ground_segmentation_ros segmentation.launch



```

### Result

![arch](https://cdn.nlark.com/yuque/0/2021/png/232596/1611807271441-3826b794-9a49-4ca5-879d-8206fbab4190.png)


### License

Copyright (c) [双愚](https://github.com/HuangCongQing/). All rights reserved.

Licensed under the [BSD 3-Clause License](./LICENSE) License.
