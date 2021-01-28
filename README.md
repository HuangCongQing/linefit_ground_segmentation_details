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

个人笔记：https://www.yuque.com/huangzhongqing/ngixrc/fxk7gt

### Setup

##### Frameworks and Packages

Make sure you have the following is installed:

- [ROS Kinetic/Melodic](http://wiki.ros.org/melodic)
- [PCL 1.7.2](http://pointclouds.org/downloads/)

##### Dataset bag

数据集已处理好，放在百度网盘上，需要自己下载

* kitti_2011_09_26_drive_0005_synced.bag
* 链接: https://pan.baidu.com/s/1sYWHzF11RpyEW25cQ_iNGA  密码: b6pd

### 编译

将本仓库下的三个文件夹移动到catkin_wp/src下，然后执行下面操作

```shell
// 创建环境变量 src中运行
mkdir -p catkin_wp/src
cd catkin_wp/src
catkin_init_workspace

// 编译（需要回到工作空间catkin_wp）
cd ..
catkin_make  // 产生build和devel文件夹


//设置环境变量，找到src里的功能包(每个新的shell窗口都要执行以下source devel/setup.bash)
source devel/setup.bash  // 不同shell，不同哦.sh  .zsh           通过设置gedit ~/.zshrc，不用每次都source
```

详情可参考：https://www.yuque.com/docs/share/e59d5c91-b46d-426a-9957-cd262f5fc241?# 《09.创建工作空间与功能包※※※》


### 修改配置文件

举例：修改输入topic

```bash
cd linefit_ground_segmentation/linefit_ground_segmentation_ros/launch/segmentation.launch

#第8行    <param name="input_topic" value="/kitti/velo/pointcloud" />修改你的雷达点云话题
<param name="input_topic" value="/kitti/velo/pointcloud" />   <!-- 输入topic -->

```


### Run运行

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

可使用此配置 `config.rviz`

注意：Fixed Frame： **velo_link**

![arch](https://cdn.nlark.com/yuque/0/2021/png/232596/1611807439212-954eb16d-fdc6-49db-8647-b4df4d4115f7.png)

##### Terminal 4

```

#  运行launch
roslaunch linefit_ground_segmentation_ros segmentation.launch



```

### Result

![arch](https://cdn.nlark.com/yuque/0/2021/png/232596/1611807271441-3826b794-9a49-4ca5-879d-8206fbab4190.png)

### Reference

* 项目代码：https://github.com/lorenwel/linefit_ground_segmentation
* 依赖1：git clone https://github.com/catkin/catkin_simple
* 依赖2：https://github.com/google/glog
* 注解：https://github.com/sysuzyc/road_detecting
* 论文：https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5548059&tag=1

### License

Copyright (c) [双愚](https://github.com/HuangCongQing/). All rights reserved.

Licensed under the [BSD 3-Clause License](./LICENSE) License.
