# LOAM-Livox
## 适用于Livox-LiDAR的稳健激光雷达里程计和建图（LOAM）软件包

<div align="center">
    <img src="pics/zym_rotate.gif" width = 45% >
    <img src="pics/hkust_stair.gif" width = 45% >
</div>

**Loam-Livox**是一个稳健、低漂移、实时的激光雷达里程计和建图软件包，适用于[*Livox LiDARs*](https://www.livoxtech.com/)，这些激光雷达具有显著的低成本和高性能，并设计用于大规模工业应用。我们的软件包解决了许多关键问题：在非常有限的视场中进行特征提取和选择、稳健的异常值排除、移动物体滤波和运动畸变补偿。此外，我们还集成了其他功能，如可并行处理的流水线、使用单元格和地图进行点云管理、闭环检测、地图保存和重新加载的实用工具等。如需了解更多详细信息，请参阅我们的相关论文:)

<div align="center">
    <img src="pics/loop_4in1.png" width = 100% >
</div>


在我们的软件包开发过程中，我们参考了LOAM、[LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED)和[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)。

**开发者：** [Jiarong Lin](https://github.com/ziv-lin)

**我们的相关论文：** 我们的相关论文现在可以在arxiv上获取：
1. [Loam_livox：用于小视场LiDAR的快速、稳健、高精度激光雷达里程计和建图软件包](https://arxiv.org/abs/1909.06700)
2. [一种快速、完整的基于点云的激光雷达里程计和建图闭环检测方法](https://arxiv.org/abs/1909.11811)

**我们的相关视频：** 我们的相关视频现在可以在YouTube上观看（点击下方图像打开）：
<div align="center">
<a href="https://youtu.be/WHbbtU-Q9-k" target="_blank"><img src="pics/video_loam.png" alt="video" width="45%" /></a>
<a href="https://youtu.be/Uq8rUEk-XnI" target="_blank"><img src="pics/video_lc.png" alt="video" width="45%" /></a>
</div>

## 1. 前提条件
### 1.1 **Ubuntu** 和 **ROS**
要求使用 Ubuntu 64 位操作系统，版本为 16.04 或 18.04。
需要安装 ROS Kinetic 或 Melodic 版本。请参考 [ROS 安装指南](http://wiki.ros.org/ROS/Installation) 安装 ROS 及其附加包:
```
sudo apt-get install ros-XXX-cv-bridge ros-XXX-tf ros-XXX-message-filters ros-XXX-image-transport
```
**注意：** 请根据你使用的 ROS 版本将上述命令中的 "XXX" 替换为相应的版本号。例如，如果你使用的是 ROS Kinetic，命令应为:
```
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport
```

### 1.2. **Ceres Solver**
请按照[Ceres 安装指南](http://ceres-solver.org/installation.html)进行安装。

### 1.3. **PCL**
请按照[PCL 安装指南](http://www.pointclouds.org/downloads/linux.html)进行安装。

**注意：** 我们最近发现 PCL 1.7 和 1.9 版本之间的体素网格滤波器输出的点云存在差异，并且 PCL 1.7 版本在一些示例中存在一些问题（[问题 #28](https://github.com/hku-mars/loam_livox/issues/28)）。因此，如果你使用的是较低版本的 PCL，请强烈建议将其更新为 1.9 版本。


## 2. 构建
克隆存储库并执行 catkin_make：
```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/loam_livox.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 3. 直接运行
### 3.1 Livox Mid-40
按照 [Livox-ros-driver 安装指南](https://github.com/Livox-SDK/livox_ros_driver) 将你的 PC 连接到 Livox LiDAR (Mid-40)，然后按照以下顺序运行（**先**启动我们的算法，然后再启动 livox-ros-driver）：
```
roslaunch loam_livox livox.launch
roslaunch livox_ros_driver livox_lidar.launch
```

### 3.2 Livox Mid-100
不幸的是，Livox-ros-driver 的默认配置会将三个激光雷达的点云混合在一起，这会对我们的特征提取和运动模糊补偿造成一些困难。因此，我们需要进行一些适应性调整（修改一些配置）来启动我们的软件包。

有关详细信息，请参考我们的 [**教程（点击此处打开）**](./Tutorial_Mid-100.md)。
## 4. Rosbag 示例
### 4.1. 一般的 rosbag
下载 [我们记录的 rosbag 文件](https://drive.google.com/drive/folders/1HWomWWPSEVvka2QVB2G41iRvSwIt5NWf?usp=sharing)，然后执行以下操作：
```
roslaunch loam_livox rosbag.launch
rosbag play YOUR_DOWNLOADED.bag
```
<div align="center">
    <img src="pics/CYT_01.png" width=45% >
    <img src="pics/CYT_02.png" width=45% >
</div>

<div align="center">
    <img src="pics/HKU_ZYM_01.png" width=45% >
    <img src="pics/HKU_ZYM_02.png" width=45% >
</div>

### 4.2. 大规模 rosbag
对于大规模的 rosbag（例如，[HKUST_01.bag](https://drive.google.com/file/d/1OoAu0WcRhyDsQB9ltPLWeZ2WZlGJz6LO/view?usp=sharing)），我们建议您使用更大的线和平面分辨率来启动（使用 *rosbag_largescale.launch*）：
```
roslaunch loam_livox rosbag_largescale.launch
rosbag play YOUR_DOWNLOADED.bag
```
<div align="center">
    <img src="pics/HKUST_01.png" width=45% >
    <img src="pics/HKUST_02.png" width=45% >
</div>

### 4.3. Livox Mid-100 示例
下载我们记录的 rosbag 文件（[mid100_example.bag](https://drive.google.com/open?id=1hvjmt4YuGROE-3HWqOfUdofc6zvXxjmU)），然后执行以下操作：
```
roslaunch loam_livox rosbag_mid100.launch
rosbag play mid100_example.bag
```
<div align="center">
    <img src="pics/for_mid100/mid_100_demo.gif" width = 80% >
</div>

## 5. 启用闭环的 Rosbag 示例

### 5.1. 闭环演示
我们提供了一个小尺寸的 rosbag 文件（名为 "loop_loop_hku_zym.bag"，[在此下载](https://drive.google.com/file/d/1rXRzbiZYeFtCWhmwCj8OuZaUUkGCsNrM)）进行演示：
<div align="center">
    <img src="pics/loop_simple.png" width=45% >
</div>

```CPP
roslaunch loam_livox rosbag_loop_simple.launch
rosbag play YOUR_PATH/loop_simple.bag
```

### 5.2. 其他示例
<div align="center">
    <img src="pics/loop_hku_main.png" width=45% >
    <img src="pics/loop_hku_zym.png" width=45% >
</div>

对于其他示例（[loop_loop_hku_zym.bag](https://drive.google.com/open?id=1J3sVQEwnkjaimf9abH1qjId1S4VEZuMu)，[loop_hku_main.bag](https://drive.google.com/open?id=1HrlFkzLMfcYbQsbOiSlSlm5blybrSVd2)），使用以下命令启动：

```
roslaunch loam_livox rosbag_loop.launch
rosbag play YOUR_DOWNLOADED.bag
```

**注意：** "rosbag_loop_simple.launch"和"rosbag_loop.launch"之间唯一的区别是回环检测的两个候选帧之间的最小关键帧数（minimum_keyframe_differen）。

## 6. 在下载 rosbag 文件时遇到问题？
如果您在从 Google 网盘下载 rosbag 文件时遇到问题（例如 [issue #33](https://github.com/hku-mars/loam_livox/issues/33)），您可以从[百度网盘](https://pan.baidu.com/s/1nMSJRuP8io8mEqLgACUT_w)下载相同的文件。

```C
链接：https://pan.baidu.com/s/1nMSJRuP8io8mEqLgACUT_w
提取码：sv9z
```

如果共享链接被禁用，请随时通过电子邮件（ziv.lin.ljr@gmail.com）联系我，以便及时更新链接。

## 7. 我们的可打印手持设备
要获取我们的以下手持设备，请访问我们的另一个[开源仓库](https://github.com/ziv-lin/My_solidworks/tree/master/livox_handhold)，所有的 3D 部件都是设计为 FDM 可打印的。我们还发布了我们的 Solidworks 文件，以便您可以自由地进行调整。

<div align="center">
    <img src="pics/handheld.png" width=45% >
    <img src="pics/system_low_cost.png" width=45% >
</div>

## 8. 鸣谢
感谢 LOAM（J. Zhang 和 S. Singh. LOAM: Lidar Odometry and Mapping in Real-time）、[LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED) 和 [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)。

## 9. 许可证
该源代码在 [GPLv2](http://www.gnu.org/licenses/) 许可证下发布。

我们仍在努力提高代码的性能和可靠性。对于任何技术问题，请通过电子邮件联系我 Jiarong Lin <ziv.lin.ljr@gmail.com>。

对于商业用途，请联系 Dr. Fu Zhang <fuzhang@hku.hk>。


