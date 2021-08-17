# lidar_align
激光雷达外参标定（激光雷达和RTK外参，6dof）

## 0. 前言
对于Lidar+IMU系统，往往需要标定Lidar与IMU的外参[4]，即从Lidar到IMU的6个位姿参数。ETH开源的lidar_align代码[0]，用于标定Lidar和里程计Odom之间的位姿参数。本文将对代码进行初步介绍，并总结一些使用中可能会遇到的问题。
## 1. 代码整体一览

1.1 代码结构

代码主要包括：头文件、cpp、以及ROS的launch启动文件等。其中头文件包括：

aligner.h：Lidar和Odom对齐（外参计算）时用到的类

loader.h：从ROS的Bag或CSV格式载入数据的相关函数

sensors.h：主要包括：里程计Odom，以及雷达Lidar相关接口

transform.h：一些SO3变化的计算以及转换，在插值、优化时使用

1.2 方法基本思想

方法本质解决的是一个优化问题，即在外参参数(6DoF)如何选择时，使Lidar采集到的数据转化到Odom系下后，前后两次scan的数据点能够尽可能的重合。

详细一点儿来说，方法将Lidar数据根据当前假设的状态变量（6DoF参数）变换到Odom系下，构成点云PointCloud，之后对每一次scan时的数据，在下一次scan中通过kdtree的方式寻找最近邻的数据点并计算距离，当总距离最小时可以认为完全匹配，即计算的外参参数正确。

1.3 主要流程

代码主要有两部分：载入数据与优化计算。载入数据部分从Bag数据载入雷达的数据(sensor_msgs/PointCloud2)，并从CSV或Bag数据中载入Odom获得的6DoF位置信息。
具体的位置信息如何获得将在后面进行介绍。优化计算部分将在第2.2小节详细展开。

## 2. 详细介绍

2.1 主要数据类型

Odom数据：主要包括两个数据：时间戳timestamp_us_与从当前时刻到初始时刻的变换T_o0_ot_。

Lidar数据：主要包括两个参数：从Lidar到Odom的外参T_o_l_与每次扫描的数据scans_，而每次的扫描scan数据类型主要包括：扫描起始时间timestamp_us_，本次扫描的点云raw_points_，某个点在Odom的变换（用于去畸变）T_o0_ot_，以及相关配置参数等。

Aligner数据：Aligner首先包含了需要优化的数据OptData（其中包括Lidar、Odom等数据），以及相应的配置参数（数据载入路径、初值、优化参数、KNN相关参数等），以及优化计算的相关参数。

2.2 优化过程详细介绍

在载入了Odom和Lidar数据之后，进行优化求解6个位姿参数。主要求解函数为：lidarOdomTransform 

### Aligner::lidarOdomTransform()

首先进行相关的优化配置。默认优化参数是6个，但可以考虑两个传感器传输造成的时间差，如果考虑这个因素，参数数量将变为7。

优化时，采用NLOPT优化库[3]，默认首先全局优化这三个参数。如果提供的初值与真值相差较大，或完全没有设置初值（默认为全0），则需要进行全局优化获得旋转参数。在局部优化这6个参数，局部优化开始时的初值就是3个为0的平移参数，以及全局优化计算出来的旋转参数。全局优化、局部优化，都是调用的optimize函数。

### Aligner::optimize()
在这个函数设置了NLOPT优化的相关参数，包括：是全局优化还是局部优化、优化问题的上下界、最大迭代次数、求解精度以及目标函数等。
最重要的是目标函数LidarOdomMinimizer 
### LidarOdomMinimizer()

这个函数在优化中会不断调用，迭代计算。首先会通过上一时刻的状态，计算新的从Lidar到Odom的变换（这里用到了Transform.h中定义的一些变换），误差是由lidarOdomKNNError函数获得。
lidarOdomKNNError()这个是一个重载函数，具有两种重载类型。首先调用的是lidarOdomKNNError(const Lidar)，处理的是Lidar的数据，首先根据估计的Lidar到Odom的变化，对完整的scans_数据计算出每次scan时每个点在Odom下的坐标（getTimeAlignedPointcloud函数，相当于点云去畸变），得到一个结合的点云(CombinedPointcloud)，之后从这个点云中寻找每个点的最近邻，在利用另一个重载类型的lidarOdomKNNError(const Pointcloud, const Pointcloud)函数进行计算最近邻误差。

计算最近邻误差时，构建了一个KD-Tree，并行计算kNNError函数，利用pcl库的nearestKSearch函数搜索一定范围（全局优化时是1m，局部优化时是0.1m）的最近邻，计算最近2个点的误差。
### 小结

优化的目标函数是每次scan的每个点在完整点云中的最近邻的距离，首先通过粗的全局优化估计一部分参数，再局部优化求解精细的6DoF参数。

## 3. 配置与运行

3.1 安装

首先在安装时需要安装NLOPT：sudo apt-get install libnlopt-dev。之后把代码拷贝到ros的工作空间，使用 catkin_make进行编译。

3.2 编译可能遇到的问题

这个代码是个人编写使用，没有在大多数的ubuntu和ros版本进行测试，所以可能会遇到各种各样的问题。以Ubuntu18与ROS-melodic为例，首先会遇到一个定义冲突的报错：

1. 定义冲突问题
2. 
error: conflicting declaration ‘typedef struct LZ4_stream_t LZ4_stream_t’ typedef struct { long long table[LZ4_STREAMSIZE_U64]; } LZ4_stream_t;

这个原因是ROS版本下有两个头文件定义发生冲突，github的issue中给出了两种解决办法，之一是重命名头文件避免冲突：

sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak

sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak

sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h

sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

（详见：https://github.com/ethz-asl/lidar_align/issues/16）

2. 找不到"FindNLOPT.cmake"

By not providing "FindNLOPT.cmake" in CMAKE_MODULE_PATH this project has asked CMake to find a package configuration file provided by "NLOPT", but CMake did not find one. Solutions: Move "NLOPTConfig.cmake" file to src directory.

解决方法：将"\lidar_align\FindNLOPT.cmake"文件移动到工作路径下中的"\src"文件夹下，再次编译即可。

3.3 测试运行

在github的issue中，由于存在准备数据（尤其是Odom数据）有错误的问题，造成运行失败。作者上传了测试数据https://drive.google.com/open?id=11fUwbVnvej4NZ_0Mntk7XJ2YrZ5Dk3Ub 可以运行测试。

3.4 数据准备

Lidar的数据直接是ros中的sensor_msgs/PointCloud2即可，但位置数据需要提供CSV格式或者ROS下的geometry_msgs/TransformStamped消息类型。后者如何获得？如果是IMU直接进行积分就好，但这样积分势必会不准确，作者也在issue中提到没有考虑noise的问题(https://github.com/ethz-asl/lidar_align/issues/5#issuecomment-432232087 )，所以目前看来对IMU进行积分，凑合使用就好。
[1]给出了一种IMU计算Odom的实现。

3.5 数据采集要求

作者在issue和readme中指出，该方法存在的局限性是，必须要求采集数据时系统进行非平面运动，对平移要求不高但要求旋转必须充分。但对数据量、运动范围没有经过严格的测试。这个局限性也限制了不能用于给无人车这种系统标定。

参考资料

[0]. 原版代码github：https://github.com/ethz-asl/lidar_align

[1]. IMU数据计算Odom的实现：https://www.cnblogs.com/gangyin/p/13366683.html

[2]. lidarr_align原理简要介绍：https://blog.csdn.net/miracle629/article/details/87854450

[3]. NLOPT优化库介绍：https://nlopt.readthedocs.io/en/latest

[4]. Lidar和IMU标定需要标什么？https://blog.csdn.net/tfb760/article/details/108532974

## A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor

**Note: Accurate results require highly non-planar motions, this makes the technique poorly suited for calibrating sensors mounted to cars.**

The method makes use of the property that pointclouds from lidars appear more 'crisp' when the calibration is correct. It does this as follows:
1) A transformation between the lidar and pose sensor is set.
2) The poses are used in combination with the above transformation to fuse all the lidar points into a single pointcloud.
3) The sum of the distance between each point and its nearest neighbor is found.
This process is repeated in an optimization that attempts to find the transformation that minimizes this distance.

## Installation

To install lidar_align, please install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu), [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

The following additional system dependencies are also required:
```
sudo apt-get install libnlopt-dev
```
## Input Transformations

The final calibrations quality is strongly correlated with the quality of the transformation source and the range of motion observed. To ensure an accurate calibration the dataset should encompass a large range of rotations and translations. Motion that is approximately planner (for example a car driving down a street) does not provide any information about the system in the direction perpendicular to the plane, which will cause the optimizer to give incorrect estimates in this direction.

## Estimation proceedure
For most systems the node can be run without tuning the parameters. By default two optimizations are performed, a rough angle only global optimization followed by a local 6-dof refinement.

The node will load all messages of type `sensor_msgs/PointCloud2` from the given ROS bag for use as the lidar scans to process. The poses can either be given in the same bag file as `geometry_msgs/TransformStamped` messages or in a separate CSV file that follows the format of [Maplab](https://github.com/ethz-asl/maplab).

## Visualization and Results

The node will output it's current estimated transform while running. To view this your launchfile must set `output="screen"` in the `<node/>` section. See the given launchfile for an example.

Once the optimization finishes the transformation parameters will be printed to the console. An example output is as follows:
```
Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor Frame to  the Lidar Frame:
[-0.0608575, -0.0758112, 0.27089, 0.00371254, 0.00872398, 1.60227]

Active Transformation Matrix from the Pose Sensor Frame to  the Lidar Frame:
-0.0314953  -0.999473  0.0078319 -0.0608575
  0.999499 -0.0314702 0.00330021 -0.0758112
 -0.003052 0.00793192   0.999964    0.27089
         0          0          0          1

Active Translation Vector (x,y,z) from the Pose Sensor Frame to  the Lidar Frame:
[-0.0608575, -0.0758112, 0.27089]

Active Hamiltonen Quaternion (w,x,y,z) the Pose Sensor Frame to  the Lidar Frame:
[0.69588, 0.00166397, 0.00391012, 0.718145]

Time offset that must be added to lidar timestamps in seconds:
0.00594481

ROS Static TF Publisher: <node pkg="tf" type="static_transform_publisher" name="pose_lidar_broadcaster" args="-0.0608575 -0.0758112 0.27089 0.00166397 0.00391012 0.718145 0.69588 POSE_FRAME LIDAR_FRAME 100" />
```
If the path has been set the results will also be saved to a text file. 

As a method of evaluating the quality of the alignment, if the needed path is set all points used for alignment will be projected into a single pointcloud and saved as a ply. An example of such a pointcloud can be seen below.

![example_pointcloud](https://user-images.githubusercontent.com/730680/48580820-7969c400-e920-11e8-9a69-e8aab2e74d20.png)

## CSV format

| Column | Description |
|--:|:--|
1 | timestamp ns 
2 | vertex index (not used) 
3 | position x
4 | position y
5 | position z 
6 | orientation quaternion w 
7 | orientation quaternion x
8 | orientation quaternion y 
9 | orientation quaternion z 

Note that Maplab has two CSV exporters. This file-format is the same as produced by [exportPosesVelocitiesAndBiasesToCsv](https://github.com/ethz-asl/maplab/blob/master/console-plugins/vi-map-data-import-export-plugin/src/export-vertex-data.cc#L39) but differs from the output of [exportVerticesAndTracksToCsv](https://github.com/ethz-asl/maplab/blob/master/tools/csv-export/src/csv-export.cc#L35)

## Parameters
------

### Scan Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `min_point_distance` |  Minimum range a point can be from the lidar and still be included in the optimization. | 0.0 |
| `max_point_distance` |  Maximum range a point can be from the lidar and still be included in the optimization. | 100.0 |
| `keep_points_ratio` |  Ratio of points to use in the optimization (runtimes increase drastically as this is increased). | 0.01 |
| `min_return_intensity` | The minimum return intensity a point requires to be considered valid. | -1.0 |
| `motion_compensation` |  If the movement of the lidar during a scan should be compensated for. | true |
| `estimate_point_times` | Uses the angle of the points in combination with `lidar_rpm` and `clockwise_lidar` to estimate the time a point was taken at. | false |
| `lidar_rpm` | Spin rate of the lidar in rpm, only used with `estimate_point_times`. | 600 |
| `clockwise_lidar` | True if the lidar spins clockwise, false for anti-clockwise, only used with `estimate_point_times`. | false |

### IO Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `use_n_scans` |  Optimization will only be run on the first n scans of the dataset. | 2147483647 |
| `input_bag_path` |  Path of rosbag containing sensor_msgs::PointCloud2 messages from the lidar. | N/A  |
| `transforms_from_csv` | True to load scans from a csv file, false to load from the rosbag. | false |
| `input_csv_path` |  Path of csv generated by Maplab, giving poses of the system to calibrate to. | N/A |
| `output_pointcloud_path` |  If set, a fused pointcloud will be saved to this path as a ply when the calibration finishes. | "" |
| `output_calibration_path` |  If set, a text document giving the final transform will be saved to this path when the calibration finishes. | "" |

### Alinger Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `local` |  If False a global optimization will be performed and the result of this will be used in place of the `inital_guess` parameter. | false |
| `inital_guess` |  Initial guess to the calibration (x, y, z, rotation vector, time offset), only used if running in `local` mode. | [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] |
| `max_time_offset` |  Maximum time offset between sensor clocks in seconds. | 0.1 |
| `angular_range` | Search range in radians around the `inital_guess` during the local optimization stage. | 0.5 |
| `translational_range` | Search range around the `inital_guess` during the local optimization stage. | 1.0 |
| `max_evals` | Maximum number of function evaluations to run | 200 |
| `xtol` | Tolerance of final solution | 0.0001 |
| `knn_batch_size` | Number of points to send to each thread when finding nearest points | 1000 |
| `knn_k` | Number of neighbors to consider in error function | 1 |
| `global_knn_max_dist` | Error between points is limited to this value during global optimization. | 1.0 |
| `local_knn_max_dist` | Error between points is limited to this value during local optimization. | 0.1 |
| `time_cal` | True to perform time offset calibration | true |
