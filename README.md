# CoolEye D1 Linux SDK & ROS 
### Version 0.4
本项目包括CoolEye D1 相机运行所需的必备文件,为了便于用于学习和使用,内容全部开源。
主流的SLAM算法,图像拼接,智能识别等功能会陆续放出。

淘宝地址: https://item.taobao.com/item.htm?spm=a1z10.3-c.w4023-18512428132.2.b68367c2Q9BWIH&id=572104551076

第三批已到货，欢迎加入。

![enter image description here](https://ws4.sinaimg.cn/large/0069RVTdgy1fv0qsffh1lj306a08274f.jpg)

-------------------------------------------------------------------------------------------------
## 主要参数介绍

- CMOS 1/3'' :  灰度图像(Mono)
- 快门模式 : 全局快门(global shutter)
- 同步方式 : 双目同步曝光
- 分辨率 : 最大752x480
- 帧率 : 双目45帧
- 视场角 : D/H/V = 140/120/75
- 双目基线长 : 120mm
- 输出格式 : RAW/畸变矫正图/视差图/XYZ点云图
- 深度范围 : 0.2m - 10米左右
- IMU输出 : ACC / GYRO 原始数据,帧率最大1K
- 接口类型 : USB2.0


--------------------------------------------------------------------------------------------------
## 目录
- 一. SDK & ROS安装
	- 1.1 SDK安装方法
	- 1.2 ROS环境安装方法
-  二. 标定相机相关
	- 2.1 ROS环境下标定相机
	- 2.2 kalibr 标定相关
- 三. 运行算法相关
	- 3.1 ORB-SLAM2算法的安装和使用
	- 3.2 okvis算法的安装和使用
	- 3.2 vins算法的安装和使用
- 四. 相机固件升级方法
	- 4.1 IMU操作固件升级
	- 4.2 图像处理固件升级
- 五. 常见问题汇总
	- 5.1 串口无法打开
	- 5.2 ORB_SLAM2的ROS环境编译不通过
- 六. 硬件尺寸图
- 修订历史

-----------------------------------------------------------

![enter image description here](https://ws1.sinaimg.cn/large/006tKfTcgy1ftnkfhworqj30ku0bqmy0.jpg)

![enter image description here](https://ws4.sinaimg.cn/large/006tKfTcgy1ftnkggoo84j30ku0bqwfg.jpg)

![enter image description here](https://ws1.sinaimg.cn/large/006tKfTcgy1ftnkgp0ekuj30ku0bqwf3.jpg)

![enter image description here](https://ws1.sinaimg.cn/large/006tKfTcgy1ftnkh0ehc3j30ku0bq3zr.jpg)

![enter image description here](https://ws1.sinaimg.cn/large/006tKfTcgy1ftnkh59ynwj30ku0bqgmp.jpg)

![enter image description here](https://ws1.sinaimg.cn/large/006tKfTcgy1ftnkh9x834j30ku0bq0tk.jpg)

![enter image description here](https://ws1.sinaimg.cn/large/006tKfTcgy1ftnkhe2b19j30ku0bq751.jpg)

![enter image description here](https://ws2.sinaimg.cn/large/006tKfTcgy1ftnkhnj6ipj30ku0bqaaw.jpg)

![enter image description here](https://ws2.sinaimg.cn/large/006tKfTcgy1ftnkhv9hpxj30ku0bq75c.jpg)

--------------------------------------------------------------------------------------------------------
VINS-Mono 测试效果图

![enter image description here](https://ws4.sinaimg.cn/large/006tNc79gy1ftqy85tb83g31hc0u0x6q.gif)

![enter image description here](https://ws4.sinaimg.cn/large/006tNc79gy1ftqy8zocseg31hc0u07wk.gif)

![enter image description here](https://ws2.sinaimg.cn/large/006tNc79gy1ftqy9h2149g31hc0u0qv7.gif)


-------------------------------------------------------------------------------------------------
ORB_SLAM2 测试效果图

![enter image description here](https://ws4.sinaimg.cn/large/0069RVTdgy1fupm9ysjmwg31hc0u0npd.gif)

![enter image description here](https://ws3.sinaimg.cn/large/0069RVTdgy1fupmacv01pg31hc0u07wh.gif)

![enter image description here](https://ws2.sinaimg.cn/large/0069RVTdgy1fupmakbo79g31hc0u0hdt.gif)

![enter image description here](https://ws3.sinaimg.cn/large/0069RVTdgy1fupmastlcpg31hc0u0qv5.gif)

---------------------------------------------------------------------------------------------------

## 一. SDK & ROS安装
- 推荐环境：ubuntu 16.04 LTS

- 推荐opencv版本：3.4.0

- 推荐ROS版本：Kinetic


__首先需要安装opencv__
建议使用3.4.0版本。
参考文档：
```
https://blog.csdn.net/u013066730/article/details/79411767
```
另视差图的计算,用到了opencv得扩展库.
请安装opencv_contrib,推荐使用3.4.0(与opencv同版本).

```
cd ~/src
git clone https://github.com/opencv/opencv_contrib/tree/3.4.0
```
建议进入opencv_contrib目录,将需要用到cuda的库文件直接删除,以便编译顺利.
```
cd  ~/src/opencv_contrib-3.4.0/modules
rm -rf xfeatures2d
```
进入opencv编译目录,重新执行cmake并编译.
_<opencv_contrib>/modules__  建议使用绝对路径,
```
$ cd <opencv_build_directory>
$ cmake -DOPENCV_EXTRA_MODULES_PATH=<opencv_contrib>/modules <opencv_source_directory>
$ make
$ sudo make install
```
至此,opencv和opencv_contrib安装完毕.

### 1.1 SDK安装方法
为了便于用户一次成功，介绍时将使用绝对使用路径。
```
sudo apt-get install libboost-dev libusb-1.0-0-dev libusb-dev git cmake 

mkdir -p ~/src

cd ~/src

git clone https://github.com/cooleyecam/cooleye_d1_linux_sdk_ros.git

cd ~/src/cooleye_d1_linux_sdk_ros/sdk/config

sudo ./cooleye_cam_set.sh

mkdir -p ~/src/cooleye_d1_linux_sdk_ros/sdk/build

cd ~/src/cooleye_d1_linux_sdk_ros/sdk/build

cmake ..

make 

```
	
编译将生成6个文件：
- CEAPP_RAW_DATA : 运行即可直接显示相机输出。如果log没有IMU数据,说明串口没有正常打开,如果没有看到图像,很有可能是OPENCV没有正确安装.
```	
./CEAPP_RAW_DATA 
```


- CETOOL_CALI_IMU : 用于校准相机自带的IMU的初始偏移。这个六面矫正法移植自PX4的飞控固件，用起来比较麻烦，但是效果还可以，在发布自动矫正的软件版本之前，先临时用来修正IMU的误差。
运行后根据操作，程序会自己将计算出的误差写入配置文件中，用户无需操作。
```	
./CETOOL_CALI_IMU
```



- CETOOL_STEREO_CAPTURE_IMG : 由于本相机使用得镜头视角很大,使用opencv自带得棋盘格标定并不容易得到正确得参数.因此需要使用鱼眼(fisheye)标定,本程序即为标定采集所需图像得程序,运行后用标定版对着摄像头,即可看到棋盘格是否正确识别,每敲一个回车,可保存一帧当前图像,
```	
./CETOOL_STEREO_CAPTURE_IMG
```



- CETOOL_CALI_STEREO_CAL : 采集完包含棋盘格得图像后,可调用本程序进行计算.内参和外参文件会直接放入config目录下.
```	
./CETOOL_STEREO_CAPTURE_IMG
```


- CEAPP_STEREO_MATCH : 运行即可调用config目录下的相机内参和外参,并输出视差图, 此程序输出得视差图没有进行后滤波处理.用于调试BM和SGM得算法参数.
```	
./CEAPP_STEREO_MATCH 
```


- CEAPP_DISP_FILTER : 运行即可输出经过后滤波得视差图.因为滤波算法尚未整合到opencv,因此需要安装扩展包opencv_contrib,如果安装不正确,则此程序无法正常编译. 本程序同要需要调用config目录下的相机内参和外参.
```	
./CEAPP_DISP_FILTER 
```


### 1.2 ROS环境安装方法
在这之前请先根据ros官网安装kinetic版本,建议使用kinetic版本.其他的版本在后续使用算法时可能会有些兼容问题.请先安装完
SDK再安装ROS.

ROS安装参考
```
http://wiki.ros.org/kinetic/Installation/Ubuntu
```
搭建好ROS的环境侯，即可运行相机的ROS驱动
```
cd ~/src/cooleye_d1_linux_sdk_ros/sdk/config/

./updata_rosfile.sh

cd ~/src/cooleye_d1_linux_sdk_ros/ros

catkin_make

source ~/src/cooleye_d1_linux_sdk_ros/ros/devel/setup.bash
```
	
至此编译完成.执行以下命令即可运行相机.


```	
roscore

rosrun cooleye_d1 CEROS_CAM_D1_NODE ~/src/cooleye_d1_linux_sdk_ros/ros/src/cooleye_d1/config/cecfg_std.txt
```
可使用ros命令观察相机运行状态
```
rostopic hz /imu0_icm20689 /cooleyed1/left/image_raw /cooleyed1/right/image_raw
```


-------------------------------------------------------------------------------------------------

## 二. 标定相机相关
### 2.1 ROS环境下标定相机
首先,可以直接参考官方文档,下文的说明只是对官方文档的注解
```
http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
```

#### 2.1.1 准备工作
1. 本相机的所有例成均使用3cm正方形的11x8棋盘格,这里的11x8是指内角的数量,其实由12x9个正方向组成.
2. 光线良好.
3. 确保CoolEye相机的ROS驱动已正确安装
#### 2.1.2 编译
安装所需要的ros文件
```
rosdep install camera_calibration

rosmake camera_calibration
```
#### 2.1.3 使用cooleye相机在ros环境下发布左右图像的topic
```
roscore

rosrun cooleye_d1 CEROS_CAM_D1_NODE ~/src/cooleye_d1_linux_sdk_ros/ros/src/cooleye_d1/config/cecfg_std.txt
```
可使用ros命令观察相机运行状态
```
rostopic hz /imu0_icm20689 /cooleyed1/left/image_raw /cooleyed1/right/image_raw
```
#### 2.1.4 启动校准
```
rosrun camera_calibration cameracalibrator.py --size 11x8 \
	--square 0.03 --no-service-check --approximate=0.1 \
	right:=/cooleyed1/right/image_raw \
	left:=/cooleyed1/left/image_raw \
	right_camera:=/cooleyed1/right \
	left_camera:=/cooleyed1/left
```

#### 2.1.5 移动棋盘格
注意事项:
1. 水平放置棋盘格,即水平方向格子数多余垂直方向.
2. 视野的左右边缘检测棋盘
3. 视野的顶部和底部检测棋盘
4. 棋盘以各种角度朝向摄像头,尽量激活XYZ三轴.
5. 棋盘格子充满整个视野.
当收集的数据足够时,CALIBRATE按钮会亮起来,按下,1分钟左右即可获得相机内参.
标定的结果像素误差小于0.25认为是可以接受的,小于0.1认为是极好的.

相机的参考标定结果如下:
```
Left:
('D = ', [-0.2816607211257644, 0.05884233698719249, 0.000989828550094559, -0.002972305970125211, 0.0])
('K = ', [370.79682914159343, 0.0, 402.25597436269385, 0.0, 371.44796549024153, 237.55089778032217, 0.0, 0.0, 1.0])
('R = ', [0.9980214136742988, -0.007980214436348127, 0.06236644951497698, 0.00797293200390816, 0.9999681488257253, 0.0003656354924037826, -0.062367380919959504, 0.00013233141028975813, 0.9980532512273995])
('P = ', [329.35169351670595, 0.0, 357.3512191772461, 0.0, 0.0, 329.35169351670595, 236.6960849761963, 0.0, 0.0, 0.0, 1.0, 0.0])

Right:
('D = ', [-0.2933299350827631, 0.06663661662397566, 0.0015052397322707109, 0.0006085899317121811, 0.0])
('K = ', [371.81317417765734, 0.0, 362.62919659065994, 0.0, 371.7958559505307, 230.5395237103048, 0.0, 0.0, 1.0])
('R = ', [0.9999953742094383, -0.002431354273887554, -0.001827587513735629, 0.002431140959735301, 0.9999970377000802, -0.00011893148697697306, 0.0018278712644524478, 0.00011448781396267668, 0.9999983228881842])
('P = ', [329.35169351670595, 0.0, 357.3512191772461, -39.191260271159535, 0.0, 329.35169351670595, 236.6960849761963, 0.0, 0.0, 0.0, 1.0, 0.0])
('self.T ', [-0.11899461806960394, 0.00028931941154418633, 0.00021747408417511582])
('self.R ', [0.9979221808144039, -0.005548872111724627, 0.06419136279058048, 0.00553922445102019, 0.9999846044989682, 0.0003282644106577577, -0.06419219602962196, 2.7988029746548842e-05, 0.9979375537505164])
# oST version 5.0 parameters


[image]

width
752

height
480

[narrow_stereo/left]

camera matrix
370.796829 0.000000 402.255974
0.000000 371.447965 237.550898
0.000000 0.000000 1.000000

distortion
-0.281661 0.058842 0.000990 -0.002972 0.000000

rectification
0.998021 -0.007980 0.062366
0.007973 0.999968 0.000366
-0.062367 0.000132 0.998053

projection
329.351694 0.000000 357.351219 0.000000
0.000000 329.351694 236.696085 0.000000
0.000000 0.000000 1.000000 0.000000

# oST version 5.0 parameters


[image]

width
752

height
480

[narrow_stereo/right]

camera matrix
371.813174 0.000000 362.629197
0.000000 371.795856 230.539524
0.000000 0.000000 1.000000

distortion
-0.293330 0.066637 0.001505 0.000609 0.000000

rectification
0.999995 -0.002431 -0.001828
0.002431 0.999997 -0.000119
0.001828 0.000114 0.999998

projection
329.351694 0.000000 357.351219 -39.191260
0.000000 329.351694 236.696085 0.000000
0.000000 0.000000 1.000000 0.000000

```



### 2.2 kalibr 标定相关
kalibr是一款常用的标定工具，可完成camera、imu等标定，很多人喜欢使用。
#### 2.2.1 kalibr安装
请先安装完ROS

1. 安装编译和依赖库

```
sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-dev libboost-all-dev doxygen libopencv-dev ros-kinetic-vision-opencv ros-kinetic-image-transport-plugins ros-kinetic-cmake-modules python-software-properties software-properties-common libpoco-dev python-matplotlib python-scipy python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev python-catkin-tools libv4l-dev 

sudo pip install python-igraph --upgrade
```

2. 建立catkin工作空间

```
mkdir -p ~/kalibr_workspace/src 

cd ~/kalibr_workspace 

source /opt/ros/kinetic/setup.bash 

catkin init 

catkin config --extend /opt/ros/kinetic

catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

3. 下载源代码至src文件目录
```
cd ~/kalibr_workspace/src 

git clone https://github.com/ethz-asl/Kalibr.git
```

4. 编译项目。__-j4__ 根据自己电脑选择，四核就用 __-j4__ ，8核就用 __-j8__ 。编译需要很久，很久，真的很久，很久。
```
cd ~/kalibr_workspace 

catkin build -DCMAKE_BUILD_TYPE=Release -j4
```

5. 添加工作空间的设置
```
source ~/kalibr_workspace/devel/setup.bash
```
#### 2.2.2 kalibr常用命令简介
#####  kalibr_calibrate_cameras
 kalibr_calibrate_cameras 可直接标定双目或多目相机。
``` 
kalibr_calibrate_cameras --target april_6x6.yaml  \
		--bag static.bag  \
		--models pinhole-equi pinhole-equi  \
		--topics /cam0/image_raw /cam1/image_raw 
```
 - --target checkerboard_11x8_3x3cm.yaml
 标定时使用的标定板描述文件，demo使用的是11x8的3cm棋盘格。
 
 - --bag  static.bag 
 使用rosbag record 命令，可采集并记录.bag文件，调用采集的图像即可进行标定。
 
 - --models pinhole-equi pinhole-equi     
 选择不同的的相机模型，CoolEye D1 的镜头视角特别大，用鱼眼模型标定效果会好很多”omni-radtan“。
 
 - --topics /camera/left/image_raw  /camera/right/image_raw  
 bag里记录的topic名称。
  
#####  kalibr_calibrate_imu_camera
kalibr_calibrate_imu_camera 可标定imu和相机之间的参数。
```
kalibr_calibrate_imu_camera --bag imu2cam.bag \
			--cam camchain-2018-05-06-21-35-46.yaml \
			--imu imu0_icm20689.yaml  \
			--target checkerboard_11x8_3x3cm.yaml \
			--time-calibration
```
 - --bag  imu2cam.bag 
 使用rosbag record 命令，可采集并记录.bag文件，调用采集的图像即可进行标定。记录imu和相机的时候，保持标定板不动，移动相机，尽量激活所有的imu维度。

 - --cam camchain-2018-05-06-21-35-46.yaml
带imu的相机标定需要先标定好相机的内参，这里输入相机的内参文件。

 - --target checkerboard_11x8_3x3cm.yaml
 标定时使用的标定板描述文件，demo使用的是11x8的3cm棋盘格。

 - --imu imu-mpu6050.yaml 
 imu的参数需要通过yaml文件输入给算法。


#### 2.2.3 kalibr标定demo
首先,ros环境下运行起相机
```
roscore

rosrun cooleye_d1 CEROS_CAM_D1_NODE ~/src/cooleye_d1_linux_sdk_ros/ros/src/cooleye_d1/config/cecfg_std.txt
```
可使用ros命令观察相机运行状态
```
rostopic hz /imu0_icm20689 /cooleyed1/left/image_raw /cooleyed1/right/image_raw
```

config/cecfg_std.txt 文件中的cf_ros_showimage参数可控制ROS是否显示图像.如果需要显示图像, 将该参数置1即可.如不需要显示,可设置为0.默认设置为1.
```
cf_ros_showimage=1;
```
运行ROS命令,采集一些图像数据.
```
rosbag record /cooleyed1/left/image_raw /cooleyed1/right/image_raw 
```
运行record命令后,系统即开始记录采集到的图像,这时可以拿出标定板对着镜头晃了,规则与ROS下标定相同.参考章节 __2.1.5 移动棋盘格__ 

然后就可以启动kalibr命令进行标定了
```
kalibr_calibrate_cameras --models pinhole-radtan pinhole-radtan \
	--target checkerboard_11x8_3x3cm.yaml\
	--bag Num-18040301-2018-05-06-21-21-34.bag \
	--topics /cooleyed1/left/image_raw /cooleyed1/right/image_raw
```
等一会,程序运行完成后即可获得标定结果.

如果还需要继续标定imu.可在完成相机标定后,使用如下命令,同时记录IMU数据和图像数据.
记录时,保持标定板静止,移动相机,尽量激活IMU的XYZ3个轴.
将命令中的文件替换为自己的文件名称即可.
运行算法后可的到IMU与相机的标定结果.
```
rosbag record /imu0_icm20689 /camera/left/image_raw  /camera/right/image_raw 

kalibr_calibrate_imu_camera --bag 2018-05-09-23-55-02.bag \
	--cam camchain-2018-05-06-21-35-46.yaml \
	--imu imu0_icm20689.yaml \
	--target checkerboard_11x8_3x3cm.yaml \
	--time-calibration
```


-------------------------------------------------------------------------------------------------

## 三. 运行算法相关

### 3.1 ORB-SLAM2算法的安装和使用
ORB_SLAM2是比较火的算法，并熟悉SLAM的人也可以通过它快速搭建SLAM算法的环境。 
这个算法依赖的包比较多，说不准哪天就更新一下，建议还是follow官方的Readme安装，网上也有很多指导教程。

#### 安装Pangolin
安装依赖包
```
sudo apt-get install libglew-dev

sudo apt-get install libpython2.7-dev

sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev

sudo apt-get install libdc1394-22-dev libraw1394-dev

sudo apt-get install libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev
```
安装一个libuvc
```
mkdir -p ~/src

cd ~/src

git clone https://github.com/ktossell/libuvc

cd libuvc

mkdir build

cd build

cmake ..

make && sudo make install
```
下载 Pangolin
```
mkdir -p ~/src

cd ~/src

git clone https://github.com/stevenlovegrove/Pangolin.git

cd Pangolin

mkdir build

cd build

cmake ..

cmake --build .
```
Pangolin python bindings
```
sudo python -mpip install numpy pyopengl Pillow pybind11

git submodule init && git submodule update
```

#### 安装OpenCV
ORB_SLAM2算法测试过OpenCV 3.2。之前安装了OpenCV 3.4，一般问题也不大，如果有问题，可以切回去试试。

#### 安装Eigen3
至少需要3.1.0版本
```
sudo apt-get install libeigen3-dev
```
#### 安装ORB_SLAM2
```
mkdir -p ~/src

cd ~/src

git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2

cd ORB_SLAM2

chmod +x build.sh

./build.sh

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/src/ORB_SLAM2/Examples/ROS

chmod +x build_ros.sh

./build_ros.sh
```
至此编译完成。

注意: ORB_SLAM2默认ROS的topic为:
```
"/camera/left/image_raw"
"/camera/right/image_raw"
```
需更改为cooleye相机默认的:
```
"/cooleyed1/left/image_raw"
"/cooleyed1/right/image_raw"
```

需要更改的文件有:
```
Examples/ROS/ORB_SLAM2/src/ros_mono.cc
Examples/ROS/ORB_SLAM2/src/ros_stereo.cc
Examples/ROS/ORB_SLAM2/src/ros_rgbd.cc
Examples/ROS/ORB_SLAM2/src/AR/ros_mono_ar.cc:
```


### 3.2 okvis算法的安装和使用
OKVIS: Open Keyframe-based Visual-Inertial SLAM. 也是常用的开源项目之一。
首先安装依赖包。
```
sudo apt-get install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libboost-dev libboost-filesystem-dev libopencv-dev
```
下载okvis算法包
```
mkdir -p ~/src

cd ~/src

git clone https://github.com/ethz-asl/okvis.git

cd ~/src/okvis

mkdir build && cd build

cmake -DCMAKE_BUILD_TYPE=Release ..

make -j8

make install
```
至此，编译安装完成。

如果，如果遇到以下ERROR，这是由于ceres-solver无法安装导致的。
```
  Failed to clone repository:
  'https://ceres-solver.googlesource.com/ceres-solver'


CMakeFiles/ceres_external.dir/build.make:89: recipe for target 'ceres/src/ceres_external-stamp/ceres_external-download' failed
make[2]: *** [ceres/src/ceres_external-stamp/ceres_external-download] Error 1
CMakeFiles/Makefile2:153: recipe for target 'CMakeFiles/ceres_external.dir/all' failed
make[1]: *** [CMakeFiles/ceres_external.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
```

__解决方案如下：__
将okvis目录下CMakeLists.txt 中，OFF修改为ON
```
option (USE_SYSTEM_CERES
        "Use ceres via find_package rather than downloading it as part of okvis" OFF)

改为
option (USE_SYSTEM_CERES
        "Use ceres via find_package rather than downloading it as part of okvis" ON)
```
根据以下文档独立安装ceres
```
https://blog.csdn.net/xiat5/article/details/79164059
```
删除okvis目录下的build文件夹，重新编译，问题应该就可以解决了。



### 3.3 vins算法的安装和使用
	
vins是香港科技大学开源的一个单目相机结合IMU的一个VIO，在github上可以下载源码，分为iOS系统下的和ros系统下的两种.本相机当然是适用ROS版本.
```
https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
```
1. 根据提示安装ROS必备得依赖包
```
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport
```
2. Ceres Solver是必备得组件,请参照下方教程安装最新版.
```
http://ceres-solver.org/installation.html#linux
```
3. 按照官方指导编译VINS-Mono的开发环境
```
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
4. 根据相机内参编写config和launch文件.即可运行VINS算法.

-------------------------------------------------------------------------------------------------

## 四. 相机固件升级方法

### 4.1 IMU操作固件升级
目前版本硬件，IMU的控制依赖于一个小型的单片机，如果需要升级，sdk/update会直接发布程序的HEX文件。windows下可使用FlyMcu进行升级，操作步骤见下图。另，熟悉STM32的用户，可根据自己习惯选择自己喜欢的升级方式。

![p1](https://ws4.sinaimg.cn/large/006tNc79gy1fs45hbvz2uj30lo0et410.jpg)

### 4.2 图像处理固件升级
 目前版本硬件，图像Sensor的控制使用了Cypress的方案，sdk/update提供了windows下的驱动文件和升级工具，同时也会直接发布用于升级的IIC文件。升级步骤见下图。
 如果设备在电脑端识别异常，请在设备管理器中卸载设备驱动，然后更新驱动指向driver文件夹，比如：
D:\Cypress\USB\CY3684_EZ-USB_FX2LP_DVK\1.1\Drivers\vista
需要注意的是，这里只需指向vista即可，请勿继续选择其子目录x86或x64，可能会造成识别异常。

 ![p2](https://ws4.sinaimg.cn/large/006tNc79gy1fs45hhn7e2j30is0f3gow.jpg)


-------------------------------------------------------------------------------------------------

### 五. 常见问题汇总
#### 5.1 串口无法打开
- celog: uart open error !: Permission denied

新环境下串口需要root权限。请执行以下命令，其中yourusername为你的当前用户名。

```
sudo gpasswd -a yourusername dialout
```
注销并且重新登录， 即可解决此问题。


#### 5.2 ORB_SLAM2的ROS环境编译不通过

如果遇到以下问题编译不过
```
/usr/bin/ld: CMakeFiles/RGBD.dir/src/ros_rgbd.cc.o: undefined reference to symbol '_ZN5boost6system15system_categoryEv'
/usr/lib/x86_64-linux-gnu/libboost_system.so: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
CMakeFiles/RGBD.dir/build.make:220: recipe for target '../RGBD' failed
make[2]: *** [../RGBD] Error 1
CMakeFiles/Makefile2:67: recipe for target 'CMakeFiles/RGBD.dir/all' failed
make[1]: *** [CMakeFiles/RGBD.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
/usr/bin/ld: CMakeFiles/Stereo.dir/src/ros_stereo.cc.o: undefined reference to symbol '_ZN5boost6system15system_categoryEv'
/usr/lib/x86_64-linux-gnu/libboost_system.so: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
CMakeFiles/Stereo.dir/build.make:220: recipe for target '../Stereo' failed
make[2]: *** [../Stereo] Error 1
CMakeFiles/Makefile2:104: recipe for target 'CMakeFiles/Stereo.dir/all' failed
make[1]: *** [CMakeFiles/Stereo.dir/all] Error 2
Makefile:127: recipe for target 'all' failed
make: *** [all] Error 2
```
解决方案为
- __将libboost_system.so与libboost_filesystem.so复制到~/src/ORB_SLAM2/lib下__
```
cp /usr/lib/x86_64-linux-gnu/libboost_system.a ~/src/ORB_SLAM2/lib/
cp /usr/lib/x86_64-linux-gnu/libboost_system.so ~/src/ORB_SLAM2/lib/
cp /usr/lib/x86_64-linux-gnu/libboost_system.so.1.58.0 ~/src/ORB_SLAM2/lib/
cp /usr/lib/x86_64-linux-gnu/libboost_filesystem.a ~/src/ORB_SLAM2/lib/
cp /usr/lib/x86_64-linux-gnu/libboost_filesystem.so ~/src/ORB_SLAM2/lib/
cp /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.58.0 ~/src/ORB_SLAM2/lib/
```
如果没有libboost_system文件，可以运行以下命令查找
```
locate boost_system
locate boost_filesystem
```

- __将~/src/ORBSLAM2/Examples/ROS/ORBSLAM2下的Cmakelists.txt中加入库目录__
```
set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS} 
${Pangolin_LIBRARIES} 
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so 
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so 
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so 
#加入以下内容
${PROJECT_SOURCE_DIR}/../../../lib/libboost_filesystem.so 
${PROJECT_SOURCE_DIR}/../../../lib/libboost_system.so 
)
```


### 六. 硬件尺寸图
下图说明了相机的IMU和双目之间的结构图关系，由于是正视图，因此左右相机正好相反，使用时注意：

![enter image description here](https://ws1.sinaimg.cn/large/006tKfTcgy1fs8og6na6pj318g0eu0tl.jpg)

-------------------------------------------------------------------------------------------------

### 修订历史
- 2018-06-03手册首次发布
- 2018-06-08增加相机升级教程，kalibr的安装教程，常见问题汇总
- 2018-06-12增加okvis算法的安装
- 2018-07-03增加视差输出,畸变矫正输出,增加opencv鱼眼标定程序
- 2018-07-26增加VINS算法


