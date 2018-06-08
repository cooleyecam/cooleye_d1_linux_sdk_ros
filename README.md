# CoolEye D1 Linux SDK & ROS 
### Version 0.1
本项目包括CoolEye D1 相机运行所需的必备文件,为了便于用于学习和使用,内容全部开源。
主流的SLAM算法,图像拼接,智能识别等功能会陆续放出。

-------------------------------------------------------------------------------------------------

## 1. SDK & ROS安装
推荐环境：ubuntu 16.04 LTS
推荐opencv版本：3.4.0
推荐ROS版本：Kinetic

__首先需要安装opencv__
建议使用3.2.0或3.4.0版本。
参考文档：
```
https://blog.csdn.net/u013066730/article/details/79411767
```

### 1.1 SDK安装方法
为了便于用户一次成功，介绍时将使用绝对使用路径。
```
sudo apt-get install libboost-dev libusb-1.0-0-dev libusb-dev git cmake 

mkdir -p ~/src

cd ~/src

git clone https://github.com/cooleyecam/cooleye_d1_linux_sdk_ros.git

mkdir -p ~/src/cooleye_d1_linux_sdk_ros/sdk/build

cd ~/src/cooleye_d1_linux_sdk_ros/sdk/build

cmake ..

make 

```
	
编译将生成2个文件：
- CEAPP_MAIN : 运行即可直接显示相机输出。
```	
./CEAPP_MAIN 
```
- CEAPP_CALI_IMU : 用于校准相机自带的IMU的初始偏移。这个六面矫正法移植自PX4的飞控固件，用起来比较麻烦，但是效果还可以，在发布自动矫正的软件版本之前，先临时用来修正IMU的误差。
运行后根据操作，程序会自己将计算出的误差写入配置文件中，用户无需操作。

```	
./CEAPP_CALI_IMU
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

source ~/src/cooleye_d1_linux_sdk_ros/devel/setup.bash

```
	
至此编译完成.执行以下命令即可运行相机.


```	
roscore

rosrun cooleye_d1 CEROS_CAM_D1_NODE 

~/src/cooleye_d1_linux_sdk_ros/ros/src/cooleye_d1/config/cecfg_std.txt
```
可使用ros命令观察相机运行状态
```
rostopic hz /imu0_icm20689 /cooleyed1/left/image_raw /cooleyed1/right/image_raw
```


-------------------------------------------------------------------------------------------------

## 2. 标定相机相关
### 2.1 ROS环境下标定相机
参考官方文档
```
http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
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

4. 编译项目。__-j4__ 根据自己电脑选择，四核就用__-j4__，8核就用__-j8__。编译需要很久，很久，真的很久，很久。
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


-------------------------------------------------------------------------------------------------

## 3. 运行算法相关

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

-------------------------------------------------------------------------------------------------

## 4. 相机固件升级方法

### 4.1 IMU操作固件升级
目前版本硬件，IMU的控制依赖于一个小型的单片机，如果需要升级，sdk/update会直接发布程序的HEX文件。windows下可使用FlyMcu进行升级，操作步骤见下图。另，熟悉STM32的用户，可根据自己习惯选择自己喜欢的升级方式。

![Alt text](./1528444485897.png)

### 4.2 图像处理固件升级
 目前版本硬件，图像Sensor的控制使用了Cypress的方案，sdk/update提供了windows下的驱动文件和升级工具，同时也会直接发布用于升级的IIC文件。升级步骤见下图。
 如果设备在电脑端识别异常，请在设备管理器中卸载设备驱动，然后更新驱动指向driver文件夹，比如：
D:\Cypress\USB\CY3684_EZ-USB_FX2LP_DVK\1.1\Drivers\vista
需要注意的是，这里只需指向vista即可，请勿继续选择其子目录x86或x64，可能会造成识别异常。
 ![Alt text](./1528445067142.png)


-------------------------------------------------------------------------------------------------

### 5. 常见问题汇总
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

-------------------------------------------------------------------------------------------------

### 修订历史
- 2018-06-03手册首次发布
- 2018-06-08增加相机升级教程，kalibr的安装教程，常见问题汇总。



