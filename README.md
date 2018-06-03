# CoolEye D1 Linux SDK & ROS 
Version 0.1
------------------------

本项目包括CoolEye D1 相机运行所需的必备文件,为了便于用于学习和使用,内容全部开源.
主流的SLAM算法,图像拼接,智能识别等功能会陆续放出.


#### SDK安装方法
为了便于用户一次成功,介绍时将使用绝对使用路径.

	mkdir -p ~/src
	cd ~/src
	git clone https://github.com/cooleyecam/cooleye_d1_linux_sdk_ros.git
	mkdir -p ~/src/cooleye_d1_linux_sdk_ros/sdk/build
	cd ~/src/cooleye_d1_linux_sdk_ros/sdk/build
	cmake ..
	make 
编译将生成2个文件
- CEAPP_MAIN   :  运行即可直接显示相机输出.
<<<<<<< HEAD

	./CEAPP_MAIN 


=======

	./CEAPP_MAIN 
>>>>>>> d84c207129d573d5591afd9a2d6627c9eb4303e2
- CEAPP_CALI_IMU : 用于校准相机自带的IMU的初始偏移. 这个六面矫正法移植自PX4的飞控固件,用起来比较麻烦,但是效果还可以,在发布自动矫正的软件版本之前,先临时用来修正IMU的误差.运行后根据操作.程序会自己将计算出的误差写入配置文件中,用户无需操作.


#### ROS环境安装方法
在这之前请先根据ros官网安装kinetic版本,建议使用kinetic版本.其他的版本在后续使用算法时可能会有些兼容问题.请先安装完SDK再安装ROS.

	cd ~/src/cooleye_d1_linux_sdk_ros/sdk/config/
	./updata_rosfile.sh
	cd ~/src/cooleye_d1_linux_sdk_ros/ros
	catkin_make
	source ~/src/cooleye_d1_linux_sdk_ros/devel/setup.bash
	
至此编译完成.执行以下命令即可运行相机.
	
	roscore
	rosrun cooleye_d1 CEROS_CAM_D1_NODE ~/src/cooleye_d1_linux_sdk_ros/ros/src/cooleye_d1/config/cecfg_std.txt

可使用ros命令观察相机运行状态

	rostopic hz /imu0_icm20689 /cooleyed1/left/image_raw /cooleyed1/right/image_raw


<<<<<<< HEAD
####修订历史

=======
#### 修订历史
>>>>>>> d84c207129d573d5591afd9a2d6627c9eb4303e2
- 2018-06-03手册首次发布



