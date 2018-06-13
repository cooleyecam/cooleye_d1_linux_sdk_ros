create_udev_rules() {
	echo "" > cooleye_d1_cam.rules
	echo 'KERNEL=="*", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ACTION=="add", ATTR{idVendor}=="04b4",ATTR{idProduct}="1003", MODE="666" ' >> cooleye_d1_cam.rules
	echo 'KERNEL=="*", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ACTION=="remove" ' >> cooleye_d1_cam.rules
}

if [ `whoami` != 'root' ]; then
   echo "You have to be root to run this script"   
   echo "请使用root权限运行此脚本"
   exit 1;
fi

create_udev_rules
cp cooleye_d1_cam.rules /etc/udev/rules.d/
rm cooleye_d1_cam.rules
