I'm hacking all of your systems.

YOUR PASSWORD IS "jer"


-------------------VM shared folder---------------------------------
sudo apt-get install virtualbox-guest-additions-iso
(or?)
sudo apt-get install virtualbox-guest-dkms
https://www.howtogeek.com/187703/how-to-access-folders-on-your-host-machine-from-an-ubuntu-virtual-machine-in-virtualbox/
sudo mount -t vboxsf -o uid=$UID,gid=$(id -g) HarCar ~/sharedfolder


---------------------RVIZ order of operations--------------------------
1. "roscore"
2. "rosparam set /use_sim_time true"
3. "rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10"
4. "roslaunch ...(whatever nodes)..."
5. "rviz"
6. "rosbag play ....bag"


---------------------to enable joystick after restart--------------------------
1. plug xbox controller to laptop via usb
2. in VM go to "devices" menu, "usb", then select "microsoft controller" to pass it through
3. "sudo modprobe xpad"
4. controller should be available (probably as js6)
5. (if connecting to harcarPi - must be running roscore!) "export ROS_MASTER_URI=http://harcarpi.local:11311"
6. 'rosparam set joy_node/dev "/dev/input/js6"'
7(?). 'rosparam set joy_node/deadzone "0.3"'
7(?). 'rosparam set joy_node/coalesce_interval  "0.05"'
8. "rosrun joy joy_node"

---------------------extract individual images from rosbag--------------------------
(this was for compressed images on topic /cv_camera/image_raw/compressed)
1. "rosbag play 2018-10-25-22-04-13.bag"
2. "rosrun image_view extract_images image:=/cv_camera/image_raw/ _image_transport:=compressed"
(note the leading underscore on _image_transport)


header: seq: 0 stamp: secs: 1541112762 nsecs: 743681849 frame_id: '' axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

