# RTK_ROS

1.cp Onboard-SDK-ROS/(all) catkin_ws/src(your computer)

2.Find the serial with gnss receiver. 
	ls /dev/tty*
	or sudo cutecom
	sudo chmod 666 /dev/ttyUSB(0) #give the serial port the permission 

3.modify the catkin_ws/src/Onboard-SDK-ROS/gnss_observations/src/config/rtkrcv.conf 
(1)Serial---modify instr1.path with your serial under gnss receiver
(2)Base---modify instr2.path with your base infomation
(3)Out---modify outstr1.path to /Onboard-SDK-ROS/gnss_observations/data/
(4)Log---modify logstr1and2.path to /Onboard-SDK-ROS/gnss_observations/data/

4.modify the below paths in the /catkin_ws/src/Onboard-SDK-ROS/gnss_observations/src/gnss_data.cpp with your path
	ws_path = "/home/syl/catkin_ws/src/Onboard-SDK-ROS/gnss_observations";
    	opt_file = "/home/syl/catkin_ws/src/Onboard-SDK-ROS/gnss_observations/config/rtkrcv.conf";

5.cd~/catkin_ws
catkin_make
source devel/setup.bash 
echo $ROS PACKAGE PATH

roscore
rosrun gnss_observations gnss_data
rostopic list...
rostopic echo /rtk
