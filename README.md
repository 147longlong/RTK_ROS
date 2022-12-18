# RTK_ROS

1.Copy the RTK package to the corresponding ros workspace

	cp Onboard-SDK-ROS/(all) catkin_ws/src(your computer)

2.Find the serial with gnss receiver serial after the handware is well connected.

	ls /dev/tty*
	
or use below GUI after installing 
	
	sudo cutecom
	
Then give the serial port the permission

	sudo chmod 666 /dev/ttyUSB(0) 

3.Modify the catkin_ws/src/Onboard-SDK-ROS/gnss_observations/src/config/rtkrcv.conf 

(1)Serial---modify instr1.path with your serial under gnss receiver

(2)Base---modify instr2.path with your base infomation

(3)Out---modify outstr1.path to /Onboard-SDK-ROS/gnss_observations/data/

(4)Log---modify logstr1and2.path to /Onboard-SDK-ROS/gnss_observations/data/

4.Modify the below paths in the /catkin_ws/src/Onboard-SDK-ROS/gnss_observations/src/gnss_data.cpp with your path

ws_path = "/home/syl/catkin_ws/src/Onboard-SDK-ROS/gnss_observations";

opt_file = "/home/syl/catkin_ws/src/Onboard-SDK-ROS/gnss_observations/config/rtkrcv.conf";

5.Environment configuration

	cd~/catkin_ws
	catkin_make
	source devel/setup.bash 
	echo $ROS PACKAGE PATH

6.Run rtkrcv and publish the topic to /rtk

	roscore
	rosrun gnss_observations gnss_data
	rostopic list...
	rostopic echo /rtk
