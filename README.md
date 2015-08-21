# ariaClientDriver
This is a Driver node for pioneer robots (or any robot) running using ARIA libraries of MoblieRobots Inc. This driver does not need a ROS installation on the robots host computer. Therefore this package is suitable for pioneer robots equipped with older computers which makes it difficult to install ROS. If you prefer to install ROS on the robots`s host computer, ROSARIA package is recomended (http://wiki.ros.org/ROSARIA).

The package is a Client software with a ROS Wrapper which connects with ARIA SDK based servers running on robots. The user simply runs the server on the robot`s host computer and runs the ROS client on a remote machine. This allows a quick and easy way to integrate robots running ARIA to a ROS multi-robot framework.

## Installation
1. Install the following from the mobile robots site (The instructions assumes a 32 bit Ubuntu system. For other systems please find the corresponding files in these links):

	* Aria SDK: Navigate to  http://robots.mobilerobots.com/wiki/ARIA and install ARIA 2.9.0 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture . 
	
	* Mobilesim Simulator: Navigate to http://robots.mobilerobots.com/wiki/MobileSim and install MobileSim 0.7.3 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture

	* ARNL Libraries (Optional- Not required for simulation): Navigate to http://robots.mobilerobots.com/wiki/ARNL,_SONARNL_and_MOGS and install Base ARNL Library 1.9.0 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture  and ARNL Laser Localization Library 1.9.0 - Ubuntu 12.04.2 (precise) or later, 32-bit i386 architecture .
	
2. The package is tested on ROS fuerte using rosbuild system on 32-bit Ubuntu 12.04  
For later distributions please follow the instructions given in section at the end. 
To install and configure ROS fuerte please use the steps detialed in the following link : http://wiki.ros.org/fuerte/Installation/Ubuntu 

3. Use the following steps to configure a ros_workspace directory (if not already configured):
 * $ cd ~
 * $ mkdir ros_workspace
 * $ echo "export ROS_PACKAGE_PATH=/opt/ros/fuerte/share:/opt/ros/fuerte/stacks:~/ros_workspace" >> ~/.bashrc
 
4. download the zip package and extract in the ros_workspace directory. Alternatively type the following commands in the terminal. 
 * $cd ~/ros_workspace
 * $git clone https://github.com/sendtooscar/ariaClientDriver.git

5. Navigate to the package directory and type make 
 * $cd ariaClientDriver
 * $make


6. If the compiler complains about CMake Cache files,  navigate to the ariaClient
Driver/build directory and remove CMakeCache.txt. Aria libraries included in the zip file are 32bit libraries so depending on your system edit the library paths in CMakeLists.txt file to the 64 bit library locations installed on your machine. The exact steps needed for a 64bit UBUNTU 12.04  system running hydro are given in section at the end.

7. The curent version of the driver provides raw laser data access and direct velocity motion commands.  For this to work, the robot or the simulating machine should compile and run serverDemo3.cpp modified server. The serverDemo3.cpp file is located in  /ariaClientDriver/Aria/ArNetworking/examples/  directory. Copy this to /usr/local/Aria/ArNetworking/examples/ directory. Open terminal and type the following to compile the modified server.
 * $ cd /usr/local/Aria/ArNetworking/examples/ 
 * $ make serverDemo3 

This completes the package installation. Use the following test examples to familiarize with the basic funcationality of the ariaClientDriver node.


