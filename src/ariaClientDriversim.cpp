/*
 * ariaClientDriver.cpp
 *
 *  Created on: Mar 2, 2013
 *      Author: islab2
 */
#include "ariaClientDriver.h"

AriaClientDriver::AriaClientDriver(ArClientBase *client, ArKeyHandler *keyHandler, std::string robotName) :
  myRobotStatePublisher(myKdlTree), myClient(client), myKeyHandler(keyHandler), 
  myTransRatio(0.0), myRotRatio(0.0), myLatRatio(0.0), myMaxVel(50),
  myPrinting(false),   myNeedToPrintHeader(false),   myGotBatteryInfo(false), 

  /* Initialize functor objects with pointers to our handler methods: */
  myUpCB(this, &AriaClientDriver::up),
  myDownCB(this, &AriaClientDriver::down),
  myLeftCB(this, &AriaClientDriver::left),
  myRightCB(this, &AriaClientDriver::right),
  myLateralLeftCB(this, &AriaClientDriver::lateralLeft),
  myLateralRightCB(this, &AriaClientDriver::lateralRight),
  mySafeDriveCB(this, &AriaClientDriver::safeDrive),
  myUnsafeDriveCB(this, &AriaClientDriver::unsafeDrive),
  myListDataCB(this, &AriaClientDriver::listData),
  myLogTrackingTerseCB(this, &AriaClientDriver::logTrackingTerse),
  myLogTrackingVerboseCB(this, &AriaClientDriver::logTrackingVerbose),
  myResetTrackingCB(this, &AriaClientDriver::resetTracking),
  myHandleOutputCB(this, &AriaClientDriver::handleOutput),
  myHandleOutputNumbersCB(this, &AriaClientDriver::handleOutputNumbers),
  myHandleOutputStringsCB(this, &AriaClientDriver::handleOutputStrings),
  myHandleBatteryInfoCB(this, &AriaClientDriver::handleBatteryInfo),
  myHandlePhysicalInfoCB(this, &AriaClientDriver::handlePhysicalInfo),
  myHandleTemperatureInfoCB(this, &AriaClientDriver::handleTemperatureInfo),
  myHandleSensorInfoCB(this, &AriaClientDriver::handleSensorInfo),
  myHandleRangeDataCB(this, &AriaClientDriver::handleRangeData)
{
   myRobotName=robotName;		//string objects best handles the errors caused by unknown size
   //printf("%s\n",(myRobotName+std::string("/Twist")).c_str());
   //strcpy(myRobotName,robotName);
   //memcpy(myRobotName,robotName, sizeof(robotName));	

   //KDL Tree and state publisher
   std::string robot_desc_string;
   myRosNodeHandle.getParam((std::string("robot_description_")+myRobotName).c_str(), robot_desc_string);
   if (!kdl_parser::treeFromString(robot_desc_string, myKdlTree)){
      ROS_ERROR("Failed to construct kdl tree\n please load urdf to parameter server:robot_description_<robotname>");
   }
   else{
	printf("KDL parser::OK\n");}
   myRobotStatePublisher=robot_state_publisher::RobotStatePublisher(myKdlTree);
   

   //critical thread performing command updates
   myCmdVelSubscribe = myRosNodeHandle.subscribe((myRobotName+std::string("/AriaCmdVel")).c_str(), 1, &AriaClientDriver::topicCallBack,this);
   myTwistSubscribe = myRosNodeHandle.subscribe((myRobotName+std::string("/Twist")).c_str(), 1, &AriaClientDriver::topicCallBack2,this);
   myNavdataPublish = myRosNodeHandle.advertise<ariaClientDriver::AriaNavData>((myRobotName+std::string("/AriaNavData")).c_str(), 1000);
   myLaserPublish = myRosNodeHandle.advertise<sensor_msgs::LaserScan>((myRobotName+std::string("/LaserData")).c_str(), 50);
   myOdomPublish = myRosNodeHandle.advertise<nav_msgs::Odometry>((myRobotName+std::string("/Odometry")).c_str(), 50);

   // Critical thread performing sensor data acquisition
     ArNetPacket sensorName;
     sensorName.strToBuf("sim_lms2xx_1"); //lms2xx_1 for actual robot sim_lms2xx_1 for sim robot
     myClient->addHandler("getSensorCurrent", &myHandleRangeDataCB);
     myClient->request("getSensorCurrent",100,&sensorName);
     myClient->addHandler("update", &myHandleOutputCB);
     myClient->request("update", 100);

  //myCmdVelSubscribe = myRosNodeHandle.subscribe<std_msgs::String>("AriaCmdVel", 1, Foo());

  // Handlers(Threads) for Keyboard input
  myKeyHandler->addKeyHandler(ArKeyHandler::UP, &myUpCB); //calls a method of an object pointer
  myKeyHandler->addKeyHandler(ArKeyHandler::DOWN, &myDownCB);//double colon used for namespace
  myKeyHandler->addKeyHandler(ArKeyHandler::LEFT, &myLeftCB);
  myKeyHandler->addKeyHandler(ArKeyHandler::RIGHT, &myRightCB);
  myKeyHandler->addKeyHandler('q', &myLateralLeftCB);
  myKeyHandler->addKeyHandler('e', &myLateralRightCB);
  myKeyHandler->addKeyHandler('s', &mySafeDriveCB);
  myKeyHandler->addKeyHandler('u', &myUnsafeDriveCB);
  myKeyHandler->addKeyHandler('l', &myListDataCB);
  myKeyHandler->addKeyHandler('t', &myLogTrackingTerseCB);
  myKeyHandler->addKeyHandler('v', &myLogTrackingVerboseCB);
  myKeyHandler->addKeyHandler('r', &myResetTrackingCB);
  myKeyHandler->addKeyHandler('o', &mySquareDriveCB);



  // Handlers(Threads) for Service calls
  
  myClient->addHandler("physicalInfo", &myHandlePhysicalInfoCB);
  myClient->requestOnce("physicalInfo");
  myClient->addHandler("batteryInfo", &myHandleBatteryInfoCB);
  myClient->requestOnce("batteryInfo");
  if (myClient->dataExists("temperatureInfo"))
  {
	  myClient->addHandler("temperatureInfo", &myHandleTemperatureInfoCB);
	  myClient->requestOnce("temperatureInfo");
  }
  if (myClient->dataExists("getSensorList"))
  {
	  myClient->addHandler("getSensorList", &myHandleSensorInfoCB);
	  myClient->requestOnce("getSensorList");
  }
  if (myClient->dataExists("updateNumbers") &&
	  myClient->dataExists("updateStrings"))
  {
	  printf("Using new updates\n");
	  myClient->addHandler("updateStrings", &myHandleOutputStringsCB);
	  myClient->request("updateStrings", -1);
  }
  unsafeDrive();
  printf("Aria Client Driver node started...\n");
printf("Debugging...\n");
}

AriaClientDriver::~AriaClientDriver(void)
{
	myClient->requestStop("update");
}

void AriaClientDriver::up(void)
{
  if (myPrinting)
    printf("Forwards\n");
  myTransRatio = 100;
}

void AriaClientDriver::down(void)
{
  if (myPrinting)
    printf("Backwards\n");
  myTransRatio = -100;
}

void AriaClientDriver::left(void)
{
  if (myPrinting)
    printf("Left\n");
  myRotRatio = 100;
}

void AriaClientDriver::right(void)
{
  if (myPrinting)
    printf("Right\n");
  myRotRatio = -100;
}

void AriaClientDriver::lateralLeft(void)
{
  if (myPrinting)
    printf("Lateral left\n");
  myLatRatio = 100;
}

void AriaClientDriver::lateralRight(void)
{
  if (myPrinting)
    printf("Lateral right\n");
  myLatRatio = -100;
}

void AriaClientDriver::safeDrive()
{
  /* Construct a request packet. The data is a single byte, with value
   * 1 to enable safe drive, 0 to disable. */
  ArNetPacket p;
  p.byteToBuf(1);

  /* Send the packet as a single request: */
  if(myPrinting)
    printf("Sending setSafeDrive 1.\n");
  myClient->requestOnce("setSafeDrive",&p);
  if(myPrinting)
    printf("\nSent enable safe drive.\n");
}

void AriaClientDriver::unsafeDrive()
{
  /* Construct a request packet. The data is a single byte, with value
   * 1 to enable safe drive, 0 to disable. */
  ArNetPacket p;
  p.byteToBuf(0);

  /* Send the packet as a single request: */
  if(myPrinting)
    printf("Sending setSafeDrive 0.\n");
  myClient->requestOnce("setSafeDrive",&p);
  if(myPrinting)
    printf("\nSent disable safe drive command. Your robot WILL run over things if you're not careful.\n");
}

void AriaClientDriver::listData()
{
  myClient->logDataList();
}

void AriaClientDriver::logTrackingTerse()
{
  myClient->logTracking(true);
}

void AriaClientDriver::logTrackingVerbose()
{
  myClient->logTracking(false);
}

void AriaClientDriver::resetTracking()
{
  myClient->resetTracking();
}

void AriaClientDriver::topicCallBack(const ariaClientDriver::AriaCommandData &msg)
{	//edit to decode the recieving packet
	  myTransRatio	=msg.TransRatio;
	  myRotRatio	=msg.RotRatio;
	  myLatRatio	=msg.LatRatio;
	  myMaxVel		=msg.MaxVel;
	}

void AriaClientDriver::topicCallBack2(const geometry_msgs::Twist &msg)
{
	printf("ok\n");
	//should modify arnl server to accept velocities
	myTransRatio	         =msg.linear.x*100;
	myRotRatio		=msg.angular.z*100;
	myLatRatio		=msg.linear.y*100;
	myMaxVel		=100.0;
}

void AriaClientDriver::sendInput()
{
  /* This method is called by the main function to send a ratioDrive
   * request with our current velocity values. If the server does
   * not support the ratioDrive request, then we abort now: */
  if(!myClient->dataExists("ratioDrive")) return;

  /* Construct a ratioDrive request packet.  It consists
   * of three doubles: translation ratio, rotation ratio, and an overall scaling
   * factor. */



  ArNetPacket packet;
  packet.doubleToBuf(myTransRatio);
  packet.doubleToBuf(myRotRatio);
  packet.doubleToBuf(myMaxVel); // use half of the robot's maximum.
  packet.doubleToBuf(myLatRatio);
  if (myPrinting)
    printf("Sending\n");
  myClient->requestOnce("ratioDrive", &packet);
  myTransRatio = 0;
  myRotRatio = 0;
  myLatRatio = 0;
}

void AriaClientDriver::handleOutput(ArNetPacket *packet)
{
  /* Extract the data from the update packet. Its format is status and
   * mode (null-terminated strings), then 6 doubles for battery voltage,
   * x position, y position and orientation (theta) (from odometry), current
   * translational velocity, and current rotational velocity. Translation is
   * always milimeters, rotation in degrees.
   */
  memset(myStatus, 0, sizeof(myStatus));
  memset(myMode, 0, sizeof(myMode));
  packet->bufToStr(myStatus, sizeof(myStatus));
  packet->bufToStr(myMode, sizeof(myMode));
  myVoltage = ( (double) packet->bufToByte2() )/10.0;
  myX = (double) packet->bufToByte4();
  myY = (double) packet->bufToByte4();
  myTh = (double) packet->bufToByte2();
  myVel = (double) packet->bufToByte2();
  myRotVel = (double) packet->bufToByte2();
  myLatVel = (double) packet->bufToByte2();
  myTemperature = (double) packet->bufToByte();

  if(myNeedToPrintHeader)
  {
    printf("\n%6s|%6s|%6s|%6s|%6s|%6s|%4s|%6s|%15s|%20s|\n",
	   "x","y","theta", "vel", "rotVel", "latVel", "temp", myVoltageIsStateOfCharge ? "charge" : "volts", "mode","status");
    fflush(stdout);
    myNeedToPrintHeader = false;
  }
  if (myGotBatteryInfo)
    printf("\r%6.0f|%6.0f|%6.1f|%6.0f|%6.0f|%6.0f|%4.0d|%6.1f|%15s|%20s|\r",
	   myX, myY, myTh, myVel, myRotVel, myLatVel, myTemperature, myVoltage, myMode, myStatus);

  fflush(stdout);


  //Assemble and Publish the custom Nav data massege
  ariaClientDriver::AriaNavData msg;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=(std::string("/")+myRobotName).c_str();
  msg.X=myX/1000.0;
  msg.Y=myY/1000.0;
  msg.Th=myTh/180*M_PI;
  msg.Vel=myVel/1000.0;
  msg.RotVel=myRotVel/180*M_PI;
  msg.LatVel=myLatVel/1000.0;
  msg.Temperature=myTemperature;
  msg.Voltage=myVoltage;
  msg.Mode=myMode;
  msg.Status=myStatus;
  //ROS_INFO("%s", msg.Status.c_str());
  myNavdataPublish.publish(msg);

  //to do - implement odometer (currently we use Arnl localization)

  ros::Time current_time=ros::Time::now();
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "/odom";
  odom.pose.pose.position.x = myX/1000.0;
  odom.pose.pose.position.y = myY/1000.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(myTh/180*M_PI);
  odom.child_frame_id = (std::string("/")+myRobotName+std::string("/base_link")).c_str();
  odom.twist.twist.linear.x = myVel;
  odom.twist.twist.linear.y = myLatVel;
  odom.twist.twist.angular.z = myRotVel;
  myOdomPublish.publish(odom);

  //Assemble and Publish the sensortransform data massege
  myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
		  current_time,(std::string("/")+myRobotName+std::string("/base_link")).c_str(), (std::string("/")+myRobotName+std::string("/base_laser")).c_str()));
  //Assemble and Publish the sensortransform data massege
  myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
		  current_time,(std::string("/")+myRobotName+std::string("/base_link")).c_str(), (std::string("/")+myRobotName+std::string("/base_reloc")).c_str()));
  //Assemble and Publish the odometrytransform data massege
  myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,myTh/180*M_PI), tf::Vector3(myX/1000.0, myY/1000.0, 0)),current_time,"/odom",(std::string("/")+myRobotName+std::string("/base_link")).c_str()));

  //publish robot transforms
  myRobotStatePublisher.publishFixedTransforms();

}

void AriaClientDriver::handleOutputNumbers(ArNetPacket *packet)
{
  /* Extract the data from the updateNumbers packet. Its format is 6
   * doubles for battery voltage, x position, y position and
   * orientation (theta) (from odometry), current translational
   * velocity, and current rotational velocity. Translation is always
   * milimeters, rotation in degrees.
   */
  myVoltage = ( (double) packet->bufToByte2() )/10.0;
  myX = (double) packet->bufToByte4();
  myY = (double) packet->bufToByte4();
  myTh = (double) packet->bufToByte2();
  myVel = (double) packet->bufToByte2();
  myRotVel = (double) packet->bufToByte2();
  myLatVel = (double) packet->bufToByte2();
  myTemperature = (double) packet->bufToByte();

  if(myNeedToPrintHeader)
  {
    printf("\n%6s|%6s|%6s|%6s|%6s|%6s|%4s|%6s|%15s|%20s|\n",
	   "x","y","theta", "vel", "rotVel", "latVel", "temp", myVoltageIsStateOfCharge ? "charge" : "volts", "mode","status");
    fflush(stdout);
    myNeedToPrintHeader = false;
  }
  if (myGotBatteryInfo)
    printf("%6.0f|%6.0f|%6.1f|%6.0f|%6.0f|%6.0f|%4.0d|%6.1f|%15s|%20s|\r",
	   myX, myY, myTh, myVel, myRotVel, myLatVel, myTemperature, myVoltage, myMode, myStatus);

  fflush(stdout);
}

void AriaClientDriver::handleOutputStrings(ArNetPacket *packet)
{
  /* Extract the data from the updateStrings packet. Its format is
   * status and mode (null-terminated strings).
   */
  memset(myStatus, 0, sizeof(myStatus));
  memset(myMode, 0, sizeof(myMode));
  packet->bufToStr(myStatus, sizeof(myStatus));
  packet->bufToStr(myMode, sizeof(myMode));
}

void AriaClientDriver::handleBatteryInfo(ArNetPacket *packet)
{
  /* Get battery configuration parameters: when the robot will begin beeping and
   * warning about low battery, and when it will automatically disconnect and
   * shutdown. */
  double lowBattery = packet->bufToDouble();
  double shutdown = packet->bufToDouble();
  printf("Low battery voltage: %6g       Shutdown battery voltage: %6g\n", lowBattery, shutdown);
  fflush(stdout);
  myNeedToPrintHeader = true;
  myGotBatteryInfo = true;

  if (packet->getDataReadLength() == packet->getDataLength())
  {
    printf("Packet is too small so its an old server, though you could just get to the bufToUByte anyways, since it'd be 0 anyhow\n");
    myVoltageIsStateOfCharge = false;
  }
  else
    myVoltageIsStateOfCharge = (packet->bufToUByte() == 1);

}


void AriaClientDriver::handlePhysicalInfo(ArNetPacket *packet)
{
  /* Get phyiscal configuration parameters: */
  char robotType[512];
  char robotSubtype[512];
  int width;
  int lengthFront;
  int lengthRear;

  packet->bufToStr(robotType, sizeof(robotType));
  packet->bufToStr(robotSubtype, sizeof(robotSubtype));
  width = packet->bufToByte2();
  lengthFront = packet->bufToByte2();
  lengthRear = packet->bufToByte2();

  printf("Type: %s Subtype: %s Width %d: LengthFront: %d LengthRear: %d\n",
	 robotType, robotSubtype, width, lengthFront, lengthRear);
  fflush(stdout);
}

void AriaClientDriver::handleTemperatureInfo(ArNetPacket *packet)
{
  char warning = packet->bufToByte();
  char shutdown = packet->bufToByte();
  printf("High temperature warning: %4d       High temperature shutdown: %4d\n", warning, shutdown);
  fflush(stdout);
  myNeedToPrintHeader = true;
}

void AriaClientDriver::handleSensorInfo(ArNetPacket *packet)
{
  int numberOfSensors = packet->bufToByte2();
  printf("I have %d Range sensors \n", numberOfSensors);
  for(int i=0;i<numberOfSensors;i++){
	  memset(mySensors, 0, sizeof(mySensors));
	  packet->bufToStr(mySensors, sizeof(mySensors));
	  printf("%d    %s\n",i+1, mySensors);
  }
  fflush(stdout);
}

void AriaClientDriver::handleRangeData(ArNetPacket *packet)
{
	int NumberOfReadings;
	char SensorName[256];
	double LaserReadingX;
	double LaserReadingY;



    memset(SensorName, 0, sizeof(SensorName));
    NumberOfReadings = (double) packet->bufToByte2();
    packet->bufToStr(SensorName, sizeof(SensorName));
    //printf("%6.0d|",NumberOfReadings);
    //printf("laser scan start\n");
    for (int i=0; i<NumberOfReadings; i++){
    	LaserReadingX = (double) packet->bufToByte4();
    	LaserReadingY = (double) packet->bufToByte4();

    	//printf("%6.0f|%6.0f|",LaserReadingX,LaserReadingY);
    	myLaserReadingX[i]=LaserReadingX;
    	myLaserReadingY[i]=LaserReadingY;
    	myLaserReading[i]=sqrt(pow(LaserReadingX-myX,2)+pow(LaserReadingY-myY,2))/1000.0;
        //printf("%6.0f, ",myLaserReadingY[i]);
        //printf("%6.0f|%6.0f|\n",myLaserReadingX[i],myLaserReadingY[i]);
    	//printf("%6.0f|%6.0f|%f\n",myLaserReadingX[i],myLaserReadingY[i],myLaserReading[i]);


    }
    //printf("\nlaser scan end\n");
    //edit to encode transmitting massege
      sensor_msgs::LaserScan msg;
      msg.header.stamp=ros::Time::now();
      msg.header.frame_id=(std::string("/")+myRobotName+std::string("/base_laser")).c_str();
      msg.angle_min=-M_PI/2;        			// start angle of the scan [rad]
      msg.angle_max=M_PI/2;       		// end angle of the scan [rad]
      msg.angle_increment=M_PI/NumberOfReadings;  	// angular distance between measurements [rad]
      msg.time_increment=0;//0.01333/180;   // time between measurements [seconds]
      msg.scan_time=0;//0.3;        		// time between scans [seconds]
      msg.range_min=0;        		// minimum range value [m]
      msg.range_max=30;        			// maximum range value [m]
      msg.ranges.resize(NumberOfReadings);
      msg.intensities.resize(NumberOfReadings);
      for (int i=0; i < NumberOfReadings; i++) {   	// range data [m] (Note: values < range_min or > range_max should be discarded)
    	  //printf("%f\n",myLaserReading[i]);
    	  msg.ranges[i]=(float)myLaserReading[i];
    	  msg.intensities[i]=1;
    	  //msg.ranges[0]=5;
      }
//    	  //msg.intensities;    			// intensity data [device-specific units]
//      //ROS_INFO("%s", msg.Status.c_str());
       myLaserPublish.publish(msg);
}

void AriaClientDriver::controlloop(){
	ros::Rate myLoopRate(10);
	sendInput();
	ros::spinOnce();
	myLoopRate.sleep();
}


