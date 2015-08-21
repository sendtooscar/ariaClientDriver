/*
 * ariaClientDriver.cpp
 *
 *  Created on: Mar 2, 2013
 *      Author: islab2
 * Change log
 * v1.1 - working bear minimum command line robot name parsing
 * v1.2 - tf prefix compatible no robot name parsing required (ros launch groups prefered instead)
 * v1.3 - raw laser data access - server modified
 * v1.4 - direct motion commands (unsafe) - server modified 
 * v1.5 - raw+pointcloud data for debug -- laser scan to point cloud (no need) --> synced_odom+laser request (server demo 2 modified) (/pioneer/Ariacmdvel for ration drive /pioneer/cmdvel for direct motion (need server demo 3))
 */
#include <std_msgs/Bool.h>
#include "ariaClientDriver.h"
#include <sensor_msgs/PointCloud.h>


AriaClientDriver::AriaClientDriver(ArClientBase *client, ArKeyHandler *keyHandler, std::string robotName) :
  myRobotStatePublisher(myKdlTree), myClient(client), myKeyHandler(keyHandler),
  myTransRatio(0.0), myRotRatio(0.0), myLatRatio(0.0), myMaxVel(50),
  myPrinting(false),   myNeedToPrintHeader(false),   myGotBatteryInfo(false), myNeedToInitialize(true), myDirectMotionEnabler(false), myXbias(0.0),myYbias(0.0),myThbias(0.0),
  /* Initialize functor objects with pointers to our handler methods: */
  myUpCB(this, &AriaClientDriver::up),
  myDownCB(this, &AriaClientDriver::down),
  myLeftCB(this, &AriaClientDriver::left),
  myRightCB(this, &AriaClientDriver::right),
  myLateralLeftCB(this, &AriaClientDriver::lateralLeft),
  myLateralRightCB(this, &AriaClientDriver::lateralRight),
  mySafeDriveCB(this, &AriaClientDriver::safeDrive),
  myUnsafeDriveCB(this, &AriaClientDriver::unsafeDrive),
  myStepBackCB(this, &AriaClientDriver::stepBack),
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
  myHandleRangeDataCB(this, &AriaClientDriver::handleRangeData),
  myHandleRangeDataCB2(this, &AriaClientDriver::handleRangeData2)
{
   //workaround for tf_prefix issue for laser
   myRosNodeHandle.getParam("tf_prefix", myRobotName);

   // robot name command line arguments are no longer used tf_prefix is used to handle multi robots 
   //myRobotName=robotName;		//string objects best handles the errors caused by unknown size
   //printf("%s\n",(myRobotName+std::string("/Twist")).c_str());
   //strcpy(myRobotName,robotName);
   //memcpy(myRobotName,robotName, sizeof(robotName));

   //KDL Tree and state publisher
   std::string robot_desc_string;
   myRosNodeHandle.getParam("/robot_description", robot_desc_string);
   if (!kdl_parser::treeFromString(robot_desc_string, myKdlTree)){
      ROS_ERROR("Failed to construct kdl tree\n please load urdf to parameter server:robot_description_pioneer1");
   }
   else{
	printf("KDL parser::OK\n");}
   myRobotStatePublisher=robot_state_publisher::RobotStatePublisher(myKdlTree);	

   //critical thread performing command updates
   myCmdVelSubscribe = myRosNodeHandle.subscribe("AriaCmdVel", 1, &AriaClientDriver::topicCallBack,this);
   //this is for direct motion command
   myTwistSubscribe = myRosNodeHandle.subscribe("cmd_vel", 1, &AriaClientDriver::topicCallBack2,this);
   myTwistSubscribe2 = myRosNodeHandle.subscribe("cmd_vel_ratio", 1, &AriaClientDriver::topicCallBack3,this);
   myDirectEnablerSubscribe = myRosNodeHandle.subscribe("direct_enable", 1, &AriaClientDriver::topicCallBack4,this);
   myNavdataPublish = myRosNodeHandle.advertise<ariaClientDriver::AriaNavData>("AriaNavData", 1000);
   myLaserPublish = myRosNodeHandle.advertise<sensor_msgs::LaserScan>("scan", 50);
   myLaserPublish2 = myRosNodeHandle.advertise<sensor_msgs::PointCloud>("scan2", 50);
   myOdomPublish = myRosNodeHandle.advertise<nav_msgs::Odometry>("odom", 50);

   // Critical thread performing sensor data acquisition
     
     // generic laser request - deprecated
     //ArNetPacket sensorName;
     //sensorName.strToBuf("sim_lms2xx_1"); //lms2xx_1 for actual robot sim_lms2xx_1 for sim robot
     //myClient->addHandler("getSensorCurrent", &myHandleRangeDataCB2);
     //myClient->request("getSensorCurrent",100,&sensorName);
     
     // raw laser request - deprecated
     //myClient->addHandler("LaserRequest", &myHandleRangeDataCB);
     //myClient->request("LaserRequest", 100);
     
     //generic odom request - deprecated
     //myClient->addHandler("update", &myHandleOutputCB);
     //myClient->request("update", 100);
     
     //row laser + synced odom request
     myClient->addHandler("LaserRequest_odom", &myHandleRangeDataCB);
     myClient->request("LaserRequest_odom", 100);

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
  myKeyHandler->addKeyHandler('o', &myStepBackCB);



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
  /*
  if (myClient->dataExists("updateNumbers") &&
	  myClient->dataExists("updateStrings"))
  {
	  printf("Using new updates\n");
	  myClient->addHandler("updateStrings", &myHandleOutputStringsCB);
	  myClient->request("updateStrings", -1);
  }
  */
  unsafeDrive();
  printf("Aria Client Driver node started...\n");
//printf("Debugging...\n");
//myClient->logDataList();
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


void AriaClientDriver::stepBack(){
  ArNetPacket request;
  request.empty();
  printf("Requesting step back %lf\n", 1000.0);
  request.byte2ToBuf((ArTypes::Byte2)(1000.0));
  myClient->requestOnce("MoveStepRequest",&request);
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

void AriaClientDriver::topicCallBack(const ariaClientDriver::AriaCommandData &msg) //deprecated
{	//edit to decode the recieving packet
	  //myTransRatio	=msg.TransRatio;
	  //myRotRatio	=msg.RotRatio;
	  //myLatRatio	=msg.LatRatio;
	  //myMaxVel		=msg.MaxVel;
	}

void AriaClientDriver::topicCallBack2(const geometry_msgs::Twist &msg) //direct motion command (need server demo 3)
{
    if(myDirectMotionEnabler){
	//should modify arnl server to accept velocities
	//myTransRatio		=msg.linear.x*10.0;
	//myRotRatio		=msg.angular.z*10.0;
	//myLatRatio		=msg.linear.y*10.0;
	//myMaxVel		=100.0;
	double vel=msg.linear.x*1000.0;
 	double rotVel=msg.angular.z/M_PI*180.0;

	ArNetPacket request;
  	request.empty();
  	//printf("Requesting set vel %lf\n", vel);
  	request.byte2ToBuf((ArTypes::Byte2)(vel));
	request.byte2ToBuf((ArTypes::Byte2)(rotVel));
  	myClient->requestOnce("SetVelRequest",&request);}
}

void AriaClientDriver::topicCallBack3(const geometry_msgs::Twist &msg) //ratio motion command
{
	//should modify arnl server to accept velocities
	myTransRatio		=msg.linear.x*10.0;
	myRotRatio		=msg.angular.z*10.0;
	myLatRatio		=msg.linear.y*10.0;
	myMaxVel		=100.0;
}

void AriaClientDriver::topicCallBack4(const std_msgs::Bool &msg){
        myDirectMotionEnabler = msg.data;
}

void AriaClientDriver::sendInput() //generic motion command
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

void AriaClientDriver::handleOutput(ArNetPacket *packet) //generic odom update -  not used
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
  msg.header.frame_id="odom";
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
  odom.header.frame_id ="odom";
  odom.pose.pose.position.x = myX/1000.0;
  odom.pose.pose.position.y = myY/1000.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(myTh/180*M_PI);
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = myVel/1000.0;
  odom.twist.twist.linear.y = myLatVel/1000.0;
  odom.twist.twist.angular.z = myRotVel/180*M_PI;
  myOdomPublish.publish(odom);

  //Assemble and Publish the sensortransform data massege - hndled by the urdf 
  //myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
  //		  current_time,"base_link","base_laser"));
  //Assemble and Publish the sensortransform data massege
  //myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
  //		  current_time,"base_link","base_reloc"));
  
  //Assemble and Publish the odometrytransform data massege
  myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,myTh/180*M_PI), tf::Vector3(myX/1000.0, myY/1000.0, 271/1000.0)),current_time,"odom","base_link"));

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

   //Decode packet (num_of readings, readings[0..180], myX, myY, myTh, myVel, myRotvel)
   int NumberOfReadings;
   NumberOfReadings = (int) packet->bufToByte4();
   for (int i=0; i<NumberOfReadings; i++){
    	myLaserReading[i]=packet->bufToByte4();
        //printf("%1.0f, ",myLaserReading[i]);
    }
    myX = (double) packet->bufToByte4();
    myY = (double) packet->bufToByte4();
    myTh = (double) packet->bufToByte4();
    myVel = (double) packet->bufToByte4();
    myRotVel = (double) packet->bufToByte4();
    myLatVel = 0;
    /* printf("\n%6s|%6s|%6s|%6s|%6s|%6s|%4s|%6s|%15s|%20s|\n",
	   "x","y","theta", "vel", "rotVel", "latVel", "temp", myVoltageIsStateOfCharge ? "charge" : "volts", "mode","status");
	    printf("%6.0f|%6.0f|%6.1f|%6.0f|%6.0f|%6.0f|%4.0d|%6.1f|%15s|%20s|\r",
	   myX, myY, myTh, myVel, myRotVel, myLatVel, myTemperature, myVoltage, myMode, myStatus);
    fflush(stdout);*/
   
  
    //Assemble and publish laser scan massege
      sensor_msgs::LaserScan msg;
      msg.header.stamp=ros::Time::now();
      msg.header.frame_id=(std::string("/")+myRobotName+std::string("/base_laser")).c_str();
      msg.angle_min=-M_PI/2;        			// start angle of the scan [rad]
      msg.angle_max=M_PI/2;       		// end angle of the scan [rad]
      msg.angle_increment=M_PI/(NumberOfReadings-1);  	// angular distance between measurements [rad]
      msg.time_increment=0.00008;//0.01333/180;   // time between measurements [seconds] ()75Hz motor
      msg.scan_time=0.01;//0.3;        		// time between scans [seconds]
      msg.range_min=0;        		// minimum range value [m]
      msg.range_max=30;        			// maximum range value [m]
      msg.ranges.resize(NumberOfReadings);
      msg.intensities.resize(NumberOfReadings);
      for (int i=0; i < NumberOfReadings; i++) {   	// range data [m] (Note: values < range_min or > range_max should be discarded)
    	  //printf("%f\n",myLaserReading[i]);
    	  msg.ranges[i]=(float)myLaserReading[i]/1000.0;
    	  msg.intensities[i]=1;
      }
      myLaserPublish.publish(msg);
       
	if(myNeedToInitialize){
	 	printf("Initializing odometer...\n ");
		
		myXbias=myX;
		myYbias=myY;
		myThbias=myTh;
		printf("Bias X:%f \tBias Y:%f \tBias Th:%f \t\n",myXbias,myYbias,myThbias);
	 	myNeedToInitialize=false;
	}

	//corected pose
        double x_corr=cos((myThbias)/180*M_PI)*(myX-myXbias)+sin((myThbias)/180*M_PI)*(myY-myYbias);
        double y_corr=-sin((myThbias)/180*M_PI)*(myX-myXbias)+cos((myThbias)/180*M_PI)*(myY-myYbias);
    

    //Assemble and publish odometry massege  
    	ros::Time current_time=ros::Time::now();
  	nav_msgs::Odometry odom;
  	odom.header.stamp = current_time;
  	odom.header.frame_id ="odom";
  	odom.pose.pose.position.x = (x_corr)/1000.0;
  	odom.pose.pose.position.y = (y_corr)/1000.0;
  	odom.pose.pose.position.z = 0.0;
  	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw((myTh-myThbias)/180*M_PI);
  	odom.child_frame_id = "base_link";
  	odom.twist.twist.linear.x = myVel/1000.0;
  	odom.twist.twist.linear.y = myLatVel/1000.0;
  	odom.twist.twist.angular.z = myRotVel/180*M_PI;
  	myOdomPublish.publish(odom);
    	myTfBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,(myTh-myThbias)/180*M_PI), tf::Vector3((x_corr)/1000.0, (y_corr)/1000.0, 271/1000.0)),current_time,"odom","base_link"));
  	//publish robot transforms
  	myRobotStatePublisher.publishFixedTransforms(); 
        //printf("Initializing pose...\n ");
	
}

void AriaClientDriver::handleRangeData2(ArNetPacket *packet) // if point cloud is needed- deprecated
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
      sensor_msgs::PointCloud pointcloud;
      pointcloud.header.stamp=ros::Time::now();
      pointcloud.header.frame_id=(std::string("/")+myRobotName+std::string("/base_laser")).c_str();
      pointcloud.points.resize(NumberOfReadings);
      for (int i=0; i<NumberOfReadings; i++){
      pointcloud.points[i].x=myLaserReadingX[i];
      pointcloud.points[i].y=myLaserReadingY[i];
      pointcloud.points[i].z=0.0;
      }
      pointcloud.channels.resize(1);
      pointcloud.channels[0].name="intensity";
      pointcloud.channels[0].values.resize(NumberOfReadings);
      for (int i=0; i<NumberOfReadings; i++){
      pointcloud.channels[0].values[i]=1;
      }
      
      //printf("publising\n");
      myLaserPublish2.publish(pointcloud);
      
    /*  sensor_msgs::LaserScan msg;
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
        // printf("%f\n",myLaserReading[i]);
      	  msg.ranges[i]=(float)myLaserReading[i];
      	  msg.intensities[i]=1;
    	  //msg.ranges[0]=5;
      }
//    	  //msg.intensities;    			// intensity data [device-specific units]
//      //ROS_INFO("%s", msg.Status.c_str());
       myLaserPublish.publish(msg);*/
}

void AriaClientDriver::controlloop(){
	ros::Rate myLoopRate(10);
	sendInput();
	ros::spinOnce();
	myLoopRate.sleep();
}


