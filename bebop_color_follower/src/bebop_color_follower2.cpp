//Coded by Alexis Guijarro
#include <bebop_simple_test/PID_ROS.h>
#include <bebop_simple_test/Altitude.h>
#include <bebop_simple_test/ArcDrone.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <time.h>
#include <cmath>
#define PI 3.141592653589793238462
static const std::string OPENCV_ORIGINAL = "Bebop camera window";		//Original image 
static const std::string OPENCV_BINARY = "Binary window";
static const std::string OPENCV_BINARYLINE = "Binary window Line";			//Binary image
bool active = true;
const double max_obj_area = 10000000; 						//Maximum area reference of the object
const double min_obj_area = 100;						//Minimum area reference of the object
const double bebop_velocity = 0.17;						//Bebop general velocity
const double bebop_turn_velocity = 0.42;					//Bebop general turn velocity
const double area_distance = 30;						//The area determines the proximity of the camera to the object/noise
double dArea;									//Holds the general area of the objects at the imageconst

float turning_speed = 3;
float angle_to_turn = 180;
time_t start_time =0;
time_t end_time = 0;

//Heading Variables
float _heading;
float _heading_t;
float _heading_t2;
float _arc;
float _arc2;
float _arc3;
float _fixed_arc;
int camera_angle = 0;
float odom_x;
float odom_y;
float odom_z;

PID pid_alt;					//PID Altitude 
				//Derivative Constant
PID_h pid_hdg;					//PID Heading

PID pid_odom;

cv::Point2f drone_center;							//Represents the center of the image, also the Bebop's nose
bebop_simple_test::Altitude alt;
cv_bridge::CvImagePtr cv_original;						//Original image container 

int iLowH = 90;		//5	//	90				//Default HSV range values
int iHighH = 150;//16	150

int iLowS = 180; //177 180
int iHighS = 255; //239 255

int iLowV = 110;//178 165
int iHighV = 255;//255 255

int posX,posY = 0;
int posXRed,posYRed = 0;									//Target Position
bool no_object = true;								//No Tracked object present
bool no_objectRed = false;
bool first_object = true;
bool exit_from_cv = false;							//Variable to indicate to exit from OpenCV to ROS
int tracking_system_armed = 0;
int landing_system_armed = 0;
int following_system_armed = 0;
int landing_mode = 0;
int follow_orange = 1;								//0 - Drone hovers, 1 - Drone move to the target
int turning_mode = 0;	
int forget_red = 1;
//AutoPicker HSV
bool auto_detect_hsv = false;
int pX,pY = 0;
int Turns = 0;
int goo = 0;
int goo2 = 0;


std_msgs::Empty take_off,land;							//Variable to take_off and land
geometry_msgs::Twist cmd_vel_bebop,cam_bebop;						//Variable that stores the linear velocity commands of Bebop
std_msgs::Float64 output_alt_pid;		//The signal of the PID to control over the alttitude
std_msgs::Float64 output_hdg_pid;
ros::Publisher takeoff_bebop;							//Sends the message to make the drone to take off 
ros::Publisher land_bebop;							//Sends the message to make the drone to land
ros::Publisher cmd_vel_pub_bebop;						//Sends the message to move the drone
ros::Publisher pub_cam_tilt;
ros::Subscriber bebopAlt;			//Receives the alttitude from the built-in sensor
ros::Publisher correct_alt;
ros::Publisher correct_hdg;		
ros::Subscriber bebopOdom;



cv::Scalar LowerYellow;
    cv::Scalar UpperYellow;
    cv::Mat img_hsv;
    cv::Mat img_mask;
    cv::Mat img;  /// Input image in opencv matrix format
    cv::Mat img_filt; 
void turn()
{
	cmd_vel_bebop.linear.x = 0;						//Turns the drone without translation
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to turn
}
void hover()
{
	cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER
	cmd_vel_bebop.linear.y = 0; 
	cmd_vel_bebop.linear.z = 0;
	cmd_vel_bebop.angular.z = 0;

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
}
void auto_pick_HSV(cv::Mat& image)
{
	cv::Vec3b temp= image.at<cv::Vec3b>(pY,pX);
	int blue = temp.val[0];
	int green = temp.val[1];
	int red = temp.val[2];
	
	cv::Mat tHSV;
	cv::cvtColor(image,tHSV,cv::COLOR_BGR2HSV);
	cv::Vec3b hsv = tHSV.at<cv::Vec3b>(pY,pX);
	int h = hsv.val[0];
	int s = hsv.val[1];
	int v = hsv.val[2];

	int range = 30;

	iLowH = h - range;
	iHighH = h + range;
	iLowS = s - range*2.5;
	iHighS = s + range*2.5;
	iLowV = v - range*3;
	iHighV = v + range*3;


	cv::setTrackbarPos("LowH", OPENCV_BINARY, iLowH);
	cv::setTrackbarPos("HighH",OPENCV_BINARY, iHighH);

	cv::setTrackbarPos("LowS", OPENCV_BINARY, iLowS);
	cv::setTrackbarPos("HighS", OPENCV_BINARY, iHighS);

	cv::setTrackbarPos("LowV", OPENCV_BINARY, iLowV);
	cv::setTrackbarPos("HighV", OPENCV_BINARY, iHighV);

	auto_detect_hsv = false;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
	auto_detect_hsv = true;
	pX = x;
	pY = y;	
     }
}

void sort_blob_by_size(std::vector<cv::KeyPoint> k)
{
	cv::KeyPoint big_keypoint_by_size;
	if(k.size() > 0)
	{
		for(int i = 0; i<k.size();i++)
		{
			if(k[i].size > big_keypoint_by_size.size)
			{
				big_keypoint_by_size = k[i];
			}
		}
		posX = big_keypoint_by_size.pt.x;
		posY = big_keypoint_by_size.pt.y;
		no_object = false;
	 //	no_objectRed = false;
	}
	else
	{
		posX = drone_center.x;
		posY = drone_center.y;
		no_object = true;
		//no_objectRed = true;
		
	}
}




void c_drone(cv::Mat& image)
{
	cv::Point2f center;
	center.x = image.cols / 2;
	center.y = image.rows / 2;
	cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);
	drone_center = center;
}

cv::Mat detect_blobs(cv::Mat& image)
{
	cv::SimpleBlobDetector::Params params;
	params.minThreshold = 10;
	params.maxThreshold = 200;
 
	params.filterByArea = true;
	params.minArea = min_obj_area;
	params.maxArea = max_obj_area;

	params.filterByColor = true;
	params.blobColor = 255;
 
	params.filterByCircularity = false;
 
	params.filterByConvexity = false;
 
	params.filterByInertia = false;
	cv::Ptr<cv::SimpleBlobDetector> d = cv::SimpleBlobDetector::create(params);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat cloned_image = image.clone();
	d->detect(cloned_image,keypoints);
	
	sort_blob_by_size(keypoints);
	cv::Mat result;
	cv::drawKeypoints( cloned_image, keypoints, result, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	return result;
}

void discriminate_color(cv::Mat& image)
{
	cv::namedWindow(OPENCV_BINARY);						//Creating Control Interface
	cv::createTrackbar("LowH", OPENCV_BINARY, &iLowH, 180);
	cv::createTrackbar("HighH",OPENCV_BINARY, &iHighH, 180);

	cv::createTrackbar("LowS", OPENCV_BINARY, &iLowS, 255);
	cv::createTrackbar("HighS", OPENCV_BINARY, &iHighS, 255);

	cv::createTrackbar("LowV", OPENCV_BINARY, &iLowV, 255);
	cv::createTrackbar("HighV", OPENCV_BINARY, &iHighV, 255);
	
	if(auto_detect_hsv){auto_pick_HSV(image);}				//AutoPicker HSV 

	cv::Mat imgHSV;								//HSV Image Container
	cv::cvtColor(cv_original->image, imgHSV, cv::COLOR_BGR2HSV);		//Convert BGR to HSV Color Space

	cv::Mat imgThresholded,imgThresholded2,imgThresholded3;
	cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS,iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded2);
	cv::inRange(imgHSV, cv::Scalar(0, 101,100), cv::Scalar(58, 251, 255), imgThresholded3);

	cv::bitwise_or(imgThresholded2,imgThresholded3,imgThresholded);
	//ROS_INFO("LowH: %d LowS: %d LowV: %d HighH: %d HighS: %d HighV: %d",iLowH, iLowS, iLowV,iHighH, iHighS, iHighV);

	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));	//Morphology Operations
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	cv::blur(imgThresholded,imgThresholded,cv::Size(5,5));			//Basic Filtering
	
	cv::Mat cntr = imgThresholded.clone();					//Contours container
	std::vector<std::vector<cv::Point> > contours;				//Contours vector
	cv::findContours(cntr, contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);	//Find contours
	cv::drawContours(cntr, contours, -1, cv::Scalar(255, 255, 255), -1);		//Draw contours
	std::vector<std::vector<cv::Point> > convexHulls(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++) 			//Find Convex Hulls
	{
		cv::convexHull(contours[i], convexHulls[i]);
	}
	cv::Mat conv(cntr.size(), CV_8UC3, cv::Scalar(0, 0, 0));		//Convex Hulls container
	cv::drawContours(conv, convexHulls, -1, cv::Scalar(255, 255, 255), -1); //Draw Convex Hulls
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)));	//Closing gaps
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)));
	cv:cvtColor(conv,conv,cv::COLOR_BGR2GRAY);				//Convert to 1-channel image
	cv::Mat blobs = detect_blobs(conv);					//Detect the blobs in the image, returns the image with detected blobs
	cv::imshow(OPENCV_BINARY,blobs);					//Show binary image with blobs
	cv::Moments mu = moments(imgThresholded);						//Calculate moments
	dArea=mu.m00;								//Area that determines if the drone is near the target and stop it
	contours.clear();
	convexHulls.clear();
}







void get_heading(float _orientationX, float _orientationY , float _orientationZ, float _orientationW)
{
	double roll;
	double pitch;
	double yaw;
	tf::Quaternion q(_orientationX,_orientationY,_orientationZ,_orientationW);
	
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
	
	//Roll, Pitch and Yaw are in radians, must convert it to degrees
	roll = roll*(180/PI);
	pitch = pitch *(180/PI);
	yaw = yaw*(180/PI);
	
	//Yaw value copied into the global variable _heading for further use 
	_heading = yaw;
}

void altCallback(const bebop_simple_test::Altitude::ConstPtr& msg)
{	
	if (active)
	{
		float var1 = msg->altitude; 		//Altitude field expects a float variable
		alt.altitude = var1;
	//	printf("\n Altura %lf \n", var1);	//Altitude in meters
	}
}
void move_drone(cv::Mat& image)
{
	


	
	if (posX >= 0 && posY >= 0)
	{


		
		if (posX < drone_center.x+10)
		{
			//cmd_vel_bebop.angular.z = bebop_turn_velocity;						
			//std::cout << "Move Left" << std::endl;
		}
		if (posX > drone_center.x-10)
			{
				//cmd_vel_bebop.angular.z = -bebop_turn_velocity;
			//	std::cout << "Move Right" << std::endl;
		}
		//printf("angle, %f",camera_angle);
		if (posY > drone_center.y+10)
		{	
			//camera_angle =camera_angle -1;
			//cam_bebop.angular.y = camera_angle;
			//pub_cam_tilt.publish(cam_bebopstd::cout << "Move Right);
			//cmd_vel_bebop.linear.z = -bebop_velocity;
			//std::cout << "Move Downwards" << std::endl;

		}
		if (posY < drone_center.y-80)
		{	
			//camera_angle =camera_angle+ 1;
			//cam_bebop.angular.y = camera_angle;
			//pub_cam_tilt.publish(cam_bebop);	
			//cmd_vel_bebop.linear.z = bebop_velocity;
			//std::cout << "Move Upwards" << std::endl;
		}
		if(tracking_system_armed)
		{
			cv::circle(image, cv::Point(posX, posY), 8, cv::Scalar(0, 255, 0), -1);
		}
		if(!tracking_system_armed)
		{
			cv::circle(image, cv::Point(posX, posY), 8, cv::Scalar(0, 255, 255), -1);
		}
		
	}
	



	


	if(tracking_system_armed)
	{
		
		int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
		//double dblFontScale = 0.75;
		double dblFontScale = 1;
		int intFontThickness = 2;
		//cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);



		if(!no_object && !first_object){
			//cmd_vel_bebop.linear.x = bebop_velocity;
			//printf("AKI NAO ERA PRA ENTRAR\n");
			
			if (posY > drone_center.y+20)
			{	
				cmd_vel_bebop.linear.x = -0.01;		
			}else{
				if (posY < drone_center.y-20)
				{	
					cmd_vel_bebop.linear.x = 0.01;
				}
				else{
					//printf("meu VALOR %f\n",(0.1*int(drone_center.x-posX)/100));
					cmd_vel_bebop.linear.x = (0.01*int(drone_center.x-posX)/100);
				}
			}
		

			


			//if (Turns==1){

				if (posX > drone_center.x+30)
					{
					
					cmd_vel_bebop.angular.z = -0.03;
					cmd_vel_bebop.linear.y = -0.02;		
				}else{
					if (posX < drone_center.x-30)
						{
						printf("absoluto2 %f\n", pid_alt.mis(std::abs(int(drone_center.x-posX)),0)/100);	
						//printf("POS x, %d\n", posX);	
						//printf("absoluto2 %d\n",std::abs(int(posX-drone_center.x)));	
				//printf("turning right");
						cmd_vel_bebop.angular.z = 0.03;
						cmd_vel_bebop.linear.y = 0.02;
						}
					else{
					
							cmd_vel_bebop.angular.z = (0.03*int(drone_center.x-posX)/100);
							cmd_vel_bebop.linear.y = (0.02*int(drone_center.x-posX)/100);
							printf("meu VALOR %f\n",cmd_vel_bebop.linear.y);
						
				
						}
					}

					
				if ((posX < drone_center.x+20) && (posX > drone_center.x-20) &&(posY < drone_center.y+20) &&(posY > drone_center.y-20) ){
					cmd_vel_bebop.linear.x = 0.0;
					cmd_vel_bebop.linear.y = 0.0;		
					cmd_vel_bebop.linear.z = 0.0;
					cmd_vel_bebop.angular.z = 0.0;
					cmd_vel_pub_bebop.publish(cmd_vel_bebop);
					land_bebop.publish(land);
					start_time=time(NULL);
					

					//ros::Duration(5.0).sleep();
					//takeoff_bebop.publish(take_off);
					//ros::Duration(5.0).sleep();
					tracking_system_armed = 0;
					turning_mode= 0 ;
					landing_mode = 0;
					angle_to_turn = 180;
					following_system_armed = 0;

				
				}
	
	
					cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			}
			

		
		if (!no_object && first_object){
			//printf("SEILA\n");
			cmd_vel_bebop.linear.x = 0.05;
			cmd_vel_bebop.linear.y = 0.0;
			cmd_vel_bebop.angular.z = 0.0;
			
			//if (Turns==1){
			//	cmd_vel_bebop.linear.z = 0.01;
			//}
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		}
		
		if(no_object){
			first_object = false;
			cmd_vel_bebop.linear.x = 0.05;
			cmd_vel_bebop.linear.y = 0.0;
			cmd_vel_bebop.angular.z = 0.0;
			
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			//cmd_vel_bebop.linear.x = 0.01;
			//cmd_vel_bebop.linear.y = 0.0;
			//cmd_vel_bebop.angular.z = 0.0;
			
			//if (Turns==1){
			//	cmd_vel_bebop.linear.z = 0.01;
			//}
			//cmd_vel_pub_bebop.publish(cmd_vel_bebop);


		
		}
		//if(dArea < area_distance && !no_object)
		//{
			//cmd_vel_bebop.linear.x = bebop_velocity;
			//cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			//cv::putText(image, "<Following>", cv::Point(image.rows-20,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
		//cv::putText(image, "<Following>", cv::Point(image.rows-20,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
		
		//else if(!(dArea < area_distance) && !no_object){turn();}
		//else if(goo>10 && turns==0 && !no_object){turn();}
	//}
		//else if(!(dArea < area_distance) && no_object){hover();}
	}
	else
	{
		//hover();	
	}





	if(landing_mode)
	{
		
		int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
		//double dblFontScale = 0.75;
		double dblFontScale = 1;
		int intFontThickness = 2;
		//cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		cv::putText(image, "Landing System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);



		if(!no_object && !first_object){
			//cmd_vel_bebop.linear.x = bebop_velocity;
			
			if (posY > drone_center.y+30)
			{	
				cmd_vel_bebop.linear.x = -0.1;		
			}
			if (posY < drone_center.y-30)
			{	
				cmd_vel_bebop.linear.x = 0.1;
			}
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);

			


		

				if (posX > drone_center.x+30)
					{	
				//printf("turning left");
					cmd_vel_bebop.angular.z = -0.3;
					cmd_vel_bebop.linear.y = -0.2;		
				}
				if (posX < drone_center.x-30)
					{	
				//printf("turning right");
					cmd_vel_bebop.angular.z = 0.3;
					cmd_vel_bebop.linear.y = 0.2;
					}


				if ((posX < drone_center.x+10) && (posX > drone_center.x-10) &&(posY < drone_center.y+20) &&(posY > drone_center.y-20) ){
					cmd_vel_bebop.linear.x = 0.0;
					cmd_vel_bebop.linear.y = 0.0;		
					cmd_vel_bebop.linear.z = 0.0;
					cmd_vel_bebop.angular.z = 0.0;
					cmd_vel_pub_bebop.publish(cmd_vel_bebop);
					land_bebop.publish(land);
					
					start_time = time(NULL);

					//ros::Duration(5.0).sleep();
					//takeoff_bebop.publish(take_off);
					landing_mode = 0;
					tracking_system_armed = 0;
					turning_mode= 0;
					//angle_to_turn =180;
					following_system_armed = 0;
				}
				
		
			
			

		}
		if(!no_object && first_object){
			cmd_vel_bebop.linear.x = 0.5;
			cmd_vel_bebop.linear.y = 0.0;
			cmd_vel_bebop.angular.z = 0.0;
			
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		}

		
		if(no_object){
			first_object=false;

			
		
		}

	}
	else
	{
		//hover();	
	}



	if(turning_mode)
	{
		
		int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
		//double dblFontScale = 0.75;
		double dblFontScale = 1;
		int intFontThickness = 2;
		//cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		cv::putText(image, "Turning System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		printf("VIRA PORRA\n");
		//printf("time %f\n",start_time);
		cmd_vel_bebop.linear.x = 0.0;
		cmd_vel_bebop.linear.y = 0.0;
		cmd_vel_bebop.angular.z = 3.0; 
		end_time = time(NULL);
		//printf("time %f\n",end_time - start_time);
		cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		if ((end_time-start_time) >= 0.013*180){
			turning_mode = 0;
			printf("HERE\n");	
			cmd_vel_bebop.linear.x = 0;
			cmd_vel_bebop.linear.y = 0;
			cmd_vel_bebop.linear.z = 0;
			cmd_vel_bebop.angular.z = 0;
			cmd_vel_pub_bebop.publish(cmd_vel_bebop); 
			land_bebop.publish(land);
					
					

			ros::Duration(5.0).sleep();
			takeoff_bebop.publish(take_off);

			ros::Duration(5.0).sleep();
			
			 if (angle_to_turn == 180){
					//angle_to_turn = 40;
					//turning_speed = -3;
			 		landing_mode = 1;
					tracking_system_armed = 0;
					following_system_armed = 0;
					first_object = true;

				}
			
				


			
			
		}
                
		
	}


	if(following_system_armed)
	{
		
		int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
		//double dblFontScale = 0.75;
		double dblFontScale = 1;
		int intFontThickness = 2;
		//cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		cv::putText(image, "Following System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);


		if(!no_object && follow_orange){
			//cmd_vel_bebop.linear.x = bebop_velocity;
			
			cmd_vel_bebop.linear.x = 0.17;
			if (posX > drone_center.x+30)
			{	
				//printf("turning left");
				cmd_vel_bebop.angular.z = -0.1;
				cmd_vel_bebop.linear.y = -0.2;		
			}
			if (posX < drone_center.x-30)
			{	
				//printf("turning right");
				cmd_vel_bebop.angular.z = 0.1;
				cmd_vel_bebop.linear.y = 0.2;
			}
			

			if ((posX < drone_center.x+10) && (posX > drone_center.x-10)){
				//land_bebop.publish(land);
				cmd_vel_bebop.linear.y = 0.0;
				cmd_vel_bebop.linear.z = 0.0;
				cmd_vel_bebop.angular.z = 0.0;
								//tracking_system_armed = 1 - tracking_system_armed;
				//ros::Duration(5.0).sleep();

			}
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);

			
		}

		if(!no_objectRed && !forget_red){
			follow_orange = 0;
			printf("PXred PYred %d %d\n",posXRed,posYRed);
		//cmd_vel_bebop.linear.x = 0.0;
		//cmd_vel_bebop.linear.y = 0.0;		
		//cmd_vel_bebop.linear.z = 0.0;
		//cmd_vel_bebop.angular.z = 0.0;
		//cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		if (posYRed > drone_center.y+20)
			{	
				cmd_vel_bebop.linear.x = -0.01;		
			}
			if (posYRed < drone_center.y-20)
			{	
				cmd_vel_bebop.linear.x = 0.01;
			}
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);

		//cmd_vel_bebop.linear.x = 0.17;
			if (posXRed > drone_center.x+10)
			{	
				//printf("turning left");
				cmd_vel_bebop.angular.z = -0.01;
				cmd_vel_bebop.linear.y = -0.02;		
			}
			if (posXRed < drone_center.x-10)
			{	
				//printf("turning right");
				cmd_vel_bebop.angular.z = 0.01;
				cmd_vel_bebop.linear.y = 0.02;
			}
			


			if ((posX < drone_center.x+10) && (posX > drone_center.x-10) &&(posYRed < drone_center.y+20) &&(posYRed > drone_center.y-20))
			{
				following_system_armed =0;
				turning_mode = 1;
			//	angle_to_turn = 360;
				cmd_vel_bebop.linear.x = 0.0;
				cmd_vel_bebop.linear.y = 0.0;
				cmd_vel_bebop.linear.z = 0.0;
				cmd_vel_bebop.angular.z = 0.0;
				start_time = time(NULL);


			}
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
		}

		if(!no_object && forget_red){
			//forget_red =0;
		}


		
		if(no_object){
			
				printf("noo object\n");
				cmd_vel_bebop.linear.x = 0.0;
				cmd_vel_bebop.linear.y = 0.0;
				cmd_vel_bebop.linear.z = 0.0;
				cmd_vel_bebop.angular.z = 0.0;
				if(angle_to_turn==180){
					angle_to_turn = 90;
					turning_speed = -3;
				}
					
				if (angle_to_turn == 10){
					angle_to_turn = 180;

				}	
				start_time = time(NULL);
				//Turns = 1;
				following_system_armed = 0;
				turning_mode = 1;
		
			
			
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);


		
		}

	}
	else
	{
		//hover();	
	}
	
	
}
void messageCallback(const nav_msgs::Odometry::ConstPtr& msg){
	float orX;
	float orY;
	float orZ;
	float orW;
	
	//Getting the data from Odometry variable and converting it
	odom_x =  msg->pose.pose.position.x;
	odom_y =  msg->pose.pose.position.y;
	odom_z =  msg->pose.pose.position.z;


	orX = msg->pose.pose.orientation.x;
	orY = msg->pose.pose.orientation.y;
	orZ = msg->pose.pose.orientation.z;
	orW = msg->pose.pose.orientation.w;
	

	
	get_heading(orX,orY,orZ,orW);	
	//printf("\n Heading %lf \n", _heading);		//In degrees

	//printf("====POSITION=====\n");
	//printf("x= %lf \ny = %lf \nz= %lf\n",odom_x,odom_y,odom_z);


}

cv::Mat Gauss(cv::Mat input) {
  cv::Mat output;
// Applying Gaussian Filter
  cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
  return output;
}

void colorthresh(cv::Mat input) {
  // Initializaing variables
  
  cv::Size s = input.size();
  std::vector<std::vector<cv::Point> > v;
  auto w = s.width;
  auto h = s.height;
  auto c_x = 0.0;
  // Detect all objects within the HSV range
  cv::cvtColor(input,img_hsv, CV_BGR2HSV);
  LowerYellow = {5,100,178};
 	UpperYellow = {16,200,255};
  cv::inRange(img_hsv, LowerYellow,
   UpperYellow, img_mask);
  img_mask(cv::Rect(0, 0, w, 0.8*h)) = 0;
  // Find contours for better visualization
  cv::findContours(img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // If contours exist add a bounding
  // Choosing contours with maximum area
  if (v.size() != 0) {
  auto area = 0;
  auto idx = 0;
  auto count = 0;
  while (count < v.size()) {
    if (area < v[count].size()) {
       idx = count;
       area = v[count].size();
    }
    count++;
  }
  cv::Rect rect = boundingRect(v[idx]);
  cv::Point pt1, pt2, pt3;
  pt1.x = rect.x;
  pt1.y = rect.y;
  pt2.x = rect.x + rect.width;
  pt2.y = rect.y + rect.height;
  pt3.x = pt1.x+5;
  pt3.y = pt1.y-5;
  // Drawing the rectangle using points obtained
  rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
  // Inserting text box
  cv::putText(input, "Line Detected", pt3,
    CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
  }
  // Mask image to limit the future turns affecting the output
  img_mask(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
  img_mask(cv::Rect(0, 0, 0.3*w, h)) = 0;
  // Perform centroid detection of line
  cv::Moments M = cv::moments(img_mask);
  if (M.m00 > 0) {
    cv::Point p1(M.m10/M.m00, M.m01/M.m00);
    cv::circle(img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
  }
  c_x = M.m10/M.m00;
  // Tolerance to chooise directions
  auto tol = 15;
  auto count = cv::countNonZero(img_mask);
  // Turn left if centroid is to the left of the image center minus tolerance
  // Turn right if centroid is to the right of the image center plus tolerance
  // Go straight if centroid is near image center
  //if (c_x < w/2-tol) {
    //LineDetect::dir = 0;
  //} else if (c_x > w/2+tol) {
    //LineDetect::dir = 2;
  //} else {
    //LineDetect::dir = 1;
 // }
  // Search if no line detected
  //if (count == 0) {
    //LineDetect::dir = 3;
 // }
  // Output images viewed by the turtlebot
  cv::namedWindow("Turtlebot View");
  imshow(OPENCV_ORIGINAL, input);
  return ;
}






void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


	/*
	 cv_bridge::CvImagePtr cv_ptr;
  	try {
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	img = cv_ptr->image;
    	cv::waitKey(30);

    	img_filt = Gauss(img);
         colorthresh(img_filt);
  	}
  	catch (cv_bridge::Exception& e) {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
  	*/
	
	try		
	{
		cv_original = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	//Copying the image and encoding it into BGR according to opencv default settings
		//iHighV = 255;
		//discriminate_color2(cv_original->image);
		//iHighV = 251;
		discriminate_color(cv_original->image);				//Discriminate colors
		cv::Mat final_img = cv_original->image.clone();			//Clone the original image
		c_drone(final_img);
		//imprime_odometry();						//Figures out bebop's center and draws a circle
		move_drone(final_img);					//Moves Drone
		cv::imshow(OPENCV_ORIGINAL,final_img);				//Show the original image
		int key = cv::waitKey(30);					//Contains the key value 
		if(key == 27)							//Press ESC to exit
		{
			exit_from_cv = true; 
			land_bebop.publish(land);
		}				
		if(key == 'k')		
		{	
			first_object = true;
			tracking_system_armed = 1 - tracking_system_armed;
			//landing_mode = 1 - landing_mode;
			if(tracking_system_armed)
			{
				ROS_INFO("TRACKING SYSTEM ARMED!!!");
			}
			else
			{
				ROS_INFO("TRACKING SYSTEM DISARMED!!!");
			}
		}
		if(key == ' ')
		{
			ROS_INFO("TAKE-OFF!!!");
			takeoff_bebop.publish(take_off);
		}			
		if(key == 'b')
		{
			ROS_INFO("LAND!!!");
			land_bebop.publish(land);
		}
		if (key == 'c'){
			ROS_INFO("TESTING!!!");
			camera_angle = 0;
			cam_bebop.angular.y = 0;
			pub_cam_tilt.publish(cam_bebop);
		}
		if (key == 'z'){
			ROS_INFO("TESTING!!!");

			cam_bebop.angular.y = -80;
			pub_cam_tilt.publish(cam_bebop);
		}

		if (key == 'w'){
			cmd_vel_bebop.linear.x = 0.1;
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);


		}
		
		if (key == 'e'){
			cmd_vel_bebop.linear.x = 0.0;
			cmd_vel_bebop.linear.y = 0.0;
			cmd_vel_bebop.linear.z = 0.0;
			cmd_vel_bebop.angular.z = 0.0;
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);

			tracking_system_armed = 0;
			turning_mode= 0 ;
			following_system_armed = 0;


		}
		


		


			
		




		
	
		
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());			//Handleling the Exception
		return;
	}			
}
int main(int argc, char** argv)
{	
	pid_alt.gpro = 0.15;				//Proportional Constant
	pid_alt.ginteg = 0.0;				//Integral Constant
	pid_alt.gder = 0.0001;
	

	pid_odom.gpro = 3.5;				//Proportional Constant
	pid_odom.ginteg = 0.0;				//Integral Constant
	pid_odom.gder = 0.0001;



	pid_hdg.gpro = 0.15;				//Proportional Constant
	pid_hdg.ginteg = 0.0;				//Integral Constant
	pid_hdg.gder = 0.00002;				//Derivative Constant



	ros::init(argc,argv,"bebop_color_follower");				//Initialize the ROS node
	ros::NodeHandle nh_;
	ros::NodeHandle n;
				//Receives the data from the sensors onboard							//Create the Node Handler
	image_transport::ImageTransport it_(nh_);				//Special message to contain the image
	image_transport::Subscriber image_sub_;					//Special subscriber to obtain the image
	//image_sub_= it_.subscribe("/usb_cam/image_raw",1,imageCallback);	//Subscribe to the Bebop image topic
	correct_alt = nh_.advertise<std_msgs::Float64>("/ardrone/output_pid_alt",1000);
	correct_hdg = nh_.advertise<std_msgs::Float64>("/ardrone/output_pid_hdg",1000);

	image_sub_= it_.subscribe("/ardrone/bottom/image_raw",1,imageCallback);
	takeoff_bebop = nh_.advertise<std_msgs::Empty>("/bebop/takeoff",1000);		//Publish data to the take-off topic
	land_bebop = nh_.advertise<std_msgs::Empty>("/bebop/land",1000);		//Publish data to the land topic
	cmd_vel_pub_bebop = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);	//Publish data to the movement topic
	bebopOdom = nh_.subscribe("/bebop/odom",1000,messageCallback);
	bebopAlt = nh_.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged",1000,altCallback);
	pub_cam_tilt = nh_.advertise<geometry_msgs::Twist>("/bebop/camera_control",1000);
	

	cv::namedWindow(OPENCV_ORIGINAL);					//Create window to visualize the original image
	cv::setMouseCallback(OPENCV_ORIGINAL, CallBackFunc, NULL);		//Receive info from the mouse
	//TODO Resolve Segmentation Fault issue due to publishers 
	while(nh_.ok())								//Asks if the node still alive
	{	

		
		

		


		ros::spinOnce();						//Refresh ROS's topics once
		if(exit_from_cv){break;}					//Exit using while focus any opencv window (ESC key)	
	}
	ROS_INFO("EXITING...");
	cv::destroyWindow(OPENCV_ORIGINAL);					//Destroy Original Window
	cv::destroyWindow(OPENCV_BINARY);					//Destroy Binary Window
	return 0;
}
