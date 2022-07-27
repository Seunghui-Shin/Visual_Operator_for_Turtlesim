#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <sys/stat.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_publisher");
    ros::NodeHandle n;

    //ros::Publisher topic_exam_pub = n.advertise<std_msgs::String>("topic_exam_message", 1000);
    ros::Publisher vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(100);

    while(ros::ok())
    {	
    VideoCapture capture(0,CAP_V4L2);
    if (!capture.isOpened()){
        //error in opening the video input
        cerr << "Unable to open file!" << endl;
        return 0;
    }
    Mat frame1, prvs;
    capture >> frame1;
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);
    while(true){
        Mat frame2, next;
        capture >> frame2;
        if (frame2.empty())
            break;
        cvtColor(frame2, next, COLOR_BGR2GRAY);
        Mat flow(prvs.size(), CV_32FC2);
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        // visualization
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
	Mat oang = angle * 3.141592 / 180.0;
        angle *= ((1.f / 360.f) * (180.f / 255.f));

        //build hsv image
        Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cvtColor(hsv8, bgr, COLOR_HSV2BGR);
        //imshow("frame2", bgr);

		// representation using vectors
		int step = 10;
		Mat img_vec = frame2;
		std_msgs::String msg;
		std::stringstream ss;
		string move;
		int x1=0, x2=0, y1=0, y2=0;
		for (int r=0; r<angle.rows; r+=step) {
			for (int c=0; c<angle.cols; c+=step){
				float ang = oang.at<float>(r,c);
				float m = magn_norm.at<float>(r,c) * 20.0;
				Point pt1 = cv::Point(c, r);
				Point pt2 = cv::Point(c + m * cos(ang) , r + m * sin(ang));
				line(img_vec, pt1, pt2, Scalar(0, 255, 0), 1, 8, 0);
				x1 += pt1.x;
				x2 += pt2.x;
				y1 += pt1.y;
				y2 += pt2.y;
			}
		}
		
		int x = abs(x2-x1);
		int y = abs(y2-y1);
		double linear_ = 0, angular_ = 0, l_scale_ = 0.3, a_scale_ = 0.5;
		
		if (x>y){
			if(x2>x1){
				move = "RIGHT";
				angular_ = -1.0;
			}
			else{
				move = "LEFT";
				angular_ = 1.0;
			}
		}
		else{
			if(y2>y1){
				move = "DOWN";
				linear_ = -1.0;
			}
			else{
				move = "UP";
				linear_ = 1.0;
			}
		}
		
        imshow("mission #3", img_vec);
	ss << move;
   	msg.data = ss.str();
    	ROS_INFO("%s", msg.data.c_str());
    	//topic_exam_pub.publish(msg);
    	geometry_msgs::Twist vel;
    	vel.angular.z = a_scale_*angular_;
    	vel.linear.x = l_scale_*linear_;
    	vel_pub_.publish(vel); 
    	ros::spinOnce();
    	loop_rate.sleep();

        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
        prvs = next;
    }	

	return 0;	
   }
}


