#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <math.h>

using namespace std;

nav_msgs::Odometry lcl_odom;
nav_msgs::Odometry tiny_odom;
nav_msgs::Odometry amu_odom;

float init_theta = 94.2971 / 180.0 * M_PI;

bool lcl_sub_flag = false;
bool tiny_sub_flag = false;
bool amu_sub_flag = false;

int lcl_count = 0;
int tiny_count = 0;
int amu_count = 0;

void lcl_callback(nav_msgs::Odometry msg){
	lcl_odom = msg;
	lcl_count++;
	lcl_sub_flag = true;
}

void tiny_callback(nav_msgs::Odometry msg){
	tiny_odom = msg;
	tiny_count++;
	tiny_sub_flag = true;
}

void amu_callback(nav_msgs::Odometry msg){
	amu_odom = msg;
	amu_count++;
	amu_sub_flag = true;
}

float rotation_x(float x, float y, float theta){
	return x * cos(theta) - y * sin(theta);
}

float rotation_y(float x, float y, float theta){
	return x * sin(theta) + y * cos(theta);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "example_trajectry_tiny");
	ros::NodeHandle n;

	ros::Subscriber sub_lcl = n.subscribe("/lcl", 10, lcl_callback);
	ros::Subscriber sub_tiny = n.subscribe("/tiny_lcl", 10, tiny_callback);
	ros::Subscriber sub_amu = n.subscribe("/amu_lcl", 10, amu_callback);

	ros::Publisher pub_line_lcl = n.advertise<visualization_msgs::Marker>("/lcl_trajectory", 1);
	ros::Publisher pub_line_tiny = n.advertise<visualization_msgs::Marker>("/tiny_trajectory", 1);
	ros::Publisher pub_line_amu = n.advertise<visualization_msgs::Marker>("/amu_trajectory", 1);
	
	visualization_msgs::Marker line_lcl, line_tiny, line_amu;

	ros::Rate loop_rate(20);

	while(ros::ok()){
		// cout<<"lcl_odom : "<<lcl_odom<<endl;
		// cout<<"tiny_odom : "<<tiny_odom<<endl;

		line_lcl.header.frame_id = line_tiny.header.frame_id = line_amu.header.frame_id =  "/map";
		line_lcl.header.stamp = line_tiny.header.stamp = line_amu.header.stamp = ros::Time::now();

		line_lcl.ns = line_tiny.ns = line_amu.ns = "trajectory";
		line_lcl.action = line_tiny.action = line_amu.action = visualization_msgs::Marker::ADD;

		line_lcl.id = 0;
		line_tiny.id = 1;
		line_amu.id = 2;

		line_lcl.type = visualization_msgs::Marker::LINE_STRIP;
		line_tiny.type = visualization_msgs::Marker::LINE_STRIP;
		line_amu.type = visualization_msgs::Marker::LINE_STRIP;

		line_lcl.scale.x = 0.5;
		line_tiny.scale.x = 0.5;
		line_amu.scale.x = 0.5;

		line_lcl.color.r = 1.0f;
		line_lcl.color.a = 1.0;
		
		line_tiny.color.g = 1.0f;
		line_tiny.color.b = 1.0f;
		line_tiny.color.a = 1.0;
		
		line_amu.color.r = 1.0f;
		line_amu.color.g = 1.0f;
		line_amu.color.a = 1.0;
		
		if(tiny_sub_flag){
			geometry_msgs::Point tiny_point;

			tiny_point.x = tiny_odom.pose.pose.position.x;
			tiny_point.y = tiny_odom.pose.pose.position.y;
			tiny_point.z = tiny_odom.pose.pose.position.z;
			
			cout<<"tiny_point.x : "<<tiny_point.x<<endl;
			cout<<"tiny_point.y : "<<tiny_point.y<<endl;
			cout<<"tiny_point.z : "<<tiny_point.z<<endl;
			
			if(tiny_count % 1 == 0){
				line_tiny.points.push_back(tiny_point);
			}
			pub_line_tiny.publish(line_tiny);
			tiny_sub_flag = false;
		}
		

		// pub_line_lcl.publish(line_lcl);
		// pub_line_amu.publish(line_amu);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
