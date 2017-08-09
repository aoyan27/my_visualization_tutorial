#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <math.h>

using namespace std;

nav_msgs::Odometry dgauss_odom;
nav_msgs::Odometry ndt_odom;
nav_msgs::Odometry ekf_odom;
nav_msgs::Odometry tiny_odom;
nav_msgs::Odometry amu_odom;

float init_theta = 94.2971 / 180.0 * M_PI;

bool dgauss_sub_flag = false;
bool ndt_sub_flag = false;
bool ekf_sub_flag = false;
bool tiny_sub_flag = false;
bool amu_sub_flag = false;

int dgauss_count = 0;
int ndt_count = 0;
int ekf_count = 0;
int tiny_count = 0;
int amu_count = 0;

void dgauss_callback(nav_msgs::Odometry msg){
	dgauss_odom = msg;
	dgauss_count++;
	dgauss_sub_flag = true;
}

void ndt_callback(nav_msgs::Odometry msg){
	ndt_odom = msg;
	ndt_count++;
	ndt_sub_flag = true;
}

void ekf_callback(nav_msgs::Odometry msg){
	ekf_odom = msg;
	ekf_count++;
	ekf_sub_flag = true;
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
	ros::init(argc, argv, "example_trajectry");
	ros::NodeHandle n;

	ros::Subscriber sub_dgauss = n.subscribe("/lcl", 10, dgauss_callback);
	ros::Subscriber sub_ndt = n.subscribe("/lcl4", 10, ndt_callback);
	ros::Subscriber sub_ekf = n.subscribe("/ekf_DgaussAndNDT", 10, ekf_callback);
	ros::Subscriber sub_amu = n.subscribe("/amu_lcl", 10, amu_callback);
	ros::Subscriber sub_tiny = n.subscribe("/tiny_lcl", 10, tiny_callback);

	ros::Publisher pub_line_dgauss = n.advertise<visualization_msgs::Marker>("/dgauss_trajectory", 1);
	ros::Publisher pub_line_ndt = n.advertise<visualization_msgs::Marker>("/ndt_trajectory", 1);
	ros::Publisher pub_line_ekf = n.advertise<visualization_msgs::Marker>("/ekf_trajectory", 1);
	ros::Publisher pub_line_tiny = n.advertise<visualization_msgs::Marker>("/tiny_trajectory", 1);
	ros::Publisher pub_line_amu = n.advertise<visualization_msgs::Marker>("/amu_trajectory", 1);
	
	visualization_msgs::Marker line_dgauss, line_ndt, line_ekf, line_tiny, line_amu;

	line_dgauss.header.frame_id = line_ndt.header.frame_id = line_ekf.header.frame_id = line_tiny.header.frame_id = line_amu.header.frame_id =  "/map";

	line_dgauss.ns = line_ndt.ns = line_ekf.ns = line_tiny.ns = line_amu.ns = "trajectory";
	line_dgauss.action = line_ndt.action = line_ekf.action = line_tiny.action = line_amu.action = visualization_msgs::Marker::ADD;

	line_dgauss.id = 0;
	line_ndt.id = 1;
	line_ekf.id = 2;
	line_tiny.id = 3;
	line_amu.id = 4;

	line_dgauss.type = visualization_msgs::Marker::LINE_STRIP;
	line_ndt.type = visualization_msgs::Marker::LINE_STRIP;
	line_ekf.type = visualization_msgs::Marker::LINE_STRIP;
	line_tiny.type = visualization_msgs::Marker::LINE_STRIP;
	line_amu.type = visualization_msgs::Marker::LINE_STRIP;

	line_dgauss.scale.x = 0.5;
	line_ndt.scale.x = 0.5;
	line_ekf.scale.x = 0.5;
	line_tiny.scale.x = 0.5;
	line_amu.scale.x = 0.5;

	line_dgauss.color.r = 1.0f;
	line_dgauss.color.a = 1.0;
	
	line_ndt.color.r = 1.0f;
	line_ndt.color.g = 0.4f;
	line_ndt.color.b = 0.2f;
	line_ndt.color.a = 1.0;

	line_ekf.color.r = 1.0f;
	line_ekf.color.g = 1.0f;
	line_ekf.color.a = 1.0;

	line_tiny.color.g = 1.0f;
	line_tiny.color.b = 1.0f;
	line_tiny.color.a = 1.0;
	
	line_amu.color.b = 1.0f;
	line_amu.color.r = 1.0f;
	line_amu.color.a = 1.0;

	ros::Rate loop_rate(20);

	while(ros::ok()){
		// cout<<"dgauss_odom : "<<dgauss_odom<<endl;
		// cout<<"tiny_odom : "<<tiny_odom<<endl;

		line_dgauss.header.stamp = line_ndt.header.stamp = line_ekf.header.stamp = line_tiny.header.stamp = line_amu.header.stamp = ros::Time::now();

		
		if(dgauss_sub_flag){
			geometry_msgs::Point dgauss_point;

			dgauss_point.x = dgauss_odom.pose.pose.position.x;
			dgauss_point.y = dgauss_odom.pose.pose.position.y;
			dgauss_point.z = dgauss_odom.pose.pose.position.z;

			cout<<"dgauss_point.x : "<<dgauss_point.x<<endl;
			cout<<"dgauss_point.y : "<<dgauss_point.y<<endl;
			cout<<"dgauss_point.z : "<<dgauss_point.z<<endl;
			
			if(dgauss_count % 1 ==0){
				line_dgauss.points.push_back(dgauss_point);
			}
			pub_line_dgauss.publish(line_dgauss);
			dgauss_sub_flag = false;
		}

		if(ndt_sub_flag){
			geometry_msgs::Point ndt_point;

			ndt_point.x = ndt_odom.pose.pose.position.x;
			ndt_point.y = ndt_odom.pose.pose.position.y;
			ndt_point.z = ndt_odom.pose.pose.position.z;

			cout<<"ndt_point.x : "<<ndt_point.x<<endl;
			cout<<"ndt_point.y : "<<ndt_point.y<<endl;
			cout<<"ndt_point.z : "<<ndt_point.z<<endl;
			
			if(ndt_count % 1 ==0){
				line_ndt.points.push_back(ndt_point);
			}
			pub_line_ndt.publish(line_ndt);
			ndt_sub_flag = false;
		}

		if(ekf_sub_flag){
			geometry_msgs::Point ekf_point;

			ekf_point.x = ekf_odom.pose.pose.position.x;
			ekf_point.y = ekf_odom.pose.pose.position.y;
			ekf_point.z = ekf_odom.pose.pose.position.z;

			cout<<"ekf_point.x : "<<ekf_point.x<<endl;
			cout<<"ekf_point.y : "<<ekf_point.y<<endl;
			cout<<"ekf_point.z : "<<ekf_point.z<<endl;
			
			if(ekf_count % 1 ==0){
				line_ekf.points.push_back(ekf_point);
			}
			pub_line_ekf.publish(line_ekf);
			ekf_sub_flag = false;
		}

		if(tiny_sub_flag){
			geometry_msgs::Point tiny_point;

			tiny_point.x = tiny_odom.pose.pose.position.x;
			tiny_point.y = tiny_odom.pose.pose.position.y;
			tiny_point.z = tiny_odom.pose.pose.position.z;

			cout<<"tiny_point.x : "<<tiny_point.x<<endl;
			cout<<"tiny_point.y : "<<tiny_point.y<<endl;
			cout<<"tiny_point.z : "<<tiny_point.z<<endl;
			
			if(tiny_count % 1 ==0){
				line_tiny.points.push_back(tiny_point);
			}
			pub_line_tiny.publish(line_tiny);
			tiny_sub_flag = false;
		}

		if(amu_sub_flag){
			geometry_msgs::Point amu_point;

			amu_point.x = amu_odom.pose.pose.position.x;
			amu_point.y = amu_odom.pose.pose.position.y;
			amu_point.z = amu_odom.pose.pose.position.z;

			cout<<"amu_point.x : "<<amu_point.x<<endl;
			cout<<"amu_point.y : "<<amu_point.y<<endl;
			cout<<"amu_point.z : "<<amu_point.z<<endl;
			
			if(amu_count % 1 ==0){
				line_amu.points.push_back(amu_point);
			}
			pub_line_amu.publish(line_amu);
			amu_sub_flag = false;
		}
		// pub_line_tiny.publish(line_tiny);
		// pub_line_amu.publish(line_amu);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
