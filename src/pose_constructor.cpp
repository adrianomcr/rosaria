#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "gps_utm.cpp"
#include <visualization_msgs/Marker.h>


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>

#include <ros/package.h>


#define GPS_TFACTOR 220


double pos[3], quat[4], yaw;

double decawave_position[3];

int count_gps, count_imu, count_deca_gps;
double cx_gps, cy_gps, center_yaw;

double deca_cx_gps, deca_cy_gps, deca_center_yaw;


void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps)
{

     // Transform to UTM reference system
     double northing, easting;
     char zone;
     LLtoUTM(gps->latitude, gps->longitude,  northing, easting , &zone);



     if (count_gps < GPS_TFACTOR){
       count_gps++;
       cx_gps += easting/(float)GPS_TFACTOR;
       cy_gps += northing/(float)GPS_TFACTOR;
       printf("count:%d\n", count_gps);
     }
     else{
       double alpha = 0.08;                                   //Beguinning of the trajectory (Dijkstra)
       pos[0] = (1-alpha)*pos[0] + alpha*(easting-cx_gps       -7.823163*0.0); //(-8.9399 * 0)
       pos[1] = (1-alpha)*pos[1] + alpha*(northing-cy_gps      +2.010794*0.0); //(+3.0343 * 0)
       //pos[0] = (easting-cx_gps); //(-8.9399 * 0)
       //pos[1] = (northing-cy_gps); //(+3.0343 * 0)
       pos[2] = 0.0;
     }



     //cout << easting-cx_gps << "\t\t"<< northing-cy_gps << endl;
//-7.823163	2.010794
}


void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{

    quat[0] = imu->orientation.x;
    quat[1] = imu->orientation.y;
    quat[2] = imu->orientation.z;
    quat[3] = imu->orientation.w;

    double r, p, y;
    tf::Matrix3x3 rotMatrix(tf::Quaternion(quat[0],quat[1],quat[2],quat[3]));

    // Get roll, pitch and yaw
    rotMatrix.getRPY(r, p, y);
    yaw = y;


    //I think that it is not necessary. But it can be done to ensure that yaw=0 means robot pointing East
    if (count_imu < GPS_TFACTOR){
      count_imu++;
      center_yaw += yaw/(float)GPS_TFACTOR;
    }
    else{
      yaw = y - center_yaw;
    }


}


// Main
int main(int argc, char **argv) {

  ros::init(argc, argv, "pose_constructor");
  ros::NodeHandle nh;

  double freq = 20.0;


  ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, gps_callback);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, imu_callback);
  // ros::Subscriber decawave_sub = nh.subscribe<nav_msgs::Odometry>("/decawave/pose", 1, decawave_callback);

  //ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/xsens/pose", 1);
  ros::Publisher pose_pub = nh.advertise<nav_msgs::Odometry>("/xsens/pose", 1);
  // ros::Publisher decawave_pub = nh.advertise<nav_msgs::Odometry>("/decawave/pose_gps", 1);
  ros::Rate loop_rate(freq);


  int i = 0;
  count_gps = 0;
  count_imu = 0;
  cx_gps = 0; cy_gps = 0; center_yaw = 0;

  //geometry_msgs::Pose espeleo_pose;

  nav_msgs::Odometry espeleo_pose;

  nav_msgs::Odometry decawave_pose;


  while (ros::ok())
  {

    // Read the callbacks
    ros::spinOnce();

    // //update decawave position
    // decawave_pose.pose.pose.position.x = decawave_position[0];
    // decawave_pose.pose.pose.position.x = decawave_position[1];
    // decawave_pose.pose.pose.position.x = decawave_position[2];



    // Uptade the rost variable to be publishe
    espeleo_pose.pose.pose.position.x = pos[0];
    espeleo_pose.pose.pose.position.y = pos[1];
    espeleo_pose.pose.pose.position.z = pos[2];
    espeleo_pose.pose.pose.orientation.x = quat[0]*0 + yaw;
    espeleo_pose.pose.pose.orientation.y = quat[1]*0 + yaw;
    espeleo_pose.pose.pose.orientation.z = quat[2]*0 + yaw;
    espeleo_pose.pose.pose.orientation.w = quat[3]*0 + yaw;

    // Publish rateThrust command
    pose_pub.publish(espeleo_pose);

    // decawave_pub.publish(decawave_pose);


    // Sleep program
    loop_rate.sleep();
  }


}
