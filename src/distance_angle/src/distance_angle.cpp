#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "ros/ros.h"
#include "distance_angle/DistanceAngle.h"
#include <math.h> 
#include <string> 
#define PI 3.14159265
#define baseline 0.12
#define flags 1
ros::Publisher DistanceAngle_pub;
distance_angle::DistanceAngle marker;




class MarkerParameters
{

public:
    float distance;
    float angle;
    float Yaw;
    float Pitch;
    float Roll;
    
    void Computedistangle(ar_track_alvar_msgs::AlvarMarkers ar_marker, int i) {
        
        float x,y;
        
        x = ar_marker.markers[i].pose.pose.position.x;
        
        y = ar_marker.markers[i].pose.pose.position.y;

        distance = sqrt(x*x + y*y);

        angle = atan(y/x)*180/PI;
        
    }
    
    void ComputeRPY(ar_track_alvar_msgs::AlvarMarkers ar_marker, int i) {  /* from quaternion to euler angles */
            
            float q0,q1,q2,q3;
            
            q0 = ar_marker.markers[i].pose.pose.orientation.x;
        
            q1 = ar_marker.markers[i].pose.pose.orientation.y;

            q2 = ar_marker.markers[i].pose.pose.orientation.z;

            q3 = ar_marker.markers[i].pose.pose.orientation.w;
            
            Yaw = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2))*180/PI;

            Pitch = asin(2*(q0*q2-q3*q1))*180/PI;
            
            Roll = atan2(2*(q0*q3 + q2*q1),1-2*(q0*q0+q1*q1))*180/PI;
            
    }

     void CopytoMarker(distance_angle::DistanceAngle &marker){

	    marker.distance = distance;
	    marker.angle = angle;
	    marker.orientation = Yaw + 90; //for having 0 degree into the perpendicular line
    }
};


void distanceangleCallback(ar_track_alvar_msgs::AlvarMarkers ar_pose_marker)
{
  
  int i;

  for (i=0;i<flags;++i)
    { 
      if ( std::to_string(ar_pose_marker.markers[i].id) == "29")
{
  ROS_INFO("29");
        MarkerParameters ARmarker;
        
        ARmarker.Computedistangle(ar_pose_marker,i);
        
        ARmarker.ComputeRPY(ar_pose_marker,i); 
  
	ARmarker.CopytoMarker(marker);

        ROS_INFO("Distance: [%f], angle: [%f], orientation: [%f]", marker.distance, marker.angle, marker.orientation);

        DistanceAngle_pub.publish(marker);
}
    }
}

int main(int argc, char **argv)
{ 
 
  ros::init(argc, argv, "distance_angle");
  try
  {
    ros::NodeHandle n;

  DistanceAngle_pub = n.advertise<distance_angle::DistanceAngle>("DistanceAngle", 10);

  ros::Subscriber sub = n.subscribe("ar_pose_marker", 15, distanceangleCallback);
  ros::spin();
  }
  catch(const std::exception& e)
  {
    std::cerr << "hataaaaaaa" << e.what() << '\n';
  }
  
  
  return 0;
}





// #include <ros/ros.h>
// #include "std_msgs/Float64.h"
// #include "tf2_msgs/TFMessage.h"
// #include "distance_angle/DistanceAngle.h"
// #define flags 1
// #define PI 3.14


// ros::Publisher OdomAngle_pub;
// ros::Publisher DistanceAngle_pub;
// distance_angle::DistanceAngle marker;
// //float chassis {0.05};

// class MarkerParameters
// {

// public:
//     float distance;
//     float angle;
//     float Yaw_odom;
//     float Pitch_odom;
//     float Roll_odom;
//     float Yaw;
//     float Pitch;
//     float Roll;
    
//     void ComputeOdomAngle(const tf2_msgs::TFMessage& robot, int i){

// 	float q0,q1,q2,q3;
   
//     	q0 = robot.transforms[0].transform.rotation.x;

//     	q1 = robot.transforms[0].transform.rotation.y;

//         q2 = robot.transforms[0].transform.rotation.z;

//         q3 = robot.transforms[0].transform.rotation.w;
    
//   	Yaw_odom = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2))*180/PI;

//         Pitch_odom = asin(2*(q0*q2-q3*q1))*180/PI;
             
//         Roll_odom = atan2(2*(q0*q3 + q2*q1),1-2*(q0*q0+q1*q1))*180/PI;

// }

//     void Computedistangle(const tf2_msgs::TFMessage& ar_marker, int i) {
        
//         float x,y,xp,yp,Yaw_odom_rad;

// 	Yaw_odom_rad = Yaw_odom*PI/180;
        
//         xp = ar_marker.transforms[i].transform.translation.z;
        
//         yp = ar_marker.transforms[i].transform.translation.x;

//         x = cos(Yaw_odom_rad)*xp + sin(Yaw_odom_rad)*yp; //+ chassis*(1-cos(Yaw_odom_rad));

// 	y = -sin(Yaw_odom_rad)*xp + cos(Yaw_odom_rad)*yp; // - chassis*sin(Yaw_odom_rad); //check minus

// 	ROS_INFO("x: [%f], y: [%f], Y: [%f]", x, y, sin(Yaw_odom_rad));

//         distance = sqrt(x*x + y*y);

//         angle = atan(y/x)*180/PI; 
        
//     }
    
//     void ComputeRPY(const tf2_msgs::TFMessage& ar_marker, int i) {  /* from quaternion to euler angles */
            
//         float q0,q1,q2,q3;
            
//         q0 = ar_marker.transforms[i].transform.rotation.x;
        
//         q1 = ar_marker.transforms[i].transform.rotation.y;

//         q2 = ar_marker.transforms[i].transform.rotation.z;

//         q3 = ar_marker.transforms[i].transform.rotation.w;
            
//         Yaw = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2))*180/PI;

//         Pitch = asin(2*(q0*q2-q3*q1))*180/PI;
            
//         Roll = atan2(2*(q0*q3 + q2*q1),1-2*(q0*q0+q1*q1))*180/PI;
            
//     }

//      void CopytoMarker(distance_angle::DistanceAngle &marker){

// 	    marker.distance = distance;
// 	    marker.angle = angle;
// 	    marker.orientation = Yaw + Yaw_odom; //for having 0 degree into the perpendicular line
//     }

// };

// MarkerParameters ARmarker;

// void odomangleCallback(const tf2_msgs::TFMessage robot)

// {

//      std_msgs::Float64 odomangle;

//      int i;

//   for (i=0;i<flags;++i)
//     {
   

//   if ((robot.transforms[0].header.frame_id == "odom")&&(robot.transforms[0].child_frame_id == "base_footprint"))

//   {
        
//         ARmarker.ComputeOdomAngle(robot,i);
//         ROS_INFO("A");

//   }


//   // if ((robot.transforms[0].header.frame_id == "camera_link")&&(robot.transforms[0].child_frame_id == "ar_marker_2")) //I should define the frames above!
//   if ((robot.transforms[0].header.frame_id == "camera_color_optical_frame")&&(robot.transforms[0].child_frame_id == "ar_marker_29")) //I should define the frames above!
//  {

//         ARmarker.Computedistangle(robot,i);
        
//         ARmarker.ComputeRPY(robot,i); 
  
// 	ARmarker.CopytoMarker(marker);
//   ROS_INFO("B");
//       ROS_INFO("Distance: [%f], angle: [%f], orientation: [%f]", marker.distance, marker.angle, marker.orientation);

//     DistanceAngle_pub.publish(marker);
    
//     odomangle.data = ARmarker.Yaw_odom;

//     OdomAngle_pub.publish(odomangle);   
//  }


  
//   }

//  }



// int main(int argc, char **argv)
// { 
 
//   ros::init(argc, argv, "distance_angle");

//   ros::NodeHandle n;

//   ros::Subscriber sub = n.subscribe("/tf", 1,odomangleCallback);

//   OdomAngle_pub = n.advertise<std_msgs::Float64>("odomangle", 10);
 
//   DistanceAngle_pub = n.advertise<distance_angle::DistanceAngle>("DistanceAngle", 10);

//   ros::spin();
  
//   return 0;
// }


