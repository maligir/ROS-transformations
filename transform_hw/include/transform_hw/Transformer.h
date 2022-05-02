#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>


/*
See this page on ROS.org for some help on the first part of the assignment!
http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
*/

class Transformer {
protected:
    /*
        You will need a ros::subscriber here, and a tf2_ros::TransformBroadcaster.
        They should be scoped to the class.

        You will need to add the relevant includes above.
        
    */
   ros::Subscriber sub;
   tf2_ros::TransformBroadcaster br;

public:
    /*
        Your constructor should take a ros::NodeHandle REFERENCE (or POINTER) as a parameter
        If it is not passed as a reference or pointer, your program will not work.
    */
    Transformer(ros::NodeHandle &handle);
    ~Transformer();

    //You will need to make a function that is a callback which subscribes to geometry_msgs::PoseStamped
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &ps);

};

#endif