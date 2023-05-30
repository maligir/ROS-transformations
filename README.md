# ROS Package: transform_hw

This ROS (Robot Operating System) package is designed to handle transformations in a robot system. The package uses ROS, tf2, and Eigen to manage transformations between frames of reference. The package listens to a geometry_msgs::PoseStamped message on the "/azure_kinect/tag_pose" topic, and then broadcasts the transform and additional transformed frames as specified in the assignment.

## Prerequisites

- ROS (Tested with ROS Noetic)
- Eigen
- tf2_ros

## Files

- `transformer.cpp`: This is the main code file where the class `Transformer` is defined. This class is responsible for listening to the pose data and handling the transformations.
  
- `transformerHW.cpp`: This is the file where the ROS node is initialized, and an instance of the Transformer class is created. The ROS spin loop is also defined in this file.
  
- `Transformer.h`: This file contains the declaration of the `Transformer` class and its methods.

## Functionality

The `Transformer` class defines a ROS node that:

1. Subscribes to "/azure_kinect/tag_pose" topic expecting PoseStamped messages.

2. Once a PoseStamped message is received, it extracts the data and logs it.

3. It then creates a TransformStamped message that represents the same data, and broadcasts this transform.

4. Additionally, it calculates and broadcasts two more transforms: 
    - One that represents a position 1 meter in front of the received pose along the z-axis.
    - Another that represents a position 1 meter in front of the received pose along the z-axis but is also rotated 180 degrees about the y-axis.

## Usage

To use this package, follow these steps:

1. Clone the repository in your catkin workspace's src folder.

2. Run `catkin_make` from the root of your catkin workspace.

3. Source the setup.bash file in your workspace's devel folder: `source devel/setup.bash`.

4. Launch the node with `rosrun transform_hw transformerHW`.

Ensure you have the required PoseStamped messages being published on the "/azure_kinect/tag_pose" topic for the node to function correctly.

**Note**: All transform information are printed using ROS_INFO_STREAM. You can view this information by running `rosrun rqt_console rqt_console` to start the rqt_console node.

## Dependencies

This package depends on the following ROS packages:

- `roscpp`
- `geometry_msgs`
- `tf2_ros`

These dependencies need to be specified in the `package.xml` file of your ROS package.
