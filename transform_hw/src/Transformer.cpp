#include <transform_hw/Transformer.h>
#include <iostream>
#include <cstdlib>

using namespace std;
Transformer::Transformer(ros::NodeHandle &handle) {
    /*
        Your program will need to subscribe to "/azure_kinect/tag_pose"
        You should use a C++ style callback, the one in this class
        You should subscribe in your constructor, because your program needs
            to subscribe to the topic ONLY ONE TIME
    */
    sub = handle.subscribe("/azure_kinect/tag_pose", 100, &Transformer::poseCallback, this);
}

Transformer::~Transformer() {}

//ask TA about callback registration

//This method DEFINITELY takes a parameter. Can you figure out what it is?
void Transformer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &ps) {
    /*
        The incoming message will be a geometry_msgs::PoseStamped.
        It will contain all of the following information EXCEPT CHILD FRAME.

        PROBLEM 1, as in the PDF:
        Get the correct information out of the incoming message.
        Print it using ROS_INFO_STREAM.

        Hint: You can look up where in the message the data are on this page!
        http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html

        The child frame will be april_tag

        Your output should look like this, with the correct information filled
            in on each line.

        "PARENT FRAME:   " << ... << endl <<
        "CHILD FRAME:    " << ... << endl <<
        "Translation X:  " << ... << endl <<
        "Translation Y:  " << ... << endl <<
        "Translation Z:  " << ... << endl <<
        "Rotation X:     " << ... << endl <<
        "Rotation Y:     " << ... << endl <<
        "Rotation Z:     " << ... << endl <<
        "Rotation W:     " << ... << endl

    */

    // add to ros info stream
    ROS_INFO_STREAM("PARENT FRAME:   " << ps->header.frame_id << endl);
    ROS_INFO_STREAM("CHILD FRAME:    " << "april_tag" << endl);
    ROS_INFO_STREAM("Translation X:  " << ps->pose.position.x << endl);
    ROS_INFO_STREAM("Translation Y:  " << ps->pose.position.y << endl);
    ROS_INFO_STREAM("Translation Z:  " << ps->pose.position.z << endl);
    ROS_INFO_STREAM("Rotation X:     " << ps->pose.orientation.x << endl);
    ROS_INFO_STREAM("Rotation Y:     " << ps->pose.orientation.y << endl);
    ROS_INFO_STREAM("Rotation Z:     " << ps->pose.orientation.z << endl);
    ROS_INFO_STREAM("Rotation W:     " << ps->pose.orientation.w << endl);

    /*
        PROBLEM 2, as in the PDF:

        Take the contents of the PoseStamped message received in this callback.
        Use them to fill in a geometry_msgs::TransformStamped, and send it
            the information as a transform using TF2.
        The tutorial about using TF2 to send transforms is in the header file
            for this class.
        Call this transform's frame "april_tag".
        Its parent frame should be "azure_kinect/camera_body".
    */

    // broadcast current time state
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = ps->header.frame_id;
    ts.child_frame_id = "april_tag";
    ts.transform.translation.x = ps->pose.position.x;
    ts.transform.translation.y = ps->pose.position.y;
    ts.transform.translation.z = ps->pose.position.z;
    ts.transform.rotation.x = ps->pose.orientation.x;
    ts.transform.rotation.y = ps->pose.orientation.y;
    ts.transform.rotation.z = ps->pose.orientation.z;
    ts.transform.rotation.w = ps->pose.orientation.w;
    br.sendTransform(ts);

    /*
        PROBLEM 3, as in the PDF:

        Publish a transform that is 1 meter (1.0) in front of "april_tag"
            and in the same orientation (all axes facing the same direction).
            This means 1 meter in front of the face of the marker, as in
            sticking out in front of the marker along its z axis.    
        1) Remember that transformations form chains from the "proximal" frame
            to the "distal" frame. In this case, the "proximal" frame is the
            camera "azure_kinect/camera_body". The "distal" frame is the point
            one meter in front of the marker .
        2) Use Eigen to build your chain of transformations:
            A) Chains of transformations are covered in
                Lec_Spatial_Transformations.pdf
            B) Eigen is covered in Lec_Eigen.pdf
            C) The mathematics are covered in Lec_Transformation_Mat.pdf
        3) The basic version of how to do this is:
            A) Use Eigen::Quaterniond to get the rotation matrix from the
                quaternion in geometry_msgs::PoseStamped that is received by
                this callback.
            B) Build a rigid transformation representing the transform between
                the camera and the marker. The rigid transformation is as in
                Lec_Transformation_Math. It is a 4x4 matrix (Eigen::MatrixXd).
                The upper-left 3x3 submatrix is the rotation (which can be)
                obtained from the Quaterniond that you just built using
                Eigen::Quaterniond::toRotationMatrix() The right column of the
                matrix is the translation, which can be obtained directly from
                the geometry_msgs::PoseStamped that is received by this
                callback.
                The matrix that you have just built is the rigid transformation
                between the camera and the marker.
            C) Build a rigid transformation representing the concept of "one
                meter in front along the z-axis." This transformation has an
                identity rotation component. It has 1.0 in the z translational
                component. Hint: Start by using Eigen::MatrixXd::Identity(4,4).
            D) Multiply the rigid transformation from the camera to the marker,
                and the one from the marker to the offset.
            E) Populate an Eigen::Quaterniond to figure out the quaternion 
                representing the orientation of the offset point.
                You can do this by initializing a new Eigen::Quaterniond with
                the submatrix from the 4x4 rigid transformation (the result)
                of the multiplication in Step D.
                This is a little tricky, so I'll show you what my code looks
                like here:
                Eigen::Quaterniond offsetQ(offsetOutMat.block<3,3>(0,0));
                Hint: Since the offset is facing the same direction as the
                marker, there *is* another way.
            F) Make a geometry_msgs::TransformStamped, pack it with the 
                transform to the offset. Send it.
	        G) HINT: You will have problems with your quaternion after you
		        send it if it is not normalized. So, normalize it before
                putting it into your TransformStamped. To do this, use
                Eigen::Quaterniond::normalize(). In my code, it looks like
                offsetQ.normalize();
    */
    // create position matrix
    Eigen::MatrixXd position(1,4);
    position(0, 0) = ps->pose.position.x;
    position(1, 0) = ps->pose.position.y;
    position(2, 0) = ps->pose.position.z;

    // create quaterniond with current state
    //this might be x y z w order
    Eigen::Quaterniond r;
    r.x() = ps->pose.orientation.x;
    r.y() = ps->pose.orientation.y;
    r.z() = ps->pose.orientation.z;
    r.w() = ps->pose.orientation.w;
 
    r.normalize();

    // create matrix of correct offset
    Eigen::MatrixXd offsetMat = Eigen::MatrixXd::Identity(4, 4);
    
    // set position block to current orientation
    offsetMat.block<3,3>(0, 0) = r.toRotationMatrix();

    Eigen::MatrixXd translation(3,1);
    translation(0,0) = 0;
    translation(1,0) = 0;
    translation(2,0) = 1;
    
    offsetMat.block<3,1>(0,3) = translation;

    Eigen::MatrixXd rt = offsetMat * position;


    // create message to broadcast
    geometry_msgs::TransformStamped psApril;
    psApril.header.frame_id = ps->header.frame_id;
    psApril.child_frame_id = "offset";
    psApril.transform.translation.x = rt(0, 0);
    psApril.transform.translation.y = rt(1, 0);
    psApril.transform.translation.z = rt(2, 0);
    psApril.transform.rotation.x = r.x();
    psApril.transform.rotation.y = r.y();
    psApril.transform.rotation.z = r.z();
    psApril.transform.rotation.w = r.w();
    br.sendTransform(psApril);

    /*
        PROBLEM 4, as in the PDF:

        Publish a transform that is 1 meter (1.0) in front of "april_tag"
            but is rotated 180 degrees about its Y axis.
    
        How to do this:
        1) Do all of the steps from PROBLEM 3.
        2) Add a final rotation at the end about the Y axis.
        3) HINT: Use Eigen::AngleAxisd to achieve this, with M_PI and
            Eigen::Vector3d::UnitY()
    */

    // create quaterniond with correct orientation
    Eigen::Quaterniond rot(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
    rot.normalize();
    Eigen::MatrixXd rotID = Eigen::MatrixXd::Identity(4, 4);
    rotID.block<3,3>(0, 0) = rot.toRotationMatrix();

    psApril.transform.translation.x = rt(0, 0);
    psApril.transform.translation.y = rt(1, 0);
    psApril.transform.translation.z = rt(2, 0);

    // update rt with new quaterniond
    rt = rt * rotID;

    Eigen::Quaterniond offsetFlip(rt.block<3,3>(0,0));

    // update psApril and broadcast
    psApril.child_frame_id = "offset_flip";
    psApril.transform.rotation.x = offsetFlip.x();
    psApril.transform.rotation.y = offsetFlip.y();
    psApril.transform.rotation.z = offsetFlip.z();
    psApril.transform.rotation.w = offsetFlip.w();
    br.sendTransform(psApril);
}
