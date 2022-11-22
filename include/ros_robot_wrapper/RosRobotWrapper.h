/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "ros/ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


class BridgeJoint{   
public:
    virtual void setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition) = 0;
    virtual void setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity) = 0;
    virtual void setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque) = 0;

    virtual void getJointState(sensor_msgs::JointState& msgJointState) = 0;
};


class BridgeKinematicsBase{   
public:
    virtual void setBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) = 0;
    virtual void setBasePosition(const geometry_msgs::Pose& msgBasePosition) = 0;

    virtual void getBaseVelocity(geometry_msgs::Twist& msgBaseVelocity) = 0;
    virtual void getBasePosition(geometry_msgs::Pose& msgBasePosition) = 0;
};


enum CONTROL_MODE{
    STOP_SEND = 0,
    JOINT_POSITION,
    JOINT_VELOCITY,
    JOINT_TORQUE,
    BASE_POSITION,
    BASE_VELOCITY,
};


class Wrapper{
    virtual void writeCmd(CONTROL_MODE CURRENT_CONTROL_MODE) = 0;
    virtual void readAndPub() = 0;
    virtual void trace() = 0;
};


class WrapperJoint : Wrapper{
public:
    WrapperJoint(const ros::NodeHandle& n, BridgeJoint& bridgeJoint);
    
    void writeCmd(CONTROL_MODE JOINT_CONTROL_MODE);
    void readAndPub();
    void trace();

private:
    BridgeJoint* bridgeJoint;
    
    void msgSetUp();
    void readAndPubJointState();

    ros::NodeHandle node;
    
    ros::Publisher pubJointState;
    ros::Subscriber subJointPosition;
    ros::Subscriber subJointVelocity;
    ros::Subscriber subJointTorque;

    sensor_msgs::JointState msgJointState;
    std_msgs::Float32MultiArray setpointJointPosition;
    std_msgs::Float32MultiArray setpointJointVelocity;
    std_msgs::Float32MultiArray setpointJointTorque;

    void callbackSetJointPosition(const std_msgs::Float32MultiArray::ConstPtr& msgJointPosition);    
    void callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity);
    void callbackSetJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque);
};


class WrapperKinematicsBase : Wrapper{
public:
    WrapperKinematicsBase(const ros::NodeHandle& n, BridgeKinematicsBase& bridgeKinematicsBase);
    
    void writeCmd(CONTROL_MODE JOINT_CONTROL_MODE);
    void readAndPub();
    void trace();

private:
    BridgeKinematicsBase* bridgeKinematicsBase;

    void msgSetUp();
    void readAndPubOdom();

    ros::NodeHandle node;

    ros::Publisher pubOdometry;
    tf2_ros::TransformBroadcaster br;  
    ros::Subscriber subBaseVelocity;
    ros::Subscriber subBasePosition;

    geometry_msgs::Twist setpointBaseVelocity;
    geometry_msgs::Pose setpointBasePosition;
    nav_msgs::Odometry msgOdom;
    geometry_msgs::TransformStamped msgTransformOdom;

    void callbackSetBaseVelocity(const geometry_msgs::Twist::ConstPtr& msgBaseVelocity);
    void callbackSetBasePosition(const geometry_msgs::Pose::ConstPtr& msgBasePosition);
};










