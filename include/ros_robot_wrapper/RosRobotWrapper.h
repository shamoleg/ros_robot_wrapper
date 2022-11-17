/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "ros/ros.h"

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

enum CONTROL_MODE{
    STOP_SEND      = 0,
    JOINT_POSITION = 1,
    JOINT_VELOCITY = 2,
    JOINT_TORQUE   = 3,
    BASE_POSITION  = 4,
    BASE_VELOCITY  = 5,
};

class Wrapper{
    virtual void writeCmd() = 0;
    virtual void readState() = 0;
    virtual void trace() = 0;
};

class DataBridgeBase{   
public:
    virtual void setJointPosition(const std_msgs::Float32MultiArray::ConstPtr& msgJointPosition) = 0;
    virtual void setJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity) = 0;
    virtual void setJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque) = 0;
    virtual void getJointState(sensor_msgs::JointState& msgJointState) = 0;

    virtual void setBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) = 0;
    virtual void getBaseVelocity(geometry_msgs::Twist& msgBaseVelocity) = 0;

    virtual void setBasePosition(const geometry_msgs::Pose2D& msgBasePosition) = 0;
    virtual void getBasePosition(geometry_msgs::Pose2D& msgBasePosition) = 0;
    
};

class WrapperBase {
public:
    WrapperBase(const ros::NodeHandle& n, DataBridgeBase* dataBridgeBase);
    
    void writeCmd();
    void trace();

private:
    ros::NodeHandle node;

    ros::Publisher pubOdometry;
    ros::Publisher pubJointState;

    ros::Subscriber subBaseVelocity;
    ros::Subscriber subBasePosition;
    ros::Subscriber subJointVelocity;
    ros::Subscriber subJointTorque;

    void callbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity);
    void callbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition);
    void callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity);
    void callbackSetJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque);

    geometry_msgs::Twist setpointBaseVelocity;
    geometry_msgs::Pose2D setpointBasePosition;
    std_msgs::Float32MultiArray::ConstPtr setpointJointVelocity;
    std_msgs::Float32MultiArray::ConstPtr setpointJointTorque;

    CONTROL_MODE BASE_CONTROL_MODE;
    DataBridgeBase* dataBridgeBase;
};


