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

enum CONTROL_MODE{
    STOP_SEND = 0,
    JOINT_POSITION,
    JOINT_VELOCITY,
    JOINT_TORQUE,
    BASE_POSITION,
    BASE_VELOCITY,
};

class Wrapper{
    virtual void writeCmd() = 0;
    virtual void readAndPub() = 0;
    virtual void trace() = 0;
};

class DataBridgeBase{   
public:
    virtual void setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition) = 0;
    virtual void setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity) = 0;
    virtual void setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque) = 0;
    virtual void getJointState(sensor_msgs::JointState& msgJointState) = 0;

    virtual void setBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) = 0;
    virtual void getBaseVelocity(geometry_msgs::Twist& msgBaseVelocity) = 0;

    virtual void setBasePosition(const geometry_msgs::Pose& msgBasePosition) = 0;
    virtual void getBasePosition(geometry_msgs::Pose& msgBasePosition) = 0;
};

class WrapperBase {
public:
    WrapperBase(const ros::NodeHandle& n, DataBridgeBase* dataBridgeBase);
    
    void writeCmd();
    void readAndPub();
    void trace();

private:
    bool startT=0;

    ros::NodeHandle node;
    
    tf2_ros::TransformBroadcaster* br;
    ros::Publisher pubOdometry;
    ros::Publisher pubJointState;

    void msgSetUp();
    
    nav_msgs::Odometry msgOdom;
    geometry_msgs::TransformStamped transformOdom;
    sensor_msgs::JointState msgJointState;

    void readAndPubOdom();
    void readAndPubJointState();

    ros::Subscriber subBaseVelocity;
    ros::Subscriber subBasePosition;
    ros::Subscriber subJointVelocity;
    ros::Subscriber subJointTorque;

    geometry_msgs::Twist setpointBaseVelocity;
    geometry_msgs::Pose setpointBasePosition;
    std_msgs::Float32MultiArray setpointJointVelocity;
    std_msgs::Float32MultiArray setpointJointTorque;

    void callbackSetBaseVelocity(const geometry_msgs::Twist::ConstPtr& msgBaseVelocity);
    void callbackSetBasePosition(const geometry_msgs::Pose::ConstPtr& msgBasePosition);
    void callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity);
    void callbackSetJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque);

    CONTROL_MODE BASE_CONTROL_MODE;
    DataBridgeBase* dataBridgeBase;
};


