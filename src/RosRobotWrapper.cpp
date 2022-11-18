#include "RosRobotWrapper.h"


WrapperBase::WrapperBase(const ros::NodeHandle& n, DataBridgeBase* db):
    node(n, "base"){

    dataBridgeBase = db;
    BASE_CONTROL_MODE = CONTROL_MODE::BASE_VELOCITY;

    pubOdometry = node.advertise<nav_msgs::Odometry>("odom", 10);
    pubJointState = node.advertise<sensor_msgs::JointState>("joint_states", 10);
    
    subBaseVelocity = node.subscribe("cmd_vel", 10, &WrapperBase::callbackSetBaseVelocity, this);
    subBasePosition = node.subscribe("cmd_pos", 10, &WrapperBase::callbackSetBasePosition, this);
    subJointVelocity = node.subscribe("cmd_joint_velocity", 10, &WrapperBase::callbackSetJointVelocity, this);
    subJointTorque = node.subscribe("cmd_joint_torque", 10, &WrapperBase::callbackSetJointTorque, this);
}

void WrapperBase::callbackSetBaseVelocity(const geometry_msgs::Twist::ConstPtr& msgBaseVelocity){
    this->setpointBaseVelocity = (*msgBaseVelocity);
}
void WrapperBase::callbackSetBasePosition(const geometry_msgs::Pose2D::ConstPtr& msgBasePosition){
    this->setpointBasePosition = msgBasePosition;
}
void WrapperBase::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity){
    this->setpointJointVelocity = msgJointVelocity;
}
void WrapperBase::callbackSetJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque){
    this->setpointJointTorque = msgJointTorque;
}

void WrapperBase::writeCmd(){
    switch (this->BASE_CONTROL_MODE){
        case CONTROL_MODE::JOINT_VELOCITY:
            break;
        case CONTROL_MODE::JOINT_TORQUE:
            break;
        case CONTROL_MODE::BASE_POSITION:
            break;
        case CONTROL_MODE::BASE_VELOCITY:
            dataBridgeBase->setBaseVelocity(this->setpointBaseVelocity);
            break;
        default:
            ROS_WARN("BASE_CONTROL_MODE is wrong");
            break;
    }
}

void WrapperBase::readState(){
    geometry_msgs::Twist currentBaseVelocity;
    dataBridgeBase->getBaseVelocity(currentBaseVelocity);

    nav_msgs::Odometry odom;
    odom.twist.twist = currentBaseVelocity;
    pubOdometry.publish(odom);
}

void WrapperBase::trace(){
    if (startT){
        std::cout << "setpointBaseVel:   " << this->setpointBaseVelocity.linear.x << std::endl;
        std::cout << "BASE_CONTROL_MODE: " << this->BASE_CONTROL_MODE << std::endl;
    }
    
}


