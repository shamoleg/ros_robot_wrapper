#include "RosRobotWrapper.h"


WrapperBase::WrapperBase(const ros::NodeHandle& n, DataBridgeBase* db):
    node(n, "base"){

    BASE_CONTROL_MODE = CONTROL_MODE::BASE_VELOCITY;

    dataBridgeBase = db;
    br = new tf2_ros::TransformBroadcaster;

    this->msgSetUp();

    pubOdometry = node.advertise<nav_msgs::Odometry>("odom", 10);
    pubJointState = node.advertise<sensor_msgs::JointState>("joint_states", 10);
    
    subBaseVelocity = node.subscribe("cmd_vel", 10, &WrapperBase::callbackSetBaseVelocity, this);
    subBasePosition = node.subscribe("cmd_pos", 10, &WrapperBase::callbackSetBasePosition, this);
    subJointVelocity = node.subscribe("cmd_joint_velocity", 10, &WrapperBase::callbackSetJointVelocity, this);
    subJointTorque = node.subscribe("cmd_joint_torque", 10, &WrapperBase::callbackSetJointTorque, this);
}

void WrapperBase::msgSetUp(){
    this->msgOdom.header.frame_id = "config->name_odomFrame";
    this->msgOdom.child_frame_id = "config->name_odomChildFrame";

    this->transformOdom.header.frame_id = this->msgOdom.header.frame_id;
    this->transformOdom.child_frame_id = this->msgOdom.child_frame_id;

    this->msgJointState.name = {"w1", "w2", "w3", "w4"};
    this->msgJointState.position.resize(4);
    this->msgJointState.velocity.resize(4);
    this->msgJointState.effort.resize(4);
}

void WrapperBase::callbackSetBaseVelocity(const geometry_msgs::Twist::ConstPtr& msgBaseVelocity){
    this->setpointBaseVelocity = (*msgBaseVelocity);
}

void WrapperBase::callbackSetBasePosition(const geometry_msgs::Pose::ConstPtr& msgBasePosition){
    this->setpointBasePosition = (*msgBasePosition);
}

void WrapperBase::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity){
    this->setpointJointVelocity = (*msgJointVelocity);
}

void WrapperBase::callbackSetJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque){
    this->setpointJointTorque = (*msgJointTorque);
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

void WrapperBase::readAndPub(){
    this->readAndPubOdom();
    this->readAndPubJointState();
}

void WrapperBase::readAndPubOdom(){
    static geometry_msgs::Twist currentBaseVelocity;
    static geometry_msgs::Pose currentBasePosition;

    dataBridgeBase->getBaseVelocity(currentBaseVelocity);    
    dataBridgeBase->getBasePosition(currentBasePosition);

    this->msgOdom.header.stamp = ros::Time::now();
    this->msgOdom.twist.twist = currentBaseVelocity;
    this->msgOdom.pose.pose = currentBasePosition;

    this->transformOdom.header.stamp = this->msgOdom.header.stamp;
    this->transformOdom.transform.translation.x = this->msgOdom.pose.pose.position.x;
    this->transformOdom.transform.translation.y = this->msgOdom.pose.pose.position.y;
    this->transformOdom.transform.rotation = this->msgOdom.pose.pose.orientation;

    this->pubOdometry.publish(this->msgOdom);
    this->br->sendTransform(this->transformOdom);
}

void WrapperBase::readAndPubJointState(){
    dataBridgeBase->getJointState(this->msgJointState);
    this->msgJointState.header.stamp = ros::Time::now();
    this->pubJointState.publish(this->msgJointState);
}

void WrapperBase::trace(){
    if (startT){
        std::cout << "setpointBaseVel:   " << this->setpointBaseVelocity.linear.x << std::endl;
        std::cout << "BASE_CONTROL_MODE: " << this->BASE_CONTROL_MODE << std::endl;
    }
}


