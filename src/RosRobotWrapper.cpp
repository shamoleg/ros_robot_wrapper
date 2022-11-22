#include "RosRobotWrapper.h"


WrapperJoint::WrapperJoint(const ros::NodeHandle& n, BridgeJoint& bridgeJoint):
    node(n){
    this->bridgeJoint = &bridgeJoint;
    this->msgSetUp();
    
    pubJointState = node.advertise<sensor_msgs::JointState>("joint_states", 10);

    subJointPosition = node.subscribe("cmd_joint_position", 10, &WrapperJoint::callbackSetJointPosition, this);
    subJointVelocity = node.subscribe("cmd_joint_velocity", 10, &WrapperJoint::callbackSetJointVelocity, this);
    subJointTorque = node.subscribe("cmd_joint_torque", 10, &WrapperJoint::callbackSetJointTorque, this);
}


void WrapperJoint::writeCmd(CONTROL_MODE JOINT_CONTROL_MODE){
    switch (JOINT_CONTROL_MODE){

        case CONTROL_MODE::JOINT_VELOCITY:
            bridgeJoint->setJointVelocity(this->setpointJointVelocity);
            break;

        case CONTROL_MODE::JOINT_POSITION:
            bridgeJoint->setJointPosition(this->setpointJointVelocity);
            break;

        case CONTROL_MODE::JOINT_TORQUE:
            bridgeJoint->setJointTorque(this->setpointJointVelocity);
            break;

        default:
            break;
    }
}


void WrapperJoint::readAndPub(){
    bridgeJoint->getJointState(this->msgJointState);

    this->msgJointState.header.stamp = ros::Time::now();
    this->pubJointState.publish(this->msgJointState);
}


void WrapperJoint::setNumOfJoint(const int& numOfJoint){
    this->setpointJointPosition.data.resize(numOfJoint);
    this->setpointJointVelocity.data.resize(numOfJoint);
    this->setpointJointTorque.data.resize(numOfJoint);
}


void WrapperJoint::trace(){
    
}


void WrapperJoint::msgSetUp(){
}


void WrapperJoint::callbackSetJointPosition(const std_msgs::Float32MultiArray::ConstPtr& msgJointPosition){
    this->setpointJointPosition = (*msgJointPosition);
}


void WrapperJoint::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity){
    this->setpointJointVelocity = (*msgJointVelocity);
}


void WrapperJoint::callbackSetJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque){
    this->setpointJointTorque = (*msgJointTorque);
}


//-------------------------------------
WrapperKinematicsBase::WrapperKinematicsBase(const ros::NodeHandle& n, BridgeKinematicsBase& bridgeKinematicsBase):
    node(n){
    this->bridgeKinematicsBase = &bridgeKinematicsBase;
    this->msgSetUp();

    pubOdometry = node.advertise<nav_msgs::Odometry>("odom", 10);
    subBaseVelocity = node.subscribe("cmd_vel", 10, &WrapperKinematicsBase::callbackSetBaseVelocity, this);
    subBasePosition = node.subscribe("cmd_pos", 10, &WrapperKinematicsBase::callbackSetBasePosition, this);
}


void WrapperKinematicsBase::writeCmd(CONTROL_MODE BASE_CONTROL_MODE){
    switch (BASE_CONTROL_MODE){

        case CONTROL_MODE::BASE_VELOCITY:
            bridgeKinematicsBase->setBaseVelocity(this->setpointBaseVelocity);
            break;

        case CONTROL_MODE::BASE_POSITION:
            bridgeKinematicsBase->setBaseVelocity(this->setpointBaseVelocity);
            break;

        default:
            break;
    }
}


void WrapperKinematicsBase::readAndPub(){
    static geometry_msgs::Twist currentBaseVelocity;
    static geometry_msgs::Pose currentBasePosition;

    bridgeKinematicsBase->getBaseVelocity(currentBaseVelocity);    
    bridgeKinematicsBase->getBasePosition(currentBasePosition);

    this->msgOdom.header.stamp = ros::Time::now();
    this->msgOdom.twist.twist = currentBaseVelocity;
    this->msgOdom.pose.pose = currentBasePosition;

    this->msgTransformOdom.header.stamp = this->msgOdom.header.stamp;
    this->msgTransformOdom.transform.translation.x = this->msgOdom.pose.pose.position.x;
    this->msgTransformOdom.transform.translation.y = this->msgOdom.pose.pose.position.y;
    this->msgTransformOdom.transform.rotation = this->msgOdom.pose.pose.orientation;

    this->pubOdometry.publish(this->msgOdom);
    this->br.sendTransform(this->msgTransformOdom);
}


void WrapperKinematicsBase::trace(){
    std::cout << "setpointBaseVel:   " << this->setpointBaseVelocity.linear.x << std::endl;
}


void WrapperKinematicsBase::msgSetUp(){
}


void WrapperKinematicsBase::setOdomFrame(std::string name_odomFrame, std::string name_odomChildFrame){
    this->msgOdom.header.frame_id = name_odomFrame;
    this->msgOdom.child_frame_id = name_odomChildFrame;

    this->msgTransformOdom.header.frame_id = this->msgOdom.header.frame_id;
    this->msgTransformOdom.child_frame_id = this->msgOdom.child_frame_id;
}


void WrapperKinematicsBase::callbackSetBaseVelocity(const geometry_msgs::Twist::ConstPtr& msgBaseVelocity){
    this->setpointBaseVelocity = (*msgBaseVelocity);
}


void WrapperKinematicsBase::callbackSetBasePosition(const geometry_msgs::Pose::ConstPtr& msgBasePosition){
    this->setpointBasePosition = (*msgBasePosition);
}
//-------------------------------------

