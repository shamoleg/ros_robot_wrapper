#include "RosRobotWrapper.h"

WrapperJoint::WrapperJoint(const ros::NodeHandle& n, DataBridgeJoint& dataBridgeJoint):
    node(n, "base"){
    this->dataBridgeJoint = &dataBridgeJoint;
    this->msgSetUp();
    
    pubJointState = node.advertise<sensor_msgs::JointState>("joint_states", 10);
    subJointVelocity = node.subscribe("cmd_joint_velocity", 10, &WrapperJoint::callbackSetJointVelocity, this);
    subJointTorque = node.subscribe("cmd_joint_torque", 10, &WrapperJoint::callbackSetJointTorque, this);
}


void WrapperJoint::writeCmd(CONTROL_MODE JOINT_CONTROL_MODE){
    switch (JOINT_CONTROL_MODE){
        case CONTROL_MODE::JOINT_VELOCITY:
            dataBridgeJoint->setJointVelocity(this->setpointJointVelocity);
            break;
        case CONTROL_MODE::JOINT_POSITION:
            dataBridgeJoint->setJointPosition(this->setpointJointVelocity);
            break;
        case CONTROL_MODE::JOINT_TORQUE:
            dataBridgeJoint->setJointTorque(this->setpointJointVelocity);
            break;
        default:
            break;
    }
}


void WrapperJoint::readAndPub(){
    dataBridgeJoint->getJointState(this->msgJointState);

    this->msgJointState.header.stamp = ros::Time::now();
    this->pubJointState.publish(this->msgJointState);
}


void WrapperJoint::trace(){
    // std::cout << "setpointBaseVel:   " << this->setpointBaseVelocity.linear.x << std::endl;
}


void WrapperJoint::msgSetUp(){
    this->msgJointState.name = {"w1", "w2", "w3", "w4"};
    this->msgJointState.position.resize(4);
    this->msgJointState.velocity.resize(4);
    this->msgJointState.effort.resize(4);

    this->setpointJointVelocity.data = {1, 0, 0, 1};
}


void WrapperJoint::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity){
    this->setpointJointVelocity = (*msgJointVelocity);
}


void WrapperJoint::callbackSetJointTorque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque){
    this->setpointJointTorque = (*msgJointTorque);
}


//-------------------------------------
WrapperKinematicsBase::WrapperKinematicsBase(const ros::NodeHandle& n, DataBridgeKinematicsBase& dataBridgeKinematicsBase):
    node(n, "base"){
    this->dataBridgeKinematicsBase = &dataBridgeKinematicsBase;
    this->msgSetUp();

    pubOdometry = node.advertise<nav_msgs::Odometry>("odom", 10);
    subBaseVelocity = node.subscribe("cmd_vel", 10, &WrapperKinematicsBase::callbackSetBaseVelocity, this);
    subBasePosition = node.subscribe("cmd_pos", 10, &WrapperKinematicsBase::callbackSetBasePosition, this);
}


void WrapperKinematicsBase::writeCmd(CONTROL_MODE BASE_CONTROL_MODE){
    switch (BASE_CONTROL_MODE){
        case CONTROL_MODE::BASE_VELOCITY:
            dataBridgeKinematicsBase->setBaseVelocity(this->setpointBaseVelocity);
            break;

        default:
            break;
    }
}


void WrapperKinematicsBase::readAndPub(){
    static geometry_msgs::Twist currentBaseVelocity;
    static geometry_msgs::Pose currentBasePosition;

    dataBridgeKinematicsBase->getBaseVelocity(currentBaseVelocity);    
    dataBridgeKinematicsBase->getBasePosition(currentBasePosition);

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
    this->msgOdom.header.frame_id = "config->name_odomFrame";
    this->msgOdom.child_frame_id = "config->name_odomChildFrame";

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

