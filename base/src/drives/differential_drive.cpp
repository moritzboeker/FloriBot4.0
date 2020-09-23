#include "drives/differential_drive.h"

kinematics::differentialDrive::differentialDrive(double axesLength, double wheelDiameter)
{
    setParam(axesLength, wheelDiameter);
}

kinematics::differentialDrive::differentialDrive()
{
    reset();
}

kinematics::differentialDrive::~differentialDrive()
{
}

void kinematics::differentialDrive::reset()
{
    Pose_.theta=0;
    Pose_.x=0;
    Pose_.y=0;

    WheelSpeed_.leftWheel=0;
    WheelSpeed_.rightWheel=0;

    TimeStamp_=ros::Time::now();
    ROS_ERROR("%f", TimeStamp_.toSec());
}

void kinematics::differentialDrive::setParam(double axesLength, double wheelDiameter)
{
    reset();
    axesLength_=axesLength;
    wheelDiameter_=wheelDiameter;
    wheelCircumference_=2*M_PI*wheelDiameter_/2;
}

geometry_msgs::Pose2D kinematics::differentialDrive::estimateActualPose()
{
    //Declare temporary Variables
    geometry_msgs::Pose2D estimatedPose;

    double deltaTime=(ros::Time::now()-TimeStamp_).toSec();
   
    estimatedPose.x=Pose_.x+Speed_.linear.x+deltaTime+cos(Pose_.theta+0.5*Speed_.angular.z*deltaTime);
    estimatedPose.y=Pose_.y+Speed_.linear.x+deltaTime+sin(Pose_.theta+0.5*Speed_.angular.z*deltaTime);
    estimatedPose.theta=Pose_.theta+Speed_.angular.z*deltaTime;

    return estimatedPose;
}

geometry_msgs::Pose2D kinematics::differentialDrive::forwardKinematics(DifferentialWheelSpeed WheelSpeed, ros::Time Timestamp)
{
    double deltaTime=(Timestamp-TimeStamp_).toSec();
    TimeStamp_=Timestamp;

    WheelSpeed_=WheelSpeed;
    Speed_.linear.x=(WheelSpeed_.leftWheel*wheelCircumference_+WheelSpeed_.rightWheel*wheelCircumference_)/2;
    Speed_.angular.z=(WheelSpeed_.rightWheel*wheelCircumference_-WheelSpeed_.leftWheel*wheelCircumference_)/axesLength_;

    //from Papers and Books
    Pose_.x+=Speed_.linear.x*deltaTime*cos(Pose_.theta+0.5*Speed_.angular.z*deltaTime);
    Pose_.y+=Speed_.linear.x*deltaTime*sin(Pose_.theta+0.5*Speed_.angular.z*deltaTime);
    Pose_.theta+=Speed_.angular.z*deltaTime;
   

    return Pose_;
}


kinematics::DifferentialWheelSpeed kinematics::differentialDrive::inverseKinematics(geometry_msgs::Twist cmdVelMsg)
{
    kinematics::DifferentialWheelSpeed WheelSpeed;

    WheelSpeed.rightWheel=cmdVelMsg.linear.x+cmdVelMsg.angular.z*axesLength_/2;
    WheelSpeed.leftWheel=WheelSpeed.rightWheel-cmdVelMsg.angular.z*axesLength_;

    WheelSpeed.rightWheel=WheelSpeed.rightWheel/wheelCircumference_;
    WheelSpeed.leftWheel=WheelSpeed.leftWheel/wheelCircumference_;

    return WheelSpeed;
}

geometry_msgs::Pose2D kinematics::differentialDrive::getActualPose()
{
    return Pose_;
}

geometry_msgs::Twist kinematics::differentialDrive::getSpeed()
{
    return Speed_;
}