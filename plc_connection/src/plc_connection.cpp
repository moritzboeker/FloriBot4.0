#include "plc_connection/plc_connection.h"

//Standard constructor
plcConnectionNode::plcConnectionNode()
{
    seq_=0;

    //Read ROSparameter
    ReadParams();

    //Initialize Socket
    InitializeSocket();

    //Subscribe to Topics
    Subscribe();

    //Publish Topics
    CreatePublisher();

    //Run Functions time triggered
    SendRecvTimer_=nh_.createTimer(ros::Duration(0.1), &plcConnectionNode::SendRecv, this);


    //ToDo: Initialize all
    Data_.From.Speed[0]=0;
    Data_.From.Speed[1]=0;
    Data_.From.Speed[2]=0;
    Data_.From.Speed[3]=0;
    Data_.From.Angle=M_PI;

    Data_.To.Speed[0]=0;
    Data_.To.Speed[1]=0;
    Data_.To.Speed[2]=0;
    Data_.To.Speed[3]=0;

    Data_.To.Accelleration[0]=0;
    Data_.To.Accelleration[1]=0;
    Data_.To.Accelleration[2]=0;
    Data_.To.Accelleration[3]=0;

    Data_.To.Torque[0]=0;
    Data_.To.Torque[1]=0;
    Data_.To.Torque[2]=0;
    Data_.To.Torque[3]=0;
    
}

//Initialize Socket
void plcConnectionNode::InitializeSocket()
{
    //temporary variables
    OwnUDP::Address tmpAddress;

    //Set timeout for connection
    ConnectionTimeout_=ros::Duration(PLCTimeout_);
    ROS_INFO("PLC Timeout is set to %f seconds", PLCTimeout_);

    //Set plcmode
    Data_.To.Mode=Mode_;
    ROS_INFO("PLC Mode is %i", Mode_);

    //Set plc Address
    Target_.IP.IP=strTargetIP_;
    Target_.IP.Port=TargetPort_;
    ROS_INFO("PLC IP is %s:%i", strTargetIP_.c_str(), Target_.IP.Port);
    
    //Initialize Target data
    Target_.LastID=0;
    Target_.ComOk=false;
    Target_.LastID=0;
    Target_.LastMsgTime=ros::Time::now();

    tmpAddress.IP.clear();
    tmpAddress.IP=strOwnIP_;
    tmpAddress.Port=OwnPort_;

    //Create UDP Socket
    ROS_INFO("Creating UDP Socket");
    try
    {    
        //set IP-Address
        PLC_Socket_.bindAddress(&tmpAddress);
        ROS_INFO("UDP Socket created on Port %i", OwnPort_);

        //set receive Timeout
        PLC_Socket_.setReceiveTime(ReceiveTimeoutUsec_, ReceiveTimeoutSec_);
        ROS_INFO("Receive Timeout for UDP Socket ist set to %i seconds and %i usec", ReceiveTimeoutSec_, ReceiveTimeoutUsec_);
    }
    catch(const std::runtime_error* e)
    {
        ROS_ERROR("Error while creating UDP Socket on Port %i\n %s", OwnPort_, e->what());
        delete e;
        exit(-1);
    }

}

//Read ROSparameter 
void plcConnectionNode::ReadParams()
{
    //Get Param from Paramserver or set Default Values
    nh_.param<std::string>("/"+ros::this_node::getName()+"/PLC_IP", strTargetIP_, "192.168.0.43");
    nh_.param<std::string>("/"+ros::this_node::getName()+"/Xavier_IP", strOwnIP_, ""); 

    TargetPort_=nh_.param("/"+ros::this_node::getName()+"/PLC_Port", 5000);
    OwnPort_=nh_.param("/"+ros::this_node::getName()+"/Xavier_Port", 5000);  
    Mode_=nh_.param("/"+ros::this_node::getName()+"/Engine_Mode", 0);
    PLCTimeout_=nh_.param("/"+ros::this_node::getName()+"/PLC_Timeout", 1.5);
    ReceiveTimeoutSec_=nh_.param("/"+ros::this_node::getName()+"/Receive_Timeout_sec", 0);
    ReceiveTimeoutUsec_=nh_.param("/"+ros::this_node::getName()+"/Receive_Timeout_usec", 500);
}

//Subscribe to topics
void plcConnectionNode::Subscribe()
{
    //Create Subscriber 
    SpeedSubscriber_=nh_.subscribe("Engine/TargetSpeed", 1, &plcConnectionNode::SpeedCallback, this);
    
    AccelerationSubscriber_=nh_.subscribe("Engine/TargetAcceleration", 1, &plcConnectionNode::AccelerationCallback, this);
    
    TorqueSubscriber_=nh_.subscribe("Engine/TargetTorque", 1, &plcConnectionNode::TorqueCallback, this);
}

//create Publisher
void plcConnectionNode::CreatePublisher()
{
    //create Publisher
    SpeedPublisher_=nh_.advertise<base::Wheels>("Engine/ActualSpeed", 1);
    
    AnglePublisher_=nh_.advertise<base::Angle>("Sensors/BodyAngle", 1);
}

//Callbacks for Subscriber
void plcConnectionNode::SpeedCallback(const base::Wheels::ConstPtr& msg)
{
    Data_.To.Speed[0]=msg->frontRight;
    Data_.To.Speed[1]=msg->frontLeft;
    Data_.To.Speed[2]=msg->rearRight;
    Data_.To.Speed[3]=msg->rearLeft;
}

void plcConnectionNode::TorqueCallback(const base::Wheels::ConstPtr& msg)
{
    Data_.To.Torque[0]=msg->frontRight;
    Data_.To.Torque[1]=msg->frontLeft;
    Data_.To.Torque[2]=msg->rearRight;
    Data_.To.Torque[3]=msg->rearLeft;
}

void plcConnectionNode::AccelerationCallback(const base::Wheels::ConstPtr& msg)
{
    Data_.To.Accelleration[0]=msg->frontRight;
    Data_.To.Accelleration[1]=msg->frontLeft;
    Data_.To.Accelleration[2]=msg->rearRight;
    Data_.To.Accelleration[3]=msg->rearLeft;
}
 
//ToDo: Add Error Handling in Protocol
void plcConnectionNode::SendRecv(const ros::TimerEvent &e)
{
    //Send and receive Data
    try
    {
        SendData();
        ReadData();    
    }
    catch(std::runtime_error* e)
    {
        ROS_WARN("%s",e->what());
        delete e;
    }

    //Publish data
    PublishData();    
}

//send data
void plcConnectionNode::SendData()
{
    //prepare data
    PLC_Data tmpData;
    htonPLC(&tmpData, &Data_);

    //send data
    PLC_Socket_.write((uint8_t*) &tmpData.To, sizeof(Data_.To), &Target_.IP);
}

//receive data
bool plcConnectionNode::ReadData()
{
    //create temporary variables
    PLC_Data tmpData;
    OwnUDP::Address tmpAddress;

    //read received data
    PLC_Socket_.read((uint8_t*) &tmpData.From, sizeof(Data_.From), &tmpAddress);


    //Check received data
    if(tmpAddress.IP==Target_.IP.IP && tmpAddress.Port==Target_.IP.Port || true)
    {
        //Check if it is a new message
        if (ntohl(tmpData.From.MessageID)==Target_.LastID)
        {  
            //Check for connection timeout
            if ((ros::Time::now() - Target_.LastMsgTime).toSec()>ConnectionTimeout_.toSec())
            {
                if (Target_.ComOk==true)
                {
                    ROS_ERROR("No Connection to PLC");
                } 
                Target_.ComOk=false;
            }
            return false;
        }

        //write data for host
        ntohPLC(&Data_, &tmpData);

        Target_.ComOk=true;
        Target_.LastID=Data_.From.MessageID;
        Target_.LastMsgTime=ros::Time::now();

        return true;
    }

    return false;
}

//Publish received Data
void plcConnectionNode::PublishData()
{
    //Create messages
    base::Angle  AngleMsg;
    base::Wheels SpeedMsg;
    geometry_msgs::TransformStamped TFAngleMsg;
    tf2::Quaternion q;

    //write data in messages and publish
    TFAngleMsg.header.seq=seq_;
    TFAngleMsg.child_frame_id="jointRear";
    TFAngleMsg.header.frame_id="jointFront";
    TFAngleMsg.header.stamp=ros::Time::now();

    SpeedMsg.header.seq=seq_;
    SpeedMsg.header.stamp=ros::Time::now();

    AngleMsg.header.seq=seq_++;
    AngleMsg.header.stamp=ros::Time::now();

    q.setRPY(0,0,Data_.From.Angle);
    TFAngleMsg.transform.translation.x=0;
    TFAngleMsg.transform.translation.y=0;
    TFAngleMsg.transform.translation.z=0;

    TFAngleMsg.transform.rotation.x=q.x();
    TFAngleMsg.transform.rotation.y=q.y();
    TFAngleMsg.transform.rotation.z=q.z();
    TFAngleMsg.transform.rotation.w=q.w();


    SpeedMsg.frontRight=Data_.From.Speed[0];
    SpeedMsg.frontLeft=Data_.From.Speed[1];
    SpeedMsg.rearRight=Data_.From.Speed[2];
    SpeedMsg.rearLeft=Data_.From.Speed[3];

    AngleMsg.angle=Data_.From.Angle;

    TFBroadcaster_.sendTransform(TFAngleMsg);

    SpeedPublisher_.publish(SpeedMsg);

    AnglePublisher_.publish(AngleMsg);
}

//Write data for network
void ntohPLC(PLC_Data* Host, PLC_Data* Network)
{
    Host->From.MessageID=ntohl(Network->From.MessageID);
    Host->From.Mode=ntohl(Network->From.Mode);
    Host->From.Angle=OwnSocket::ntohf(Network->From.Angle);
    Host->From.Voltage=OwnSocket::ntohf(Network->From.Voltage);
    Host->From.HomingError=ntohl(Network->From.HomingError);

    for (int i=0;i<4;i++)  
    {
        Host->From.Speed[i]=OwnSocket::ntohf(Network->From.Speed[i]);
        Host->From.SpeedError[i]=ntohl(Network->From.SpeedError[i]);
        Host->From.ResetError[i]=ntohl(Network->From.ResetError[i]);
    }
}

//write data for host
void htonPLC(PLC_Data* Network, PLC_Data* Host)
{
    Network->To.MessageID=htonl(Host->To.MessageID++);
    Network->To.Mode=htonl(Host->To.Mode);
    
    for (int i=0; i<4;i++)
    {
        Network->To.Speed[i]=OwnSocket::htonf(Host->To.Speed[i]);
        Network->To.Torque[i]=OwnSocket::htonf(Host->To.Torque[i]);
        Network->To.Accelleration[i]=OwnSocket::htonf(Host->To.Accelleration[i]);
    }
}