//=================================================================================================
// Copyright (c) 2018, Bowen Wen, Ohio State University
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.


// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <cmath>
#include <fstream>
#include "ll2utm.h"

using boost::asio::ip::udp;


class Quaterniond
{
public:
    double w,x,y,z;
    Quaterniond(double x1,double x2,double x3,double x4)
    {
        w=x1;
        x=x2;
        y=x3;
        z=x4;
    }
    Quaterniond() {}

};


inline static Quaterniond ToQuaternion(double pitch, double roll, double yaw) // Here, yaw is heading angle in GPS
{
    Quaterniond q;
    pitch=0.0;
    roll=0.0;
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(roll * 0.5);
    double t3 = std::sin(roll * 0.5);
    double t4 = std::cos(pitch * 0.5);
    double t5 = std::sin(pitch * 0.5);

    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}

inline double GetYaw(Quaterniond q) // from quaternion form
{
    return std::atan2(2*(q.w*q.z+q.y*q.x),1-2*(std::pow(q.y,2)+std::pow(q.z,2)));
}


class Udpros
{
private:
    boost::asio::io_service io_service;
    udp::endpoint receiver_endpoint;
    udp::endpoint sender_endpoint;
    udp::socket socket;
    boost::system::error_code error;
    boost::system::error_code ignored_error;
    std::vector<int> message;
    int recv_buf[25];


    ros::Subscriber loc_sub;
    ros::Subscriber dist_sub;
    ros::Publisher gps_pub;
    nav_msgs::Odometry location;

    nav_msgs::Odometry gps_pub_;

    bool ros_listen_, ros_talk_;
    std::string pose_topic_,dist_topic_;

    double initial_x,initial_y,initial_heading;
    bool isfirst;

    std::ofstream myfile;

public:
    Udpros(ros::NodeHandle nh, ros::NodeHandle Private_nh);
    ~Udpros(){myfile.close();}
    void ProcessData(const nav_msgs::Odometry::ConstPtr &location);
    void ProcessGPS();
    void FillBuffer(double val);
    double GetBuffer(int * recv_buf, int n);
    void PoseROS2World(double (&p)[2]);
    void PoseWorld2ROS(double (&pose)[2]);


};

Udpros::Udpros(ros::NodeHandle nh, ros::NodeHandle Private_nh):socket(io_service, udp::endpoint(udp::v4(), 5555)),
    receiver_endpoint(boost::asio::ip::address::from_string("192.168.0.89"), 5002)
{
//    initial_heading=-M_PI/2;
    myfile.open("/home/wbw/log.txt");
    isfirst=true;
    Private_nh.param("ros_listen",ros_listen_,false);
    Private_nh.param("ros_talk",ros_talk_,true);
    Private_nh.param<std::string>("pose_topic",pose_topic_,"scanmatch_odom");
    Private_nh.param<std::string>("dist_topic",dist_topic_,"mindistance");
    Private_nh.param<double>("GPSh",initial_heading,-90); // [deg]

    gps_pub=nh.advertise<nav_msgs::Odometry>("odometry",100);
    loc_sub=nh.subscribe(pose_topic_,10,&Udpros::ProcessData,this,ros::TransportHints().tcpNoDelay(true));
    std::cout<<"initial heading="<<initial_heading<<std::endl;

    if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

}

void Udpros::ProcessData(const nav_msgs::Odometry::ConstPtr &location)
{

    double yaw;
    message.clear();
    double pose[2];

    pose[0]=location->pose.pose.position.x;  // message:{x,y,yaw}
    pose[1]=location->pose.pose.position.y;
//    std::cout<<"raw:"<<pose[0]<<"       "<<pose[1]<<std::endl<<std::endl;



    PoseROS2World(pose);

//    std::cout<<"new:"<<pose[0]<<"       "<<pose[1]<<std::endl<<std::endl;
    FillBuffer(pose[0]);
    FillBuffer(pose[1]);

    Quaterniond q;
    q.w=location->pose.pose.orientation.w;
    q.x=location->pose.pose.orientation.x;
    q.y=location->pose.pose.orientation.y;
    q.z=location->pose.pose.orientation.z;

    yaw=GetYaw(q);

    yaw=yaw*180/M_PI;
    yaw=(initial_heading-yaw); // to world coordinate [deg]
    if (yaw<-180)
    {
        yaw=yaw+360;
    }
    if (yaw>180)
    {
        yaw=yaw-360;
    }

    FillBuffer(yaw);

    std::vector<int>::iterator ite;
//    ite=message.begin();
//    for (ite;ite!=message.end();ite++)
//    {
//        std::cout<<*ite<<" ";
//    }
//    std::cout<<std::endl;

    std::cout<<"sending: x="<<pose[0]<<",y="<<pose[1]<<",yaw="<<yaw<<std::endl;

    socket.send_to(boost::asio::buffer(message), receiver_endpoint, 0, ignored_error);  //return The number of bytes sent.

    size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);


    for (int k=0;k<12;k++)
    {
        myfile<<recv_buf[k]<<" ";
    }
     myfile<<"\n";
    ProcessGPS(); // get gps and publish as topic

}

void Udpros::ProcessGPS()
{
//    if (isfirst==true)
//    {
//        initial_x=GetBuffer(recv_buf,0);
//        initial_y=GetBuffer(recv_buf,3);
////        double tempx,tempy;
//        LLtoUTM(initial_x,initial_y,initial_x,initial_y);
//        isfirst=false;
//    }
    std::cout<<"initial pose="<<initial_x<<","<<initial_y<<std::endl;
//    gps_pub_.header.stamp=location->header.stamp; // borrow location's stamp
    gps_pub_.header.stamp=ros::Time::now();

    gps_pub_.header.frame_id="odom";
    gps_pub_.child_frame_id="base_link";
    gps_pub_.pose.pose.position.z=0;

    //recv_buf should be {x,y,yaw,vE,vN,aE,aN}




    double pose[2],vel[2],yaw,yaw_rate;
    pose[0]=GetBuffer(recv_buf,0);

//    std::cout<<pose[0]<<"++++++++++++++++++++++++++++++++"<<std::endl;


    pose[1]=GetBuffer(recv_buf,3);
//    std::cout<<pose[1]<<"++++++++++++++++++++++++++++++++"<<std::endl;
//    LLtoUTM(pose[0],pose[1],pose[0],pose[1]);

//    std::cout<<"by conversion: "<<pose[0]<<std::endl;

//    pose[0]-=initial_x;
//    pose[1]-=initial_y;
    std::cout<<"from gps,x="<<pose[0]<<", y="<<pose[1]<<std::endl;
    yaw=GetBuffer(recv_buf,6);  // [deg]
    yaw=yaw*M_PI/180.0;

    vel[0]=GetBuffer(recv_buf,9);
    vel[1]=GetBuffer(recv_buf,12);
    yaw_rate=GetBuffer(recv_buf,15);



    Quaterniond q=ToQuaternion(0.0,0.0,yaw);
//    PoseWorld2ROS(pose);
    gps_pub_.pose.pose.position.x=pose[0];
    gps_pub_.pose.pose.position.y=pose[1];
    gps_pub_.pose.pose.position.z=0.0;

    gps_pub_.twist.twist.linear.x=vel[0];
    gps_pub_.twist.twist.linear.y=vel[1];
    gps_pub_.twist.twist.angular.z=yaw_rate;

//    gps_pub_.twist.twist.angular.z=     // yaw angular speed

//    std::cout<<"gps: x:"<<gps_pub_.pose.pose.position.x<<" y:"<<gps_pub_.pose.pose.position.y<<" vx:"<<gps_pub_.twist.twist.linear.x<<" vy:"
//            <<gps_pub_.twist.twist.linear.y<<" ax:"<<" ay:"<<" yaw"<<std::endl;



    gps_pub_.pose.pose.orientation.w=q.w;
    gps_pub_.pose.pose.orientation.x=q.x;
    gps_pub_.pose.pose.orientation.y=q.y;
    gps_pub_.pose.pose.orientation.z=q.z;

    std::cout<<"gps heading="<<yaw<<" "<<vel[0]<<" "<<vel[1]<<" "<<yaw_rate<<std::endl;
    gps_pub.publish(gps_pub_);
}

void Udpros::PoseROS2World(double (&p)[2])
{


    double theta=initial_heading*M_PI/180-M_PI/2;
    double x,y;

    x=std::cos(theta)*p[0]+std::sin(theta)*p[1];
    y=-std::sin(theta)*p[0]+std::cos(theta)*p[1];
    p[0]=x;
    p[1]=y;
}

void Udpros::PoseWorld2ROS(double (&pose)[2])
{
    double theta=M_PI/2-initial_heading*M_PI/180;
    double pose1[2];
    pose1[0]=std::cos(theta)*pose[0]+std::sin(theta)*pose[1];
    pose1[1]=-std::sin(theta)*pose[0]+std::cos(theta)*pose[1];
    pose[0]=pose1[0];
    pose[1]=pose1[1];
}



void Udpros::FillBuffer(double val)
{

    if (val>=0)
    {
        message.push_back(0);
    }
    else
    {
        message.push_back(1);
        val=-val;
    }

    message.push_back((int)val);
    std::vector<int>::iterator i=message.end();
    message.push_back((int)((val-*(i-1))*1000000));
}

double Udpros::GetBuffer(int * recv_buf,int n)
{
//    std::cout<<"..................."<<((double)recv_buf[n+2]/1000000.0)<<std::endl;
    if (recv_buf[n]==0) // positive
    {
//        printf("%f\n",(double)recv_buf[n+1]+((double)recv_buf[n+2]/1000000.0));
//        std::cout<<"..................."<<((double)recv_buf[n+2]/1000000.0)<<"//"<<(double)recv_buf[n+1]<<"//"<<(double)recv_buf[n+1]+((double)recv_buf[n+2]/1000000.0)<<std::endl;
        return (double)recv_buf[n+1]+((double)recv_buf[n+2]/1000000.0);
    }
    else // negatvie
    {
        return -(double)recv_buf[n+1]-((double)recv_buf[n+2]/1000000.0);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"Udpros");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    Udpros udpros(nh,priv_nh);
    ros::Rate r(100);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
