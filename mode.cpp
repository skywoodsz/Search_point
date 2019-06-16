#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <string> 
using namespace std;
class TO_MODE{
public:
    TO_MODE()
    {
        mode_pub_  = n_.advertise<std_msgs::Int32>("mode",1);
        marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        car_sub_ = n_.subscribe("/odometry/filtered", 1, &TO_MODE::receive_odom,this);
        curve_sub_ = n_.subscribe("path_curve_av",1,&TO_MODE::receive_curve,this);
        
        timer1 = n_.createTimer(ros::Duration((0.1/10)), &TO_MODE::to_cal_mode, this); // 100hz
    }
    ~TO_MODE()
    {

    }
    void receive_curve(const std_msgs::Float64::ConstPtr &curveMsg)
    {
        std_msgs::Float64 curve_ros = *curveMsg;
        curve = curve_ros.data;

    }
    void receive_odom(const nav_msgs::Odometry::ConstPtr& odomMsg)//获取小车的map下的坐标
    {
       nav_msgs::Odometry odom_car_pose =  *odomMsg;

       geometry_msgs::PoseStamped odom_car;
       geometry_msgs::PoseStamped map_car;

       odom_car.pose = odom_car_pose.pose.pose;
       odom_car.header = odom_car_pose.header;
    
       try
       {
           listener_.transformPose("map",ros::Time(0),odom_car,"odom",map_car); // 从odom变map
       }
       catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        car_pose.header = map_car.header;
        car_pose.pose.pose = map_car.pose;
        // 第一次接收 设为原点
        if(!is_inited)
        {
            car_pose_init = car_pose;
            car_pose_init_x = car_pose_init.pose.pose.position.x;
            car_pose_init_y = car_pose_init.pose.pose.position.y;
            is_inited = true;
        }
          
    }
    // 距离
    void to_cal_mode(const ros::TimerEvent&)
    {
        /*
        * mode = 0: 直道
        * mode = 1；第一弯道
        * mode = 2; 第二弯道
        * mode = 3; 180度  
        */
        double dis = 0;
        if(is_inited)
        {
            int mode = 0; 

            double car_pose_x = car_pose.pose.pose.position.x;
            double car_pose_y = car_pose.pose.pose.position.y;
            dis = pow(pow(car_pose_x-car_pose_init_x,2) + pow(car_pose_y-car_pose_init_y,2),0.5);
            
            if(dis>wan1_ru && dis<wan1_chu)
            {
                mode = 1;
            }
            if(dis>wan2_ru && dis<wan2_chu)
            {
                mode = 2;
            }
            if(dis>wan180)
            {
                mode = 3;
            }

            std_msgs::Int32 st_mode;
            st_mode.data = mode;
            mode_pub_.publish(st_mode);

            string str = "the mode = ";
            draw_txt(0,car_pose_x,car_pose_y,str,mode,marker_pub_);
        }
        
    }
inline void draw_txt(int id,double x,double y,string s,double num,ros::Publisher pub)
{
  visualization_msgs::Marker visual_temp;

  visual_temp.header.frame_id = "/map";
  visual_temp.header.stamp = ros::Time::now();
  visual_temp.ns = "points_and_txt";
  visual_temp.id = id;
  visual_temp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  visual_temp.scale.z = 0.3;
  visual_temp.color.b = 1.0;
  visual_temp.color.a = 1;

  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 1;
  ostringstream str;
  str<<s<<num;
  visual_temp.text=str.str();
  visual_temp.pose = pose;
  pub.publish(visual_temp);
}
public:
    ros::NodeHandle n_;
    ros::Publisher mode_pub_,marker_pub_;
    ros::Subscriber car_sub_,curve_sub_;
    nav_msgs::Odometry car_pose;
    nav_msgs::Odometry car_pose_init;
    tf::TransformListener listener_;
    bool is_inited = false;
    ros::Timer timer1;

    double curve;
    double car_pose_init_x,car_pose_init_y;
    double wan1_ru,wan1_chu,wan2_ru,wan2_chu,wan180;
    double curve_wan1,curve_dianzhen;
    
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mode");
    ros::NodeHandle n;

    TO_MODE to_mode;

    double wan1_ru,wan1_chu,wan2_ru,wan2_chu,wan180;
    double curve_wan1,curve_dianzhen;

    n.getParam("/mode/wan1_ru",wan1_ru);
    n.getParam("/mode/wan1_chu",wan1_chu);
    n.getParam("/mode/wan2_ru",wan2_ru);
    n.getParam("/mode/wan2_chu",wan2_chu);
    n.getParam("/mode/wan180",wan180);

    to_mode.wan1_ru = wan1_ru;
    to_mode.wan1_chu = wan1_chu;
    to_mode.wan2_ru = wan2_ru;
    to_mode.wan2_chu = wan2_chu;
    to_mode.wan180 = wan180;

    ros::spin();
    return 0;
}
