#include <string>
#include <std_msgs/String.h>
#include <ur_msgs/urscript.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//#include <tf/transform_datatypes.h>
#include <calibration_glasgow/parameters.h>
#include <calibration_glasgow/calibration_services.h>

/*
void getPose(const std::string &tool_link, const std::string &base_link, const ros::Time &ts) 
{
    string base_link_ ;
    if ( base_link.empty() ) base_link_ = "base_link" ;
    else base_link_ = base_link ;

    string tool_link_;
    if(tool_link.empty()) tool_link_ = "tool0_controller";
    else tool_link_ = tool_link ;

    ros::Publisher trans_pub = nh.advertise<geometry_msgs::TransformStamped>(TRANFORM_SUB, 10);
    tf::TransformListener listener(ros::Duration(1.0));
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped msg_trans;    

    try 
    {
        listener.waitForTransform(base_link_, tool_link_, ts, ros::Duration(1) );//第二个参数到第一个参数的变换T
        listener.lookupTransform(base_link_, tool_link_, ts, transform);
        tf::TransformStampedTFToMsg(transform, msg_trans);
        trans_pub.publish(msg_trans);
              
    } 
    catch (tf::TransformException ex) 
    {
        ROS_ERROR("%s",ex.what());     
    }

}*/

//写一串urscript
std::string cmd_str;
std::string cmd_str1;

int main(int argc,char** argv)
{
    ros::init(argc, argv, "robot_msg_listener");
   
    cmd_str = "movej([1.57075, -1.396, -1.57075, 0.349, 1.396, 0.2094],1.4,1.05,5)\n"; 
   
    std::string joint_prefix = "";
    std::string base_link_ = joint_prefix + "base" ;
    std::string tool_link_ = joint_prefix + "tool0_controller";
    ros::NodeHandle nh;
    ros::Rate rate(50.0);

    ros::ServiceClient joint_state_client = nh.serviceClient<ur_msgs::urscript>("ur_driver/URScript_srv");
    ur_msgs::urscript srv;
    srv.request.script = cmd_str;

    ROS_INFO("calling the service...");   
    while(!joint_state_client.call(srv));
    ROS_INFO("succeeded in calling the service.");   
    
    
    ros::Publisher trans_pub = nh.advertise<geometry_msgs::TransformStamped>(TRANFORM_SUB, 10);
    tf::TransformListener listener(ros::Duration(1.0));
    ros::Rate loop_rate(10);


    while (nh.ok())
    {
        
        tf::StampedTransform transform;
        geometry_msgs::TransformStamped msg_trans;    

        try 
        {
            listener.waitForTransform(tool_link_, base_link_, ros::Time(0), ros::Duration(1) );//第二个参数到第一个参数的变换T
            listener.lookupTransform(tool_link_, base_link_, ros::Time(0), transform);
            tf::transformStampedTFToMsg(transform, msg_trans);
            trans_pub.publish(msg_trans);
                
        } 
        catch (tf::TransformException ex) 
        {
            ROS_ERROR("%s",ex.what());     
        }

            rate.sleep();
    }

    return 0;

}


/*
//与之message_filters::Subscriber<geometry_msgs::TransformStamped> trans_sub_;对应的发布器：
ros::Publisher trans_pub = nh.advertise<geometry_msgs::TransformStamped>(TRANFORM_SUB, 10);
trans_pub.publish(msg_trans);

TransformBroadcaster
TransformListener
ros::Time(0)
ros::Time::now()

tf::TransformStampedTFToMsg(transform, msg_trans);
StampedTransform (const tf::Transform &input, const ros::Time &timestamp,
                   const std::string &frame_id, const std::string &child_frame_id)
lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, StampedTransform &transform) const
 	Get the transform between two frames by frame ID. 
 */