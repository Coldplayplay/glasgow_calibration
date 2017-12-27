#include <ctime>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <ur_msgs/urscript.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//#include <tf/transform_datatypes.h>
#include <calibration_glasgow/parameters.h>
#include <calibration_glasgow/calibration_services.h>

vector<double> joint_vels (3, 0.1) ;

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

void joint_msg_process(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_vels = msg->velocity;
    //ROS_INFO_STREAM("ur joint states received.");    
}

bool checkJointState(vector<double> joint_vel)
{
    for(int i=0;i<joint_vel.size();i++)
    {
        if(abs(joint_vel[i])<=1e-15)
            continue;
        else
        {
            ROS_INFO_STREAM("move...");
            return false;
        }        
        
    }
    ROS_INFO_STREAM("stop.");
    return true;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "robot_msg_listener");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    std::string joint_prefix = "";
    std::string base_link_ = joint_prefix + "base" ;
    std::string tool_link_ = joint_prefix + "tool0_controller";

    vector<string> cmd_str;
    cmd_str.push_back("movej([1.57075, -1.396, -1.57075, 0.349, 1.396, 0.2094],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.57075, -1.396, -1.47075, 0.549, 1.196, 0.3094],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.57075, -1.396, -1.37075, 0.749, 1.096, 0.4094],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.57075, -1.396, -1.77075, 0.949, 0.896, 0.5094],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.57075, -1.396, -1.97075, 1.349, 0.696, 0.1094],1.4,1.05,5)\n");

    cmd_str.push_back("movej([1.67075, -1.296, -1.97075, 1.349, 0.996, 0.3094],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.87075, -1.096, -1.97075, 1.349, 0.696, 0.4094],1.4,1.05,5)\n");
    cmd_str.push_back("movej([2.07075, -1.096, -1.77075, 1.349, 0.596, 0.5594],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.57075, -1.396, -1.97075, 1.349, 1.896, 0.2394],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.57075, -1.396, -1.97075, 1.349, 1.696, 0.1394],1.4,1.05,5)\n");

    

    double minX = -0.25 ;
    double maxX = 0.25 ;
    double minY = -1.3 ;
    double maxY = -0.8 ;
    double minZ = 0.9 ;
    double maxZ = 1.5 ;

    int count = 0;
    int totalNum = cmd_str.size();
    int num_ensured, num_ensured2;

    
    calibration_glasgow::TargetProcess srv1;
    srv1.request.doIt = true;

    ros::ServiceClient joint_state_client   = nh.serviceClient<ur_msgs::urscript>("ur_driver/URScript_srv");
    ros::ServiceClient targetProcess_client = nh.serviceClient<calibration_glasgow::TargetProcess>(TGT_PROCESS);

    ros::Subscriber joint_msg_sub = nh.subscribe("joint_states", 1, joint_msg_process);
    ros::Publisher trans_pub = nh.advertise<geometry_msgs::TransformStamped>(TRANFORM_SUB, 1);
    tf::TransformListener listener(ros::Duration(1.0));

    tf::StampedTransform transform;
    geometry_msgs::TransformStamped msg_trans;
    
    while (count<totalNum)
    {
        num_ensured2 = num_ensured = 4;
        joint_vels[0] = 0.1;
        ur_msgs::urscript srv;
        srv.request.script = cmd_str[count];
        while(!joint_state_client.call(srv));
        ROS_INFO("============================================================");
        ROS_INFO_STREAM("succeeded in calling the move robot service: num "<<++count);   

        sleep(1);//给机械臂启动的等待时间
        while(!checkJointState(joint_vels))//等机械臂停下来的时候再去listen
        {
            ros::spinOnce();
            rate.sleep();
        }
        
        while(num_ensured--)
        {
            try 
            {
                listener.waitForTransform(tool_link_, base_link_, ros::Time(0), ros::Duration(1) );//第二个参数到第一个参数的变换T
                listener.lookupTransform(tool_link_, base_link_, ros::Time(0), transform);
                tf::transformStampedTFToMsg(transform, msg_trans);
                trans_pub.publish(msg_trans);
                //ROS_INFO_STREAM("succeeded in publishing joint_states: num "<<5-num_ensured);
            } 
            catch (tf::TransformException ex) 
            {
                ROS_ERROR("%s",ex.what());     
            }            
            
        } 
        ROS_INFO_STREAM("succeeded in publishing joint_states: "<<num_ensured2<<" times");
       
        sleep(1);//给mainRoutine接收消息的等待时间    
        while(!targetProcess_client.call(srv1));
        ROS_INFO_STREAM("succeeded in call the process target service: num "<<count);
        //count++;    
           
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