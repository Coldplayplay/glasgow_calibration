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
    /*
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
*/
     //cmd_str.push_back("movej([0, -1.57075, 0, -1.57075, 0, 6.2831852],1.4,1.05,5)\n");
    
/*    
    cmd_str.push_back("movej([1.57271618897209,	-1.40952790391062,	-1.57428698529889,	0.227416401534861,	1.70064882314327,	6.139451807234347],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.55526289645215,	-1.33866753627965,	-1.55665915985374,	-0.152716309549504,	1.64410015537866,	6.16624824729597],1.4,1.05,5)\n");
    
    cmd_str.push_back("movej([1.40532438142728,	-1.52384696991625,	-1.36397481043357,	-0.266511776779534,	1.45967041641778,	6.05349997761713],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.9515,   -1.0125,   -1.8513,   -0.1229,    1.6139,    0.0290],1.4,1.05,5)\n");     
    cmd_str.push_back("movej([1.9515,   -0.9037,   -1.6963,   -0.1229,    1.4258,    0.0290],1.4,1.05,5)\n");

    cmd_str.push_back("movej([1.54304559168819,	-1.12922802604033,	-2.08392312688123,	0.530754625531476,	1.75457949702990,	6.26712827806124],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.54269652583779,	-1.11299646399678,	-1.88844625065786,	-0.110828407501640,	1.62245807265393,	6.26712827806124],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.68895511715491,	-1.06796696929533,	-1.96925499502520,	0.122871179340401,	1.23953283476637,	0.425162205785819],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.57707951210208,	-1.14999744413906,	-1.95441969638325,	0.267384441405531,	1.46171324854525,	0.559727091114582],1.4,1.05,5)\n");
    cmd_str.push_back("movej([1.58249003278326,	-1.00426245159754,	-1.88321026290188,	-0.351509311351658,	1.44687794990330,	0.522551578047102],1.4,1.05,5)\n");

    cmd_str.push_back("movej([1.53536614297941,	-1.03655104275943,	-1.89001704698466,	-0.267035375555132,	1.54409278923938,	0.360410490536829],1.4,1.05,5)\n");
*/    

   
    cmd_str.push_back("movej([-2.57271618897209,	-1.40952790391062,	-1.57428698529889,	0.227416401534861,	1.70064882314327,	6.139451807234347],1.4,1.05,5)\n");
    
    cmd_str.push_back("movej([-2.55526289645215,	-1.33866753627965,	-1.55665915985374,	-0.152716309549504,	1.64410015537866,	6.16624824729597],1.4,1.05,5)\n");
    
    cmd_str.push_back("movej([-2.40532438142728,	-1.52384696991625,	-1.36397481043357,	-0.266511776779534,	1.45967041641778,	6.05349997761713],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.9515,   -1.0125,   -1.8513,   -0.1229,    1.6139,    0.0290],1.4,1.05,5)\n");     
    //cmd_str.push_back("movej([-2.9515,   -0.9037,   -1.6963,   -0.1229,    1.4258,    0.0290],1.4,1.05,5)\n");

    cmd_str.push_back("movej([-2.54304559168819,	-1.12922802604033,	-2.08392312688123,	0.530754625531476,	1.75457949702990,	6.26712827806124],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.54269652583779,	-1.11299646399678,	-1.88844625065786,	-0.110828407501640,	1.62245807265393,	6.26712827806124],1.4,1.05,5)\n");
    
    //cmd_str.push_back("movej([-2.68895511715491,	-1.06796696929533,	-1.96925499502520,	0.122871179340401,	1.23953283476637,	0.325162205785819],1.4,1.05,5)\n");
    //cmd_str.push_back("movej([-2.57707951210208,	-1.14999744413906,	-1.95441969638325,	0.267384441405531,	1.46171324854525,	0.359727091114582],1.4,1.05,5)\n");
    //cmd_str.push_back("movej([-2.58249003278326,	-1.00426245159754,	-1.88321026290188,	-0.351509311351658,	1.44687794990330,	0.402551578047102],1.4,1.05,5)\n");

    cmd_str.push_back("movej([-2.72131732, -0.78382735, -1.76452784,  0.03211406,  1.70623385, 0.07068583],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.7185248 , -1.8947294 , -0.92048663,  0.25132741,  1.50377566, 0.06248279],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-1.73119206, -1.90485231, -0.87423541,  0.41434116,  0.45204027, 5.72712331],1.4,1.05,5)\n");
    
    
    cmd_str.push_back("movej([-2.53536614297941,	-1.03655104275943,	-1.89001704698466,	0.267035375555132,	1.54409278923938,	0.560410490536829],1.4,1.05,5)\n");
 


    


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
                listener.waitForTransform(base_link_, tool_link_, ros::Time(0), ros::Duration(1) );//第二个参数到第一个参数的变换T
                listener.lookupTransform(base_link_, tool_link_, ros::Time(0), transform);
                tf::transformStampedTFToMsg(transform, msg_trans);
                trans_pub.publish(msg_trans);
                /*
                tf::Quaternion a = transform.getRotation();
                tf::Vector3 b = transform.getOrigin();
                ROS_INFO_STREAM("tool02base:transition "<<b[0]<<" "<<b[1]<<" "<<b[2]);
                ROS_INFO_STREAM("tool02base:rotation "<<a[0]<<" "<<a[1]<<" "<<a[2]<<" "<<a[3]);
                std::cout<<std::endl;
                ROS_INFO_STREAM("succeeded in publishing joint_states: num "<<5-num_ensured);
                */
            } 
            catch (tf::TransformException ex) 
            {
                ROS_ERROR("%s",ex.what());     
            }            
            
        } 
        ROS_INFO_STREAM("succeeded in publishing joint_states: "<<num_ensured2<<" times");
        //ROS_INFO_STREAM("transform is:"<<transform);
       
        sleep(3);//给mainRoutine接收消息的等待时间    
        while(!targetProcess_client.call(srv1));
        ROS_INFO_STREAM("succeeded in call the process target service: num "<<count);
           
          
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
表示child_frame_id在frame_id下的位姿

lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, StampedTransform &transform) const
 	Get the transform between two frames by frame ID. 
表示source_frame相对于target_frame的位置和姿态
*/
