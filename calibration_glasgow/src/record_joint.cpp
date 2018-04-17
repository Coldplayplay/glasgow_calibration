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

#include <iostream>  
#include <fstream>
#include <termios.h>  
#include <unistd.h>  
#include <fcntl.h> 

vector<double> joint_vels (6, 0.0) ;
vector<double> joint_pos (6, 0.0) ;

void joint_msg_process(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_vels = msg->velocity;
    joint_pos = msg->position;
    //ROS_INFO_STREAM("ur joint states received.");    
}

int kbhit(void)  
{  
      struct termios oldt, newt;  
      int ch;  
      int oldf;  
      tcgetattr(STDIN_FILENO, &oldt);  
      newt = oldt;  
      newt.c_lflag &= ~(ICANON | ECHO);  
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
      oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
      fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
      ch = getchar();  
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
      fcntl(STDIN_FILENO, F_SETFL, oldf);  
      if(ch != EOF)  
      {  
        ungetc(ch, stdin);  
        return 1;  
      }  
      return 0; 
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
    

    cmd_str.push_back("movej([0.59166661, -0.77370444, -1.14301611, -1.12399202, -0.60091685, 5.6168185],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.57271618897209,	-1.40952790391062,	-1.57428698529889,	0.227416401534861,	1.70064882314327,	6.139451807234347],1.4,1.05,5)\n");
    
    cmd_str.push_back("movej([-2.55526289645215,	-1.33866753627965,	-1.55665915985374,	-0.152716309549504,	1.64410015537866,	6.16624824729597],1.4,1.05,5)\n");
    
    cmd_str.push_back("movej([-2.40532438142728,	-1.52384696991625,	-1.36397481043357,	-0.266511776779534,	1.45967041641778,	6.05349997761713],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.9515,   -1.0125,   -1.8513,   -0.1229,    1.6139,    0.0290],1.4,1.05,5)\n");     
    //cmd_str.push_back("movej([-2.9515,   -0.9037,   -1.6963,   -0.1229,    1.4258,    0.0290],1.4,1.05,5)\n");

    cmd_str.push_back("movej([-2.54304559168819,	-1.12922802604033,	-2.08392312688123,	0.530754625531476,	1.75457949702990,	6.26712827806124],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.54269652583779,	-1.11299646399678,	-1.88844625065786,	-0.110828407501640,	1.62245807265393,	6.26712827806124],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.68895511715491,	-1.06796696929533,	-1.96925499502520,	0.122871179340401,	1.23953283476637,	0.425162205785819],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.57707951210208,	-1.14999744413906,	-1.95441969638325,	0.267384441405531,	1.46171324854525,	0.559727091114582],1.4,1.05,5)\n");
    cmd_str.push_back("movej([-2.58249003278326,	-1.00426245159754,	-1.88321026290188,	-0.351509311351658,	1.44687794990330,	0.522551578047102],1.4,1.05,5)\n");

    cmd_str.push_back("movej([-2.53536614297941,	-1.03655104275943,	-1.89001704698466,	-0.267035375555132,	1.54409278923938,	0.360410490536829],1.4,1.05,5)\n");
 


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
    

    ros::Subscriber joint_msg_sub = nh.subscribe("joint_states", 1, joint_msg_process);
    //ros::Publisher trans_pub = nh.advertise<geometry_msgs::TransformStamped>(TRANFORM_SUB, 1);
    //tf::TransformListener listener(ros::Duration(1.0));

    //tf::StampedTransform transform;
    //geometry_msgs::TransformStamped msg_trans;
    ofstream out("joint_states_record.txt");
    
    sleep(1);//给机械臂启动的等待时间
    ros::spinOnce();
    if(out.is_open())
    {
        
    }
    while(ros::ok())
    {
        out<<joint_pos[0]<<", ";
        out<<joint_pos[1]<<", ";
        out<<joint_pos[2]<<", ";
        out<<joint_pos[3]<<", ";
        out<<joint_pos[4]<<", ";
        out<<joint_pos[5];

        out<<"\n";
        cout<<"记录了第"<<++count<<"数据"<<endl;
        
    
        ros::spinOnce();
        rate.sleep();
    }
    out.close();
     
  

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
