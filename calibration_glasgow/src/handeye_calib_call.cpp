#include <calibration_glasgow/parameters.h>
#include <calibration_glasgow/handeye_calibration.h>
#include <calibration_glasgow/calibration_services.h>


//针对这个服务写的客户端
//he_calib_srv_ = nh_.advertiseService(HE_CALIB, &CmainCalibration::HandEyeCalibrationSrv, this);

void readCalibration(string child_frame, Eigen::Affine3d& cal)
{
  string calibFile;

  calibFile = ros::package::getPath("calibration_glasgow") + "/" + child_frame + ".calib";
  ROS_INFO("Calibration read from %s: ",calibFile.c_str());

  std::ifstream file;
  file.open((calibFile).c_str());

  string buffer;
  for (int i=0; i<4; i++)
  {
    for (int j=0; j<4; j++)
    {
      file >> buffer;
      cal(i,j) = stof(buffer);
    }          

  }

  file.close();
}


std::vector<float> computeEularAngles(Eigen::Matrix3d& R, bool israd)
{
	std::vector<float> result(3, 0);
	const float pi = 3.14159265397932384626433;
 
	float theta = 0, psi = 0, pfi = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN){ // abs(R(2, 0)) != 1
		float theta1 = -asin(R(2, 0));
		float theta2 = pi - theta1;
		float psi1 = atan2(R(2,1)/cos(theta1), R(2,2)/cos(theta1));
		float psi2 = atan2(R(2,0)/cos(theta2), R(2,2)/cos(theta2));
		float pfi1 = atan2(R(1,0)/cos(theta1), R(0,0)/cos(theta1));
		float pfi2 = atan2(R(1,0)/cos(theta2), R(0,0)/cos(theta2));
		theta = theta1;
		psi = psi1;
		pfi = pfi1;
	} else{
		float phi = 0;
		float delta = atan2(R(0,1), R(0,2));
		if (R(2, 0) > -1 - FLT_MIN && R(2, 0) < -1 + FLT_MIN){ // R(2,0) == -1
			theta = pi / 2;
			psi = phi + delta;
		} else{
			theta = -pi / 2;
			psi = -phi + delta;
		}
	}
 
	// psi is along x-axis, theta is along y-axis, pfi is along z axis
	if (israd){ // for rad 
		result[0] = psi;
		result[1] = theta;
		result[2] = pfi;
	} else{
		result[0] = psi * 180 / pi;
		result[1] = theta * 180 / pi;
		result[2] = pfi * 180 / pi;
	}
	return result;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hand_eye_calib_call");
    ros::NodeHandle nh;
    ros::ServiceClient handeye_calib_client = nh.serviceClient<calibration_glasgow::HandEyeCalibration>(HE_CALIB);
    calibration_glasgow::HandEyeCalibration srv;
    srv.request.doIt = true;
    ROS_INFO("Calling handeye calibration service...");
    while(!handeye_calib_client.call(srv));

    ROS_INFO("Service finished.Results are as followed: ");
    ROS_INFO_STREAM("status message: "<<srv.response.status_message);
    ROS_INFO_STREAM("success?  "<<srv.response.success);

    //读取标定文件内的矩阵变换数据,将4×4的transform转换成[x, y, z, Rx, Ry, Rz]
    //用于修改kinect2与ur3的相对位置
    //两个设备显示在rviz里更加清楚地知道标定的结果（的准确度）
    Eigen::Affine3d cam2rb;  
    readCalibration("camera2rb", cam2rb); //OK!
    cout<<cam2rb.matrix()<<endl;
    Eigen::Matrix3d r = cam2rb.rotation();
    Eigen::Vector3d t = cam2rb.translation();

    vector<float> angles = computeEularAngles(r, true);//由旋转矩阵变换成欧拉角
    cout<<"x, y, z分别为："<<"["<<t[0]<<" "<<t[1]<<" "<<t[2]<<"]"<<endl;
    cout<<"Rx, Ry, Rz分别为："<<"["<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<"]"<<endl;

    return 0;

}
