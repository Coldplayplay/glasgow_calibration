#include <calibration_glasgow/parameters.h>
#include <calibration_glasgow/calibration_services.h>


//针对这个服务写的客户端
//he_calib_srv_ = nh_.advertiseService(HE_CALIB, &CmainCalibration::HandEyeCalibrationSrv, this);

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


    return 0;



}
