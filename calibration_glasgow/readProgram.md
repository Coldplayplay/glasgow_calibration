
在calibration_services.cpp中
定义类CmainCalibration
===============================================================================================================================
1.CmainCalibration()
	2.void mainRoutine(const sensor_msgs::ImageConstPtr& imIn, const sensor_msgs::CameraInfoConstPtr& cam_info, const geometry_msgs::TransformStampedConstPtr& msg_trans)//是msg_trans提供了图片、相机信息和robot2grabber的变换信息
		3.processTarget()//做了最核心的工作：角点，收集每一张图片中标定板2相机的变换和robot2grabber的变换。即给rvecs_cam，tvecs_cam，rvecs_rb2gripper，tvecs_rb2gripper赋值
			

1.1 public数据成员：
================================================================================================================================
Calibration c_;
Chandeye hE_robot; // To find the transformation between calibration target and gripper
Chandeye hE_camera; // To find the transformation between camera and robot
tf::TransformListener tf_;

vector<vector<Point2f> > imagePoints;
vector<vector<Point2f> > points_handEye;
vector<Mat> rvecs_cam, tvecs_cam;
vector<Mat> rvecs_rb2gripper, tvecs_rb2gripper;

string ROBOT_GRIPPER;
string GRIPPER_LINK;
string ROBOT_BASE;

Mat cameraMatrix, distCoeffs;
Mat imLeft, cornersImgLeft;
bool found_chees;

cv_bridge::CvImagePtr cv_ptr;

tf::Quaternion transform_rotation;
tf::Vector3 transform_translation;

ros::ServiceServer process_target_srv_;
ros::ServiceServer he_calib_srv_;

bool debugQ;
int noImgPtu;
vector<Mat> bigMatQ, bigMatE;

1.2 private数据成员：
========================================================================================================================================
vector<Point2f> pointBuf;
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
typedef image_transport::SubscriberFilter ImageSubscriber;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::TransformStamped> syncPolicy;
ImageSubscriber im_sub_;
message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_;
message_filters::Subscriber<geometry_msgs::TransformStamped> trans_sub_;
message_filters::Synchronizer<syncPolicy> sync;


2.1 public函数成员：
=============================================================================================================================================
1.CmainCalibration() : it_(nh_), im_sub_(it_, CAM_SUB, 5), cam_info_sub_(nh_, CAMERA_INFO, 5),
        	      trans_sub_(nh_, TRANFORM_SUB, 5),  sync(syncPolicy(5), im_sub_, cam_info_sub_, trans_sub_)

2.~CmainCalibration()
3.void mainRoutine(const sensor_msgs::ImageConstPtr& imIn, 
		   const sensor_msgs::CameraInfoConstPtr& cam_info, 
		   const geometry_msgs::TransformStampedConstPtr& msg_trans)
4.bool HandEyeCalibrationSrv(calibration_glasgow::HandEyeCalibration::Request& req, 
			     calibration_glasgow::HandEyeCalibration::Response& rsp)//服务的回调函数

2.2 private函数成员：
=======================================================================================================================================================
1.bool isRotationMatrix(Mat &R)
2.Vec3f rotationMatrixToEulerAngles(Mat &R)
3.void processTarget()
4.void display_image(cv::Mat inImg)
5.Mat getCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
6.void saveCalibration(cv::Mat cal, string child_frame)
7.void printMatrix(Mat M, bool printType = true)
8.void saveImages(string str1, const Mat& imL)
9.Mat getRotation(double x, double y, double z, double w)
10.void savePose(double x, double y, double z, double qx, double qy, double qz, double qw, double yaw, double pitch, double roll)
==================================================important!!!!!
11.void runBA(vector<vector<Point2f> > imagePoints, Mat&  cameraMatrix_in, vector<Mat>& rvecs, vector<Mat>& tvecs)
12.Mat findCorners(const Mat& view)






