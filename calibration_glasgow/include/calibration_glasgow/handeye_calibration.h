//
//  handEye_routines.h
//  Hand Eye Calibration
//
//  Created by Gerardo Aragon on 31/11/2012.
//  Copyright (c) 2012 Gerardo Aragon. All rights reserved.
//

#ifndef handeye_calibration_h
#define handeye_calibration_h

#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/Dense> 
#include <Eigen/StdVector>
#include <unsupported/Eigen/NonLinearOptimization>
#include <vector>
#include <cv.h>

using namespace std;
using namespace cv;
using namespace Eigen;
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d)//一句话的事。。。解决vector中Eigen对齐的问题
//#define EIGEN_DONT_VECTORIZE ;

class Chandeye
{

public:

    //Variables
    vector<Mat> bHg; // robot base 2 gripper
    vector<Mat> tHc; // calibration grid 2 camera
    vector<Mat> bHc; // base 2 camera (left or right)
    vector<Mat> bHct; // base to calibration target (either left or right chain)
    
    Mat gHc;
    Mat residuals;

    int M; // no of stations
    bool invH;

    //我添加的
    vector<Eigen::Affine3d> gripper2base;
    vector<Eigen::Affine3d> calib2cam;
    Eigen::Affine3d sensor_to_base; 
    Eigen::Affine3d sensor_to_gripper;

    //Chandeye(const char * filename); // For debug only
    Chandeye()
    {
        tHc.resize(0);
        bHg.resize(0);
    }

    ~Chandeye()
    {
        ROS_INFO("Closing hand eye routines");
    }

    // Rotation gripper, translation gripper, rotation camera, translation camera
    bool loadParameters(const vector<Mat>& rG_in, const vector<Mat>& tG_in,
                        const vector<Mat>& rC_in, const vector<Mat>& tC_in, bool invCamera = true);

    void calibrate();

    
    Mat avgTransformation(const vector<Mat>& H, bool decomposeQ);

    //我添加的
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void vector_Mat2Affine3d();
    Eigen::Affine3d Mat2Affine3d(Mat &A);

//从clopema_certh_ros中拿过来的

    Eigen::Matrix3d crossprod(const Eigen::Vector3d &a);

    void getDualQuaternion(const Eigen::Affine3d &A, Eigen::Quaterniond &q, Eigen::Quaterniond &qp);

    void sortStationMovements(std::vector<Eigen::Affine3d> &gripper_to_base, std::vector<Eigen::Affine3d> &target_to_sensor);

    enum HandEyeMethod { Horaud, Tsai, DualQuat } ;

    bool solveHandEyeFixed(const std::vector<Eigen::Affine3d> &gripper_to_base, const std::vector<Eigen::Affine3d> &target_to_sensor,
                    HandEyeMethod method, bool refine, Eigen::Affine3d &sensor_to_base ) ;

    bool solveHandEyeMoving(const std::vector<Eigen::Affine3d> &gripper_to_base, const std::vector<Eigen::Affine3d> &target_to_sensor,
                    HandEyeMethod method, bool refine, Eigen::Affine3d &sensor_to_gripper ) ;

    // Method by Horaud and Dornaika
    bool solveHandEyeLinearHD(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;

    // Method by R. Tsai
    bool solveHandEyeLinearTsai(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;

    // Method by K. Danielidis
    bool solveHandEyeLinearDualQuaternion(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;

    // minimize the frobenius norm of residual errors in rotation and translation (according to Dornaika and Horaud)
    // uses numerical differentiation for computing Jacobians
    bool solveHandEyeNonLinear(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X ) ;
   


private:

    //Functions
    Mat decompose_rotation(Mat R, Mat T);

    Mat rodrigues_custom(Mat R);
    
    Mat quat2rot(Mat q);

    Mat diagonal(float p);

    Mat skew(Mat V);

    Mat rot2quat(Mat R);

    Mat CrossProduct(Mat in);

    void printMatrix(Mat M, bool printType = true);

};

#endif
