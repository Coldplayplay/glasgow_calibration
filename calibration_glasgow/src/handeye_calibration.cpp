//
//  Hand-Eye Calibration class definition
//
//  Created by Gerardo Aragon on 03/2014.
//  Copyright (c) 2014 Gerardo Aragon. All rights reserved.

#include <calibration_glasgow/calibration_services.h>
#include <calibration_glasgow/handeye_calibration.h>


// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
};

struct SolverFunctor: public Functor<double>
{

    SolverFunctor(const std::vector<Eigen::Affine3d> &A_, const std::vector<Eigen::Affine3d> &B_):
        A(A_), B(B_) {}

  int operator()(const VectorXd &x, VectorXd &fvec) const
  {
      Eigen::Matrix4d X;

      for(int i=0, k=0 ; i<3 ; i++)
          for(int j=0 ; j<4 ; j++, k++)
          {
              X(i, j) = x[k] ;
          }

      int n = A.size() ;

      Eigen::Matrix3d Rx = X.block<3, 3>(0, 0) ;
      Eigen::Vector3d Tx = X.block<3, 1>(0, 3) ;

      int k = 0 ;

      for(int r=0 ; r<n ; r++ )
      {
          Eigen::Matrix3d Ra = A[r].rotation(), Rb = B[r].rotation() ;
          Eigen::Vector3d Ta = A[r].translation(), Tb = B[r].translation() ;

          Eigen::Matrix3d C = Ra * Rx - Rx * Rb ;
          Eigen::Vector3d S = Ra * Tx + Ta - Rx * Tb - Tx ;

          for(int i=0 ; i<3 ; i++ )
              for(int j=0 ; j<3 ; j++ )
                  fvec[k++] = C(i, j) ;

          for(int i=0 ; i<3 ; i++ )
              fvec[k++] = S[i] ;
      }

      const double cfactor = 1.0e6 ;

      Eigen::Matrix3d Cx = Rx.transpose() * Rx - Matrix3d::Identity() ;

      for(int i=0 ; i<3 ; i++ )
          for(int j=0 ; j<3 ; j++ )
              fvec[k++] = cfactor * Cx(i, j) ;

      return 0;
  }

  int inputs() const { return 12 ; }
  int values() const { return 12*A.size() + 9; } // number of constraints

  const std::vector<Eigen::Affine3d> &A,  &B ;

};

//补的几个函数放在最前面*************************来自wo
Eigen::Affine3d Chandeye::Mat2Affine3d(Mat &A)
{
    Eigen::Affine3d B;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            B(i,j) = A.at<float>(i,j);
    return B;
}
void Chandeye::vector_Mat2Affine3d()
{
    for(int i=0; i<bHg.size(); i++)
        gripper2base.push_back(Mat2Affine3d(bHg[i]));
    
    for(int i=0; i<tHc.size(); i++)
        calib2cam.push_back(Mat2Affine3d(tHc[i]));
        
}

//补的几个函数放在最前面*************************来自clopema_certh_ros
Eigen::Matrix3d Chandeye::crossprod(const Eigen::Vector3d &a)
{
    Matrix3d r ;
    r << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0 ;
    return r ;
}

void Chandeye::getDualQuaternion(const Eigen::Affine3d &A, Eigen::Quaterniond &q, Eigen::Quaterniond &qp)
{
    AngleAxisd rod(A.rotation()) ;

    double theta = rod.angle() ;
    double hc = cos(theta/2.0) ;
    double hs = sin(theta/2.0) ;
    Vector3d a = rod.axis() ;

    Eigen::Vector3d as = hs * a ;

    q = Quaterniond(hc, as.x(), as.y(), as.z()) ;

    Eigen::Vector3d t = A.translation() ;

    double qpw = -t.dot(as) / 2.0 ;

    Eigen::Vector3d qpv = (t.cross(as) + hc * t) / 2.0 ;

    qp = Quaterniond(qpw, qpv.x(), qpv.y(), qpv.z()) ;
}


void Chandeye::sortStationMovements(std::vector<Eigen::Affine3d> &gripper_to_base, std::vector<Eigen::Affine3d> &target_to_sensor)
{
    int nFrames = gripper_to_base.size() ;

    vector<Eigen::Affine3d> gripper_to_base_, target_to_sensor_ ;

    double dist ;

    int i=0 ;

    set<int> checked ;

    while ( 1 )
    {
        checked.insert(i) ;
        gripper_to_base_.push_back(gripper_to_base[i]) ;
        target_to_sensor_.push_back(target_to_sensor[i]) ;

        double maxAngle = 0.0 ;
        int bestj = -1 ;

        for( int j = 0 ; j<gripper_to_base.size() ; j++ )
        {
            if ( checked.count(j) == 0 )
            {
                Quaterniond qi(gripper_to_base[i].rotation()) ;
                Quaterniond qj(gripper_to_base[j].rotation()) ;

                double angle = qi.angularDistance(qj) ;

                if ( angle > maxAngle )
                {
                    maxAngle = angle ;
                    bestj = j ;
                }

            }
        }

        if ( bestj < 0 ) break ;

        i = bestj ;
    }


    gripper_to_base = gripper_to_base_ ;
    target_to_sensor = target_to_sensor_ ;

}

bool Chandeye::solveHandEyeFixed(const vector<Eigen::Affine3d> &gripper_to_base, const vector<Eigen::Affine3d> &target_to_sensor,
                  HandEyeMethod method, bool refine,
                  Eigen::Affine3d &sensor_to_base )
{
    vector<Eigen::Affine3d> A, B ;

    vector<Eigen::Affine3d> gripper_to_base_(gripper_to_base), target_to_sensor_(target_to_sensor) ;

    sortStationMovements(gripper_to_base_, target_to_sensor_) ;

    for( uint i=0 ; i<target_to_sensor_.size()-1 ; i++)
    {
        A.push_back(gripper_to_base_[i+1] * gripper_to_base_[i].inverse() ) ;
        B.push_back(target_to_sensor_[i+1] * target_to_sensor_[i].inverse() ) ;
    }

    bool res ;

    if ( method == Horaud )
        res = solveHandEyeLinearHD(A, B, sensor_to_base) ;
    else if ( method == Tsai )
        res = solveHandEyeLinearTsai(A, B, sensor_to_base) ;
    else
        res = solveHandEyeLinearDualQuaternion(A, B, sensor_to_base) ;

    if ( !res ) return false ;

    if ( refine )
        res = solveHandEyeNonLinear(A, B, sensor_to_base) ;

    return true ;

}

bool Chandeye::solveHandEyeMoving(const vector<Eigen::Affine3d> &gripper_to_base, const vector<Eigen::Affine3d> &target_to_sensor,
                  HandEyeMethod method, bool refine,
                  Eigen::Affine3d &sensor_to_gripper )
{
    vector<Eigen::Affine3d> A, B ;

    vector<Eigen::Affine3d> gripper_to_base_(gripper_to_base), target_to_sensor_(target_to_sensor) ;

    sortStationMovements(target_to_sensor_, gripper_to_base_) ;

    for( uint i=0 ; i<target_to_sensor_.size()-1 ; i++)
    {
        A.push_back(gripper_to_base_[i+1].inverse() * gripper_to_base_[i] ) ;
        B.push_back(target_to_sensor_[i+1] * target_to_sensor_[i].inverse() ) ;
    }

    bool res ;

    if ( method == Horaud )
        res = solveHandEyeLinearHD(A, B, sensor_to_gripper) ;
    else if ( method == Tsai )
        res = solveHandEyeLinearTsai(A, B, sensor_to_gripper) ;
    else
        res = solveHandEyeLinearDualQuaternion(A, B, sensor_to_gripper) ;

    if ( !res ) return false ;

    if ( refine )
        res = solveHandEyeNonLinear(A, B, sensor_to_gripper) ;

    return true ;

}

// Method by Horaud and Dornaika

bool Chandeye::solveHandEyeLinearHD(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X )
{
    assert(A.size() == B.size()) ;

    int n = A.size() ;

    // Solve for rotation

    MatrixXd A_(4*n, 4) ;

    for(int i=0 ; i<n ; i++)
    {
        Quaterniond qa, qb ;
        qa = Quaterniond(A[i].rotation()) ;
        qb = Quaterniond(B[i].rotation()) ;

        // compute  (Q-W)'(Q-W)

        Matrix4d Q, W, QW ;

        Q <<     qa.w(), -qa.x(), -qa.y(), -qa.z(),
                 qa.x(),  qa.w(), -qa.z(),  qa.y(),
                 qa.y(),  qa.z(),  qa.w(), -qa.x(),
                 qa.z(), -qa.y(),  qa.x(),  qa.w();

        W <<     qb.w(), -qb.x(), -qb.y(), -qb.z(),
                 qb.x(),  qb.w(),  qb.z(), -qb.y(),
                 qb.y(), -qb.z(),  qb.w(),  qb.x(),
                 qb.z(),  qb.y(), -qb.x(),  qb.w();

        A_.block<4, 4>(4*i, 0) = Q - W ;

    }

    // Perform SVD to find rotation

    JacobiSVD<MatrixXd> svd(A_, ComputeFullV) ;

    const MatrixXd V = svd.matrixV();

    Vector4d vq = V.col(3) ;

    Quaterniond q(vq[0], vq[1], vq[2], vq[3]) ;

    Matrix3d R = q.toRotationMatrix() ;

    // Solve for translation

    MatrixXd MA(3*n, 3) ;
    VectorXd MB(3*n) ;

    for(int i=0 ; i<n ; i++)
    {
        MA.block<3, 3>(3*i, 0) = A[i].rotation() - Matrix3d::Identity() ;
        MB.segment<3>(3*i) = R * B[i].translation() - A[i].translation() ;
    }

    Vector3d T = MA.jacobiSvd(ComputeThinU | ComputeThinV).solve(MB) ;

    X = Translation3d(T) * R ;

    return true ;
}

// Method by Tsai
bool Chandeye::solveHandEyeLinearTsai(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B, Eigen::Affine3d &X )
{
    assert(A.size() == B.size()) ;

    int n = A.size() ;

    assert(n>2) ;

    MatrixXd A_(3*n, 3) ;
    VectorXd B_(3*n) ;

    for(int i=0 ; i<n ; i++)
    {
        AngleAxisd rg(A[i].rotation()) ;
        AngleAxisd rc(B[i].rotation()) ;

        double theta_g = rg.angle() ;
        double theta_c = rc.angle() ;

        Vector3d rng = rg.axis() ;
        Vector3d rnc = rc.axis() ;

        Vector3d Pg = 2*sin(theta_g/2)*rng;
        Vector3d Pc = 2*sin(theta_c/2)*rnc;

        A_.block<3, 3>(3*i, 0) = crossprod(Pg + Pc);
        B_.segment<3>(3*i) = Pc - Pg;
    }

    // Solve problem with SVD

    JacobiSVD<MatrixXd> svdR(A_, ComputeThinU | ComputeThinV) ;

    // compute rotation

    VectorXd Pcg_prime = svdR.solve(B_) ;
    double err = (A_* Pcg_prime - B_).norm()/n ;

    VectorXd Pcg = 2*Pcg_prime/(sqrt(1+Pcg_prime.squaredNorm()));
    MatrixXd Rcg = (1-Pcg.squaredNorm()/2)*Matrix3d::Identity() +
            0.5*(Pcg*Pcg.adjoint() + sqrt(4 - Pcg.squaredNorm())*crossprod(Pcg));

    // compute translation

    for(int i=0 ; i<n ; i++)
    {
        A_.block<3, 3>(3*i, 0) = A[i].rotation() - Matrix3d::Identity() ;
        B_.segment<3>(3*i) = Rcg * B[i].translation() - A[i].translation() ;
    }

    JacobiSVD<MatrixXd> svdT(A_, ComputeThinU | ComputeThinV) ;

    VectorXd Tcg = svdT.solve(B_) ;
    err = (A_* Tcg - B_).norm()/n ;

    X = Translation3d(Tcg) * Rcg ;

    return true ;
}

// Method by K. Danielidis

bool Chandeye::solveHandEyeLinearDualQuaternion(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B,
                  Eigen::Affine3d &X )
{
    assert(A.size() == B.size()) ;

    int n = A.size() ;

    MatrixXd T(6*n, 8) ;

    for(int i=0 ; i<n ; i++)
    {
        Quaterniond qa, qb, qpa, qpb ;

        // compute dual quaternion representations

        getDualQuaternion(A[i], qa, qpa) ;
        getDualQuaternion(B[i], qb, qpb) ;

        // Form the A problem matrix (eq. 31)
        Vector3d s1 = qa.vec() - qb.vec() ;
        Matrix3d s2 = crossprod(qa.vec() + qb.vec()) ;
        Vector3d t1 = qpa.vec() - qpb.vec() ;
        Matrix3d t2 = crossprod(qpa.vec() + qpb.vec()) ;

        T.block<3, 1>(6*i, 0) = s1 ;
        T.block<3, 3>(6*i, 1) = s2 ;
        T.block<3, 1>(6*i, 4) = Vector3d::Zero() ;
        T.block<3, 3>(6*i, 5) = Matrix3d::Zero() ;

        T.block<3, 1>(6*i+3, 0) = t1 ;
        T.block<3, 3>(6*i+3, 1) = t2 ;
        T.block<3, 1>(6*i+3, 4) = s1 ;
        T.block<3, 3>(6*i+3, 5) = s2 ;
    }

    // Solve problem with SVD

    JacobiSVD<MatrixXd> svd(T, ComputeFullV) ;

    const MatrixXd V = svd.matrixV();
    const VectorXd S = svd.singularValues() ;

    // The last two singular values should be ideally zero

    const double singValThresh = 1.0e-1 ;

  //  if ( S[6] > singValThresh || S[7] > singValThresh ) return false ;

    // obtain right eigen-vectors spanning the solution space

    VectorXd v7 = V.col(6) ;
    VectorXd v8 = V.col(7) ;

    // find lambda1, lambda2 so that lambda1*v7 + lambda2*v8 = [q^T ;qprime^T]
    // Form the quadratic eq. 35 and solve it to obtain lamba1, lambda2

    Vector4d u1 = v7.segment<4>(0);
    Vector4d v1 = v7.segment<4>(4);

    Vector4d u2 = v8.segment<4>(0);
    Vector4d v2 = v8.segment<4>(4);

    double a = u1.dot(v1) ;
    double b = u1.dot(v2) + u2.dot(v1) ;
    double c = u2.dot(v2) ;

    // solve for s = lambda1/lambda2

    double det = sqrt(b * b - 4 * a * c) ;
    double s1 = (-b + det)/2/a ;
    double s2 = (-b - det)/2/a ;

    double a_ = u1.dot(u1) ;
    double b_ = u1.dot(u2) ;
    double c_ = u2.dot(u2) ;
    double s, val ;

    double val1 = s1 * s1 * a_ + 2 * s1 * b_ + c_ ;
    double val2 = s2 * s2 * a_ + 2 * s2 * b_ + c_ ;

    if ( val1 > val2 )  {
        s = s1 ;
        val = val1 ;
    }
    else  {
        s = s2 ;
        val = val2 ;
    }

    double lambda2 = sqrt(1/val) ;
    double lambda1 = s * lambda2 ;

    // compute the solution

    VectorXd sol = lambda1 * v7 + lambda2 * v8 ;

    Quaterniond q(sol[0], sol[1], sol[2], sol[3]) ;
    Quaterniond qp(sol[4], sol[5], sol[6], sol[7]) ;

    // obtain rotation and translation from dual quaternion

    Matrix3d rot = q.toRotationMatrix() ;
    Vector3d trans = 2 * (qp * q.conjugate()).vec() ;

    X = Translation3d(trans) * rot ;

    return true ;
}

bool Chandeye::solveHandEyeNonLinear(const std::vector<Eigen::Affine3d> &A, const std::vector<Eigen::Affine3d> &B,
                  Eigen::Affine3d &X )
{
    SolverFunctor functor(A, B);
    NumericalDiff<SolverFunctor> numDiff(functor);

    LevenbergMarquardt<NumericalDiff<SolverFunctor>, double> lm(numDiff);

    VectorXd Y(12) ;

    for(int i=0, k=0 ; i<3 ; i++)
        for(int j=0 ; j<4 ; j++, k++)
           Y[k] = X(i, j) ;

    LevenbergMarquardtSpace::Status status = lm.minimizeInit(Y);
    do {
        status = lm.minimizeOneStep(Y);
        double fnorm = lm.fvec.blueNorm();

    } while ( status == LevenbergMarquardtSpace::Running );

    for(int i=0, k=0 ; i<3 ; i++)
        for(int j=0 ; j<4 ; j++, k++)
            X(i, j) = Y[k] ;

    Matrix3d Rx = X.rotation() ;
    Vector3d Tx = X.translation() ;

   // double fnorm = lm.fvec.blueNorm();

    X = Translation3d(Tx) * Rx ;

    return true ;
}



// Rotation gripper, translation gripper, rotation camera, translation camera
bool Chandeye::loadParameters(const vector<Mat>& rG_in, const vector<Mat>& tG_in,
                    const vector<Mat>& rC_in, const vector<Mat>& tC_in, bool invCamera)
{
    invH = invCamera;
    ROS_INFO_STREAM("Size G: " << rG_in.size() << " and " << tG_in.size());
    ROS_INFO_STREAM("Size C: " << rC_in.size() << " and " << tC_in.size());
    if(rG_in.size() == tG_in.size())
        M = rG_in.size();
    else
        return false;

    ROS_INFO_STREAM("Number of stations: " << M);
    ROS_INFO("bHg，grabber to base");
    bHg.resize(0);
    // poses from robot base 2 gripper
    for(int i=0; i<M; i++)
    {
        Mat tempM = Mat::zeros(4, 4, CV_32F);
        Mat rTemp;

        tempM.at<float>(3,3) = 1;

        //Rodrigues(rG_in[i], rTemp);//这个应该是opencv自带的Rodrigues函数，为什么没有用。。。
        rTemp = rodrigues_custom(rG_in[i]);//应该是旋转向量变换成旋转矩阵，还没有验证具体函数内部跟opencv自带的Rodrigues一不一样。。。

        for(int ii = 0; ii < 3; ii++)
        {
            tempM.at<float>(ii,3) = tG_in[i].at<float>(ii);
            for(int jj = 0; jj < 3; jj++)
                tempM.at<float>(ii,jj) = rTemp.at<float>(ii,jj);
        }

        bHg.push_back(tempM);
        //bHg.push_back(tempM.inv());//tempM可以取逆看看。。
    }

    ROS_INFO("tHc, calib_board to camera");
    tHc.resize(0);
    // calibration grid 2 camera
    // solvePnP finds the transformation from camera to grid
    for(int i=0; i<M; i++)
    {
        Mat tempM = Mat::zeros(4, 4, CV_32F);
        Mat rTemp;

        tempM.at<float>(3,3) = 1;

        Mat forRod = Mat::zeros(1, 3, CV_32F);
        forRod.at<float>(0) = (float)rC_in[i].at<double>(0);
        forRod.at<float>(1) = (float)rC_in[i].at<double>(1);
        forRod.at<float>(2) = (float)rC_in[i].at<double>(2);
        rTemp = rodrigues_custom(forRod);

        for(int ii = 0; ii < 3; ii++)
        {
            tempM.at<float>(ii,3) = (float)tC_in[i].at<double>(ii);
            for(int jj = 0; jj < 3; jj++)
                tempM.at<float>(ii,jj) = rTemp.at<float>(ii,jj);
        }

        // inverse so it is camera to grid
        Mat invTemp;
        if(invCamera)
            invTemp = tempM.inv();
        else
            invTemp = tempM;

        tHc.push_back(invTemp);
    }

    ROS_INFO("Finish loading matrices for hand-eye calibration!!");

    return true;

}
void Chandeye::calibrate()//眼在手上的标定过程
{
    ROS_INFO("Entering hand-eye calibration...");
    int K = (M*M-M) / 2; // Number of unique camera position pairs
    Mat A = Mat::zeros(3*K,3, CV_32F); // will store: skew(Pgij+Pcij)
    Mat B = Mat::zeros(3*K,1,  CV_32F); // will store: Pcij - Pgij
    Mat A_trans = Mat::zeros(3*K,3, CV_32F);
    Mat B_trans = Mat::zeros(3*K,1,  CV_32F);
    int k = 0;

    // Now convert from tHc notation to Hc notation used in Tsai paper.
    vector<Mat> Hg = bHg;
    // Hc = cHw = inv(tHc); We do it in a loop because tHc is given, not cHw
    vector<Mat> Hc;
    Hc.resize(0);

    for(int i = 0; i < M; i++)
        Hc.push_back(tHc[i].inv());

    for(int i = 0; i < M; i++)
    {
        for(int j = i+1; j < M; j++)
        {
            Mat Hgij = Hg[j].inv() * Hg[i]; //Transformation from i-th to j-th gripper pose
            Mat Pgij = rot2quat(Hgij) * 2; //... and the corresponding quaternion

            Mat Hcij = Hc[j] * Hc[i].inv(); //Transformation from i-th to j-th camera pose
            Mat Pcij = rot2quat(Hcij) * 2; //... and the corresponding quaternion

            k += 1;
            // Form linear system of equations for rotational component
            Mat skewMat = skew(Pgij + Pcij);
            Mat tempSubstract = Pcij - Pgij;

            for(int ii = 1; ii <= 3; ii++)
            {
                B.at<float>((3*k-3) + ii - 1, 0) = tempSubstract.at<float>(ii - 1); //right-hand side
                for(int jj = 1; jj <= 3; jj++)
                    A.at<float>((3*k-3) + ii - 1, jj - 1) = skewMat.at<float>(ii - 1,jj - 1); //left-hand side
            }

        }
    }

    // Computing Rotation
    // Rotation from camera to gripper is obtained from the set of equations:
    //    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij
    // Gripper with camera is first moved to M different poses, then the gripper
    // .. and camera poses are obtained for all poses. The above equation uses
    // .. invariances present between each pair of i-th and j-th pose.
    Mat A_inv = A.inv(DECOMP_SVD);
    Mat Pcg_ = A_inv * B;

    // Obtained non-unit quaternin is scaled back to unit value that
    // .. designates camera-gripper rotation A.dot(B);
    Mat Pcg = 2 * Pcg_ / sqrt(1 + Pcg_.dot(Pcg_) );
    Mat Rcg = quat2rot(Pcg * 0.5); // Rotation matrix

    k = 0;
    Mat Rcg_rot = Rcg(Range(0, 3), Range(0, 3));
    for(int i = 0; i < M; i++)
    {
        for(int j = i+1; j < M; j++)
        {
            Mat Hgij = Hg[j].inv() * Hg[i]; //Transformation from i-th to j-th gripper pose
            Mat Hcij = Hc[j] * Hc[i].inv(); //Transformation from i-th to j-th camera pose

            k += 1;
            Mat Hgij_rot = Hgij(Range(0, 3), Range(0, 3));
            Mat tempT1 = Hgij_rot - Mat::eye(3,3,CV_32F);

            Mat Hcij_trans = Mat::zeros(3,1,CV_32F);
            Mat Hgij_trans = Mat::zeros(3,1,CV_32F);
            for(int ii = 0; ii < 3; ii++)
            {
                Hcij_trans.at<float>(ii) = Hcij.at<float>(ii,3);
                Hgij_trans.at<float>(ii) = Hgij.at<float>(ii,3);
            }

            Mat tempT2 = Rcg_rot * Hcij_trans - Hgij_trans;

            for(int ii = 1; ii <= 3; ii++)
            {
                B_trans.at<float>((3*k-3) + ii - 1, 0) = tempT2.at<float>(ii - 1); //right-hand side
                for(int jj = 1; jj <= 3; jj++)
                    A_trans.at<float>((3*k-3) + ii - 1, jj - 1) = tempT1.at<float>(ii - 1,jj - 1);
            }
        }
    }

    Mat A1_inv = A_trans.inv(DECOMP_SVD);
    Mat Tcg = A1_inv * B_trans;

    Mat transl = Mat::eye(4,4,CV_32F);
    transl.at<float>(0,3) = Tcg.at<float>(0);
    transl.at<float>(1,3) = Tcg.at<float>(1);
    transl.at<float>(2,3) = Tcg.at<float>(2);

    gHc = transl * Rcg; // incorporate translation with rotation

    ROS_INFO("gHc:");
    printMatrix(gHc, false);

    bHc.resize(0);
    bHct.resize(0);
    for(int i = 0; i < M; i++)
    {
        Mat bHc_temp = bHg[i] * gHc;
        Mat bHct_temp = bHg[i] * gHc * tHc[i].inv();

        // if(invH)
        //     Mat bHct_temp = bHg[i] * gHc * tHc[i].inv();
        // else
        //     Mat bHct_temp = bHg[i] * gHc * tHc[i];

        bHc.push_back(bHc_temp);
        bHct.push_back(bHct_temp);
    }

    //printMatrix(bHc[3], false);
    //printMatrix(bHct[3], false);

    return;

}

Mat Chandeye::avgTransformation(const vector<Mat>& H, bool decomposeQ)
{
    // Average orientation
    Mat tempR = Mat::zeros(3,3,CV_32F);
    for(int i=0; i < M; i++)
    {
        Mat R_add = Mat::zeros(3,3,CV_32F);
        for(int j = 0; j < 3; j++)
            for(int k = 0; k < 3; k++)
                R_add.at<float>(j,k) = H[i].at<float>(j,k);

        tempR = tempR + R_add;
    }

    Mat R_bar = tempR / (float)M;
    Mat RTR = R_bar.t() * R_bar;

    Mat D = Mat::eye(3,3,CV_32F);
    Mat V;
    Mat d, Vt;
    SVD::compute(RTR, d, V, Vt);

    Mat mul_temp = V * D * V.t();
    Mat Ravg = R_bar * mul_temp;

    // Average translations
    Mat Tavg_temp = Mat::zeros(3,1,CV_32F);
    Mat Tavg = Mat::zeros(3,1,CV_32F);
    for(int i = 0; i < M; i++)
    {
        Mat temp = Mat::zeros(3,1,CV_32F);
        for(int j = 0; j < 3; j++)
            temp.at<float>(j) = H[i].at<float>(j,3);

        Tavg_temp = Tavg_temp + temp;
    }

    Tavg_temp = Tavg_temp / M;

    // Do this so in xacro file x, y and z corresponds in order to avoid confussions
    Tavg.at<float>(0) = Tavg_temp.at<float>(0);
    Tavg.at<float>(1) = Tavg_temp.at<float>(1);
    Tavg.at<float>(2) = Tavg_temp.at<float>(2);

    // For debug
    /*Mat Havg = Mat::zeros(4,4,CV_32F);
    for(int i = 0; i < 3; i++)
    {
        Havg.at<float>(i,3) = Tavg.at<float>(i);
        for(int j = 0; j < 3; j++)
            Havg.at<float>(i,j) = Ravg.at<float>(i,j);
    }
    printMatrix(Havg,false);*/

    if(decomposeQ)
        return decompose_rotation(Ravg, Tavg);
    else
    {
        Mat Havg = Mat::zeros(4,4,CV_32F);
        for(int i = 0; i < 3; i++)
        {
            Havg.at<float>(i,3) = Tavg.at<float>(i);
            for(int j = 0; j < 3; j++)
                Havg.at<float>(i,j) = Ravg.at<float>(i,j);
        }
        Havg.at<float>(3,3) = 1;
        return Havg;
    }
}

// Private functions
Mat Chandeye::decompose_rotation(Mat R, Mat T)
{
    Mat vec = Mat::zeros(1,6,CV_32F);
    vec.at<float>(0) = atan2(R.at<float>(2,1), R.at<float>(2,2));
    vec.at<float>(1) = atan2(-1*R.at<float>(2,0), sqrt(R.at<float>(2,1) * R.at<float>(2,1) + R.at<float>(2,2) * R.at<float>(2,2)));
    vec.at<float>(2) = atan2(R.at<float>(1,0), R.at<float>(0,0));

    vec.at<float>(3) = T.at<float>(0);
    vec.at<float>(4) = T.at<float>(1);
    vec.at<float>(5) = T.at<float>(2);

    return vec;
}

Mat Chandeye::rodrigues_custom(Mat R)
{
    Mat wx = Mat::zeros(3,3,CV_32F);
    wx.at<float>(0,1) = -1*R.at<float>(2);
    wx.at<float>(0,2) = R.at<float>(1);
    wx.at<float>(1,0) = R.at<float>(2);
    wx.at<float>(1,2) = -1*R.at<float>(0);
    wx.at<float>(2,0) = -1*R.at<float>(1);
    wx.at<float>(2,1) = R.at<float>(0);

    float R1_norm = sqrt(double(pow(R.at<float>(0),2) + pow(R.at<float>(1),2) + pow(R.at<float>(2),2)));

    Mat first = wx * (sin(R1_norm)/R1_norm);
    float cte = (1-cos(R1_norm))/pow(R1_norm,2);
    Mat second = (wx * wx) * cte;
    Mat R2 = Mat::eye(3,3,CV_32F) + first + second;

    return R2;

}


Mat Chandeye::quat2rot(Mat q)
{
    //Mat q_tr = q.t();
    double p = q.dot(q);
    if(p > 1)
        ROS_ERROR("HANDEYE::quat2rot: quaternion greater than 1");

    float w = (float)sqrt(1 - p);
    Mat R = Mat::eye(4,4,CV_32F);

    Mat primero = q * q.t();
    Mat segundo = skew(q) * 2 * w;
    Mat tercero = diagonal((float)p) * 2;
    Mat tempR = (primero * 2) + segundo + Mat::eye(3,3,CV_32F) - tercero;

    for(int i = 0; i<3; i++)
        for(int j = 0; j<3; j++)
            R.at<float>(i,j) = tempR.at<float>(i,j);

    return R;
}

Mat Chandeye::diagonal(float p)
{
    Mat out = Mat::zeros(3,3,CV_32F);
    out.at<float>(0,0) = p;
    out.at<float>(1,1) = p;
    out.at<float>(2,2) = p;

    return out;
}

Mat Chandeye::skew(Mat V)
{
    Mat S = Mat::zeros(3,3, CV_32F);

    S.at<float>(0,1) = -1*V.at<float>(0,2);
    S.at<float>(0,2) = V.at<float>(0,1);
    S.at<float>(1,0) = V.at<float>(0,2);
    S.at<float>(1,2) = -1*V.at<float>(0,0);
    S.at<float>(2,0) = -1*V.at<float>(0,1);
    S.at<float>(2,1) = V.at<float>(0,0);

    return S;
}


Mat Chandeye::rot2quat(Mat R)
{
    Mat q = Mat::zeros(1,3, CV_32F);

    Mat R_temp(R, Range(0,3), Range(0,3));
    CvScalar t = trace(R_temp);
    float w4 = 2 * float(sqrt( 1.0 + t.val[0] )); // can this be imaginary?

    q.at<float>(0) = ( R.at<float>(2,1) - R.at<float>(1,2) ) / w4;
    q.at<float>(1) = ( R.at<float>(0,2) - R.at<float>(2,0) ) / w4;
    q.at<float>(2) = ( R.at<float>(1,0) - R.at<float>(0,1) ) / w4;

    return q;

}

Mat Chandeye::CrossProduct(Mat in)
{
    Mat out = Mat::zeros(3,3, CV_32F);

    out.at<float>(0,1) = -1*in.at<float>(2,0);
    out.at<float>(0,2) = in.at<float>(1,0);
    out.at<float>(1,0) = in.at<float>(2,0);
    out.at<float>(1,2) = -1*in.at<float>(0,0);
    out.at<float>(2,0) = -1*in.at<float>(1,0);
    out.at<float>(2,1) = in.at<float>(0,0);

    return out;
}

void Chandeye::printMatrix(Mat M, bool printType)
{
    if(printType)
        ROS_INFO_STREAM("Matrix type:" << M.type());
    // dont print empty matrices
    if (M.empty()){
        ROS_INFO("---");
        return;
    }
    // loop through columns and rows of the matrix
    for(int i=0; i < M.rows; i++){
        for(int j=0; j < M.cols ; j++){
            cout << M.at<float>(i,j) << "\t";
        }
        cout<<endl;
    }
    cout<<endl;
}
