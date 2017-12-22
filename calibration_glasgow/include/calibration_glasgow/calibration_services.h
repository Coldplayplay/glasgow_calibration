// main include file

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cvsba/cvsba.h>

#include <vector>
#include <errno.h>
#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>

#include <calibration_glasgow/HandEyeCalibration.h>
#include <cvsba/cvsba.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;
#define DEBUG true



