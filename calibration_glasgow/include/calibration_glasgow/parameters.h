
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <string>
#include <iostream>
#include <sstream>
#include <calibration_glasgow/kinect2_definitions.h>

static const char WINDOW_LEFT[] = "RGB camera image";

// Parameter server
static const char CALIB_TARGET[] = "/glasgow_calibration/target";
static const char WIDTH_MARKER[] = "/glasgow_calibration/marker_width";
static const char MARKER_SIZE_X[] = "/glasgow_calibration/marker_size_x";
static const char MARKER_SIZE_Y[] = "/glasgow_calibration/marker_size_y";
static const char MAX_ERROR_TH[] = "/glasgow_calibration/max_error";

static const char OUTPUT_IMAGE_DIR[] = "/glasgow_calibration/outputImageDir";
static const char OUTPUT_CALIB_DIR[] = "/glasgow_calibration/outputCalibDir";

static const char SAVE_MODE[] = "/glasgow_calibration/save_mode";
static const char INPUT_SCALE[] = "/glasgow_calibration/resize_imgs_factor";

static const char HE_CALIB_FILE_URL[] = "/glasgow_calibration/gHc_calibration_file";
static const char DEBUGQ[] = "/glasgow_calibration/debug";

// Messages
/*
static const char CAM_SUB[] = "/Image_for_Calibration";
static const char CAMERA_INFO[] = "Cam_info_for_Calibration";
static const char TRANFORM_SUB[] = "/transform_GrippertoBase";
*/
const std::string CAM_SUB1 = K2_DEFAULT_NS K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR;
const std::string CAM_SUB = "/" + CAM_SUB1;
const std::string CAMERA_INFO = CAM_SUB.substr(0, CAM_SUB.rfind('/')) + "/camera_info"; ;
static const char TRANFORM_SUB[] = "/transform_GrippertoBase";


//Services
static const char HE_CALIB[] = "/glasgow_calibration/HandEyeCalibration";

#endif
