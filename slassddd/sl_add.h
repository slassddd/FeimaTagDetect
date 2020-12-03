#pragma once

// #include <stdlib.h>
// #include "matd.h"
// #include <math.h>
// #include <stdio.h>

extern "C" {
    #include "sl_disp.h"
}

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

typedef struct TagInfo{
    float Rcl[9]; // DCM = [dcm0 dcm1 dcm2;dcm3 dcm4 dcm5;dcm6 dcm7 dcm8],  V_camera = DCM*V_world
    float eulerdcl[3]; // tag coordinate to camera coordinate,(yawd,pitchd,rolld), rotate sequence ZYX
    float tcl[3]; // center of Tag(LandMark) w.r.t. center of camera, in camera coordinate
    float err;
    bool isDetect; // 
}TagInfo;
// Data received from flight controller
typedef struct UFeimaTagStruct {
    int timeGPS;
    float timeFlightController;
}UFeimaTagStruct;
// Data that need send to flight controller
typedef struct YFeimaTagStruct {
    float time;
    uint64 timegps;
    TagInfo Tags; // small tag
    TagInfo Tagm; // middle tag
    TagInfo Tagl; // large tag
    int nDetect; // num
    int frameRate; 
    int idxValid; // number of frames in which at least 1 tag is detected
    int idxInvalid;
    int idxAll;
    bool validFlag; //
    double timePerFrame;
    bool isCameraOpen;
}YFeimaTagStruct;

typedef struct TagConfig{
    // std::string familyNameTags;
    // std::string familyNameTagm;
    // std::string familyNameTagl;     
    std::string familyNameTags;
    std::string familyNameTagm;
    std::string familyNameTagl; 
    float sizeTags;
    float sizeTagm;
    float sizeTagl;
    int idTags;
    int idTagm;
    int idTagl;
}TagConfig;

typedef struct CameraConfig{
    float fx;
    float fy;
    float cx;
    float cy;
}CameraConfig;

void resetTagInfo(TagInfo* tagInfo);
void initYFeimaTagStruct(YFeimaTagStruct* FeimaTagOutput);
void CreateImageLegend(int clock_minute,int clock_sec,YFeimaTagStruct FeimaTagOutput, DebugFlagFeimaTagStruct FeimaTagDebug, cv::Mat frame);
void ShowTagImage(YFeimaTagStruct FeimaTagOutput, DebugFlagFeimaTagStruct FeimaTagDebug, cv::Mat frame);
void CreateImagePlot(apriltag_detection_t* det, DebugFlagFeimaTagStruct FeimaTagDebug,cv::Mat frame);
bool CalTagPOSE(apriltag_detection_t* det, TagConfig feimaTagConfig, apriltag_detection_info_t infoDetection, YFeimaTagStruct* FeimaTagOutput, TagInfo* tempTagInfo, apriltag_pose_t* pose);
bool ParseAlgoParamFile(string strSettingPath, apriltag_detector_t* td);
bool ParseParamFile(string strSettingPath,CameraConfig* feimaCameraConfig,TagConfig* tagConfig);