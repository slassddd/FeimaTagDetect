#pragma once
#include <stdlib.h>
#include "matd.h"
#include <math.h>
#include <stdio.h>

#include "apriltag.h"
// #include "apriltag_pose.h"
// #include "tag36h11.h"
// #include "tag25h9.h"
// #include "tag16h5.h"
// #include "tagCircle21h7.h"
// #include "tagCircle49h12.h"
// #include "tagCustom48h12.h"
// #include "tagStandard41h12.h"
// #include "tagStandard52h13.h"
// #include "common/getopt.h"

// #include "common/sl_add.h"

// #include "apriltag.h"
//#include<iostream>
//#ifndef __cplusplus
//    #include<iostream>
//#endif

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

struct DebugFlagFeimaTagStruct {
    bool basic_show;
    bool detail_func_process;
    bool imshow;
};
static struct DebugFlagFeimaTagStruct FeimaTagDebug = { true,false,true }; // Debug������

struct TagInfo{
    float Rcl[9]; // DCM = [dcm0 dcm1 dcm2;dcm3 dcm4 dcm5;dcm6 dcm7 dcm8],  V_camera = DCM*V_world
    float eulerdcl[3]; // tag coordinate to camera coordinate,(yawd,pitchd,rolld), rotate sequence ZYX
    float tcl[3]; // center of Tag(LandMark) w.r.t. center of camera, in camera coordinate
    bool isDetect; // 
};
// Data received from flight controller
struct UFeimaTagStruct {
    int timeGPS;
    float timeFlightController;
};
// Data that need send to flight controller
struct YFeimaTagStruct {
    struct TagInfo Tags; // small tag
    struct TagInfo Tagm; // middle tag
    struct TagInfo Tagl; // large tag
    int nDetect; // num
    int frameRate; 
    int idxValid; // number of frames in which at least 1 tag is detected
    int idxInvalid;
    bool validFlag; //
    double timePerFrame;
    bool isCameraOpen;
};

struct TagConfig{
    char* familyNameTags;
    char* familyNameTagm;
    char* familyNameTagl; 
    float sizeTags;
    float sizeTagm;
    float sizeTagl;
    int idTags;
    int idTagm;
    int idTagl;
};

struct CameraConfig{
    float fx;
    float fy;
    float cx;
    float cy;
};

// char charTagCustom48h12[] = "tagCustom48h12";
#ifdef __cplusplus
extern "C" {
#endif
    void dcm2angle(float dcm[9], int type, float* rolld, float* pitchd, float* yawd);
    int testadd(int a, int b);
    void disp_H(matd_t* H_disp, char spaceStrPerFrame3tab[]);
    void disp_R_t(matd_t* R, matd_t* t, char spaceStrPerFrame3tab[]);
    void disp_euler(float roll, float pitch, float yaw, char spaceStrPerFrame3tab[]);
    void disp_apriltag_detection_t(apriltag_detection_t* det, int idx, char spaceStr[]);
    // void assignTagInfo(apriltag_pose_t pose, struct TagInfo& tempTagInfo);
    // void updateU(); // update input
    // void updateY(); // update output
#ifdef __cplusplus
}
#endif
