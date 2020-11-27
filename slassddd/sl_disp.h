#pragma once

#include <stdlib.h>
#include "matd.h"
#include <math.h>
#include <stdio.h>
// extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
// #include "tag36h11.h"
// #include "tag25h9.h"
// #include "tag16h5.h"
// #include "tagCircle21h7.h"
// #include "tagCircle49h12.h"
// #include "tagCustom48h12.h"
// #include "tagStandard41h12.h"
// #include "tagStandard52h13.h"
// #include "common/getopt.h"
// }

// #include "apriltag.h"
// #include <iostream>

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

typedef struct DebugFlagFeimaTagStruct {
    bool basic_show;
    bool detail_func_process;
    bool imshow;
}DebugFlagFeimaTagStruct;
static struct DebugFlagFeimaTagStruct FeimaTagDebug = { true,false,true }; // Debug������

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
