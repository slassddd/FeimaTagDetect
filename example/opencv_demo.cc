/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>
#include "opencv2/opencv.hpp"

extern "C" {
    #include "apriltag.h"
    #include "apriltag_pose.h"
    #include "tag36h11.h"
    #include "tag25h9.h"
    #include "tag16h5.h"
    #include "tagCircle21h7.h"
    #include "tagCircle49h12.h"
    #include "tagCustom48h12.h"
    #include "tagStandard41h12.h"
    #include "tagStandard52h13.h"
    #include "common/getopt.h"

    #include "slassddd/sl_disp.h"
}
#include "slassddd/sl_add.h"

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    // Parse main argvs
    char* videoName;
    string configFileName;
    int min0 = 0, sec0 = 0;
    
    if (argc>=2){
        cout << "argc:" << argc << endl;
        for (int i = 0; i != argc; i++)
            cout << "argv[" << i << "]: " << argv[i] << endl;

        configFileName = string(argv[1]);
        videoName = argv[2];   
        if (argc==5){
            min0 = atoi(argv[3]);
            sec0 = atoi(argv[4]);                 
        }
    }
    else{
        // syn_tag36_11_00002.png  syn_tagCustom48h12_tagCustom48h12_tagCustom48h12.png v1 v1_1123 v_determine_axis v1000_april
        videoName = "./demo/v1000_april.avi";
        configFileName = "./configFiles/p640_480.yaml";
        min0 = 0;
        sec0 = 0;
    }

    // Load configurations
    struct TagConfig feimaTagConfig;
    struct CameraConfig feimaCameraConfig;
    bool isSuccess = ParseParamFile(configFileName,&feimaCameraConfig,&feimaTagConfig);
    if (!isSuccess){
        cout << "Fail to parse camera and tag parameters." << endl;
        return -1;
    }
    apriltag_detector_t* td = apriltag_detector_create();
    isSuccess = ParseAlgoParamFile(configFileName, td);
    if (!isSuccess){
        cout << "Fail to parse algorithm parameters." << endl;
        return -1;
    }
    struct YFeimaTagStruct FeimaTagOutput;// tag检测模块的输出结构体
    initYFeimaTagStruct(&FeimaTagOutput);

    struct UFeimaTagStruct FeimaTagInput;
    
    float duPerFrame = 0;
    int rtFrameRate = 0;

    int framerate;
    int framecount;
    VideoCapture cap(videoName);
    try
    {
        framerate = cap.get(CAP_PROP_FPS);
        framecount = cap.get(CV_CAP_PROP_FRAME_COUNT);
    }
    catch(const std::exception& e)
    {
        framerate = 30;
        framecount = 0;
        std::cerr << e.what() << '\n';
    }
    
    cout << "video name: " << videoName << "\t\t" << "fps (" << framerate << ")   " << "frame (" << framecount << ")" << endl;

    if (!cap.isOpened()){
        FeimaTagOutput.isCameraOpen = false;
        cout << "fail to open!" << endl;
    }
    else {
        FeimaTagOutput.isCameraOpen = true;
        cout << "success to open!" << endl;
    }
    // Initialize tag detector with options
    apriltag_family_t* tf_tagCustom48h12 = NULL;
    apriltag_family_t* tf_tag36h11 = NULL;

    tf_tagCustom48h12 = tagCustom48h12_create();
    tf_tag36h11 = tag36h11_create();    
    // 添加需要进行识别的Tag family，可以添加多个
    apriltag_detector_add_family(td, tf_tag36h11);
    apriltag_detector_add_family(td, tf_tagCustom48h12);

    Mat frame, gray;
    int frameidx = 0;

    int framestamps = 0;    
    int time_sec = 0;
    int clock_minute = 0;
    int clock_sec = 0;

    int time_sec_start = min0*60 + sec0;

    float yawd,pitchd,rolld;
    // 调试START
    if (FeimaTagDebug.basic_show) {
        apriltag_family_t* family;
        int nTagFamily = zarray_size(td->tag_families);
        cout << "Tag族数量: " << nTagFamily << endl;
        for (int famidx = 0; famidx < nTagFamily; famidx++) {
            zarray_get(td->tag_families, famidx, &family);
            cout << "\t 族 " << famidx << endl;
            cout << "\t\t 名称 :" << family->name << endl;
            cout << "\t\t 宽度 :" << family->total_width << endl;
        }        
        cout << "开始执行循环" << endl;
    }
    // 调试END    
    apriltag_detection_t* det;
    try
    {    
        while (true) {
            framestamps ++;  
            time_sec = framestamps/framerate;
            clock_minute = time_sec/60;
            clock_sec = time_sec%60;
            try
            {
                cap >> frame;
                if (frame.empty()){
                    cout << "第 " << framestamps << " 帧 empty. " << endl;
                    continue;
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            

            if (time_sec < time_sec_start){
                if (framestamps%(10*framerate) == 0)
                {
                    cout << "skip image: " << clock_minute << " min" << clock_sec << " sec\t\t";
                    cout << "( work after  " << min0 << " min" << sec0 << " sec )" << endl;
                }
                continue;
            }

            cvtColor(frame, gray, COLOR_BGR2GRAY); // 将原始图像转换为灰度图

            // Make an image_u8_t header for the Mat data
            image_u8_t im = { .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
            };

            // 调试START
            if (FeimaTagDebug.detail_func_process) {
                printf("frame %d  ( apriltag_detector_detect )\n",frameidx);
            }
            // 调试END    
            // ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
            // ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
            // ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||            
            // *****************************************************  核心函数  *******************************************************
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            zarray_t* detections = apriltag_detector_detect(td, &im);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            // ************************************************************************************************************************
            // ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
            // ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
            // ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

            FeimaTagOutput.timePerFrame = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            if(duPerFrame>1)
            {
                duPerFrame = 0.0;
                FeimaTagOutput.frameRate = rtFrameRate;
                rtFrameRate = 0;
            }
            else
            {
                rtFrameRate++;
                duPerFrame += FeimaTagOutput.timePerFrame;
            }
            FeimaTagOutput.nDetect = zarray_size(detections);
            if (FeimaTagOutput.nDetect>0){
                FeimaTagOutput.idxValid ++;
                FeimaTagOutput.validFlag = true;
            }
            else{
                FeimaTagOutput.idxInvalid ++;
                FeimaTagOutput.validFlag = false;
            }
            // 调试START
            char spaceStrPerFrame2tab[] = "\t\t ";
            char spaceStrPerFrame3tab[] = "\t\t\t ";
            frameidx++;
            if (FeimaTagDebug.basic_show) {
                cout << "\t 帧 " << frameidx << "\t( 帧率 " << FeimaTagOutput.frameRate << " )" <<  endl;
                cout << spaceStrPerFrame2tab << "检测到tag个数: " << FeimaTagOutput.nDetect << endl;
            }
            // 调试END         

            // First create an apriltag_detection_info_t struct using your known parameters.
            apriltag_detection_info_t infoDetection;

            infoDetection.fx = feimaCameraConfig.fx;
            infoDetection.fy = feimaCameraConfig.fy;
            infoDetection.cx = feimaCameraConfig.cx;
            infoDetection.cy = feimaCameraConfig.cy;

            FeimaTagOutput.Tags.isDetect = false;
            FeimaTagOutput.Tagm.isDetect = false;
            FeimaTagOutput.Tagl.isDetect = false;
            // Cal POSE
            for (int i = 0; i < FeimaTagOutput.nDetect; i++) {
                zarray_get(detections, i, &det);
                infoDetection.det = det;

                double err;
                TagInfo tempTagInfo;
                apriltag_pose_t pose;
                CalTagPOSE(det, feimaTagConfig, infoDetection, &FeimaTagOutput, &tempTagInfo, &pose);

                if ((!FeimaTagOutput.Tags.isDetect) && (!FeimaTagOutput.Tagm.isDetect) && (!FeimaTagOutput.Tagl.isDetect)){
                    cout << "unexpected tag is detected: " << det->family->name << " (" << det->id << ")" << endl;
                    continue;
                }

                if (FeimaTagDebug.basic_show) {
                    cout << spaceStrPerFrame2tab << "提取Tag检测结果 " << i + 1 << "/" << FeimaTagOutput.nDetect << ":\t" << det->family->name << "\t" << det->id << endl;
                    // disp_H(det->H, spaceStrPerFrame3tab);
                    disp_R_t(pose.R, pose.t, spaceStrPerFrame3tab);
                    disp_euler(tempTagInfo.eulerdcl[2], tempTagInfo.eulerdcl[1], tempTagInfo.eulerdcl[0], spaceStrPerFrame3tab);
                }
                // 图片标注
                CreateImagePlot(det, FeimaTagDebug, frame);      
            }
            apriltag_detections_destroy(detections);
            // 图片标注
            CreateImageLegend(clock_minute, clock_sec, FeimaTagOutput, FeimaTagDebug, frame);
            if (FeimaTagDebug.imshow){
                imshow("image show",frame);
                int wait_msec;
                if (FeimaTagOutput.nDetect==0)
                    wait_msec = 10;
                else
                    wait_msec = 10;
                    
                if (waitKey(wait_msec) >= 0)
                    break;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        int pause = 1;
    }
    apriltag_detector_destroy(td);

    tag36h11_destroy(tf_tag36h11);
    tagCustom48h12_destroy(tf_tagCustom48h12);

    return 0;
}