#include "sl_add.h"

//
void resetTagInfo(TagInfo* tagInfo){
    for (int i = 0; i != 3; i++){
        tagInfo->tcl[i] = 0.0;
        tagInfo->eulerdcl[i] = 0.0;  
    }
    for (int i = 0; i != 9; i++){
        tagInfo->Rcl[i] = 0.0;  
    }
    tagInfo->err = 0.0;
    tagInfo->isDetect = false;
}
// 
void initYFeimaTagStruct(YFeimaTagStruct* FeimaTagOutput){
    resetTagInfo(&FeimaTagOutput->Tags);
    resetTagInfo(&FeimaTagOutput->Tagm);
    resetTagInfo(&FeimaTagOutput->Tagl);
    FeimaTagOutput->nDetect = 0;
    FeimaTagOutput->frameRate = 0;    
    FeimaTagOutput->idxValid = 0;
    FeimaTagOutput->idxInvalid = 0;
    FeimaTagOutput->validFlag = false;
    FeimaTagOutput->timePerFrame = 0.0;
    FeimaTagOutput->isCameraOpen = false;    
}

// Parse parameters from yaml file
bool ParseParamFile(string strSettingPath,CameraConfig* cameraConfig,TagConfig* tagConfig){
    bool isParamMiss = false;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    cv::FileNode node = fSettings["Camera.fx"];
    if(!node.empty() && node.isReal())
    {
        cameraConfig->fx = node.real();
    }
    else
    {
        std::cerr << "fx doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }    

    node = fSettings["Camera.fy"];
    if(!node.empty() && node.isReal())
    {
        cameraConfig->fy = node.real();
    }
    else
    {
        std::cerr << "fy doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["Camera.cx"];
    if(!node.empty() && node.isReal())
    {
        cameraConfig->cx = node.real();
    }
    else
    {
        std::cerr << "cx doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }    

    node = fSettings["Camera.cy"];
    if(!node.empty() && node.isReal())
    {
        cameraConfig->cy = node.real();
    }
    else
    {
        std::cerr << "cy doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }         

    //
    
    string familyNameTags = fSettings["Tag.familyNameTags"];
    tagConfig->familyNameTags = familyNameTags;
    // tagConfig->familyNameTags = new char[familyNameTags.size()+1];
    // strcpy(tagConfig->familyNameTags,familyNameTags.c_str());
    if(familyNameTags.size()==0){
        std::cerr << "familyNameTags doesn't exist or is not an string" << std::endl;
        isParamMiss = true;
    }

    string familyNameTagm = fSettings["Tag.familyNameTagm"];
    tagConfig->familyNameTagm = familyNameTagm;
    // tagConfig->familyNameTagm = new char[familyNameTagm.size()+1];
    // strcpy(tagConfig->familyNameTagm,familyNameTagm.c_str());
    if(familyNameTagm.size()==0){
        std::cerr << "familyNameTagm doesn't exist or is not an string" << std::endl;
        isParamMiss = true;
    }

    string familyNameTagl = fSettings["Tag.familyNameTagl"];
    tagConfig->familyNameTagl = familyNameTagl;
    // tagConfig->familyNameTagl = new char[familyNameTagl.size()+1];
    // strcpy(tagConfig->familyNameTagl,familyNameTagl.c_str());
    if(familyNameTagl.size()==0){
        std::cerr << "familyNameTagl doesn't exist or is not an string" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["Tag.sizeTags"];
    if(!node.empty() && node.isReal())
    {
        tagConfig->sizeTags = node.real();
    }
    else
    {
        std::cerr << "sizeTags doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["Tag.sizeTagm"];
    if(!node.empty() && node.isReal())
    {
        tagConfig->sizeTagm = node.real();
    }
    else
    {
        std::cerr << "sizeTagm doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["Tag.sizeTagl"];
    if(!node.empty() && node.isReal())
    {
        tagConfig->sizeTagl = node.real();
    }
    else
    {
        std::cerr << "sizeTagl doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["Tag.idTags"];
    if(!node.empty() && node.isInt())
    {
        tagConfig->idTags = node.operator int();
    }
    else
    {
        std::cerr << "idTags doesn't exist or is not an integer number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["Tag.idTagm"];
    if(!node.empty() && node.isInt())
    {
        tagConfig->idTagm = node.operator int();
    }
    else
    {
        std::cerr << "idTagm doesn't exist or is not an integer number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["Tag.idTagl"];
    if(!node.empty() && node.isInt())
    {
        tagConfig->idTagl = node.operator int();
    }
    else
    {
        std::cerr << "idTagl doesn't exist or is not an integer number" << std::endl;
        isParamMiss = true;
    }

    if(isParamMiss)
    {
        return false;
    }

    return true; 
}
//
void CreateImageLegend(int clock_minute,int clock_sec,YFeimaTagStruct FeimaTagOutput, DebugFlagFeimaTagStruct FeimaTagDebug, cv::Mat frame){
    if (FeimaTagDebug.imshow){
    int idxText = -1;
    int textSpaceVertical = 25;
    int textVertical0 = 20;
    int textHorizon0 = 10;
    Scalar textColor = Scalar(0, 255, 0);
    double textFontScale = 0.6;
    char imshow_time[40];
    char imshow_valid_frame[40];
    char imshow_framerate[40];
    char tags_tcl[40];
    char tagm_tcl[40];
    char tagl_tcl[40];

    idxText ++;
    sprintf(imshow_time,"%d : %d  (frame: %d)",clock_minute,clock_sec,FeimaTagOutput.idxValid + FeimaTagOutput.idxInvalid);
    putText(frame, imshow_time, Point(textHorizon0,textVertical0+idxText*textSpaceVertical),
            FONT_HERSHEY_SIMPLEX, textFontScale, textColor, 2);
    idxText ++;
    sprintf(imshow_valid_frame,"valid frame: %d / %d", FeimaTagOutput.idxValid, FeimaTagOutput.idxValid + FeimaTagOutput.idxInvalid);
    putText(frame, imshow_valid_frame, Point(textHorizon0,textVertical0+idxText*textSpaceVertical),
            FONT_HERSHEY_SIMPLEX, textFontScale, textColor, 2);
    idxText ++;
    sprintf(imshow_framerate,"frame rate: %d", FeimaTagOutput.frameRate);
    putText(frame, imshow_framerate, Point(textHorizon0,textVertical0+idxText*textSpaceVertical),
            FONT_HERSHEY_SIMPLEX, textFontScale, textColor, 2);
    idxText ++;
    idxText ++;
    if (FeimaTagOutput.Tags.isDetect){
        sprintf(tags_tcl,"small tag : %3.2f %3.2f %3.2f live", FeimaTagOutput.Tags.tcl[0], FeimaTagOutput.Tags.tcl[1], FeimaTagOutput.Tags.tcl[2]);
    }
    else {
        sprintf(tags_tcl,"small tag : %3.2f %3.2f %3.2f dead", FeimaTagOutput.Tags.tcl[0], FeimaTagOutput.Tags.tcl[1], FeimaTagOutput.Tags.tcl[2]);
    }
    putText(frame, tags_tcl, Point(textHorizon0,textVertical0+idxText*textSpaceVertical),
            FONT_HERSHEY_SIMPLEX, textFontScale, textColor, 2);

    idxText ++;
    idxText ++;
    if (FeimaTagOutput.Tagm.isDetect){
        sprintf(tagm_tcl,"middle tag: %3.2f %3.2f %3.2f live", FeimaTagOutput.Tagm.tcl[0], FeimaTagOutput.Tagm.tcl[1], FeimaTagOutput.Tagm.tcl[2]);
    }
    else {
        sprintf(tagm_tcl,"middle tag: %3.2f %3.2f %3.2f dead", FeimaTagOutput.Tagm.tcl[0], FeimaTagOutput.Tagm.tcl[1], FeimaTagOutput.Tagm.tcl[2]);
    }
    putText(frame, tagm_tcl, Point(textHorizon0,textVertical0+idxText*textSpaceVertical),
            FONT_HERSHEY_SIMPLEX, textFontScale, textColor, 2);      

    idxText ++;
    idxText ++;
    if (FeimaTagOutput.Tagl.isDetect){
        sprintf(tagl_tcl,"large tag : %3.2f %3.2f %3.2f live", FeimaTagOutput.Tagl.tcl[0], FeimaTagOutput.Tagl.tcl[1], FeimaTagOutput.Tagl.tcl[2]);
    }
    else {
        sprintf(tagl_tcl,"large tag : %3.2f %3.2f %3.2f dead", FeimaTagOutput.Tagl.tcl[0], FeimaTagOutput.Tagl.tcl[1], FeimaTagOutput.Tagl.tcl[2]);
    }
    putText(frame, tagl_tcl, Point(textHorizon0,textVertical0+idxText*textSpaceVertical),
            FONT_HERSHEY_SIMPLEX, textFontScale, textColor, 2);    
    }
}
//
void ShowTagImage(YFeimaTagStruct FeimaTagOutput, DebugFlagFeimaTagStruct FeimaTagDebug, cv::Mat frame){
    // if (FeimaTagDebug.imshow){
    //     imshow("image show",frame);
    //     int wait_msec;
    //     if (FeimaTagOutput.nDetect==0)
    //         wait_msec = 10;
    //     else
    //         wait_msec = 10;
            
    //     if (waitKey(wait_msec) >= 0)
    //         break;
    // }
}
//
void CreateImagePlot(apriltag_detection_t* det, DebugFlagFeimaTagStruct FeimaTagDebug,cv::Mat frame){
    if (FeimaTagDebug.imshow){
        line(frame, Point(det->p[0][0], det->p[0][1]),
            Point(det->p[1][0], det->p[1][1]),
            Scalar(0, 0xff, 0), 2);
        line(frame, Point(det->p[0][0], det->p[0][1]),
            Point(det->p[3][0], det->p[3][1]),
            Scalar(0, 0, 0xff), 2);
        line(frame, Point(det->p[1][0], det->p[1][1]),
            Point(det->p[2][0], det->p[2][1]),
            Scalar(0xff, 0, 0), 2);
        line(frame, Point(det->p[2][0], det->p[2][1]),
            Point(det->p[3][0], det->p[3][1]),
            Scalar(0xff, 0, 0), 2);
    }
}

//
bool CalTagPOSE(apriltag_detection_t* det, TagConfig feimaTagConfig, apriltag_detection_info_t infoDetection, YFeimaTagStruct* FeimaTagOutput, TagInfo* tempTagInfo, apriltag_pose_t* pose){
    bool isContinue = false;
    float yawd,pitchd,rolld;

    if (det->id == feimaTagConfig.idTags && !strcmp(det->family->name,feimaTagConfig.familyNameTags.c_str()))
    {
        FeimaTagOutput->Tags.isDetect = true;
        infoDetection.tagsize = feimaTagConfig.sizeTags; // 米
        FeimaTagOutput->Tags.err = estimate_tag_pose(&infoDetection, pose);
        
        for (int i = 0; i != 9; i++) {
            FeimaTagOutput->Tags.Rcl[i] = pose->R->data[i];
        }
        dcm2angle(FeimaTagOutput->Tags.Rcl, 321, &rolld, &pitchd, &yawd);
        FeimaTagOutput->Tags.eulerdcl[0] = yawd;
        FeimaTagOutput->Tags.eulerdcl[1] = pitchd;
        FeimaTagOutput->Tags.eulerdcl[2] = rolld;
        for (int i = 0; i != 3; i++){
            FeimaTagOutput->Tags.tcl[i] = pose->t->data[i];
        }                
        *tempTagInfo = FeimaTagOutput->Tags;
    }
    // calculate middle tag pose
    if (det->id == feimaTagConfig.idTagm && !strcmp(det->family->name,feimaTagConfig.familyNameTagm.c_str()))
    {
        FeimaTagOutput->Tagm.isDetect = true;
        infoDetection.tagsize = feimaTagConfig.sizeTagm; // 米
        FeimaTagOutput->Tagm.err = estimate_tag_pose(&infoDetection, pose);

        for (int i = 0; i != 9; i++) {
            FeimaTagOutput->Tagm.Rcl[i] = pose->R->data[i];
        }
        dcm2angle(FeimaTagOutput->Tagm.Rcl, 321, &rolld, &pitchd, &yawd);
        FeimaTagOutput->Tagm.eulerdcl[0] = yawd;
        FeimaTagOutput->Tagm.eulerdcl[1] = pitchd;
        FeimaTagOutput->Tagm.eulerdcl[2] = rolld;
        for (int i = 0; i != 3; i++){
            FeimaTagOutput->Tagm.tcl[i] = pose->t->data[i];
        }          
        *tempTagInfo = FeimaTagOutput->Tagm;      
    }
    // calculate large tag pose
    if (det->id == feimaTagConfig.idTagl && !strcmp(det->family->name,feimaTagConfig.familyNameTagl.c_str()))
    {
        FeimaTagOutput->Tagl.isDetect = true;
        infoDetection.tagsize = feimaTagConfig.sizeTagl; // 米
        FeimaTagOutput->Tagl.err = estimate_tag_pose(&infoDetection, pose);

        for (int i = 0; i != 9; i++) {
            FeimaTagOutput->Tagl.Rcl[i] = pose->R->data[i];
        }
        dcm2angle(FeimaTagOutput->Tagl.Rcl, 321, &rolld, &pitchd, &yawd);
        FeimaTagOutput->Tagl.eulerdcl[0] = yawd;
        FeimaTagOutput->Tagl.eulerdcl[1] = pitchd;
        FeimaTagOutput->Tagl.eulerdcl[2] = rolld;
        for (int i = 0; i != 3; i++){
            FeimaTagOutput->Tagl.tcl[i] = pose->t->data[i];
        }                
        *tempTagInfo = FeimaTagOutput->Tagl;
    }
}
// Parse algorithm parameters from yaml file
bool ParseAlgoParamFile(string strSettingPath, apriltag_detector_t* td){
    bool isParamMiss = false;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    cv::FileNode node;
    node = fSettings["AlgoParam.debug"];
    if(!node.empty() && node.isInt())
    {
        td->debug = node.operator int();
    }
    else
    {
        std::cerr << "AlgoParam.debug doesn't exist or is not an integer number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["AlgoParam.threads"];
    if(!node.empty() && node.isInt())
    {
        td->nthreads = node.operator int();
    }
    else
    {
        std::cerr << "AlgoParam.threads doesn't exist or is not an integer number" << std::endl;
        isParamMiss = true;
    }

    node = fSettings["AlgoParam.decimate"];
    if(!node.empty() && node.isReal())
    {
        td->quad_decimate = node.real();
    }
    else
    {
        std::cerr << "AlgoParam.decimate doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }  

    node = fSettings["AlgoParam.blur"];
    if(!node.empty() && node.isReal())
    {
        td->quad_sigma = node.real();
    }
    else
    {
        std::cerr << "AlgoParam.blur doesn't exist or is not an real number" << std::endl;
        isParamMiss = true;
    }    

    node = fSettings["AlgoParam.refine_edges"];
    if(!node.empty() && node.isInt())
    {
        td->refine_edges = node.operator int();
    }
    else
    {
        std::cerr << "AlgoParam.refine_edges doesn't exist or is not an integer number" << std::endl;
        isParamMiss = true;
    }    

    if(isParamMiss)
    {
        return false;
    }

    return true; 
}