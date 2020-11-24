#include "sl_add.h"

// ���Ժ���
int testadd(int a, int b) {
    return a + b;
}
// ��ʾ��Ӧ����H
void disp_H(matd_t* H_disp, char spaceStrPerFrame3tab[]) {
    printf("%sH:\t[%15.4f%15.4f%15.4f\n", spaceStrPerFrame3tab, H_disp->data[0], H_disp->data[1], H_disp->data[2]);
    printf("%s  \t %15.4f%15.4f%15.4f\n", spaceStrPerFrame3tab, H_disp->data[3], H_disp->data[4], H_disp->data[5]);
    printf("%s  \t %15.4f%15.4f%15.4f]\n", spaceStrPerFrame3tab, H_disp->data[6], H_disp->data[7], H_disp->data[8]);
}

// ��ʾ R t
void disp_R_t(matd_t* R, matd_t* t, char spaceStrPerFrame3tab[]) {
    printf("%sR:\t%10.4f%10.4f%10.4f\t\t t:%12.3f\n", spaceStrPerFrame3tab, R->data[0], R->data[1], R->data[2], t->data[0]);
    printf("%s  \t%10.4f%10.4f%10.4f\t\t   %12.3f\n", spaceStrPerFrame3tab, R->data[3], R->data[4], R->data[5], t->data[1]);
    printf("%s  \t%10.4f%10.4f%10.4f\t\t   %12.3f\n", spaceStrPerFrame3tab, R->data[6], R->data[7], R->data[8], t->data[2]);
}

// ��ʾŷ����euler
void disp_euler(float rolld, float pitchd, float yawd, char spaceStrPerFrame3tab[]) {
    printf("%s(roll,pitch,yaw)deg :\t%12.4f%12.4f%12.4f\t\n", spaceStrPerFrame3tab, rolld, pitchd, yawd);
}

// DCMת��euler�� Z-Y-X˳��
void dcm2angle(float dcm[9], int type, float* rolld, float* pitchd, float* yawd) {
    // ��ת˳��zyx
    float r11 = dcm[2 - 1]; // r11 = m( 1,2 )
    float r12 = dcm[1 - 1]; // r12 = m( 1,1 )
    float r21 = -dcm[3 - 1]; // r21 =-m( 1,3 )
    float r31 = dcm[6 - 1]; // r31 = m( 2,3 )
    float r32 = dcm[9 - 1]; // r32 = m( 3,3 )

    *yawd = atan2(r11, r12)*180/M_PI;
    if (r21 < -1) {
        r21 = -1;
    }
    else if (r21 > 1) {
        r21 = 1;
    }
    *pitchd = asin(r21)*180/M_PI;
    *rolld = atan2(r31, r32)*180/M_PI;

    int pause = 0;
}

// ��ʾ apriltag_detection_t ��Ϣ
void disp_apriltag_detection_t(apriltag_detection_t* det, int idx, char spaceStr[]) {
    printf("%s tag %d:\t%s\t\tNumber( %d )\t\tHamming( %d )\t\tMargin( %.2f )\n", spaceStr, idx, det->family->name, det->id, det->hamming, det->decision_margin);
    disp_H(det->H,"");
}

// caculate frame rate
void cal_frame_rate()
{
    // static idx = 0;
    // idx ++;
    // utime_now(e)
}