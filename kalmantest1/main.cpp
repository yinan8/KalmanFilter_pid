#include "kalmantest.h"
#include "pid.h"
#include<opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>

using namespace cv;
using namespace std;


Point2f g_predict_pt;

Mat measurement;
Mat control;

double g_runtime;

Point2f g_current_point;

void on_MouseHandle(int event, int x, int y, int flags, void* param);



int main()
{

    Point i;
    PIDcontrolor pid;
    Point2i p1, p2;





    Mat src_img(960, 1024, CV_8UC3, Scalar(255,255,255));
    RM_kalmanfilter kalman;
    Point2f kalman_point;



    while(1)
    {
        setMouseCallback("test", on_MouseHandle);

        g_runtime = ((double)getTickCount() - g_runtime) / getTickFrequency();

        kalman_point=kalman.point_Predict(g_runtime,g_current_point);

        p1 = g_current_point;  // 上一次的真实点

        p2 = kalman_point;


        i = pid.pid_Control_predictor(p1, p2);

        cout << i << endl;


        src_img.setTo(Scalar(255,255,255,0));
        circle(src_img,g_current_point,5,Scalar(0,0,255),3);    //predicted point with green
        circle(src_img,kalman_point,5,Scalar(255,0,0),3);
        circle(src_img,i,5,Scalar(0,255,0),3);

        cout<<"latest: ["<<g_current_point.x<<","<<g_current_point.y<<"]"<<endl;
        cout<<"predict: ["<<kalman_point.x<<","<<kalman_point.y<<"]"<<endl;
        cout << endl;

        imshow("test", src_img);
        int key=waitKey(10);
        if (key==27){//esc
            break;

    }}
}

void on_MouseHandle(int event, int x, int y, int flags, void* param)
{
    //鼠标移动消息
    if (event == EVENT_MOUSEMOVE)
    {
        g_current_point.x = x;
        g_current_point.y = 300;

    }

}

