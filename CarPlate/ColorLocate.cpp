//
// Created by SchUser on 17/1/16.
//
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//! 根据一幅图像与颜色模板获取对应的二值图
//! 输入RGB图像, 颜色模板（蓝色、黄色）
//! 输出灰度图（只有0和255两个值，255代表匹配，0代表不匹配）
enum Color {BLUE,YELLOW};


Mat colorMatch(const Mat& src, Mat& match, const Color r, const bool adaptive_minsv)
{
    // S和V的最小值由adaptive_minsv这个bool值判断
    // 如果为true，则最小值取决于H值，按比例衰减
    // 如果为false，则不再自适应，使用固定的最小值minabs_sv
    // 默认为false

    const float max_sv = 255;
    const float minref_sv = 64;

    const float minabs_sv = 95;

    //blue的H范围
    const int min_blue = 100;  //100
    const int max_blue = 140;  //140

    //yellow的H范围
    const int min_yellow = 15; //15
    const int max_yellow = 40; //40

    Mat src_hsv;
    // 转到HSV空间进行处理，颜色搜索主要使用的是H分量进行蓝色与黄色的匹配工作
    cvtColor(src, src_hsv, CV_BGR2HSV);

    vector<Mat> hsvSplit;
    split(src_hsv, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, src_hsv);

    //匹配模板基色,切换以查找想要的基色
    int min_h = 0;
    int max_h = 0;
    switch (r) {
        case BLUE:
            min_h = min_blue;
            max_h = max_blue;
            break;
        case YELLOW:
            min_h = min_yellow;
            max_h = max_yellow;
            break;
    }

    float diff_h = float((max_h - min_h) / 2);
    int avg_h = min_h + diff_h;

    int channels = src_hsv.channels();
    int nRows = src_hsv.rows;
    //图像数据列需要考虑通道数的影响；
    int nCols = src_hsv.cols * channels;

    if (src_hsv.isContinuous())//连续存储的数据，按一行处理
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i, j;
    uchar* p;
    float s_all = 0;
    float v_all = 0;
    float count = 0;
    for (i = 0; i < nRows; ++i)
    {
        p = src_hsv.ptr<uchar>(i);
        for (j = 0; j < nCols; j += 3)
        {
            int H = int(p[j]); //0-180
            int S = int(p[j + 1]);  //0-255
            int V = int(p[j + 2]);  //0-255

            s_all += S;
            v_all += V;
            count++;

            bool colorMatched = false;

            if (H > min_h && H < max_h)
            {
                int Hdiff = 0;
                if (H > avg_h)
                    Hdiff = H - avg_h;
                else
                    Hdiff = avg_h - H;

                float Hdiff_p = float(Hdiff) / diff_h;

                // S和V的最小值由adaptive_minsv这个bool值判断
                // 如果为true，则最小值取决于H值，按比例衰减
                // 如果为false，则不再自适应，使用固定的最小值minabs_sv
                float min_sv = 0;
                if (true == adaptive_minsv)
                    min_sv = minref_sv - minref_sv / 2 * (1 - Hdiff_p); // inref_sv - minref_sv / 2 * (1 - Hdiff_p)
                else
                    min_sv = minabs_sv; // add

                if ((S > min_sv && S < max_sv) && (V > min_sv && V < max_sv))
                    colorMatched = true;
            }

            if (colorMatched == true) {
                p[j] = 0; p[j + 1] = 0; p[j + 2] = 255;
            }
            else {
                p[j] = 0; p[j + 1] = 0; p[j + 2] = 0;
            }
        }
    }

    //cout << "avg_s:" << s_all / count << endl;
    //cout << "avg_v:" << v_all / count << endl;

    // 获取颜色匹配后的二值灰度图
    Mat src_grey;
    vector<Mat> hsvSplit_done;
    split(src_hsv, hsvSplit_done);
    src_grey = hsvSplit_done[2];

    match = src_grey;
    Mat img_threshold;
    Mat element = getStructuringElement(MORPH_RECT, Size(7, 7) );
    morphologyEx(match, img_threshold, MORPH_CLOSE, element);
    vector< vector< Point> > contours;
    findContours(img_threshold, contours, // a vector of contours
                 CV_RETR_EXTERNAL, // 提取外部轮廓
                 CV_CHAIN_APPROX_NONE); // all pixels of each contours
    Mat result;
    src.copyTo(result);
    //Start to iterate to each contour founded
    vector<vector<Point> >::iterator itc = contours.begin();

    vector<RotatedRect> rects;
    //Remove patch that are no inside limits of aspect ratio and area.
    int t = 0;
    while (itc != contours.end())
    {
        //Create bounding rect of object
        RotatedRect mr = minAreaRect(Mat(*itc));

        //large the rect for more
        if(mr.size.height<10)
        {
            itc = contours.erase(itc);
        }
        else
        {
            ++itc;
            rects.push_back(mr);
        }
    }
    cout<<rects.size()<<endl;
//    drawContours(result, contours,
//                 -1, // draw all contours
//                 Scalar(0,0,255), // in blue
//                 1); // with a thickness of 1
    Point2f rect_points[4];
    rects[0].points( rect_points );
    for( int j = 0; j < 4; j++ )
        line( result, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,255), 1, 8 );
    imshow("match",result);
    waitKey(0);
    //test(result,contours);
    return src_grey;
}

int main(){
    cout<<"color Locate"<<endl;
    Mat match;
    Mat src = imread("../img/1.jpg");
    colorMatch(src,match,BLUE, false);
    return 0;
}