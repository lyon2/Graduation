#include <iostream>
#include "opencv2/opencv.hpp"
#include "CPlateLocate.h"
using namespace cv;



int main() {
    std::cout << "Hello, World!" << std::endl;
    CPlateLocate cPlateLocate=CPlateLocate();
    Mat in = imread("../img/3.jpg");
    cout<<CV_VERSION<<endl;
    cPlateLocate.setM_GaussianBlurSize(5);
    cPlateLocate.setM_MorphSizeWidth(17);
    cPlateLocate.setM_MorphSizeHeight(3);
    cPlateLocate.setM_debug(1);
    //imshow("source",in);
    //可能是车牌的图块集合
    vector<Mat> matVec;
    cPlateLocate.plateLocate(in,matVec);
    return 0;
}