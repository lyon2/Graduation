//
// Created by SchUser on 17/1/11.
//

#ifndef CARPLATE_CPLATELOCATE_H
#define CARPLATE_CPLATELOCATE_H
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;
using namespace cv::ml;

class CPlateLocate {
public:
    CPlateLocate();
    //svmCallback extractFeature;
    //! 车牌定位
    int plateLocate(Mat, vector<Mat>& resultVec);

    //! 车牌的尺寸验证
    bool verifySizes(RotatedRect mr);

    //! 结果车牌显示
    Mat showResultMat(Mat src, Size rect_size, Point2f center,int index);

    //! 设置与读取变量
    //...


protected:
    //! 高斯模糊所用变量
    int m_GaussianBlurSize;

    //! 连接操作所用变量
    int m_MorphSizeWidth;
public:
    int getM_GaussianBlurSize() const;

    void setM_GaussianBlurSize(int m_GaussianBlurSize);

    int getM_MorphSizeWidth() const;

    void setM_MorphSizeWidth(int m_MorphSizeWidth);

    int getM_MorphSizeHeight() const;

    void setM_MorphSizeHeight(int m_MorphSizeHeight);

    float getM_error() const;

    void setM_error(float m_error);

    float getM_aspect() const;

    void setM_aspect(float m_aspect);

    int getM_verifyMin() const;

    void setM_verifyMin(int m_verifyMin);

    int getM_verifyMax() const;

    void setM_verifyMax(int m_verifyMax);

    int getM_angle() const;

    void setM_angle(int m_angle);

    int getM_debug() const;

    void setM_debug(int m_debug);

    //! PlateLocate所用常量
    static const int DEFAULT_GAUSSIANBLUR_SIZE = 5;
    static const int SOBEL_SCALE = 1;
    static const int SOBEL_DELTA = 0;
    static const int SOBEL_DDEPTH = CV_16S;
    static const int SOBEL_X_WEIGHT = 1;
    static const int SOBEL_Y_WEIGHT = 0 ;
    static const int DEFAULT_MORPH_SIZE_WIDTH = 17;
    static const int DEFAULT_MORPH_SIZE_HEIGHT = 3;

    //! showResultMat所用常量
    static const int WIDTH = 136;
    static const int HEIGHT = 36;
    static const int TYPE = CV_8UC3;

    //! verifySize所用常量
    static const int DEFAULT_VERIFY_MIN = 3;
    static const int DEFAULT_VERIFY_MAX = 20;

    //! 角度判断所用常量
    static const int DEFAULT_ANGLE = 30;

    //! 是否开启调试模式常量，默认0代表关闭
    static const int DEFAULT_DEBUG = 0;

protected:
    int m_MorphSizeHeight;

    //! verifySize所用变量
    float m_error;
    float m_aspect;
    int m_verifyMin;
    int m_verifyMax;

    //! 角度判断所用变量
    int m_angle;

    //! 是否开启调试模式，0关闭，非0开启
    int m_debug;
};


#endif //CARPLATE_CPLATELOCATE_H
