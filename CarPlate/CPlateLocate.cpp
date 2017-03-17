//
// Created by SchUser on 17/1/11.
//

#include "CPlateLocate.h"
const float DEFAULT_ERROR = 0.6;
const float DEFAULT_ASPECT = 3.75;

CPlateLocate::CPlateLocate() {
    //cout << "CPlateLocate" << endl;
    m_GaussianBlurSize = DEFAULT_GAUSSIANBLUR_SIZE;
    m_MorphSizeWidth = DEFAULT_MORPH_SIZE_WIDTH;
    m_MorphSizeHeight = DEFAULT_MORPH_SIZE_HEIGHT;

    m_error = DEFAULT_ERROR;
    m_aspect = DEFAULT_ASPECT;
    m_verifyMin = DEFAULT_VERIFY_MIN;
    m_verifyMax = DEFAULT_VERIFY_MAX;

    m_angle = DEFAULT_ANGLE;

    m_debug = DEFAULT_DEBUG;

}

bool CPlateLocate::verifySizes(RotatedRect mr) {
    float error = m_error;
    //Spain car plate size: 52x11 aspect 4,7272
    //China car plate size: 440mm*140mm，aspect 3.142857
    float aspect = m_aspect;
    //Set a min and max area. All other patchs are discarded
    //int min= 1*aspect*1; // minimum area
    //int max= 2000*aspect*2000; // maximum area
    int min= 44*14*m_verifyMin; // minimum area
    int max= 44*14*m_verifyMax; // maximum area
    //Get only patchs that match to a respect ratio.
    float rmin= aspect-aspect*error;
    float rmax= aspect+aspect*error;

    int area= mr.size.height * mr.size.width;
    float r = (float)mr.size.width / (float)mr.size.height;
    if(r < 1)
    {
        r= (float)mr.size.height / (float)mr.size.width;
    }

    if(( area < min || area > max ) || ( r < rmin || r > rmax ))
    {
        return false;
    }
    else
    {
        return true;
    }
}

//Mat CPlateLocate::showResultMat(Mat src, Size rect_size, Point2f center) {
//
//}

//! 显示最终生成的车牌图像，便于判断是否成功进行了旋转。
Mat CPlateLocate::showResultMat(Mat src, Size rect_size, Point2f center, int index)
{
    Mat img_crop;
    getRectSubPix(src, rect_size, center, img_crop);

    if(m_debug)
    {
        stringstream ss(stringstream::in | stringstream::out);
        ss << "./tmp/debug_crop_" << index << ".jpg";
        cout<<ss.str()<<endl;
        imwrite(ss.str(), img_crop);
    }

    Mat resultResized;
    resultResized.create(HEIGHT, WIDTH, TYPE);

    resize(img_crop, resultResized, resultResized.size(), 0, 0, INTER_CUBIC);

    if(m_debug)
    {
        stringstream ss(stringstream::in | stringstream::out);
        ss << "./tmp/debug_resize_" << index << ".jpg";
        imwrite(ss.str(), resultResized);
    }

    return resultResized;
}

int CPlateLocate::plateLocate(Mat src, vector<Mat>& resultVec) {
    Mat out;
    GaussianBlur( src, out, Size( getM_GaussianBlurSize(), getM_GaussianBlurSize() ), 0, 0 );
    cvtColor( out, out, CV_RGB2GRAY );
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Mat dst;
    Sobel( out, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );
    Sobel( out, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );
    addWeighted( abs_grad_x, 1, abs_grad_y, 0, 0, dst);
    Mat img_threshold;
    threshold(dst, img_threshold, 0, 255, CV_THRESH_OTSU+CV_THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT, Size(getM_MorphSizeWidth(), getM_MorphSizeHeight()) );
    morphologyEx(img_threshold, img_threshold, MORPH_CLOSE, element);
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
        if( !verifySizes(mr))
        {
            itc = contours.erase(itc);
        }
        else
        {
            ++itc;
            rects.push_back(mr);
        }
    }
    drawContours(result, contours,
                 -1, // draw all contours
                 Scalar(0,0,255), // in blue
                 1); // with a thickness of 1




    //imshow("closeOperation",result);
    int k = 1;
    for(int i=0; i< rects.size(); i++)
    {
        RotatedRect minRect = rects[i];
        //cout<<verifySizes(minRect)<<endl;
        if(verifySizes(minRect))
        {
            // rotated rectangle drawing
            // Get rotation matrix
            // 旋转这部分代码确实可以将某些倾斜的车牌调整正，
            // 但是它也会误将更多正的车牌搞成倾斜！所以综合考虑，还是不使用这段代码。
            // 2014-08-14,由于新到的一批图片中发现有很多车牌是倾斜的，因此决定再次尝试
            // 这段代码。
            if(m_debug)
            {
                Point2f rect_points[4];
                minRect.points( rect_points );
                for( int j = 0; j < 4; j++ )
                    line( result, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,255), 1, 8 );
            }

            //imshow("rotate",result);

            float r = (float)minRect.size.width / (float)minRect.size.height;
            float angle = minRect.angle;
            Size2f rect_size = minRect.size;
            if (r < 1)
            {
                angle = 90 + angle;
                swap(rect_size.width, rect_size.height);
            }
            //cout<<angle<<endl;
            //如果抓取的方块旋转超过m_angle角度，则不是车牌，放弃处理
            if (angle - m_angle < 0 && angle + m_angle > 0)
            {
                //Create and rotate image
                Mat rotmat = getRotationMatrix2D(minRect.center, angle, 1);
                Mat img_rotated;
                warpAffine(src, img_rotated, rotmat, src.size(), CV_INTER_CUBIC);
                //imshow("rotate",img_rotated);
                Mat resultMat;

                resultMat = showResultMat(img_rotated, rect_size, minRect.center, k++);

                resultVec.push_back(resultMat);
            }
        }
    }


    waitKey(0);
    return 0;
}

int CPlateLocate::getM_GaussianBlurSize() const {
    return m_GaussianBlurSize;
}

void CPlateLocate::setM_GaussianBlurSize(int m_GaussianBlurSize) {
    CPlateLocate::m_GaussianBlurSize = m_GaussianBlurSize;
}

int CPlateLocate::getM_MorphSizeWidth() const {
    return m_MorphSizeWidth;
}

void CPlateLocate::setM_MorphSizeWidth(int m_MorphSizeWidth) {
    CPlateLocate::m_MorphSizeWidth = m_MorphSizeWidth;
}

int CPlateLocate::getM_MorphSizeHeight() const {
    return m_MorphSizeHeight;
}

void CPlateLocate::setM_MorphSizeHeight(int m_MorphSizeHeight) {
    CPlateLocate::m_MorphSizeHeight = m_MorphSizeHeight;
}

float CPlateLocate::getM_error() const {
    return m_error;
}

void CPlateLocate::setM_error(float m_error) {
    CPlateLocate::m_error = m_error;
}

float CPlateLocate::getM_aspect() const {
    return m_aspect;
}

void CPlateLocate::setM_aspect(float m_aspect) {
    CPlateLocate::m_aspect = m_aspect;
}

int CPlateLocate::getM_verifyMin() const {
    return m_verifyMin;
}

void CPlateLocate::setM_verifyMin(int m_verifyMin) {
    CPlateLocate::m_verifyMin = m_verifyMin;
}

int CPlateLocate::getM_verifyMax() const {
    return m_verifyMax;
}

void CPlateLocate::setM_verifyMax(int m_verifyMax) {
    CPlateLocate::m_verifyMax = m_verifyMax;
}

int CPlateLocate::getM_angle() const {
    return m_angle;
}

void CPlateLocate::setM_angle(int m_angle) {
    CPlateLocate::m_angle = m_angle;
}

int CPlateLocate::getM_debug() const {
    return m_debug;
}

void CPlateLocate::setM_debug(int m_debug) {
    CPlateLocate::m_debug = m_debug;
}
