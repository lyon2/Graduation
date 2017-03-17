//
// Created by SchUser on 17/1/15.
//

// SVMTrain.cpp : 定义控制台应用程序的入口点。

#include <iostream>
#include "opencv2/opencv.hpp"

#include <dirent.h>
#include <unistd.h>
using namespace std;
using namespace cv;
using namespace cv::ml;

int readFileList(char *basePath,vector<string>& files)
{
    XMLNode::openFileHelper
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }
    string result;
    char temp[100];
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8) {   ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            sprintf(temp, "%s/%s", basePath, ptr->d_name);
            //cout<<temp<<endl;
            result = temp;
            files.push_back(result);
            //cout<<result<<endl;
        }
        else if(ptr->d_type == 10)    ///link file
            printf("d_name:%s/%s\n",basePath,ptr->d_name);
        else if(ptr->d_type == 4)    ///dir
        {
            memset(base,'\0',sizeof(base));
            strcpy(base,basePath);
            strcat(base,"/");
            strcat(base,ptr->d_name);
            readFileList(base,files);
        }
    }
    closedir(dir);
    return 1;
}



void getPlate(Mat& trainingImages, vector<int>& trainingLabels)
{

    char * filePath = "../img/HasPlate";
    vector<string> files;

    ////获取该路径下的所有文件
    //getFiles(filePath, files );
    readFileList(filePath,files);

    int size = files.size();
    if (0 == size)
        cout << "No File Found in train HasPlate!" << endl;

    for (int i = 0;i < size;i++)
    {
        cout << files[i].c_str() << endl;
        Mat img = imread(files[i].c_str());
        //img= img.reshape(1, 1);
        img= img.reshape(1, 1);
        trainingImages.push_back(img);
        trainingLabels.push_back(1);
    }
}

void getNoPlate(Mat& trainingImages, vector<int>& trainingLabels)
{

    char * filePath = "../img/NoPlate";
    vector<string> files;

    ////获取该路径下的所有文件
    //getFiles(filePath, files );
    readFileList(filePath,files);
    int size = files.size();
    if (0 == size)
        cout << "No File Found in train NoPlate!" << endl;

    for (int i = 0;i < size;i++)
    {
        cout << files[i].c_str() << endl;
        Mat img = imread(files[i].c_str());
        //img= img.reshape(1, 1);
        img= img.reshape(1, 1);
        trainingImages.push_back(img);
        trainingLabels.push_back(0);
    }
}


int main()
{
//    vector<string> files;
//    readFileList("../img/NoPlate",files);
//    cout<<files.size()<<files[2]<<endl;


    Mat classes;//(numPlates+numNoPlates, 1, CV_32FC1);
    Mat trainingData;//(numPlates+numNoPlates, imageWidth*imageHeight, CV_32FC1 );

    Mat trainingImages;
    vector<int> trainingLabels;

    getPlate(trainingImages, trainingLabels);
    getNoPlate(trainingImages, trainingLabels);

    Mat(trainingImages).copyTo(trainingData);
    trainingData.convertTo(trainingData, CV_32FC1);
    Mat(trainingLabels).copyTo(classes);

//    CvSVMParams SVM_params;
//    SVM_params.svm_type = CvSVM::C_SVC;
//    SVM_params.kernel_type = CvSVM::LINEAR; //CvSVM::LINEAR;
//    SVM_params.degree = 0;
//    SVM_params.gamma = 1;
//    SVM_params.coef0 = 0;
//    SVM_params.C = 1;
//    SVM_params.nu = 0;
//    SVM_params.p = 0;
//    SVM_params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 1000, 0.01);

    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setDegree(0);
    svm->setGamma(1);
    svm->setCoef0(0);
    svm->setC(1);
    svm->setNu(0);
    svm->setP(0);
    svm->setTermCriteria(cvTermCriteria(CV_TERMCRIT_ITER, 1000, 0.01));
    //Train SVM

    cout << "Begin to generate svm" << endl;
    //CvSVM svmClassifier(trainingData, classes, Mat(), Mat(), SVM_params);
    svm->train(trainingData,0,classes);
    svm->trainAuto(trainingData,classes);
    cout << "Svm generate done!" << endl;

    //FileStorage fsTo("train/svm.xml", cv::FileStorage::WRITE);
    //svmClassifier.write(*fsTo, "svm");
    svm->save("../train/svm.xml");
    int a;
    cin >> a;

    return 0;
}

