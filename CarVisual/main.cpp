#include <iostream>
#include <dirent.h>
#include "easypr.h"

namespace easypr{
    namespace demo {
        int readFileList(char *basePath,vector<string>& files)
        {
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

        vector<string> splitEx(const string& src, string separate_character)
        {
            vector<string> strs;

            int separate_characterLen = separate_character.size();//分割字符串的长度,这样就可以支持如“,,”多字符串的分隔符
            int lastPosition = 0,index = -1;
            while (-1 != (index = src.find(separate_character,lastPosition)))
            {
                strs.push_back(src.substr(lastPosition,index - lastPosition));
                lastPosition = index + separate_characterLen;
            }
            string lastString = src.substr(lastPosition);//截取最后一个分隔符后的内容
            if (!lastString.empty())
                strs.push_back(lastString);//如果最后一个分隔符后还有内容就入队
            return strs;
        }

        int simpletest() {

            cout << "test_plate_recognize" << endl;

            Mat src = imread("../image/8.jpg");

            CPlateRecognize pr;
            pr.setLifemode(true);
            pr.setDebug(false);
            pr.setMaxPlates(1);
            pr.setResultShow(true);
            pr.setDetectType(PR_DETECT_COLOR | PR_DETECT_SOBEL);
            pr.setDetectType(easypr::PR_DETECT_CMSER);

            //vector<string> plateVec;
            vector<CPlate> plateVec;

            int result = pr.plateRecognize(src, plateVec);
            //int result = pr.plateRecognizeAsText(src, plateVec);
            if (result == 0) {
                int temp=1;
                size_t num = plateVec.size();
                for (size_t j = 0; j < num; j++) {
                    cout << "plateRecognize: " << plateVec[j].getPlateStr() << endl;


                }

            }

            if (result != 0) cout << "result:" << result << endl;

            return result;
        }

        string copy2test(string s){
            string result="";
            int size = s.size();
            for(int i=3;i<size;i++){
                result+=s[i];
            }
            return result;
        }

        int test(string str) {

            cout << "test_plate_recognize" << endl;

            Mat src = imread(str);

            CPlateRecognize pr;
            pr.setLifemode(true);
            pr.setDebug(false);
            pr.setMaxPlates(4);
            pr.setResultShow(false);
            //pr.setDetectType(PR_DETECT_COLOR | PR_DETECT_SOBEL);
            pr.setDetectType(easypr::PR_DETECT_CMSER);

            //vector<string> plateVec;
            vector<CPlate> plateVec;

            int result = pr.plateRecognize(src, plateVec);
            //int result = pr.plateRecognizeAsText(src, plateVec);
            if (result == 0) {
                int temp=1;
                size_t num = plateVec.size();
                for (size_t j = 0; j < num; j++) {
                    cout << "plateRecognize: " << plateVec[j].getPlateStr() <<str<< endl;
                    vector<string> strs = splitEx(str, "/");
                    vector<string> ssts2 = splitEx(strs[2].c_str(),".");

                    vector<string> myresult = splitEx(plateVec[j].getPlateStr(),":");
                    if(myresult.size()>=2){

//                        string s1=copy2test(myresult[1].c_str());
//                        string s2=copy2test(ssts2[0].c_str());
                        string s1=myresult[1].c_str();
                        string s2=ssts2[0].c_str();
                        cout<<"test,"<<s1<<","<<s2<<endl;

                    if(strcmp(s1.c_str(),s2.c_str())==0){
                        temp=0;


                    }}
                }

                result=temp;
            }

            if (result != 0) cout << "result:" << result << endl;

            return result;
        }
    }

}

int main() {
    easypr::demo::simpletest();
//    easypr::demo::test("../image/3.jpg");
//    char *filePath = "../general_test";
//    vector<string> files;
//
//    easypr::demo::readFileList(filePath, files );
//    cout<<files.size()<<files[0].c_str()<<endl;
//    int right=0;
//    for(int i=0;i<files.size();i++){
//        if(easypr::demo::test(files[i].c_str())==0){
//            right++;
//        }
//    }
//    cout<<"共有256个样本"<<"，正确"<<right<<"个"<<endl;
    return 0;
}