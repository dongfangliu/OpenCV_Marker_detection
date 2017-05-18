#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv/cv.hpp>
#include "Marker.h"

typedef vector<vector<cv::Point>> contourVec;
void findContoursAndFilter(Mat& thresedImg,vector<vector<cv::Point>>& contoursOut,int ContourRetriMethod,int ContourApproxChoice,Point offset,size_t minSize,size_t maxSize);
float LengthOfPoly(const Marker& m);
void findMarkers(const contourVec& contours,vector<Marker>& finalMarkers) ;

void getMarkersWrapped(Mat& grayImg,vector<Marker>& markers);

Point2f * GetP2farr(vector<Point2f>& p2f);

using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
    if(argc != 2)
    {
        cout << "Usage: display_image ImageToLoadAndDisplay" << std::endl;
        return -1;
    }

    Mat input,thresoutput;
    contourVec contoursFound,contoursFiltered;
    vector<Marker> markersOut;
    input= imread(argv[1], 0); // Read the file
    if(input.empty()) // Check for invalid input
    {
        std::cout << "Could not open or find the img" << std::endl;
        return -1;
    }
    threshold(input,thresoutput,100,255,0); // do threshold ,set max_gray_scale as 100

    findContoursAndFilter(thresoutput,contoursFiltered,CV_RETR_LIST,CV_CHAIN_APPROX_NONE,Point(),200,4000);

    findMarkers(contoursFiltered,markersOut);
    getMarkersWrapped(thresoutput,markersOut);
//    Mat Contours_out(input.size(),CV_8U,Scalar(0));
//    drawContours( Contours_out,contoursFiltered,-1,Scalar(255),2);
   // namedWindow("Window",WINDOW_NORMAL); // Create a window for display.
    //imshow("Window", Contours_out); // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window
    return 0;
}
void getMarkersWrapped(Mat& grayImg,vector<Marker>& markers){
    Mat warppedMat;
    Point2f * dst = GetP2farr(markers[0].marker_coordinate_2d);
    int CellSize_Y= markers[0].markerSize.height/7;
    int CellSize_X =markers[0].markerSize.width/7;
    Point2f * src;
    for(int i = 0 ;i<markers.size();i++){
        src = GetP2farr(markers[i].corners);
//        Mat markerImage = grayImg.clone();
//        Mat markerSubImage = markerImage(boundingRect(markers[i].corners));
//        imwrite("Source marker" + to_string(i) + ".png",markerSubImage);
        Mat tranform_mat = getPerspectiveTransform(src,dst);
        warpPerspective(grayImg,warppedMat,tranform_mat,markers[i].markerSize); // 从透视图转换成正视图
//        imwrite("Marker " + to_string(i) + ".png",warppedMat);
        threshold(warppedMat,warppedMat,125,255,THRESH_BINARY|THRESH_OTSU);//对正视图做OTSU二值化
        Mat bit_matrix(5,5,CV_8UC1); //数值矩阵
        for(int y =0;y<7;y++){  //遍历黑色边框
            int step = (y==0||y==6)?1:6;//如果是第一行或者第6行，步长为1，否则设置为6
            int cell_y = y*CellSize_Y; //计算Y坐标
            for(int x = 0 ; x<7;x+=step){ //遍历X方向
                int cell_x = x*CellSize_X; //计算x坐标
                int count = countNonZero(warppedMat(Rect(cell_x,cell_y,CellSize_X,CellSize_Y)));//计算白色像素数
                if(count>CellSize_X*CellSize_Y/4){  //如果白色像素占了超过1/4
                    goto _wrongmarker;//跳过这次检测
                }
            }
        }
        //边框检测通过，遍历单位格，得到矩阵
        for (int y = 0; y < 5; ++y)
        {
            int cell_y = (y+1)*CellSize_Y;

            for (int x = 0; x < 5; ++x)
            {
                int cell_x = (x+1)*CellSize_X;
                int none_zero_count = countNonZero(warppedMat(Rect(cell_x, cell_y, CellSize_X, CellSize_Y)));
                if (none_zero_count > CellSize_X*CellSize_Y/2)
                    bit_matrix.at<uchar>(y, x) = 1;
                else
                    bit_matrix.at<uchar>(y, x) = 0;
            }
        }
        //输出矩阵
        cout<<bit_matrix<<endl;
        _wrongmarker:
        continue;
    }


}
void findMarkers(const contourVec& contours,vector<Marker>& finalMarkers) {
    vector<Point2f> approxCurve;
    vector<Marker> possibleMarkers;
    float minLengthBetweenPoints = 10;
    float minLengthOfSide = 1e10;
    vector<pair<int,int>> closeMarkerIndexPair;
    for(int i = 0 ; i<contours.size();i++){ //近似轮廓到多边形
        approxPolyDP(contours[i],approxCurve,contours[i].size()*0.05,true);
        if((!isContourConvex(approxCurve))||(approxCurve.size()!=4)){//如果不是四边形或不具有凸性，就不是一个合法的标记
            continue;
        }
        //筛去边长过短的四边形
        for(int j = 0 ; j<4;j++){
            Point2f x = approxCurve[j]-approxCurve[(j+1)%4];
            minLengthOfSide =std::min(minLengthOfSide,x.dot(x));
        }
        if(minLengthOfSide<minLengthBetweenPoints*minLengthBetweenPoints){
            continue;
        }
        //基本筛选完毕，添加进入marker，并修正四个顶点要逆时针排列
        Marker marker  =Marker(Point2f(approxCurve[0].x,approxCurve[0].y),Point2f(approxCurve[1].x,approxCurve[1].y),Point2f(approxCurve[2].x,approxCurve[2].y),Point2f(approxCurve[3].x,approxCurve[3].y));
        marker.FixantiClockWise();
        possibleMarkers.push_back(marker);
    }
    //筛选距离相近的四边形：筛去周长更短的四边形marker
    for(int i = 0 ; i<possibleMarkers.size();i++){
        for(int j = i+1 ;j<possibleMarkers.size();j++){
            float avg_distSquared = 0;
            for(int k = 0 ;k<4;k++){
                Point2f p = possibleMarkers[i].corners[k]-possibleMarkers[j].corners[k];
                avg_distSquared+=p.dot(p);
            }
            if(avg_distSquared/4 <100){
                closeMarkerIndexPair.push_back(pair<int,int>(i,j));
            }
        }
    }
    bool removeTable[possibleMarkers.size()]={false};
    for(int i = 0 ;i<closeMarkerIndexPair.size();i++){
        int index1 =closeMarkerIndexPair[i].first;
        int index2 =closeMarkerIndexPair[i].second;
        if(LengthOfPoly(possibleMarkers[index1])<LengthOfPoly(possibleMarkers[index2])){
            removeTable[index1]=true;
        }else{
            removeTable[index2]=true;
        }
    }
    for(int i = 0 ; i<possibleMarkers.size();i++){
        if(!removeTable[i]){
            finalMarkers.push_back(possibleMarkers[i]);
        }
    }
}

void findContoursAndFilter(Mat& thresedImg,contourVec& contoursOut,int ContourRetriMethod,int ContourApproxChoice,Point offset,size_t minSize,size_t maxSize){
    vector<vector<cv::Point>> contoursFound;
    findContours(thresedImg,contoursFound,ContourRetriMethod,ContourApproxChoice,offset); // Find the contours

    for(int i = 0 ;i<contoursFound.size();i++){     //filter out unnecessary contours
        if(contoursFound[i].size()>minSize&&contoursFound[i].size()<maxSize){
            contoursOut.push_back(contoursFound[i]);
        }
    }
}

float LengthOfPoly(const Marker& m){
    float sum=0,dist_x=0,dist_y=0;int i,j;
    for(i = 0 ;i<m.corners.size();i++){
        j =(i+1)%m.corners.size();
        dist_x=m.corners[i].x-m.corners[j].x;
        dist_y=m.corners[i].y-m.corners[j].y;
        sum+=sqrt(dist_x*dist_x+dist_y*dist_y);
    }
    return sum;
}

Point2f * GetP2farr(vector<Point2f>& p2f){
    Point2f * p = new Point2f[p2f.size()];
    for(int i = 0 ; i<p2f.size();i++){
        *(p+i) = p2f[i];
    }
    return p;
}
