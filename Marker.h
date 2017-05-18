//
// Created by liu on 2017/5/14.
//

#ifndef OPENCVTEST0_MARKER_H
#define OPENCVTEST0_MARKER_H

#include <opencv/cv.hpp>
using namespace std;
using namespace cv;
class Marker {
public:
    int id;
    Size markerSize;
    vector<Point2f>  corners;//specify four corners of a Marker
    Marker();
    Marker(Point2f corner0,Point2f corner1,Point2f corner2,Point2f corner3);
    void FixantiClockWise();

    vector<Point2f> marker_coordinate_2d;
};


#endif //OPENCVTEST0_MARKER_H
