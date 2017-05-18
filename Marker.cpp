//
// Created by liu on 2017/5/14.
//

#include "Marker.h"

Marker::Marker(Point2f corner0, Point2f corner1, Point2f corner2, Point2f corner3) :Marker(){
    corners.push_back(corner0);corners.push_back(corner1);corners.push_back(corner2);corners.push_back(corner3);
}

void Marker::FixantiClockWise() {
    Point2f v1 = corners[1] - corners[0];
    Point2f v2 = corners[2] - corners[0];
    /*行列式的几何意义是什么呢？有两个解释：一个解释是行列式就是行列式中的行或列向量所构成的超平行多面体的有向面积或有向体积；另一个解释是矩阵A的行列式detA就是线性变换A下的图形面积或体积的伸缩因子。
    //以行向量a=(a1,a2)，b=(b1,b2)为邻边的平行四边形的有向面积：若这个平行四边形是由向量沿逆时针方向转到b而得到的，面积取正值；若这个平行四边形是由向量a沿顺时针方向转到而得到的，面积取负值； */
    double o = (v1.x * v2.y) - (v1.y * v2.x);

    if (o < 0.0) //如果第三个点在左边，那么交换第一个点和第三个点，逆时针保存
        swap(corners[1], corners[3]);

}

Marker::Marker() {
    id = -1;
    markerSize=Size(100,100);
    marker_coordinate_2d.push_back(Point2f(0,0));
    marker_coordinate_2d.push_back(Point2f(markerSize.width-1,0));
    marker_coordinate_2d.push_back(Point2f(markerSize.width-1,markerSize.height-1));
    marker_coordinate_2d.push_back(Point2f(0,markerSize.height-1));
}
