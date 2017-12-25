#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "MarkerRecognizer.hpp"
using namespace cv;
using namespace std;

float f_x = 640.0f;
float f_y = 640.0f;
float c_x = 320.0f;
float c_y = 240.0f;

float camera_matrix[] =
{
    f_x, 0.0f, c_x,
    0.0f, f_y, c_y,
    0.0f, 0.0f, 1.0f
};
float dist_coeff[] = {0.0f, 0.0f, 0.0f, 0.0f};
MarkerRecognizer m_markerrecognizer;
int main() {
    VideoCapture cap(0);
    
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 280);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    
    Mat image;
    while (true) {
        
        cap>>image;
        Mat image_gray;
        cvtColor(image, image_gray, CV_BGR2GRAY);
        m_markerrecognizer.update(image_gray, 100, 10);
        m_markerrecognizer.drawToImage(image, Scalar(255,0,0), 2);
        //FileStorage fs("out_camera_data.xml", FileStorage::READ);
        Mat intrinsics =Mat(3, 3, CV_32FC1, camera_matrix);
        Mat distortion =Mat(1, 4, CV_32FC1, dist_coeff);
        //fs["Camera_Matrix"] >> intrinsics;
        //fs["Distortion_Coefficients"] >> distortion;
        cout << intrinsics.rows;
        vector<Point3f> objectPoints;
        objectPoints.push_back(Point3f(-1, 1, 0));
        objectPoints.push_back(Point3f(1, 1, 0));
        objectPoints.push_back(Point3f(1, -1, 0));
        objectPoints.push_back(Point3f(-1, -1, 0));
        Mat objectPointsMat(objectPoints);
        
        Mat rvec;
        Mat tvec;
        if(m_markerrecognizer.getMarkers().size()!=0)
        {
            for(int i=0;i<m_markerrecognizer.getMarkers().size();i++)
            {
        solvePnP(objectPointsMat, m_markerrecognizer.getMarkers()[i].m_corners, intrinsics, distortion, rvec, tvec);
        
        cout << "rvec: " << rvec << endl;
        cout << "tvec: " << tvec << endl;
        
        vector<Point3f> line3dx = {{0, 0, 0}, {1, 0, 0}};
        vector<Point3f> line3dy = {{0, 0, 0}, {0, 1, 0}};
        vector<Point3f> line3dz = {{0, 0, 0}, {0, 0, -1}};
        
        vector<Point2f> line2dx;
        vector<Point2f> line2dy;
        vector<Point2f> line2dz;
        projectPoints(line3dx, rvec, tvec, intrinsics, distortion, line2dx);
        projectPoints(line3dy, rvec, tvec, intrinsics, distortion, line2dy);
        projectPoints(line3dz, rvec, tvec, intrinsics, distortion, line2dz);
        
        
        line(image, line2dx[0], line2dx[1], Scalar(255,0,0),10);
        line(image, line2dy[0], line2dy[1], Scalar(0,255,0),10);
        line(image, line2dz[0], line2dz[1], Scalar(0,0,255),10);
            }
        }
        imshow("ss", image);
        int c = cv::waitKey(20);
        if((char)c == 'q')
        {
            break;
        }
        if(c >= 0 && c!= 'q')
        {
            cv::waitKey();
        }
        
    }
    
    
    return 0;
}
