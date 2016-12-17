//#include "opencv/cv.h"
#include "OpenNI2/OpenNI.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "OniSampleUtilities.h"
#include "libfreenect.h"
//#include <stdio.h>
//#include <iostream>
//#include <string>

using namespace openni;
using namespace cv;
using namespace std;
#define MAX_DEPTH 10000
float posX = 0;
 float posY = 0;
 Mat cameraFeed(cv::Size(640, 480), CV_8UC3, NULL);
Mat HSV(cv::Size(640, 480), CV_8UC3, NULL);
Mat thresholdMat(cv::Size(640, 480), CV_8UC1, NULL);
cv::Mat depthcv1(cv::Size(640, 480), CV_16UC1, NULL);
IplImage*  Contours = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
IplImage*  drawingIpl = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
Mat coloredDepth;
int H_MIN = 0;
int H_MAX = 28;
int S_MIN = 133;
int S_MAX = 256;
int V_MIN = 130;
int V_MAX =256;
int counter = 0;
const string trackbarWindowName = "HSV ColorPicker";
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

void Objecttracking();
void createTrackbars();
void on_trackbar(int, void*);
void drawObject(int x, int y, Mat &frame);

void on_trackbar(int, void*)
{//This function gets called whenever a
	// trackbar position is changed



}

int main()
{
    createTrackbars();
    Device device;

    openni::VideoStream depth, color;
    openni::VideoFrameRef depthFrame, colorFrame;


    //NamedWindow("Camera", CV_WINDOW_AUTOSIZE);
    OpenNI::initialize();
    if (device.open(openni::ANY_DEVICE) != 0)
    {
        std::cout << "Kinect not found !" << endl;
        openni::OpenNI::getExtendedError();
        std::cout << "Press ESC to exit" << endl;

    }
    std::cout << "Kinect opened" << endl;

    color.create(device, SENSOR_COLOR);
    color.start();
    std::cout << "Camera ok" << endl;
    depth.create(device, SENSOR_DEPTH);
    depth.start();
    std::cout << "Depth sensor ok" << endl;

    openni::VideoMode video;
    video.setResolution(640, 480);
    video.setFps(30);
    video.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
    depth.setVideoMode(video);
    video.setPixelFormat(PIXEL_FORMAT_RGB888);
    color.setVideoMode(video);
    while(true)
    {

        color.readFrame(&colorFrame);
        depth.readFrame(&depthFrame);

        cameraFeed.data = (uchar*)colorFrame.getData();
        cv::Mat depthImage(depthFrame.getHeight(), depthFrame.getWidth(), CV_16U);
        depthImage.data = (uchar*)depthFrame.getData();
        Mat depthImage2;
        depthImage.convertTo(depthImage2, CV_8U, 0.1);

        cvtColor(cameraFeed, cameraFeed, CV_BGR2RGB);
        cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
        inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), thresholdMat);
        cvtColor(depthImage2, coloredDepth, CV_GRAY2RGB);
        Objecttracking();
        float x, y, z;
	//posX = posX + posX * 0.1;
	//posY = posY + posY * 0.1;
        float posZ = ((float)400 * ((float)depthImage2.at<uchar>(posY, posX)) / (float)255) - 40;
        int a = CoordinateConverter::convertDepthToWorld(depth, posX, posY, posZ * 10, &x, &y, &z);
        if(counter%10 == 0)
        {
            //std::cout << posZ << endl;
            std::cout << x << "   " << y << "   " << z << endl;
        }
        imshow("RGB", cameraFeed);
        cv::imshow("depth", coloredDepth);
        cv::imshow("Threshold", thresholdMat);
        cvtColor(cameraFeed, cameraFeed, CV_BGR2RGB);
        int key = cv::waitKey(50);
        if(key==27)
        {
            color.stop();
            color.destroy();
            depth.stop();
            depth.destroy();
            device.close();
            OpenNI::shutdown();
            break;
        }
        counter++;
    }

    color.stop();
    color.destroy();
    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();

     return 1;

}
void Objecttracking()
	{


		Mat temp;
		cvSet(drawingIpl, cvScalar(0));
		CvMoments *colorMoment = (CvMoments*)malloc(sizeof(CvMoments));
		static const int thickness = 3;
		static const int lineType = 8;
		Scalar           color = CV_RGB(255, 255, 255);

		temp = thresholdMat.clone();
		Contours->imageData = (char*)temp.data;


		CvMemStorage*   storage = cvCreateMemStorage(0);
		CvSeq*          contours = 0;
		CvSeq*			biggestContour = 0;
		int             numCont = 0;
		int             contAthresh = 45;

		cvFindContours(Contours, storage, &contours, sizeof(CvContour),
			CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

		int largest_area = 0;


		for (; contours != 0; contours = contours->h_next)
		{
			double a = cvContourArea(contours, CV_WHOLE_SEQ);

			if (a > largest_area){

				largest_area = a;

				biggestContour = contours;
			}


		}

		if (biggestContour != 0)
		{
			cvDrawContours(drawingIpl, biggestContour, color, color, -1, thickness, lineType, cvPoint(0, 0));
		}

			cvMoments(drawingIpl, colorMoment, 1);
			if (true)
			{

		double moment10 = cvGetSpatialMoment(colorMoment, 1, 0);
		double moment01 = cvGetSpatialMoment(colorMoment, 0, 1);
		double area = cvGetCentralMoment(colorMoment, 0, 0);
		if (area > 30)
		{

			posX = (moment10 / area);
			posY = moment01 / area;

			float xd = posX -320;
			float yd = posY -240;
			float xd2, yd2;
                 if(xd >= 0) 
		{
			xd2 = posX + ((xd * xd) * 0.0008);
		}
		else
		{
			//xd2 = posX;
			xd2 = posX - ((xd * xd) * 0.0004);
		}
 		if(yd >= 0) 
		{
			yd2 = posY + ((yd * yd) * 0.0008);
		}
		else
		{
			//xd2 = posX;
			yd2 = posY - ((yd * yd) * 0.0004);
		}
				
		if(xd2 >= 0 && xd2<= 640 && yd2 >= 0 && yd2 <= 480)
		{
            		drawObject(xd2, yd2, coloredDepth);
		}	
			drawObject(posX, posY, cameraFeed);
		posX = xd2;
		posY = yd2;
		}


			else
			{
				posX = 0;
				posY = 0;
				putText(cameraFeed, "No Object", Point(30, 450), 1, 1, Scalar(0, 255, 0), 2);

			}

	}
		else
		{
			posX = 0;
			posY = 0;
		}

}

void drawObject(int x, int y, Mat &frame)
{

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);


}

void createTrackbars()
{
		namedWindow(trackbarWindowName,0);

		char TrackbarName[100];
		printf(TrackbarName, "Hue_MIN", H_MIN);
		printf(TrackbarName, "Hue_MAX", H_MAX);
		printf(TrackbarName, "Saturation_MIN", S_MIN);
		printf(TrackbarName, "Saturation_MAX", S_MAX);
		printf(TrackbarName, "Value_MIN", V_MIN);
		printf(TrackbarName, "Value_MAX", V_MAX);


		createTrackbar("Hue_MIN", trackbarWindowName, &H_MIN, 256, on_trackbar);
		createTrackbar("Hue_MAX", trackbarWindowName, &H_MAX, 256, on_trackbar);
		createTrackbar("Saturation_MIN", trackbarWindowName, &S_MIN, 256, on_trackbar);
		createTrackbar("Saturation_MAX", trackbarWindowName, &S_MAX, 256, on_trackbar);
		createTrackbar("Value_MIN", trackbarWindowName, &V_MIN, 256, on_trackbar);
		createTrackbar("Value_MAX", trackbarWindowName, &V_MAX, 256, on_trackbar);
}
