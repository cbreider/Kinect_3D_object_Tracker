//#include "opencv/cv.h"
#include "OpenNI2/OpenNI.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "OniSampleUtilities.h"
#include "libfreenect.h"

using namespace openni;
using namespace cv;
using namespace std;
#define MAX_DEPTH 10000

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size,chunk_size) * (chunk_size))

#define TEXTURE_SIZE	512
Mat dst, detected_edges;
int edgeThresh = 1;
int lowThreshold =1 ;
int highThreshold =100 ;
int const max_lowThreshold = 1000;
int ratio = 3;
int kernel_size = 3;
        Mat gray;
             Mat cameraFeed(cv::Size(640, 480), CV_8UC3, NULL);
char* window_name = "Edge Map";
void CannyThreshold()
{
  /// Reduce noise with a kernel 3x3
  blur( gray, detected_edges, Size(3,3) );
 lowThreshold = getTrackbarPos("Min Threshold:", window_name);
 highThreshold = getTrackbarPos("Max Threshold:", window_name);
  /// Canny detector
  Canny( gray, detected_edges, lowThreshold, highThreshold);

  /// Using Canny's output as a mask, we display our result
 dst = Scalar::all(0);
  std::cout << detected_edges.rows << endl;
  cameraFeed.copyTo( dst, detected_edges);
  imshow(window_name, dst );
 }
int main()
{

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
        Mat gray;
        cameraFeed.data = (uchar*)colorFrame.getData();
        cvtColor( cameraFeed, gray, CV_BGR2GRAY );
        cv::Mat depthImage(depthFrame.getHeight(), depthFrame.getWidth(), CV_16U);
        depthImage.data = (uchar*)depthFrame.getData();
        depthImage.convertTo(depthImage, CV_8U, 0.1);
        dst.create( cameraFeed.size(), cameraFeed.type() );
       // GaussianBlur( gray, gray, Size(9, 9), 0, 0 );
        /*for(int i = 0; i < depthImage.rows; i++)
        {
            for(int j = 0; j < depthImage.cols; j++)
            {
                 depthImage.at<uchar>(i, j) += 65000;

                // do something with BGR values...
            }
        }*/
          createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold);
           createTrackbar( "Max Threshold:", window_name, &highThreshold, max_lowThreshold);

        /// Show the image
        CannyThreshold();
        vector<Vec3f> circles;

        /// Apply the Hough Transform to find the circles
        HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, depthImage.rows/8, 200, 100, 0, 0);



        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( depthImage, center, 3, 255, 2, 8, 0 );
            // circle outline
            circle( depthImage, center, radius, 255, 3, 8, 0 );
        }
        cv::imshow("depth", depthImage);
        cv::imshow("camera", gray);
        int key = cv::waitKey(100);
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
    }

    color.stop();
    color.destroy();
    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();

     return 1;

}
