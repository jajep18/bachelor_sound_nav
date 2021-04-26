#pragma once
/*
 * Description:     Vision (camera and detection) class
 *
 * Author:			Jacob Fløe Jeppesen
 *					University of Southern Denmark
 * Creation date:   21-02-2021
 */

/*
* Yolo Object DISCLAIMER:
* This code is written at BigVision LLC. It is based on the OpenCV project. It is subject to the license terms in the LICENSE file found in this distribution and at http://opencv.org/license.html
* The code used in this project is inspired by https://learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/
* Code source: https://github.com/spmallick/learnopencv/blob/master/ObjectDetection-YOLO/object_detection_yolo.cpp
*/

#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>
#include <unistd.h>
#include <raspicam/raspicam.h>
#include <atomic>
#include <fstream>
#include <sstream>




using namespace cv;
using namespace dnn;
using namespace std;

class Vision
{
private:
	//Camera
	raspicam::RaspiCam camera;									// Create Raspberry Pi camera object
	int img_buf_len;											// Create buffer of correct size to store image data
	unsigned char* img_buf;										// Create buffer of correct size to store image data
	cv::Mat imageMat;											// Initialise OpenCV image Mat

	/**********************YOLO************************************/
	float confThreshold = 0.5; // Confidence threshold
	float nmsThreshold = 0.4;  // Non-maximum suppression threshold
	int inpWidth = 416;  // Width of network's input image
	int inpHeight = 416; // Height of network's input image


	vector<string> classes;
	string str, outputFile;
	VideoCapture cap;
	VideoWriter video;
	Mat frame, blob;

	Net net;

	string kWinName = "Object detection";


	void setUpYOLO();


	// Remove the bounding boxes with low confidence using non-maxima suppression
	void postprocess(Mat& frame, const vector<Mat>& out);

	// Draw the predicted bounding box
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);



public:
	Vision();
	~Vision();
	void setupSimpleBlobDetector();
	void updateCamera();
	void releaseCamera();
    void YOLOProcess();

	// Get the names of the output layers
	vector<String> getOutputsNames(const Net& net);

	std::atomic<char> inputKey;
};
