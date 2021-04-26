#include "../includes/camera.h"


Vision::Vision() {
	/*****************************************************************************
	*********************   INITIALISE RASPBERRY PI CAMERA   *********************
	*****************************************************************************/
	// Create Raspberry Pi camera object
	//raspicam::RaspiCam Camera;

	// Set Raspberry Pi camera parameters
	// Set camera image format to BGR as used by  OpenCV
	camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);
	// Set image resolution
	camera.setWidth(640);
	camera.setHeight(480);
	// Flip camera image vertically and horizontally
	// because camera is mounted upside down
	camera.setVerticalFlip(true);
	camera.setHorizontalFlip(true);

//	// Display current camera parameters
//	std::cout << "Format: " << camera.getFormat() << std::endl;
//	std::cout << "Width: " << camera.getWidth() << std::endl;
//	std::cout << "Height: " << camera.getHeight() << std::endl;
//	std::cout << "Brightness: " << camera.getBrightness() << std::endl;
//	std::cout << "Rotation: " << camera.getRotation() << std::endl;
//	std::cout << "ISO: " << camera.getISO() << std::endl;
//	std::cout << "Sharrpness: " << camera.getSharpness() << std::endl;
//	std::cout << "Contrast: " << camera.getContrast() << std::endl;
//	std::cout << "Saturation: " << camera.getSaturation() << std::endl;
//	std::cout << "ShutterSpeed: " << camera.getShutterSpeed() << std::endl;
//	std::cout << "Exopsure: " << camera.getExposure() << std::endl;
//	std::cout << "AWB: " << camera.getAWB() << std::endl;
//	std::cout << "Image effect: " << camera.getImageEffect() << std::endl;
//	std::cout << "Metering: " << camera.getMetering() << std::endl;
//	std::cout << "Format:" << camera.getFormat() << std::endl;
//	std::cout << "Body: " << "Ready" << std::endl;

	// Open camera
	if (!camera.open())
	{
		std::cerr << "Error opening camera." << std::endl;
		throw("Error opening camera.");
	}

	// Wait 3 seconds for camera image to stabilise
	std::cout << "Waiting for camera to stabilise...";
	usleep(3000000);
	std::cout << " Done." << std::endl;



	setupSimpleBlobDetector();
	setUpYOLO();

	/*****************************************************************************
	*******************************   CONTROLLER   *******************************
	*****************************************************************************/

	// Create buffer of correct size to store image data
	img_buf_len = camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_BGR);
	img_buf = new unsigned char[img_buf_len];

	// Initialise OpenCV image Mat
	imageMat = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, img_buf);

	// Create window to display original image
	cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
}


Vision::~Vision()
{
}

void Vision::setupSimpleBlobDetector()
{
	/*****************************************************************************
	*********************  SETUP SIMPLE BLOB DETECTOR   **************************
	*****************************************************************************/
	/*// Create OpenCV SimpleBlobDetector parameter objects (One for white object detection (red), one for black object detection).
	//cv::SimpleBlobDetector::Params sbdPar_red, sbdPar_black;
	// Set blob detection parameters
	// Change thresholds
	sbdPar_red.minThreshold = 1;
	sbdPar_red.maxThreshold = 1000;
	sbdPar_black.minThreshold = 1;
	sbdPar_black.maxThreshold = 1000;
	// Filter by colour
	sbdPar_red.filterByColor = true;
	sbdPar_black.filterByColor = true;
	// Look for colours that match grayscale value of 255 (white) or 0 (black)
	sbdPar_red.blobColor = 255;
	sbdPar_black.blobColor = 0;
	// Filter by area
	sbdPar_red.filterByArea = true;
	sbdPar_red.minArea = 100; // 10x10 pixels
	sbdPar_red.maxArea = 160000; // 400x400 pixels
	sbdPar_black.filterByArea = true;
	sbdPar_black.minArea = 1225; // 100x100 pixels
	sbdPar_black.maxArea = 160000; // 400x400 pixels
	// Create OpenCV SimpleBlobDetector object based on assigned parameters
	sbd_red = cv::SimpleBlobDetector::create(sbdPar_red);
	sbd_black = cv::SimpleBlobDetector::create(sbdPar_black);
	vector<cv::KeyPoint> keypts_red, keypts_black;*/

	/*****************************************************************************
	************************   INITIALISE VISUALISATION   ************************
	*****************************************************************************/
	/*// Black threshold values
	int iLowH_black = 0;
	int iHighH_black = 179;
	int iLowS_black = 0;
	int iHighS_black = 255;
	int iLowV_black = 30;
	int iHighV_black = 255;
	// Red threshold values Remember blobcolor = 255
	int iLowH_red = 0;
	int iHighH_red = 179;
	int iLowS_red = 74;
	int iHighS_red = 255;
	int iLowV_red = 60;
	int iHighV_red = 255;
	//~ // Create a window for displaying HSV
	//~ cv::namedWindow("HSV controls",cv::WINDOW_NORMAL);
	//~ // Create trackbars for H, S and V in the window
	//~ cv::createTrackbar("LowH", "HSV controls", &iLowH, 179); //Hue (0 - 179)
	//~ cv::createTrackbar("HighH", "HSV controls", &iHighH, 179);
	//~ cv::createTrackbar("LowS", "HSV controls", &iLowS, 255); //Saturation (0 - 255)
	//~ cv::createTrackbar("HighS", "HSV controls", &iHighS, 255);
	//~ cv::createTrackbar("LowV", "HSV controls", &iLowV, 255); //Value (0 - 255)
	//~ cv::createTrackbar("HighV", "HSV controls", &iHighV, 255);
	// Create a bunch of windows for displaying image processing steps
	// Create window to display original HSV image
	//cv::namedWindow("HSV image",cv::WINDOW_AUTOSIZE);
	// Create window to display thresholded image
	//cv::namedWindow("Thresholded image - Red",cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Thresholded image - Black",cv::WINDOW_AUTOSIZE);
	// Create window to display blob image
	cv::namedWindow("Blobs - Red", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Blobs - Black", cv::WINDOW_AUTOSIZE);*/
	/*********************************   DONE   *********************************/
}

void Vision::updateCamera() {

    while(true){

        // Grab image into internal buffer
        camera.grab();

        // Copy latest camera buffer into our defined buffer
        camera.retrieve(img_buf);

        // Copy image buffer data into OpenCV Mat image
        imageMat = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, img_buf);

        // Exit if there is no image data in OpenCV image Mat
        if (!imageMat.data)
        {
            std::cout << "No data in Mat imageMat." << std::endl;

            break;
        }

        // Display Image
        cv::imshow("Image", imageMat);
        //cv::waitKey(30);
        inputKey = cv::waitKey(100);
        if(inputKey == 27) //27 = 'ESC'
            break;
	}
	releaseCamera();             // Release Raspberry Pi camera resources
}

void Vision::releaseCamera()
{
	camera.release();
	std::cout << "Camera resources released." << std::endl;
}

void Vision::setUpYOLO()
{
	// Load names of classes
	string classesFile = "coco.names";
	ifstream ifs(classesFile.c_str());
	string line;
	while (getline(ifs, line)) classes.push_back(line);

	// Give the configuration and weight files for the model
	String modelConfiguration = "yolov3.cfg";
	String modelWeights = "yolov3.weights";

	// Load the network
	net = readNetFromDarknet(modelConfiguration, modelWeights);

	cout << "Using CPU device" << endl;
	net.setPreferableBackend(DNN_TARGET_CPU);

//	outputFile = "yolo_out_cpp.avi";
//
//	// Open the image file
//	str = parser.get<String>("image");
//	ifstream ifile(str);
//	if (!ifile) throw("error in 'ifile(str)\n");
//	cap.open(str);
//	str.replace(str.end() - 4, str.end(), "_yolo_out_cpp.jpg");
//	outputFile = str;


	std::cout << "YOLO ready!\n";

}

void Vision::YOLOProcess()
{
    std::cout << "Y.O.L.O RUNNIG...\n";



	while (waitKey(1) < 0)
	{
		// get frame from the video
		frame = imageMat.clone();

		// Create a 4D blob from a frame.
		blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);

		//Sets the input to the network
		net.setInput(blob);

		// Runs the forward pass to get output of the output layers
		vector<Mat> outs;
		net.forward(outs, getOutputsNames(net));

		// Remove the bounding boxes with low confidence
		postprocess(frame, outs);

		// Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
		//vector<double> layersTimes;
		//double freq = getTickFrequency() / 1000;
		//double t = net.getPerfProfile(layersTimes) / freq;
		//string label = format("Inference time for a frame : %.2f ms", t);
		//putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

		// Write the frame with the detection boxes
		Mat detectedFrame;
		frame.convertTo(detectedFrame, CV_8U);


		imshow("YOLO", frame);

	}

}

// Remove the bounding boxes with low confidence using non-maxima suppression
void Vision::postprocess(Mat& frame, const vector<Mat>& outs)
{
	vector<int> classIds;
	vector<float> confidences;
	vector<Rect> boxes;

	for (size_t i = 0; i < outs.size(); ++i)
	{
		// Scan through all the bounding boxes output from the network and keep only the
		// ones with high confidence scores. Assign the box's class label as the class
		// with the highest score for the box.
		float* data = (float*)outs[i].data;
		for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
		{
			Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
			Point classIdPoint;
			double confidence;
			// Get the value and location of the maximum score
			minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
			if (confidence > confThreshold)
			{
				int centerX = (int)(data[0] * frame.cols);
				int centerY = (int)(data[1] * frame.rows);
				int width = (int)(data[2] * frame.cols);
				int height = (int)(data[3] * frame.rows);
				int left = centerX - width / 2;
				int top = centerY - height / 2;

				classIds.push_back(classIdPoint.x);
				confidences.push_back((float)confidence);
				boxes.push_back(Rect(left, top, width, height));
			}
		}
	}

	// Perform non maximum suppression to eliminate redundant overlapping boxes with
	// lower confidences
	vector<int> indices;
	NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		drawPred(classIds[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame);
	}
}

// Draw the predicted bounding box
void Vision::drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
	//Draw a rectangle displaying the bounding box
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

	//Get the label for the class name and its confidence
	string label = format("%.2f", conf);
	if (!classes.empty())
	{
		CV_Assert(classId < (int)classes.size());
		label = classes[classId] + ":" + label;
	}

	//Display the label at the top of the bounding box
	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = max(top, labelSize.height);
	rectangle(frame, Point(left, top - round(1.5 * labelSize.height)), Point(left + round(1.5 * labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}

// Get the names of the output layers
vector<String> Vision::getOutputsNames(const Net& net)
{
	static vector<String> names;
	if (names.empty())
	{
		//Get the indices of the output layers, i.e. the layers with unconnected outputs
		vector<int> outLayers = net.getUnconnectedOutLayers();

		//get the names of all the layers in the network
		vector<String> layersNames = net.getLayerNames();

		// Get the names of the output layers in names
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}
