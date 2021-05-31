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
	camera.setWidth(640); //640
	camera.setHeight(480); //480
	// Flip camera image vertically and horizontally
	// because camera is mounted upside down
	camera.setVerticalFlip(true);
	camera.setHorizontalFlip(true);

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

	setUpYOLO();

	/*****************************************************************************
	*******************************   CONTROLLER   *******************************
	*****************************************************************************/

	// Create buffer of correct size to store image data
	img_buf_len = camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_BGR);
	img_buf = new unsigned char[img_buf_len];

	// Initialise OpenCV image Mat
	imageMat = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, img_buf);
	processedFrame = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, img_buf);



	// Create window to display original image
	cv::namedWindow("Image", cv::WINDOW_NORMAL);
	resizeWindow("Image", 640, 480);

    cv::namedWindow(kWinName, WINDOW_NORMAL);
    resizeWindow(kWinName, 640, 480);
}


Vision::~Vision()
{
}


void Vision::updateCamera() {

    while(true){

        // Grab image into internal buffer
        camera.grab();

        while (true) { //MUTEX Loop

            if (imageMatMutex.try_lock()) {

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
                imshow(kWinName, processedFrame);
                cv::imshow("Image", imageMat);
                imageMatMutex.unlock();
                break;
            }
        } //End of MUTEX LOOP

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

void Vision::getImageMat()
{
    while (true) {

        if (imageMatMutex.try_lock()) {
            frame = imageMat.clone();
            imageMatMutex.unlock();
            break;
        }
    }
}


void Vision::setUpYOLO()
{
	// Load names of classes
	string classesFile = "coco.names";
	ifstream ifs(classesFile.c_str());
	string line;
	while (getline(ifs, line)){
            classes.push_back(line);
    }

	// Give the configuration and weight files for the model
	String modelConfiguration = "yolov3-tiny.cfg";
	String modelWeights = "yolov3-tiny.weights";

	// Load the network
	net = readNetFromDarknet(modelConfiguration, modelWeights);

	cout << "Using CPU device" << endl;
	net.setPreferableBackend(DNN_TARGET_CPU);

    ifs.close();

	std::cout << "YOLO ready!\n";

}

void Vision::YOLOProcess()
{
    std::cout << "YOLO RUNNING...\n";
    while(true){


        getImageMat();
        if(frame.empty()){
            cout << "No frame from UpdateCamera!\n";
            return;
        }
        // Create a 4D blob from a frame.
        blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);

        //Sets the input to the network
        net.setInput(blob);

        // Runs the forward pass to get output of the output layers
        vector<Mat> outs;
        net.forward(outs, getOutputsNames(net));

        // Remove the bounding boxes with low confidence
        postprocess(frame, outs);

        //Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
        vector<double> layersTimes;
        freq = getTickFrequency() / 1000;
        t = net.getPerfProfile(layersTimes) / freq;

        label = format("Inference time for a frame in YOLOv3-lite: %.2f ms", t);
        //std::cout << label << endl;
        putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

        // Write the frame with the detection boxes
        Mat detectedFrame;
        frame.convertTo(detectedFrame, CV_8U);


        processedFrame = frame.clone();


        if(inputKey == 27) //27 = 'ESC'
            break;
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
	if (!classes.empty() )
	{
		CV_Assert(classId < (int)classes.size());
		label = classes[classId] + ":" + label;

		if(classes[classId] == "person" && conf >= CONFIDENCE_THRESHOLD){
            object = classes[classId];
            if(conf > confidence)
                confidence = conf;
		}
		else{
			object == "n/a";
			confidence = 0;
		}

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

std::string Vision::getObject()
{
	return object;
}

double Vision::getConfidence()
{
	return confidence;
}




