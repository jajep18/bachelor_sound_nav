g++ testOfYOLO.cpp -o yolo.out \
sources/camera.cpp \
-Wall \
-pthread \
-L/usr/local/lib \
-ljson-c \
-I/usr/local/include/opencv4 \
-lm \
-lraspicam \
-lopencv_core \
-lopencv_features2d \
-lopencv_highgui \
-lopencv_imgcodecs \
-lopencv_imgproc \
-lopencv_dnn \
-lopencv_videoio \
-lgflags

echo "To run, type:"
echo "./yolo.out"