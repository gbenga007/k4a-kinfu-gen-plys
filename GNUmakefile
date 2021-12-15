TARGET := Azure_Kinect_Capture

CXXFLAGS := -std=c++11 -lk4a `pkg-config --cflags --libs opencv`
SOURCES := grab_images_k4a.cpp


$(TARGET) : $(SOURCES)
	$(CXX) $^ $(CXXFLAGS) -o $@
