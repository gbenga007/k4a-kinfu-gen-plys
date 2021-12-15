// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <k4a/k4a.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
// OpenCV header files.
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>



// Create the directory to store the images.
int CreateDirectory(std::string dirDest){
    int mkdRet = mkdir(dirDest.c_str(), 0777);
    if (mkdRet == 0){
        std::cout << "The \"" << dirDest.c_str() << "\"is created successfully"
                  << std::endl;
	return mkdRet;
    }
    else{
        std::cout << "The folder \"" << dirDest.c_str()
                  << "\" couldn't be created." << std::endl;
	return mkdRet;
    }

        return mkdRet;
}




int main(int argc, char **argv)
{
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    int captureFrameCount;
    k4a_capture_t capture = NULL;

    if (argc < 3)
    {
        printf("Usage:\n\t%s <FRAMECOUNT> <destination dirName>\n", argv[0]);
        printf("Capture FRAMECOUNT color and depth frames from the device using the separate get frame APIs\n");
        returnCode = 2;
	
        if (device != NULL)
	    {
	        k4a_device_close(device);
		return returnCode;
	    }
	return returnCode;
    }
    std::ostringstream saveDir;
    saveDir << "/home/vigir/Images/" << argv[2];
    if( CreateDirectory( saveDir.str() ) != 0) {
     	
        if (device != NULL)
	    {
	        k4a_device_close(device);
		return returnCode;
	    }
	return returnCode;
     }
    captureFrameCount = atoi(argv[1]);
    printf("Capturing %d frames\n", captureFrameCount);

    uint32_t device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
        if (device != NULL)
	    {
	        k4a_device_close(device);
		return returnCode;
	    }
    }

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start device\n");
        if (device != NULL)
	    {
	        k4a_device_close(device);
		return returnCode;
	    }
    }

    while (captureFrameCount-- > 0)
    {
        k4a_image_t image;

        // Get a depth frame
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            //goto Exit;
	   if (device != NULL)
	    {
	        k4a_device_close(device);
		return returnCode;
	    }
        }

        printf("Capture");

        // Probe for a color image
        image = k4a_capture_get_color_image(capture);
        if (image)
        {
            printf(" | Color res:%4dx%4d stride:%5d ",
                   k4a_image_get_height_pixels(image),
                   k4a_image_get_width_pixels(image),
                   k4a_image_get_stride_bytes(image));
	  /* source : https://stackoverflow.com/questions/57222190/how-to-convert-k4a-image-t-to-opencv-matrix-azure-kinect-sensor-sdk */
	 // you can check the format with this function
	    k4a_image_format_t format = k4a_image_get_format(image); // K4A_IMAGE_FORMAT_COLOR_BGRA32 

	    // get raw buffer
	    uint8_t* buffer = k4a_image_get_buffer(image);

	    // convert the raw buffer to cv::Mat
	    int rows = k4a_image_get_height_pixels(image);
	    int cols = k4a_image_get_width_pixels(image);
	    cv::Mat colorMat(rows , cols, CV_8UC4, (void*)buffer, cv::Mat::AUTO_STEP);
	    // could also be ..
	    // cv::Mat colorMat(image.get_height_pixels(),image.get_width_pixels(),CV_8UC4,image.get_buffer(),(size_t)image.get_stride_bytes());
	    std::ostringstream color_image_name;
	    color_image_name <<"/home/vigir/Images/" << argv[2] << "/rgbImg_" << captureFrameCount << ".jpg";
	    cv::imwrite(color_image_name.str(), colorMat);
	   
		
            k4a_image_release(image);
        }
        else
        {
            printf(" | Color None                       ");
        }

        // probe for a IR16 image
        image = k4a_capture_get_ir_image(capture);
        if (image != NULL)
        {
            printf(" | Ir16 res:%4dx%4d stride:%5d ",
                   k4a_image_get_height_pixels(image),
                   k4a_image_get_width_pixels(image),
                   k4a_image_get_stride_bytes(image));
            k4a_image_release(image);
        }
        else
        {
            printf(" | Ir16 None                       ");
        }

        // Probe for a depth16 image
        image = k4a_capture_get_depth_image(capture);
        if (image != NULL)
        {
            printf(" | Depth16 res:%4dx%4d stride:%5d\n",
                   k4a_image_get_height_pixels(image),
                   k4a_image_get_width_pixels(image),
                   k4a_image_get_stride_bytes(image));
		k4a_image_format_t format = k4a_image_get_format(image); // K4A_IMAGE_FORMAT_COLOR_BGRA32 

		// get raw buffer
		uint8_t* buffer = k4a_image_get_buffer(image);

		// convert the raw buffer to cv::Mat
		int rows = k4a_image_get_height_pixels(image);
		int cols = k4a_image_get_width_pixels(image);
		cv::Mat depthMat(rows , cols, CV_16U, (void*)buffer, cv::Mat::AUTO_STEP);
		// same song different verse
		//  cv::Mat depthMat(image.get_height_pixels(),
		//  			image.get_width_pixels(),
		//  			CV_16U,
		//  			image.get_buffer,
		//  			(size_t)image.get_stride_in_bytes()
		//  			);
		std::ostringstream depth_image_name;
		depth_image_name << "/home/vigir/Images/" << argv[2] << "/depthImg_" << captureFrameCount << ".png";
		cv::imwrite(depth_image_name.str(), depthMat);
		k4a_image_release(image);
        }
        else
        {
            printf(" | Depth16 None\n");
        }

        // release capture
        k4a_capture_release(capture);
        fflush(stdout);
    }

    returnCode = 0;
//Exit:
    if (device != NULL)
    {
        k4a_device_close(device);
    }

    return returnCode;
}
