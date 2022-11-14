// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>
#include <string>
#include <algorithm>
// OpenCV header files.
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>

#define INVALID INT32_MIN

using namespace cv;
using namespace std;
typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
    int x;
    int y;
    float weight[4];
} coordinate_t;

struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};
template<typename T> Mat create_mat_from_buffer(T *data, int width, int height, int channels = 1)
{
    Mat mat(height, width, CV_MAKETYPE(DataType<T>::type, channels));
    memcpy(mat.data, data, width * height * channels * sizeof(T));
    return mat;
}
void initialize_kinfu_params(kinfu::Params &params,
                             const int width,
                             const int height,
                             const float fx,
                             const float fy,
                             const float cx,
                             const float cy);

typedef enum
{
    INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
    INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
    INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                                 data with value 0 */
} interpolation_t;

// some function prototypes
static void compute_xy_range(const k4a_calibration_t *, 
                             const k4a_calibration_type_t ,
                             const int ,
                             const int ,
                             float &,
                             float &,
                             float &,
                             float &);
static pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera);
static void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type);
static void create_undistortion_lut(const k4a_calibration_t *calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t *pinhole,
                                    k4a_image_t lut,
                                    interpolation_t type);

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name);   

void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
                                             const k4a_image_t color_image,
                                             const char *file_name);

// Create the directory to store the images.
int CreateDirectory(std::string dirDest){
    int mkdRet = mkdir(dirDest.c_str(), 0777);
    if (mkdRet == 0){
        std::cout << "The \"" << dirDest.c_str() << "\" is created successfully"
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


//k4arecorder -l 5 -d WFOV_UNBINNED -r 5 --imu OFF recording.mkv



int main(int argc, char **argv)
{

    if(argc < 3){
        std::cout << "\n\tUsage : " << argv[0] << " <recording.mkv> <output ply file name>" << std::endl;
        return -1;
    }
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;

    setUseOptimized(true);
    
    k4a_image_t depth_image = NULL;
    k4a_image_t undistorted_depth_image = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t lut = NULL;
    k4a_transformation_t transformation = NULL;
    interpolation_t interpolation_type = INTERPOLATION_BILINEAR_DEPTH;
    pinhole_t pinhole;
    k4a_calibration_t calibration;

    /* Read data from Capture MKV file */
    /* Apply Kinect Fusion to the stream */
    string inputFname = argv[1];
    k4a_playback_t playback = NULL;
    k4a_result_t result;
    k4a_stream_result_t stream_result = K4A_STREAM_RESULT_SUCCEEDED; // init to success state and watch for change later

    // this opens the recording stored in inputFname
    result = k4a_playback_open(inputFname.c_str(),&playback);
    
    if(result != K4A_RESULT_SUCCEEDED || playback == NULL){
        std::cout<<"Failed to open the recording: " << inputFname << std::endl;
        
        if(playback != NULL){
            k4a_playback_close(playback);
        }
        return -1;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration)){
        std::cout << "Failed to get calibration" << std::endl;
        if(playback != NULL){
            k4a_playback_close(playback);
        }
        return -1;
            
    }

    transformation = k4a_transformation_create(&calibration);
    // Generate a pinhole model for depth camera
    pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     pinhole.width,
                     pinhole.height,
                     pinhole.width * (int)sizeof(coordinate_t),
                     &lut);

    create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_DEPTH, &pinhole, lut, interpolation_type);
    // Retrieve calibration parameters
    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.depth_camera_calibration.intrinsics.parameters;
    const int width = calibration.depth_camera_calibration.resolution_width;
    const int height = calibration.depth_camera_calibration.resolution_height;

    // Initialize kinfu parameters
    Ptr<kinfu::Params> params;
    params = kinfu::Params::defaultParams();
    initialize_kinfu_params(
        *params, width, height, pinhole.fx, pinhole.fy, pinhole.px, pinhole.py);

    // Distortion coefficients
    Matx<float, 1, 8> distCoeffs;
    distCoeffs(0) = intrinsics->param.k1;
    distCoeffs(1) = intrinsics->param.k2;
    distCoeffs(2) = intrinsics->param.p1;
    distCoeffs(3) = intrinsics->param.p2;
    distCoeffs(4) = intrinsics->param.k3;
    distCoeffs(5) = intrinsics->param.k4;
    distCoeffs(6) = intrinsics->param.k5;
    distCoeffs(7) = intrinsics->param.k6;

   // Create KinectFusion module instance
    Ptr<kinfu::KinFu> kf;
    kf = kinfu::KinFu::create(params);
    //namedWindow("AzureKinect KinectFusion Example");
    //viz::Viz3d visualization("AzureKinect KinectFusion Example");

    bool stop = false;
    bool renderViz = false;  
    UMat points;
    UMat normals;
    // Streaming here
    int iter = 0;
    while(stream_result == K4A_STREAM_RESULT_SUCCEEDED){
        
        stream_result = k4a_playback_get_next_capture(playback,&capture);
        
        if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            // End of file reached
            break;
        }
        // Fetch frame
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0)
        {
            std::cout<<"Failed to get depth image from capture" << std::endl;
            break;
        }

        // Once we acquire the depth, we run it through the kinect fusion pipeline
        
        // Undistort the depth
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                         pinhole.width,
                         pinhole.height,
                         pinhole.width * (int)sizeof(uint16_t),
                         &undistorted_depth_image);
        remap(depth_image, lut, undistorted_depth_image, interpolation_type);

        // Create frame from depth buffer
        uint8_t *buffer = k4a_image_get_buffer(undistorted_depth_image);
        uint16_t *depth_buffer = reinterpret_cast<uint16_t *>(buffer);
        UMat undistortedFrame;
        create_mat_from_buffer<uint16_t>(depth_buffer, width, height).copyTo(undistortedFrame);

        if (undistortedFrame.empty())
        {
            k4a_image_release(depth_image);
            k4a_image_release(undistorted_depth_image);
            k4a_capture_release(capture);
            continue;
        }

        // Update KinectFusion
        if (!kf->update(undistortedFrame))
        {
            printf("Reset KinectFusion\n");
            kf->reset();
            k4a_image_release(depth_image);
            k4a_image_release(undistorted_depth_image);
            k4a_capture_release(capture);
            continue;
        }

        // Retrieve rendered TSDF
        UMat tsdfRender;
        kf->render(tsdfRender);

        // Retrieve fused point cloud and normals
        kf->getCloud(points, normals);

        // Show TSDF rendering
        //imshow("AzureKinect KinectFusion Example", tsdfRender);

        // Show fused point cloud and normals
        /*
        if (!points.empty() && !normals.empty() && renderViz)
        {
            viz::WCloud cloud(points, viz::Color::white());
            viz::WCloudNormals cloudNormals(points, normals, 1, 0.01, viz::Color::cyan());
            visualization.showWidget("cloud", cloud);
            visualization.showWidget("normals", cloudNormals);
            visualization.showWidget("worldAxes", viz::WCoordinateSystem());
            Vec3d volSize = kf->getParams().voxelSize * kf->getParams().volumeDims;
            visualization.showWidget("cube", viz::WCube(Vec3d::all(0), volSize), kf->getParams().volumePose);
            visualization.spinOnce(1, true);
        }
        */

       
       
        k4a_image_release(depth_image);
        k4a_image_release(undistorted_depth_image);
        k4a_capture_release(capture);
    }

     // Output the fused point cloud from KinectFusion
        Mat out_points;
        Mat out_normals;
        points.copyTo(out_points);
        normals.copyTo(out_normals);

        std::cout<<"Saving fused point cloud into ply file ..." << std::endl;

        // Save to the ply file
    #define PLY_START_HEADER "ply"
    #define PLY_END_HEADER "end_header"
    #define PLY_ASCII "format ascii 1.0"
    #define PLY_ELEMENT_VERTEX "element vertex"
        std::ostringstream output_file_name;
        output_file_name << argv[2] <<".ply";
        ofstream ofs(output_file_name.str()); // text mode first
        ofs << PLY_START_HEADER << endl;
        ofs << PLY_ASCII << endl;
        ofs << PLY_ELEMENT_VERTEX << " " << out_points.rows << endl;
        ofs << "property float x" << endl;
        ofs << "property float y" << endl;
        ofs << "property float z" << endl;
        ofs << "property float nx" << endl;
        ofs << "property float ny" << endl;
        ofs << "property float nz" << endl;
        ofs << PLY_END_HEADER << endl;
        ofs.close();

        stringstream ss;
        for (int i = 0; i < out_points.rows; ++i)
        {
            ss << out_points.at<float>(i, 0) << " "
                << out_points.at<float>(i, 1) << " "
                << out_points.at<float>(i, 2) << " "
                << out_normals.at<float>(i, 0) << " "
                << out_normals.at<float>(i, 1) << " "
                << out_normals.at<float>(i, 2) << endl;
        }
        ofstream ofs_text(output_file_name.str(), ios::out | ios::app);
        ofs_text.write(ss.str().c_str(), (streamsize)ss.str().length());



    if (playback != NULL)
    {
        k4a_playback_close(playback);
        returnCode = -1;
    }

    if (capture != NULL)
    {
        k4a_capture_release(capture);
        returnCode = -1;
    }

    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
        returnCode = -1;
    }

    if (stream_result == K4A_STREAM_RESULT_FAILED){
        std::cout<<"Failed to read entire recording" << std::endl;
        returnCode = -1;
    }
   
    k4a_image_release(lut);
    
    destroyAllWindows();

    return returnCode;
}


void initialize_kinfu_params(kinfu::Params &params,
                             const int width,
                             const int height,
                             const float fx,
                             const float fy,
                             const float cx,
                             const float cy)
{
    const Matx33f camera_matrix = Matx33f(fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);
    params.frameSize = Size(width, height);
    params.intr = camera_matrix;
    params.depthFactor = 1000.0f;
}



static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              transformed_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

// Compute a conservative bounding box on the unit plane in which all the points have valid projections
static void compute_xy_range(const k4a_calibration_t *calibration,
                             const k4a_calibration_type_t camera,
                             const int width,
                             const int height,
                             float &x_min,
                             float &x_max,
                             float &y_min,
                             float &y_max)
{
    // Step outward from the centre point until we find the bounds of valid projection
    const float step_u = 0.25f;
    const float step_v = 0.25f;
    const float min_u = 0;
    const float min_v = 0;
    const float max_u = (float)width - 1;
    const float max_v = (float)height - 1;
    const float center_u = 0.5f * width;
    const float center_v = 0.5f * height;

    int valid;
    k4a_float2_t p;
    k4a_float3_t ray;

    // search x_min
    for (float uv[2] = { center_u, center_v }; uv[0] >= min_u; uv[0] -= step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_min = ray.xyz.x;
    }

    // search x_max
    for (float uv[2] = { center_u, center_v }; uv[0] <= max_u; uv[0] += step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_max = ray.xyz.x;
    }

    // search y_min
    for (float uv[2] = { center_u, center_v }; uv[1] >= min_v; uv[1] -= step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_min = ray.xyz.y;
    }

    // search y_max
    for (float uv[2] = { center_u, center_v }; uv[1] <= max_v; uv[1] += step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_max = ray.xyz.y;
    }
}



static pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera)
{
    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        width = calibration->color_camera_calibration.resolution_width;
        height = calibration->color_camera_calibration.resolution_height;
    }

    float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    compute_xy_range(calibration, camera, width, height, x_min, x_max, y_min, y_max);

    pinhole_t pinhole;

    float fx = 1.f / (x_max - x_min);
    float fy = 1.f / (y_max - y_min);
    float px = -x_min * fx;
    float py = -y_min * fy;

    pinhole.fx = fx * width;
    pinhole.fy = fy * height;
    pinhole.px = px * width;
    pinhole.py = py * height;
    pinhole.width = width;
    pinhole.height = height;

    return pinhole;
}

static void create_undistortion_lut(const k4a_calibration_t *calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t *pinhole,
                                    k4a_image_t lut,
                                    interpolation_t type)
{
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    k4a_float3_t ray;
    ray.xyz.z = 1.f;

    int src_width = calibration->depth_camera_calibration.resolution_width;
    int src_height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        src_width = calibration->color_camera_calibration.resolution_width;
        src_height = calibration->color_camera_calibration.resolution_height;
    }

    for (int y = 0, idx = 0; y < pinhole->height; y++)
    {
        ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

        for (int x = 0; x < pinhole->width; x++, idx++)
        {
            ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

            k4a_float2_t distorted;
            int valid;
            k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

            coordinate_t src;
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                // Remapping via nearest neighbor interpolation
                src.x = (int)floorf(distorted.xy.x + 0.5f);
                src.y = (int)floorf(distorted.xy.y + 0.5f);
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                // Remapping via bilinear interpolation
                src.x = (int)floorf(distorted.xy.x);
                src.y = (int)floorf(distorted.xy.y);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }

            if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
            {
                lut_data[idx] = src;

                if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // Compute the floating point weights, using the distance from projected point src to the
                    // image coordinate of the upper left neighbor
                    float w_x = distorted.xy.x - src.x;
                    float w_y = distorted.xy.y - src.y;
                    float w0 = (1.f - w_x) * (1.f - w_y);
                    float w1 = w_x * (1.f - w_y);
                    float w2 = (1.f - w_x) * w_y;
                    float w3 = w_x * w_y;

                    // Fill into lut
                    lut_data[idx].weight[0] = w0;
                    lut_data[idx].weight[1] = w1;
                    lut_data[idx].weight[2] = w2;
                    lut_data[idx].weight[3] = w3;
                }
            }
            else
            {
                lut_data[idx].x = INVALID;
                lut_data[idx].y = INVALID;
            }
        }
    }
}

static void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t *src_data = (uint16_t *)(void *)k4a_image_get_buffer(src);
    uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint16_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),
                                               std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                                               std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint16_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                                         neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                                         0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}



void tranformation_helpers_write_point_cloud(const k4a_image_t point_cloud_image,
                                             const k4a_image_t color_image,
                                             const char *file_name)
{
    std::cout << "Writing PLY ... " << file_name << std::endl;
    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);

    for (int i = 0; i < width * height; i++)
    {
        color_point_t point;
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0)
        {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        {
            continue;
        }

        points.push_back(point);
    }

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << PLY_START_HEADER << std::endl;
    ofs << PLY_ASCII << std::endl;
    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << PLY_END_HEADER << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
        ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
