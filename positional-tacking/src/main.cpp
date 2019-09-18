///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/*************************************************************************
** This sample demonstrates how to use the ZED for positional tracking  **
** and display camera motion in an OpenGL window. 		                **
**************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <time.h>
#include <math.h>


// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "TrackingViewer.hpp"

#include "netutil.h"

#define ZAJKO_IP                       "169.254.0.100"
#define ZAJKO_POSITION_AND_DEPTH_PORT  9772

#define DEPTH_REPORT_PERIOD            200    // 5 Hz
#define POSITION_REPORT_PERIOD         50    // 20 Hz
#define POSITION_PRINT_PERIOD          500   // 2 Hz
#define POSITION_PACKET_TYPE	       3
#define DEPTH_PACKET_TYPE	       4

#define RECONNECT_FREQUENCY            70

// Using std namespace
using namespace std;
using namespace sl;

// Create ZED objects
sl::Camera zed;
sl::Pose camera_pose;
std::thread zed_callback;
bool quit = false;

int connected = 0;
int reconnect_counter = 0;

// OpenGL window to display camera motion
//GLViewer viewer;

const int MAX_CHAR = 128;

static int running;
static int depth_socket;

static int retry = 0;

const double hd_cx = 619.874;
const double hd_cy = 359.724;
const double hd_fx = 699.326;
const double hd_fy = 699.326;

const double camera_inclination_rad = (17 / 180.0 * M_PI);

// Sample functions
void startZED();
void run();
void close();
void transformPose(sl::Transform &pose, float tx);

long long msec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000L * tv.tv_sec + tv.tv_usec / 1000L;
}

long long usec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return (1000000L * (long long)tv.tv_sec) + tv.tv_usec;
}

static double view_angle_vertical[720];
static double view_angle_horizontal[1280];

void precompute_angles_for_each_coordinate()
{
    for (int i = 0; i < 720; i++)
      view_angle_vertical[i] = atan((hd_cy - i) / hd_fy);

    for (int i = 0; i < 1280; i++)
      view_angle_horizontal[i] = atan((i - hd_cx) / hd_fx);
}

int main(int argc, char **argv) {

    precompute_angles_for_each_coordinate();

    // Set configuration parameters for the ZED
    InitParameters initParameters;
    initParameters.camera_resolution = RESOLUTION_HD720;
    initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
    initParameters.coordinate_units = UNIT_METER;
    initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;

    if (argc > 1 && std::string(argv[1]).find(".svo"))
        initParameters.svo_input_filename.set(argv[1]);

    // Open the camera
    ERROR_CODE err = zed.open(initParameters);
    if (err != sl::SUCCESS) {
        std::cout << sl::toString(err) << std::endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // our camera is inclined 17 degrees down, we do not care about its height above the ground
    sl::Transform initial_position;
    sl::Rotation camera_rotation;
    sl::float3 our_camera_rotation(camera_inclination_rad, 0, 0);
    camera_rotation.setEulerAngles(our_camera_rotation);
    initial_position.setRotation(camera_rotation);

    // Set positional tracking parameters
    TrackingParameters trackingParameters;
    //trackingParameters.initial_world_transform = sl::Transform::identity();
    trackingParameters.initial_world_transform = initial_position;
    trackingParameters.enable_spatial_memory = true;

    // Start motion tracking
    zed.enableTracking(trackingParameters);

    // Initialize OpenGL viewer
    //viewer.init(zed.getCameraInformation().camera_model);

    retry = 0;
    do {
      depth_socket = connect_to_server(ZAJKO_IP, ZAJKO_POSITION_AND_DEPTH_PORT);
      if (depth_socket == 0)
      {
        printf("connect to zajko depth server failed, retrying %d...\n", retry++);
        sleep(1);
        continue;
      }
      connected = 1;
    } while (!connected);

    printf("starting zed\n");
    running = 1;
    // Start ZED callback
    startZED();

    
    // Set the display callback
    //glutCloseFunc(close);
    //glutMainLoop();
    while (running) usleep(100000);

    close(depth_socket);
    return 0;
}


/**
 *   Launch ZED thread. Using a thread here allows to retrieve camera motion and display it in a GL window concurrently.
 **/
void startZED() {
    quit = false;
    zed_callback = std::thread(run);
}

void fill_neighbors(uint8_t *map, int map_x, int map_y)
{
  for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++)
    {
      int y = map_y + dy;
      int x = map_x + dx;
      if ((x >= 0) && (x < 60) && (y >= 0) && (y < 60))
      map[y * 60 + x] = 1;
    }
}

void try_reconnecting()
{
  reconnect_counter++;
  if (reconnect_counter > RECONNECT_FREQUENCY)
  {
    reconnect_counter = 0;
    depth_socket = connect_to_server(ZAJKO_IP, ZAJKO_POSITION_AND_DEPTH_PORT);
    if (depth_socket == 0)
    printf("reconnecting failed %d, will retry...\n", retry++);
    else 
    {
      printf("reconnected\n");
      connected = 1; retry = 0;
    }
  }
}

/**
 *  This function loops to get image and motion data from the ZED. It is similar to a callback.
 *  Add your own code here.
 **/
void run() {
    static long long last_msec_reported = 0;
    static long long last_msec_depth_reported = 0;
    static long long last_msec_printed = 0;

    uint8_t position_packet[100];

    float tx = 0, ty = 0, tz = 0;
    float rx = 0, ry = 0, rz = 0;

    // Get the distance between the center of the camera and the left eye
    float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;

    // Create text for GUI
    char text_rotation[MAX_CHAR];
    char text_translation[MAX_CHAR];

    // Create a CSV file to log motion tracking data
    std::ofstream outputFile;
    std::string csvName = "Motion_data";
    outputFile.open(csvName + ".csv");
    if (!outputFile.is_open())
        cout << "WARNING: Can't create CSV file. Run the application with administrator rights." << endl;
    else
        outputFile << "Timestamp(ns);Rotation_X(rad);Rotation_Y(rad);Rotation_Z(rad);Position_X(m);Position_Y(m);Position_Z(m);" << endl;

    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    sl::Mat zed_dep(zed.getResolution(), MAT_TYPE_32F_C1);
    //FILE *f = fopen("depth.log", "w+");

    while (!quit && zed.getSVOPosition() != zed.getSVONumberOfFrames() - 1) {
        //if (zed.grab(runtime_parameters) == SUCCESS) {
        if (zed.grab() == SUCCESS) {
            // Get the position of the camera in a fixed reference frame (the World Frame)
            TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);
            long long tm = msec();
	    zed.retrieveMeasure(zed_dep, MEASURE_DEPTH);

            if (tm - last_msec_reported >= POSITION_REPORT_PERIOD)
            {
                last_msec_reported = tm;
                if (tracking_state == TRACKING_STATE_OK) {
                    // getPosition() outputs the position of the Camera Frame, which is located on the left eye of the camera.
                    // To get the position of the center of the camera, we transform the pose data into a new frame located at the center of the camera.
                    // The generic formula used here is: Pose(new reference frame) = M.inverse() * Pose (camera frame) * M, where M is the transform between two frames.
                    transformPose(camera_pose.pose_data, translation_left_to_center); // Get the pose at the center of the camera (baseline/2 on X axis)
    
                    // Update camera position in the viewing window
                    //viewer.updateZEDPosition(camera_pose.pose_data);
    
                    // Get quaternion, rotation and translation
                    sl::float4 quaternion = camera_pose.getOrientation();
                    sl::float3 rotation = camera_pose.getEulerAngles(); // Only use Euler angles to display absolute angle values. Use quaternions for transforms.
                    sl::float3 translation = camera_pose.getTranslation();
    
                    // Display translation and rotation (pitch, yaw, roll in OpenGL coordinate system)
                    snprintf(text_rotation, MAX_CHAR, "%3.2f; %3.2f; %3.2f", rotation.x, rotation.y, rotation.z);
                    snprintf(text_translation, MAX_CHAR, "%3.2f; %3.2f; %3.2f", translation.x, translation.y, translation.z);
    
                    // Save the pose data in a csv file
                    if (outputFile.is_open())
                        outputFile << zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE) << "; " << text_rotation << "; " << text_translation << ";" << endl;
                    if (tm - last_msec_printed >= POSITION_PRINT_PERIOD)
                    {
                        last_msec_printed = tm;
    	                printf("rx;ry;rz = %s x,y,z = %s\n", text_rotation, text_translation);
                    }
    
                    // TODO: transform position according camera inclination

                    position_packet[0] = POSITION_PACKET_TYPE;
                    /* *((double *)(position_packet + 4)) = translation.x;
                    *((double *)(position_packet + 12)) = translation.y;
                    *((double *)(position_packet + 20)) = translation.z;
                    *((double *)(position_packet + 28)) = quaternion.x;
                    *((double *)(position_packet + 36)) = quaternion.y;
                    *((double *)(position_packet + 44)) = quaternion.z;
                    *((double *)(position_packet + 52)) = quaternion.w; 
                    printf("sending %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n", translation.x,
                        translation.y, translation.z, quaternion.x, quaternion.y, quaternion.z, quaternion.w);
                    */
    
                    *((double *)(position_packet + 4)) = translation.x;
                    *((double *)(position_packet + 12)) = translation.z;
                    *((double *)(position_packet + 20)) = rotation.y;

                    if (connected)
                    {
                      if (!send_packet(depth_socket, position_packet, 4 + 3 * sizeof(double)))
                      {
                         close(depth_socket);
                         connected = 0;
                         printf("could not sent. will try reconnecting...\n");
                      }
                      else printf("sent %.2lf %.2lf %.2lf\n", translation.x, translation.z, rotation.y / M_PI * 180.0);
                    }
                    else try_reconnecting();
                }
            }
            if (tm - last_msec_depth_reported >= DEPTH_REPORT_PERIOD)
            {
                last_msec_depth_reported = tm;
                uint8_t depth_packet[3604];
                uint8_t *map = depth_packet + 4;
                for (int i = 0; i < 3600; i++) map[i] = 0;

                for (int i = 0; i < zed_dep.getWidth(); i++)
                {
                  for (int j = 0; j < zed_dep.getHeight(); j++)
                  {
                     float z;
                     zed_dep.getValue(i, j, &z);
                     //fprintf(f, "%.4f ", z);
                     if (isnan(z)) continue;

                     double projection_to_top_plane = z * cos((camera_inclination_rad - view_angle_vertical[j]));
                     double vertical_dist = z * sin((camera_inclination_rad - view_angle_vertical[j]));
                     double perpendicular_dist = projection_to_top_plane * sin(view_angle_horizontal[i]);
                     double forward_dist = projection_to_top_plane * cos(view_angle_horizontal[i]);
                     if (projection_to_top_plane == 0.0) continue;

                     if ((vertical_dist > -0.45) && (vertical_dist < 0.5) && (forward_dist < 6.0) && (fabs(perpendicular_dist) < 3.0))
                     {
                        double map_y = (0.5 + forward_dist * 10); 
                        double map_x = (0.5 + perpendicular_dist * 10);
                        double dy = map_y / (0.5 + projection_to_top_plane * 10);
                        double dx = map_x / (0.5 + projection_to_top_plane * 10);
                        double map_x_save = map_x;
                        double map_y_save = map_y;
                        
                        while (map_y < 6.0)
                        {
                          fill_neighbors(map, (int)(0.5 + 30 + map_x), (int)(0.5 + map_y));
                          map_x += dx;
                          map_y += dy;
                        }

                        map_x = map_x_save;
                        map_y = map_y_save;
                        while (map_y > 0.0)
                        {
                          map_x -= dx;
                          map_y -= dy;
                          int index = 60 * (int)(0.5 + map_y) + (int)(0.5 + 30 + map_x);
                          if ((index >= 0) && (index < 3600)) if (map[index] == 0) map[index] = 2;  // 0=undefined, 1=obstacle, 2=free
                        }
                     }
                  }
                  //fprintf(f, "\n");
                }
                //fprintf(f, "---\n");
                if (connected)
                {
                    uint8_t depth_packet[4];
                    depth_packet[0] = DEPTH_PACKET_TYPE;
                    if (!send_packet(depth_socket, depth_packet, 3604))
                    {
                       close(depth_socket);
                       connected = 0;
                       printf("could not sent dpth. will try reconnecting...\n");
                    }
                    else printf("sent depth\n");
                }
                else try_reconnecting();
            }

            // Update rotation, translation and tracking state values in the OpenGL window
            //viewer.updateText(string(text_translation), string(text_rotation), tracking_state);
        } else sl::sleep_ms(1);
    }
}

/**
 *  Trasnform pose to create a Tracking Frame located in a separate location from the Camera Frame
 **/
void transformPose(sl::Transform &pose, float tx) {
    sl::Transform transform_;
    transform_.setIdentity();
    // Move the tracking frame by tx along the X axis
    transform_.tx = tx;
    // Apply the transformation
    pose = Transform::inverse(transform_) * pose * transform_;
}

/**
 * This function closes the ZED camera, its callback (thread) and the GL viewer
 **/
void close() {
    quit = true;
    zed_callback.join();
    zed.disableTracking("./ZED_spatial_memory"); // Record an area file

    zed.close();
    //viewer.exit();
}
