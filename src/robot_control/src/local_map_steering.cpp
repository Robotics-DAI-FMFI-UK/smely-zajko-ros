#include <stdlib.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "ros/ros.h"
#include "localMap/LocalMap.h"
#include "message_types/SbotMsg.h"
#include "message_types/RpLidarObstacle.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include <std_msgs/Float64.h>
#include "message_types/GpsAngles.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "string.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "netutil.h"

#define RAD2DEG 57.295779513

#define LOCALMAP_LOG_IMAGE_PATH "/home/zajko/logs/"

#define SOCKET_BUFFER_SIZE  65535
static uint8_t buffer[SOCKET_BUFFER_SIZE];
static uint8_t depth_buffer[SOCKET_BUFFER_SIZE];

static char localmap_log_filename[150];

ros::Publisher steeringPublisher;

bool sbot = false, hokuyo = false;

LocalMap* localMap;

int this_module_running;

//---------------------------------------
static char log_file_name[100];

long long msec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000L * tv.tv_sec + tv.tv_usec / 1000L;
}

void log_msg(const char *msg)
{
	FILE *f = fopen(log_file_name, "a+");
	long long tm = msec();
	fprintf(f, "%.2lf %s\n", tm / 1000.0, msg);
	fclose(f);
}

void log_msg(const char *msg, double val)
{
	FILE *f = fopen(log_file_name, "a+");
	long long tm = msec();
	fprintf(f, "%.2lf %s %.4lf\n", tm / 1000.0, msg, val);
	fclose(f);
}

void log_msg(const char *msg, double val1, double val2)
{
	FILE *f = fopen(log_file_name, "a+");
	long long tm = msec();
	fprintf(f, "%.2lf %s %.4lf %.4lf\n", tm / 1000.0, msg, val1, val2);
	fclose(f);
}

void setup_log_file()
{
    time_t tm;
    time(&tm);
    snprintf(log_file_name, 100, "%s%ld_local_map.log", LOCALMAP_LOG_IMAGE_PATH, tm);
    printf("logging to: %s\n", log_file_name);
    log_msg("started");	
}
//---------------------------------------

double d2r(double d) {
    return d * (pi / 180);
}

void say(const char *msg) {
    char out[256];
    snprintf(out, 255, "echo \"%s\" | espeak -a 200 -p 20 -s 80", msg);
    system(out);
}


int data[1081];
void hokuyoCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int i = 0;
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        data[i] = *it;
        i++;
    }
    localMap->setHokuyoData(data);
}

double angles[400];
double distances[400];
void rplidarCallback(const message_types::RpLidarObstacle &msg) {
    int rays = msg.rays;
    for(int i = 0; i < rays; i++) {
        distances[i] = msg.distances.data[i];
        angles[i] = msg.angles.data[i];
    }
    localMap->setRpLidarData(rays, distances, angles);
}

void sbotCallback(const message_types::SbotMsg &msg) {
    localMap->updateRobotPosition((long) msg.lstep, (long) msg.rstep);
}

void globalMapCallback(const message_types::GpsAngles &msg) {
    localMap->setGlobalMapData(d2r(msg.currWayHeading), d2r(msg.nextWayHeading), msg.distToWayEnd);
}

void imuCallback(const sensor_msgs::Imu &msg) {
    localMap->setCompassHeading(d2r(msg.orientation.x / 10));
}

void cameraCallback(const std_msgs::UInt8MultiArray::ConstPtr &array) {
    unsigned char data[3600];
    int i = 0;
    for (std::vector<unsigned char>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        data[i] = *it;
        i++;
    }
    localMap->setImageData(data);
}

//void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//void poseCallback(double tx, double ty, double tz, double qx, double qy, double qz, double qw) {
void poseCallback(double x, double y, double heading) {
    static time_t last_print = 0;

    // Camera position in map frame
    //double tx = msg->pose.position.x;
    //double ty = msg->pose.position.y;
    //double tz = msg->pose.position.z;

    // Orientation quaternion
    //tf2::Quaternion q(
    //        msg->pose.orientation.x,
    //        msg->pose.orientation.y,
    //        msg->pose.orientation.z,
    //        msg->pose.orientation.w);
    //tf2::Quaternion q(qx, qy, qz, qw);

    // 3x3 Rotation matrix from quaternion
    //tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    //double roll, pitch, yaw;
    //m.getRPY(roll, pitch, yaw);
//    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
//             msg->header.frame_id.c_str(),
//             tx, ty, tz,
//             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
//    ROS_INFO("X: %.2fcm Y: %.2fcm A: %.2f°", tx*100, ty*100, yaw * RAD2DEG);

    // coordinate system in local map is stupid (left handed with X left, Y back and Z down)
    // zed returns pose in not-stupid system (right handed with X forward, Y left and Z up)
    // need to flip axes and directions to make it work
    time_t tm;
    time(&tm);
    if (tm - last_print > 1)
    {
      printf("x=%.2lf, y=%.2lf, h=%.2lf\n", -y*100, x*100, -(heading / M_PI * 180.0));
      last_print = tm;
      log_msg("pose x, y: ", -y*100, x*100);
      log_msg("heading: ", -heading);
    }
	static double last_heading = 0;

    // do not update pose if it changed more than 30 deg per iteration	
    if (angleDiffAbs(last_heading, heading) < 30 / 180.0 * pi)
		localMap->setPose(-y*100, x*100, -heading);	
	
	last_heading = heading;
}

int c = 0;

void drawMap(const ros::TimerEvent&) {
    if (c > 1) {
        //localMap->doUpdate();
        c = 0;
    }
    c++;

    static char filename[200];
    static int log_file_counter = 0;
    
    cv::Mat img = localMap->getGui();
    
    snprintf(filename, 200, "%s%s/%dM.png", LOCALMAP_LOG_IMAGE_PATH, localmap_log_filename, log_file_counter);
    cv::imwrite(filename, img);
    
    localMap->addArrows(img);
    
    cv::imshow("local map", img);
    
    snprintf(filename, 200, "%s%s/%dA.png", LOCALMAP_LOG_IMAGE_PATH, localmap_log_filename, log_file_counter++);
    cv::imwrite(filename, img);
    
    cv::waitKey(1);
}

void process_packet(uint8_t *buffer, int size)
{
    uint8_t packet_type = buffer[0];
    if (packet_type == EVALUATED_IMAGE_PACKET_TYPE)
    {
       if (size != 3604)
       {
          printf("unexpected packet size %d, expected: %d\n", size, 3604);
          return;
       }
       localMap->setImageData(buffer + 4);
    }
}

void process_position_and_depth_packet(uint8_t *buffer, int size)
{
    uint8_t packet_type = buffer[0];
    if (packet_type == POSITION_PACKET_TYPE)
    {
       if (size != 4 + 3 * sizeof(double))
       {
          printf("unexpected packet size %d, expected: %ld\n", size, 4 + 3 * sizeof(double));
          return;
       }
      /* double tx, ty, tz, qx, qy, qz, qw;
       tx = *((double *)(buffer + 4));
       ty = *((double *)(buffer + 12));
       tz = *((double *)(buffer + 20));
       qx = *((double *)(buffer + 28));
       qy = *((double *)(buffer + 36));
       qy = *((double *)(buffer + 44));
       qw = *((double *)(buffer + 52)); */

       double x = *((double *)(buffer + 4));
       double y = *((double *)(buffer + 12));
       double heading = *((double *)(buffer + 20));

       // ODOMETRIA ZO ZED
       //poseCallback(x, y, heading);
    } else if (packet_type == DEPTH_MAP_PACKET_TYPE)
    {
      if (size != 3604)
      {
        printf("unexpected packet size %d, expected 3604\n", size);
        return;
      }
      localMap->setDepthMap(buffer + 4);
    }
}

int server_fd;
int socket_fd;
void *evaluated_image_subscriber_thread(void *arg)
{
    server_fd = create_server(EVALUATED_IMAGE_LISTENER_PORT);
    if (!server_fd)
    {
        printf("could not create evaluated image server\n");
        return 0;
    }
    do {
        socket_fd = wait_for_client_connection(server_fd);
	printf("jetson connected\n");
	do {
            int len = receive_packet(socket_fd, buffer, SOCKET_BUFFER_SIZE);

	    if (len == 0)
            {
                printf("could not receive packet\n");
                break;
            }
            process_packet(buffer, len);
        } while (this_module_running);
        printf("jetson disconnected\n");
        close(socket_fd);
    } while (this_module_running);
    close(server_fd); 
    printf("eval image thread terminated\n");
    return 0;
}

int depth_server_fd;
int depth_socket_fd;
void *position_and_depth_map_thread(void *arg)
{
    depth_server_fd = create_server(POSITION_AND_DEPTH_MAP_PORT);
    if (!depth_server_fd)
    {
        printf("could not create position and depth server\n");
        return 0;
    }
    do {
        depth_socket_fd = wait_for_client_connection(depth_server_fd);
	printf("jetson2 connected depth\n");
	do {
            int len = receive_packet(depth_socket_fd, depth_buffer, SOCKET_BUFFER_SIZE);
	    if (len == 0)
            {
                printf("could not receive depth packet\n");
                break;
            }
            process_position_and_depth_packet(depth_buffer, len);
        } while (this_module_running);
        printf("jetson2 disconnected depth\n");
        close(depth_socket_fd);
    } while (this_module_running);
    close(depth_server_fd); 
    printf("position and depth thread terminated\n");
    return 0;
}

void start_evaluated_image_subscriber()
{
    pthread_t t;
    if (pthread_create(&t, 0, evaluated_image_subscriber_thread, 0) != 0)
    {
        perror("could not start eval image thread");
        return;
    }
}

void start_position_and_depth_map_thread()
{
    pthread_t t;
    if (pthread_create(&t, 0, position_and_depth_map_thread, 0) != 0)
    {
        perror("could not start position and depth map thread");
        return;
    }
}

void prepare_log_directory()
{
    time_t tm;
    time(&tm);    
    snprintf(localmap_log_filename, 150, "%s%ld_localMap", LOCALMAP_LOG_IMAGE_PATH, tm);
    mkdir(localmap_log_filename, 0777);
    snprintf(localmap_log_filename, 150, "%ld_localMap", tm);
}	

int main(int argc, char **argv) {

    prepare_log_directory();
    setup_log_file();
        
    this_module_running = 1;
    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;

    ros::Subscriber sbot_subscriber = nh.subscribe("/control/base_data", 2, sbotCallback);
    ros::Subscriber hokuyo_subscriber = nh.subscribe("/sensors/hokuyo_publisher", 2, hokuyoCallback);
    ros::Subscriber rplidar_subscriber = nh.subscribe("/sensors/rplidar_publisher", 2, rplidarCallback);
    ros::Subscriber global_map_subscriber = nh.subscribe("/control/localization_and_planning", 2, globalMapCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("/sensors/imu_publisher", 2, imuCallback);
//    ros::Subscriber camera_subscriber = nh.subscribe("/sensors/camera/evaluated_image", 2, cameraCallback);
    start_evaluated_image_subscriber();
    start_position_and_depth_map_thread();

    ros::Publisher heading_publisher = nh.advertise<std_msgs::Float64>("/control/local_map", 10);

    localMap = new LocalMap(600, 600, heading_publisher);

    ros::Timer draw = nh.createTimer(ros::Duration(0.5), drawMap);

//    cv::imshow("local map", localMap->getGui());//    ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
//             msg->header.frame_id.c_str(),
//             tx, ty, tz,
//             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);

//    cv::waitKey(0);

    ros::spin();
    this_module_running = 0;
    close(server_fd);
    close(socket_fd);

    return 0;
}
