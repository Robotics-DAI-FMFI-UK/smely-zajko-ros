#include <stdlib.h>
#include "ros/ros.h"
#include "localizationAndPlanning/LocalizationAndPlanning.h"
#include "message_types/GpsAngles.h"
#include "message_types/SbotMsg.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8.h>
#include "string.h"
  
#include "netutil.h" 

#define QR_PORT 9770

#define SOCKET_BUFFER_SIZE 100

//int cameraAction;

const int DETECT_ROAD = 0;
const int DETECT_QR = 1;

//ros::Publisher cameraActionPublisher;

ros::Publisher locPublisher;
LocalizationAndPlanning *localizationAndPlanning = new LocalizationAndPlanning(500, 500);

message_types::SbotMsg sbot_msg;

sensor_msgs::NavSatFix loadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix unloadingPoint = sensor_msgs::NavSatFix();
sensor_msgs::NavSatFix destinationPoint = sensor_msgs::NavSatFix();

sensor_msgs::NavSatFix newTarget = sensor_msgs::NavSatFix();
bool targetValid = false;

int headingState;

static int this_module_running;
static int server_fd, socket_fd;
static uint8_t socket_buffer[SOCKET_BUFFER_SIZE];

const int START = 0;
const int HEADING_LOADING = 1;
const int LOADING = 2;
const int HEADING_UNLOADING = 3;
const int UNLOADING = 4;
const int HEADING_DEST = 5;
const int END = 6;

const float FULL = 1;
const float EMPTY = 0;

int debug_payload = 0;

//---------------------------------------
#define LOG_FILE_DIR "/home/zajko/logs/"
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
	fprintf(f, "%.2lf %s %.8lf\n", tm / 1000.0, msg, val);
	fclose(f);
}

void log_msg(const char *msg, double val1, double val2)
{
	FILE *f = fopen(log_file_name, "a+");
	long long tm = msec();
	fprintf(f, "%.2lf %s %.8lf %.8lf\n", tm / 1000.0, msg, val1, val2);
	fclose(f);
}

void setup_log_file()
{
    time_t tm;
    time(&tm);
    snprintf(log_file_name, 100, "%s%ld_localization_and_planning.log", LOG_FILE_DIR, tm);
    printf("logging to: %s\n", log_file_name);
    log_msg("started");	
}
//---------------------------------------

void sbotCallback(const message_types::SbotMsg &msg) {
    sbot_msg = msg;
}

void targetCallback(const sensor_msgs::NavSatFix &dest) {
    newTarget = dest;
    targetValid = true;
    printf("new target from QR: %f %f\n", dest.latitude, dest.longitude);
}

void resetTarget() {
    targetValid = false;
}

void setState(int state) {
    headingState = state;
}

//void setCameraAction(int action) {
//    cameraAction = action;
//}

void send_qr_request(uint8_t msg)
{
    send(socket_fd, &msg, 1, 0);  
    printf("sending action\n");
}

void say(const char *msg) {
    char out[256];
    snprintf(out, 255, "echo \"%s\" | espeak -a 200 -p 20 -s 80", msg);
    system(out);
}

int spamCounter = 0;
static sensor_msgs::NavSatFix service_area_gps_remember;

void gpsCallback(const sensor_msgs::NavSatFix &gps) {
    message_types::GpsAngles actualHeading = localizationAndPlanning->update(gps);
    std_msgs::UInt8 actionMsg;

    log_msg("gps ", gps.latitude, gps.longitude);

    if (headingState == START) {
        if (!targetValid) {
            //setCameraAction(DETECT_QR);
            send_qr_request(DETECT_QR);
            if (spamCounter == 0) say("WAITING FOR QR CODE");
            spamCounter++;
            if (spamCounter > 10) spamCounter = 0;
        } else {
            say("HEADING LOADING");
            log_msg("heading loading");
            destinationPoint = gps;
            log_msg("Final destination point lat, long: ", destinationPoint.latitude, destinationPoint.longitude);
            localizationAndPlanning->setDestination(newTarget);
            setState(HEADING_LOADING);
            send_qr_request(DETECT_ROAD);
            //setCameraAction(DETECT_ROAD);
        }
    }
    if (actualHeading.map == DBL_MAX) {
        if (headingState == HEADING_LOADING) {
            printf("LOADING\n");
            say("LOADING");
            log_msg("loading");

            setState(LOADING);
            resetTarget();
            send_qr_request(DETECT_QR);
            //setCameraAction(DETECT_QR);
            send_qr_request(DETECT_QR);
        } else if (headingState == LOADING) {
            if (sbot_msg.payload == FULL && targetValid) {
                printf("HEADING_UNLOADING\n");
                say("HEADING UNLOADING");
                log_msg("heading unloading");

                localizationAndPlanning->setDestination(newTarget);
                setState(HEADING_UNLOADING);
                send_qr_request(DETECT_ROAD);
                //setCameraAction(DETECT_ROAD);
            } else {
                //setCameraAction(DETECT_QR);
                send_qr_request(DETECT_QR);
                if (spamCounter == 0) {
                    if (sbot_msg.payload != FULL) say("LOADING");
                    else say("WAITING FOR QR CODE");
                }
                spamCounter++;
                if (spamCounter > 10) spamCounter = 0;
            }
        } else if (headingState == HEADING_UNLOADING) {
            printf("UNLOADING\n");
            say("UNLOADING");
            log_msg("unloading");

            setState(UNLOADING);
            resetTarget();
        } else if (headingState == UNLOADING) {
            if (sbot_msg.payload == EMPTY) {
                printf("HEADING_DEST\n");
                say("HEADING DESTINATION");
                log_msg("heading destination");

                localizationAndPlanning->setDestination(destinationPoint);
                setState(HEADING_DEST);
                //setCameraAction(DETECT_ROAD);
            } else {
                if (spamCounter == 0) say("UNLOADING");
                spamCounter++;
                if (spamCounter > 10) spamCounter = 0;
            }
        } else if (headingState == HEADING_DEST) {
            printf("FINISH\n");
            log_msg("finish");
            setState(END);
            say("I am done. Can we go again?");
        }
    }

    actualHeading.headingState = headingState;
    //actionMsg.data = cameraAction;

//    printf("Distance: %f\n", localizationAndPlanning->distance(localizationAndPlanning->destinationPoint,
//                                                             localizationAndPlanning->curPoint) * 1000);
    locPublisher.publish(actualHeading);
    //cameraActionPublisher.publish(actionMsg);
    //TODO: send QR code read request
   

    cvShowImage("loc and planning", localizationAndPlanning->getGui());
    cv::waitKey(1);
}

static void process_packet(const uint8_t *packet, int length)
{
    double longitude, latitude;

    sensor_msgs::NavSatFix dest;
    sscanf((const char *)packet, "%lf %lf", &(dest.latitude), &(dest.longitude));
    printf("new target: lat=%lf long=%lf\n", dest.latitude, dest.longitude);
    
    newTarget = dest;
    targetValid = true;
}

void *qr_thread(void *arg)
{
    server_fd = create_server(QR_PORT);
    if (!server_fd)
    {
        printf("could not create qr port server\n");
        return 0;
    }
    do {
        socket_fd = wait_for_client_connection(server_fd);
        printf("jetson connected qr\n");

        do {
            int len = receive_packet(socket_fd, socket_buffer, SOCKET_BUFFER_SIZE);
            if (len == 0)
            {
                printf("could not receive qr packet\n");
                break;
            }
            process_packet(socket_buffer, len);
        } while (this_module_running);
        printf("jetson disconnected qr\n");
        close(socket_fd);
    } while (this_module_running);
    close(server_fd); 
    printf("qr thread terminated\n");
    return 0;
}


void create_qr_notify_thread()
{
    pthread_t t;
    if (pthread_create(&t, 0, qr_thread, 0) != 0)
    {   
        perror("could not start eval image thread");
        return;
    }
}

int main(int argc, char **argv) {

    setup_log_file();
    
    this_module_running = 1;
    create_qr_notify_thread();

    log_msg("ros init");

    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    log_msg("subscribing...");

    ros::Subscriber gps_subscriber = nh.subscribe("/sensors/gps_publisher", 1, gpsCallback);

    ros::Subscriber sbot_subscriber = nh.subscribe("/control/base_data", 10, sbotCallback);

    //ros::Subscriber target_subscriber = nh.subscribe("/control/camera_qr_target", 1, targetCallback);

    locPublisher = nh.advertise<message_types::GpsAngles>("localization_and_planning", 10);
    //cameraActionPublisher = nh.advertise<std_msgs::UInt8>("/control/camera_action", 10);

    //localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/zilina.osm");
    //localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/matfyz.osm");
    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/deggendorf2019.osm");
//    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/lednice_velka.osm");
//    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/botanicka.osm");
//    localizationAndPlanning->readMap((char *) "/home/zajko/Projects/smely-zajko-ros/resources/maps/homologacie_fei.osm");

//    loadingPoint.latitude = 49.2129610;
//    loadingPoint.longitude = 18.7447602;

/*    unloadingPoint.latitude = 49.2126778;
    unloadingPoint.longitude = 18.7439919;

    destinationPoint.latitude = 49.2128361;
    destinationPoint.longitude = 18.7446799;
*/

/* homologacia fei: */
 
//    loadingPoint.latitude = 48.15143;
//    loadingPoint.longitude = 17.0729;
//
//    unloadingPoint.latitude = 48.15159;
//    unloadingPoint.longitude = 17.07298;
//
//    destinationPoint.latitude = 48.1517;
//    destinationPoint.longitude = 17.07329;

    /*
    loadingPoint.latitude = 48.1457841;
    loadingPoint.longitude = 17.0740046;


    unloadingPoint.latitude = 48.1458947;
    unloadingPoint.longitude = 17.0722153;


    destinationPoint.latitude = 48.1473408;
    destinationPoint.longitude = 17.0725359;
    */

    // default bod aby LocalizationAndPlanning nerobilo chyby kym sa caka na QRkod
    loadingPoint.latitude = 48.14703;
    loadingPoint.longitude = 17.07314;
    localizationAndPlanning->setDestination(loadingPoint);

    destinationPoint.latitude = 48.80182;
    destinationPoint.longitude = 16.8059;

    headingState = START;

    say("WAITING FOR GPS");

    log_msg("spinning ros...");

    ros::spin();

    return 0;
}
