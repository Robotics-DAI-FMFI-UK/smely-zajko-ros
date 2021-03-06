#include <stdlib.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "robot/AbstractRobot.h"
#include "robot/Robot.h"
#include "message_types/SbotMsg.h"
#include "message_types/GpsAngles.h"
#include "message_types/HeadingState.h"
#include "message_types/HokuyoObstacle.h"
#include "message_types/SteeringMsg.h"

#define USE_LOCAL_MAP 1

message_types::GpsAngles gps_msg;
message_types::HeadingState state_msg;

double local_map_heading;

sensor_msgs::Imu imu_msg;
cv::Mat image;
std::vector<double> hokuyo_algo_msg;
std::vector<double> camera_prediction_msg;
message_types::SbotMsg sbot_msg;

ros::Publisher directionPublisher;

//ros::Publisher steeringPublisher;

const int START = 0;
const int HEADING_LOADING = 1;
const int LOADING = 2;
const int HEADING_UNLOADING = 3;
const int UNLOADING = 4;
const int HEADING_DEST = 5;
const int END = 6;

int previousState = -1;

int direction = 0;

Robot *robot;

static volatile int8_t hokuyo_sees_obstacle;

int said_wrong = 0;

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
    snprintf(log_file_name, 100, "%s%ld_robot_control.log", LOG_FILE_DIR, tm);
    printf("logging to: %s\n", log_file_name);
    log_msg("started");	
}
//---------------------------------------

void localMapCallback(const std_msgs::Float64 &msg) {
    local_map_heading = msg.data;
}

void sbotCallback(const message_types::SbotMsg &msg) {
    sbot_msg = msg;
}

void localizationAndPlanningCallback(const message_types::GpsAngles &msg) {
    gps_msg = msg;
}

void imuCallback(const sensor_msgs::Imu &msg) {
    imu_msg = msg;
}

void setSteering(int direction, int speed) {
    robot->set_direction(direction);
    robot->set_speed(speed);
}

void stopNowAbruptly() {
    robot->stop_now();
    robot->set_direction(0);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
    }
}

void say(const char * msg) 
{
    char out[256];
    snprintf(out, 255, "echo \"%s\" | espeak -a 200 -p 20 -s 80", msg);
    system(out);
}

//TODO: refactor this
double predicted_dir = 0.0;
double running_mean = 0.0;
double running_mean_weight = 0.5;

void hokuyoAlgoCallback(const message_types::HokuyoObstacle::ConstPtr &msg) {
//    double max = 0.0;
//    int actual_direction = 0;
//    int j = 0;

    hokuyo_algo_msg = msg->distances.data;
    hokuyo_sees_obstacle = msg->obstacle;

/*
    for (std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
        if (*it > max) {
            max = *it;
            actual_direction = j;
        }
        j++;
    }
*/

    // FIXME: Naco dva krat?
    /*if (max < 0.3) {
        //TODO: Dozadu backing dorobit
        printf("Tu by sa niekedy v buducnosti mohlo aj cuvat\n");
    } else {
        actual_direction = 5 * (actual_direction - 5);

        predicted_dir = (running_mean * running_mean_weight) + (actual_direction * (1 - running_mean_weight));
        running_mean = (running_mean * 3.0 + predicted_dir) / 4.0;

        direction = (int) (running_mean + 0.5);
    }*/
}

void cameraPredictionCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    camera_prediction_msg = msg->data;
}

int move() {
    int display_direction = 0;
    bool autonomy = true;
    bool status_from_subroutines;

    std::vector<double> vDist;
    std::vector<double> lDist;
    std::vector<double> move_probs;
    int isChodnik = 0;
    double delta;
    double computed_dir;
    int wrong_dir = 0;
    double imuAngle = imu_msg.orientation.x;

    double max_neural_dir = 0;
    double max_neural_dir_val = 0.0;
    int neuron_dir = 0;
    // TODO: z lokalizacie
    double mapAngle = gps_msg.map;

    double speed_down_dst = 0.003;
    std::string move_status;

    for (int i = 0; i < 11; i++) {
        double f = (camera_prediction_msg.size() ? camera_prediction_msg[i] : 0.0);
        double g = hokuyo_algo_msg.size() ? hokuyo_algo_msg[i] : 0.0;
        if (USE_LOCAL_MAP) {
            f = 1.0;
            g = 1.0;
        }
        if (f < 0) {
            f = 0;

        } else if (f > 0.3 && g > 0.5) {
            isChodnik = 1;
        }

        if (f > max_neural_dir_val) {
            max_neural_dir_val = f;
            max_neural_dir = i;
        }

        vDist.push_back(f);
        lDist.push_back(g);
    }
    neuron_dir = max_neural_dir;

    // delta
    if (mapAngle == DBL_MIN) {
        delta = 0;
    } else {
        // delta = ((int)(mapAngle /* - (imuData.xAngle / 10) */ + 360))%360 ;
        delta = ((int) (mapAngle - imuAngle / 10 + 360)) % 360;
    }
    if (delta > 180)
        delta = delta - 360;

    // heading roughly the right way
    if (abs(delta) < 40) {
        wrong_dir = 0;
    }

    // significantly off course(> 150deg), turn
    //if (abs(delta) > 150 || wrong_dir) {
    if (0) {
        if (!status_from_subroutines)
            move_status = "turning";
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Going in wrong direction, \
         delta %f turning...\n",delta);
        log_msg("going in wrong direction", delta);
        say("its the other way, zyco");
        wrong_dir = 1;
        if (delta > 0) {
            if (autonomy) {
                setSteering(80, 1);
            }
            display_direction = 5;
        } else {
            if (autonomy) {
                setSteering(-80, 1);
            }
            display_direction = -5;
        }

    } else if (isChodnik == 0) {
        // no valid direction from vision
        // TODO presun do subroutines

        if (!status_from_subroutines)
            move_status = "searching";

         printf("!!!!!!!!!!!!!!!!!!!!!!! Chodnik missing searching..\n");
        if (delta > 0) {
            if (autonomy) {
                setSteering(40, -1);
            }
            display_direction = 5;
        } else {
            if (autonomy) {
                setSteering(-40, -1);
            }
            display_direction = -5;
        }
    } else {
        if (!status_from_subroutines) {
            move_status = "running";
        }
        if (delta > 90) {
            delta = 89.3;
        } else if (delta < -90) {
            delta = -89.3;
        }
        delta /= 18.0;

        double fmax = -1;
        int maxdir = 5;
        move_probs.clear();
        for (int i = 0; i <= 10; i++) {
            double coeff = 5 - abs(delta - (i - 5));
            if (coeff < 0.0)
                coeff = 0.000001; //0.1;
            coeff /= 25.0; // vyskusat 5, 12 ...
            coeff += 1.0;
            // TODO x 1if hoku sez > 1.5m else 0.1
            // lDist[i] = 1.0;
            double f = vDist[i] * coeff * lDist[i];
            move_probs.push_back(f);
            if (f > fmax) {
                fmax = f;
                maxdir = i;
            }
        }
        for (int i = 0; i < move_probs.size(); i++) {
            move_probs[i] /= fmax;
            move_probs[i] = 1 - move_probs[i];
        }

        int sdir = (maxdir - 5) * 8;
        double heading;
        if (USE_LOCAL_MAP) {
                        if (local_map_heading >= 9999) // GOING_WRONG
                        {
                          setSteering(80, 6);
                          if (!said_wrong)
                          {
                            say("its the other way, zyco");
                            said_wrong = 1;
                          }
                          return 0;
                        }
                        else if (said_wrong) 
                        {
                          said_wrong = 0;
                          say("corrected");
                        }
			heading = local_map_heading * (180 / M_PI);
			// 0.4444 comes from 180 deg => turning rate 80              
			sdir = 0.5 + heading * 0.4444;

			// we try to amplify turning for small angles		
			if (fabs(heading) < 45)
			  sdir *= 2;
        }
        // sdir -= 3;

        if (sbot_msg.away_from_left)
            sdir += 20;
        if (sdir > 25)
            sdir = 25;
        if (sbot_msg.away_from_right)
            sdir -= 20;
        if (sdir < -25)
            sdir = -25;

        predicted_dir = (running_mean * running_mean_weight) + (sdir * (1 - running_mean_weight));

        //running_mean = (running_mean * 3.0 + predicted_dir) / 4.0;
        running_mean = (running_mean * 1.0 + predicted_dir * 3.0) / 4.0;

         printf("Inferred dir: %d\tProposed dir: %f\n", sdir, predicted_dir);
         log_msg("local_map_heading, sdir: ", heading, sdir);
         log_msg("pdir, rmean: ", predicted_dir, running_mean);

        computed_dir = sdir;

        if (autonomy) {
            // printf("%.10f %.10f\n", angles.dstToHeadingPoint, speed_down_dst);
            if (gps_msg.dstToHeadingPoint <= speed_down_dst) {
                setSteering(predicted_dir, 7);
                // printf("setSpeed: 7\n");
            } else {
                setSteering(predicted_dir, 14);
                // printf("setSpeed: 10\n");
            }
        }
        std_msgs::Float64 directionMsg;
        directionMsg.data = predicted_dir;
        directionPublisher.publish(directionMsg);
        display_direction = maxdir;
    }

    // printf("delta= %f\n", delta);

    return display_direction;
}

void avoid_obstacle(ros::Rate *loop_rate)
{
    setSteering(0, 0); // TODO: check if we need this
    stopNowAbruptly();

    say("step away, please");
    // wait for the obstacle to go away
    int waiting = 0;
    int not_see_counter = 0;
    while (waiting < 400)
    {
      if (ros::ok())
      {
        ros::spinOnce();
        loop_rate->sleep();
      } 
      waiting++;
      if (!hokuyo_sees_obstacle) 
      {
         not_see_counter++;
         if (not_see_counter > 20)
         {
           say("thanks");
           return;
         }
      }
      else not_see_counter = 0;
    } 

    say("watch out behind me");
    // obstacle is still in front of us after 30 seconds, try backing up a little bit
    setSteering(0, -8);
    waiting = 0;
    
    // just a couple of seconds of backing up
    while (waiting < 200)
    {
      if (ros::ok())
      {
        ros::spinOnce();
        loop_rate->sleep();
      } 
      waiting++;
    } 
    // now try to resume... or end up in this function again if obstacle still seen (todo: try to turn)
}

int main(int argc, char **argv) {
	setup_log_file();
    ros::init(argc, argv, "ros_control");
    ros::NodeHandle nh;
    ros::Subscriber sbot_subscriber = nh.subscribe("/control/base_data", 3, sbotCallback);
    ros::Subscriber localization_and_planning_subscriber = nh.subscribe("localization_and_planning", 10,
                                                                        localizationAndPlanningCallback);
    ros::Subscriber hokuyo_algo_subscriber = nh.subscribe("basic_algo", 10, hokuyoAlgoCallback);
    ros::Subscriber imu_subscriber = nh.subscribe("/sensors/imu_publisher", 10, imuCallback);
    ros::Subscriber camera_prediction_traingle_subscriber = nh.subscribe("/control/camera_triangles_prediction", 10,
                                                                         cameraPredictionCallback);

    ros::Subscriber local_map_subscriber = nh.subscribe("/control/local_map", 10, localMapCallback);

//    steeringPublisher = nh.advertise<message_types::SteeringMsg>("/control/steering", 3);

//    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber sub = it.subscribe("/sensors/camera/image", 1, imageCallback);

    directionPublisher = nh.advertise<std_msgs::Float64>("directionPublisher", 10);

    ros::Publisher base_publisher = nh.advertise<message_types::SbotMsg>("/control/base_data", 3);

    robot = new Robot(base_publisher);

    ros::Rate loop_rate(20);

    int index = 0;
    while (ros::ok()) {
        // printf("CURRENT STATE: %d PAYLOAD: %f\n", gps_msg.headingState, sbot_msg.payload);

        if (previousState == LOADING && gps_msg.headingState == HEADING_UNLOADING ||
            previousState == UNLOADING && gps_msg.headingState == HEADING_DEST) {
            ros::Duration(3).sleep();
        }

        if (gps_msg.headingState == START ||
            gps_msg.headingState == LOADING ||
            gps_msg.headingState == UNLOADING ||
            gps_msg.headingState == END) {
            setSteering(0, 0);
        } else {
            if (hokuyo_sees_obstacle)
               avoid_obstacle(&loop_rate);
            else
               move();
        }

        previousState = gps_msg.headingState;

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
