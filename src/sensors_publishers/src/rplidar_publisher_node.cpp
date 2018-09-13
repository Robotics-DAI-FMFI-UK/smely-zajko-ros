#include "ros/ros.h"

#include "rplidar/RpLidar.h"
#include "message_types/RpLidarObstacle.h"

using namespace std;

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "rplidar_publisher");
    ros::NodeHandle nh;
    ros::Publisher rplidar_publisher = nh.advertise<message_types::RpLidarObstacle>("rplidar_publisher", 1);

    ros::Rate loop_rate(10);

    RpLidar *rplidar = new RpLidar();
    rplidar->init();

    ros::Duration(0.5).sleep();

    while (ros::ok()) 
    {
        if (rplidar_publisher.getNumSubscribers() > 0) 
        rplidar->get_data(msg);
        rplidar_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    rplidar->program_runs = 0;

    return 0;
}
