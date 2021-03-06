#include "ros/ros.h"

#include "rplidar/RpLidar.h"
#include "message_types/RpLidarObstacle.h"

#define MIN_OBSTACLE_DISTANCE 300.0

using namespace std;

const double pi = 3.141592653589793238463;

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "rplidar_publisher");
    ros::NodeHandle nh;
    ros::Publisher rplidar_publisher = nh.advertise<message_types::RpLidarObstacle>("rplidar_publisher", 1);

    ros::Rate loop_rate(10);

    RpLidar *rplidar = new RpLidar();
    rplidar->init();

    ros::Duration(0.5).sleep();

    lidar_data_type msg;
    message_types::RpLidarObstacle rpl_obstacle;

    while (ros::ok()) 
    {
        if (rplidar_publisher.getNumSubscribers() > 0)  {
            rplidar->get_data(&msg);

//	        rpl_obstacle.rays = msg.count;
            uint8_t possible_obstacles = 0;
            rpl_obstacle.distances.data.clear();
            rpl_obstacle.angles.data.clear();

            int filter_count = 0;
            for (size_t i = 0; i < msg.count; ++i) {
                if (msg.distance[i] < 2000) continue;
                rpl_obstacle.distances.data.push_back(msg.distance[i]);
                rpl_obstacle.angles.data.push_back((msg.angle[i] / 64) * (pi / 180.0));
                filter_count++;

		        if (msg.distance[i] < MIN_OBSTACLE_DISTANCE) possible_obstacles++;
            }
            rpl_obstacle.obstacle = (uint8_t) (possible_obstacles > 3);
            rpl_obstacle.rays = filter_count;

            rplidar_publisher.publish(rpl_obstacle);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    rplidar->stop();

    return 0;
}
