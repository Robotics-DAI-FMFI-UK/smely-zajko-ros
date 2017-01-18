#include <vector>

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "hokuyo/Hokuyo.h"


void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int arr[AbstractHokuyo::RANGE_DATA_COUNT];
    int i = 0;
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        arr[i] = *it;
        i++;
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "hokuyo_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscriber = n.subscribe("hokuyo_publisher", 100, arrayCallback);

    ros::spin();

    return 0;
}