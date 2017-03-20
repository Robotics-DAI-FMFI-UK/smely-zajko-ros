#include "ros/ros.h"

#include "std_msgs/Int32MultiArray.h"
#include "hokuyo_algos/openPath/OpenPath.h"
#include "hokuyo_algos/basicAlgo/BasicAlgo.h"

ros::Publisher pubPtr;
OpenPath op = OpenPath();
BasicAlgo *ba;

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int arr[1081];
    int i = 0;
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        arr[i] = *it;
        i++;
    }
    std_msgs::Float64MultiArray a = ba->getPaths(arr);
    for (int i = 0; i <= 10; i++) {
        //ROS_ERROR("%lf", a[i]);
        std::cout << a.data[i];
    }
    std::cout << std::endl;
    pubPtr.publish(a);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "hokuyo_algo");
    ros::NodeHandle n;
    ba = new BasicAlgo();
    ba->init();
    ros::Subscriber subscriber = n.subscribe("/sensors/hokuyo_publisher", 100, arrayCallback);
    pubPtr = n.advertise<std_msgs::Float64MultiArray>("hokuyo_algo", 10);

    ros::spin();

    return 0;
}