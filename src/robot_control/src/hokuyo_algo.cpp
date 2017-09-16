#include "ros/ros.h"

#include "std_msgs/Int32MultiArray.h"
#include "message_types/HokuyoObstacle.h"
#include "hokuyo_algos/openPath/OpenPath.h"
#include "hokuyo_algos/basicAlgo/BasicAlgo.h"
#include "hokuyo_algos/previousAlgo/PreviousAlgo.h"

ros::Publisher prevPtr;
ros::Publisher basicPtr;
OpenPath op = OpenPath();
BasicAlgo *ba;
PreviousAlgo *pa;

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr &array) {
    int arr[1081];
    int i = 0;
    for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        arr[i] = *it;
        i++;
    }
    message_types::HokuyoObstacle a = ba->getPaths(arr);

    basicPtr.publish(a);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "hokuyo_algo");
    ros::NodeHandle n;
    pa = new PreviousAlgo();
    ba = new BasicAlgo();
    ba->init();
    pa->init();
    ros::Subscriber subscriber = n.subscribe("/sensors/hokuyo_publisher", 100, arrayCallback);
    basicPtr = n.advertise<message_types::HokuyoObstacle>("basic_algo", 10);

    ros::spin();

    return 0;
}