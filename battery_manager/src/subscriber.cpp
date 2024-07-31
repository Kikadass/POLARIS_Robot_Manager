#include <ros/ros.h>
#include <std_msgs/Float32.h>

void writeMsgToLog(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("The message that we received was: %f", msg->data);

    if (msg->data <= 50.0f) ROS_ERROR("The message that we received was: %f", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber topic_pub = nh.subscribe("battery_percentage", 1000, writeMsgToLog);

    ros::spin();
}