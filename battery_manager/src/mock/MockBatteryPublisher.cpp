#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_percentage_publisher");
    ros::NodeHandle nh;

    ros::Publisher topic_pub = nh.advertise<std_msgs::Float32>("battery_percentage", 1000);
    ros::Rate loop_rate(1);

    const float initialPercentage = 100.0f;
    const float limit = 51.0f;
    float currentPercentage = initialPercentage;
    const float reductionAmount = (initialPercentage - limit) / 30.0f; // Total decrement over 30 seconds

    while (ros::ok())
    {
        std_msgs::Float32 msg;
        msg.data = currentPercentage;

        topic_pub.publish(msg);
        ROS_INFO("Published Battery Percentage: %f", msg.data);

        // Update the current value
        if (currentPercentage <= limit) currentPercentage = 49.0f;
        else currentPercentage -= reductionAmount;
        
        loop_rate.sleep();
    }
}