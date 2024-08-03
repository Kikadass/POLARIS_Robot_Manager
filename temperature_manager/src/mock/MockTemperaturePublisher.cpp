#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "temperature_publisher");
  ros::NodeHandle nh;

  ros::Publisher topic_pub = nh.advertise<std_msgs::Float32>("temperature", 1000);
  ros::Rate loop_rate(1);

  const float initialTemperature = 30.0f;
  const float limit = 55.0f;
  float currentTemperature = initialTemperature;
  const float additionAmount = (limit - initialTemperature) / 30.0f;  // Total decrement over 30 seconds

  while (ros::ok())
  {
    std_msgs::Float32 msg;
    msg.data = currentTemperature;

    topic_pub.publish(msg);
    ROS_INFO("Published Temperature (C): %f", msg.data);

    // Update the current value
    if (currentTemperature >= limit)
      currentTemperature = 60.0f;
    else
      currentTemperature += additionAmount;

    ros::spinOnce();
    loop_rate.sleep();
  }
}