# POLARIS_Robot_Manager

This s a tiny Robot manager Using Gazebo, ROS and POLARIS_GEM_e2 done within a week, with the free time I could find.

# Dependencies

## Docker

Please install docker in you machine if you don´t already have it.
https://docs.docker.com/engine/install/

## VcXsrv

To be able to get the windows from Docker display in a Windows machine, you would need VcXsrv
https://sourceforge.net/projects/vcxsrv/

All other dependencies will be downloaded and installed in the Docker container

# Docker

## Build

First of all the Docker container needs to be built from the repository directory with:

```
docker compose build
```

## Run Docker container

Docker compose has been used to add useful parameters to be able to get Gazebo and RViz windows on Windows machines.
Run the following from the repository directory:

```
docker-compose up
```

## Attach a terminal to the container

If needed, attach a terminal to the container like so:

First, Get the container ID or NAME

```
docker ps
```

Example:

```
$ docker ps
CONTAINER ID   IMAGE                          COMMAND                  CREATED         STATUS         PORTS     NAMES
0e387b9d222f   polaris_simulator_user_image   "/ros_entrypoint.sh …"   2 minutes ago   Up 4 seconds             polaris_robot_manager-polaris_simulator_user-1
682ebed3d465   polaris_simulator_dev_image    "/ros_entrypoint.sh …"   2 minutes ago   Up 5 seconds             polaris_robot_manager-polaris_simulator_dev-1
```

You will see there are two docker containers, one will share a volume between the host machine and the docker container for development.
The other will just pull the git repository, compile and set things up to just run the latest code.
Use the user container.

Second, make execute a new bash terminal inside the docker container (RECOMMENDED):

```
docker exec -it <container_name_or_id> /bin/bash
```

Or just attach it to the main process like so:

> [!WARNING]
> Note that if you do this, when exiting this terminal, it will stop the container.

```
docker attach <container_name_or_id>
```

# Run

First of all follow the steps to set up Docker, run the docker container and attach a terminal to the polaris_simulator_user_image container shown above.

## Run Robot Status Manager

An alias has been created for easy use.
Just run:

```
launchRobotStatusManager
```

After launching the Robot Status Manager feel free to mock any of the sensors with some of the following commands in a different terminal:

```
mockBattery
mockGpsAccuracy
mockNoSignal
mockMixedUnstableSignal
mockTemperature
```

# Design choices

The software architecture of this project is simple. First of all, there is a manager for each sensor that monitors its values and interprets them into acceptable behaviour or into error states. The final state is then published for other systems to read and do with that information what they wish. This adds abstraction and flexibility to the code, reducing code duplication in the case that multiple parts of the robotic platform need the same data.

In this project the Robot Status Manager is the one in charge of reading all different sensor status and determining whether the robot is in a healthy condition or not. This gives, yet, another level of abstraction for other parts of the system to not worry about the sensors at all, but just read the overall Robot Status when needed.

Additionally, there would be a Navigation Manager that would subscribe to the Robot Status, and send commands to the robot to move around the map from one waypoint to another, only when the robot status would be either IDLE or RUNNING. During those movements it would report to the Robot Status Manager that the robot is moving, so the Robot Status would be RUNNING. When in ERROR state, the Navigation Manager would stop the robot until the error clears.

This software architecture gives clear responsability to each node and makes for easy to use communication, as well as adding in flexibility and safety.

# Planned improvements

## Behavior Trees

It would be benefitial to add Behavior Trees to the Robot Status Manager, as it can easily become overcomplicated with if statements, when its functionality grows, taking care of all other possible scenarios the robot could be involved in.

## Config Management

Currently the thresholds, such as the maximum temperature, or the minimum accepted battery percentage, are harcoded to some extent. This is not a good way to deal with parameters that may need to be changed by the user at runtime. Therefore, they should be dealt in a more flexible manner in config files, or having some sort of database for it.

## Hardware integration

In the simulation we can see there are some red emergency buttons which could be used to simulate the button presses by interacting with them with the mouse, perhaps. That would make it a better simulation and use the hardware found in simulation as in the real robot, instead of manually mocking the button press by manually sending a message on the /emergency_button_press topic.

Furthermore, that could also be extrapolated to the temperature, gps, battery level, by using simulated hardware and sensors, instead of just mocking its values. It can be seen that gazebo has an Atmosphere with a temperature (which seems to be in Kelvin), which could be used to easily modify it and test in gazebo.

# Implementation Challenges

This was the first project using ROS and Gazebo, which was a challenge on itself. For a long time, there was the desire to learn how to use it, and finally that was achieved. Perhaps, this was one of the main contributing factors to the simplicity of the code and the lack of some functionality, as there was a tight time limit for this assignment.

Another major challenge, was trying to implement the robot behaviour in a separate repository from the main POLARIS_GEM_e2 code. As the main repository could not be modified and some of the desired functionality and communication methods were missing from there, it added some constraint to the design.

Finally, as this is a side project there was a lack of time within the week required to finish it. It would have been ideal to have be able to invest more into it and really have a perfect finished product.

# System Performance

The PC used in this project is 10 years old, therefore, it could not really run the simulation with more than 3 fps, even without any of this project running.
Therefore, it is not really possible to do a valid performance analysis on it.
