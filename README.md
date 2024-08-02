# POLARIS_Robot_Manager

This s a tiny Robot manager Using Gazebo, ROS and POLARIS_GEM_e2

# Dependencies

## Docker

Please install docker in you machine if you don´t already have it.
https://docs.docker.com/engine/install/

## VcXsrv

To be able to get the windows from Docker display in a Windows machine, you would need VcXsrv
https://sourceforge.net/projects/vcxsrv/

All other dependencies will be downloaded and installed in the Docker container

# Build

First of all the Docker container needs to be built from the repository directory with:

```
docker build -t polaris_simulator .
```

# Run Docker container

Docker compose has been used to add useful parameters to be able to get Gazebo and RViz windows on Windows machines.
Run the following from the repository directory:

```
docker-compose up
```

# Attach a terminal to the container

If needed, attach a terminal to the container like so:

First, Get the container ID or NAME

```
docker ps
```

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
