#!/bin/bash

container_name=HRISim
image_name=hrisim-docker
host_folder=$(pwd)/shared
container_folder=/root/shared

xhost +local:

echo " "
echo "Starting docker engine..."
systemctl start docker

echo " "
echo "Building ${container_name} docker..."
docker build -f hrisim_docker/Dockerfile -t ${image_name} .

echo " "
echo "Removing old images..."
# docker rmi $(docker images -qa -f 'dangling=true')
dangling_images=$(docker images -qa -f 'dangling=true')
if [ -n "$dangling_images" ]; then
    docker rmi $dangling_images
fi

echo " "
echo "Running docker container..."
# docker run --name ${container_name} -it --rm --network host -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e QT_X11_NO_MITSHM=1 -v $HOME/.Xauthority:/root/.Xauthority:rw -v /tmp/.X11-unix:/tmp/.X11-unix ${image_name} --privileged --gpus all --entrypoint /usr/local/bin/robot_entrypoint.sh $1 $2
docker run --privileged --network host \
           --gpus all \
           --env="NVIDIA_DRIVER_CAPABILITIES=all" \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           -v /dev/dri:/dev/dri \
           -v $host_folder:$container_folder \
           --rm \
           --entrypoint /usr/local/bin/robot_entrypoint.sh \
           -it ${image_name}  \
           $1 $2

echo " "
echo "Closing docker container..."
xhost -local: