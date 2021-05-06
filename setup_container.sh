#  For making the GUI stuff work
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

CONTAINER_NAME=gpd-ros
CONTAINER_USER=gpd

docker run -itd \
           --gpus all \
           --volume=$XSOCK:$XSOCK \
           --volume=$XAUTH:$XAUTH \
           --net host \
           --env="XAUTHORITY=${XAUTH}" \
           --env="DISPLAY" \
           --name="${CONTAINER_NAME}" \
           --cap-add sys_ptrace \
           -p 127.0.0.1:2222:22 \
           --volume=`pwd`/docker_dir:/home/$CONTAINER_USER/docker_dir \
           --volume=`pwd`/ws_gpd_tiago:/home/$CONTAINER_USER/docker_dir/ws_gpd_tiago \
           --user=$CONTAINER_USER \
           --privileged \
           pcl-docker:gpd-ros-full \
           terminator