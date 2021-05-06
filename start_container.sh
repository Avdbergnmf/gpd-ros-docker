# allow x server connection
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:root
# to set up the right environment variables in CLion
echo "Set \$DISPLAY parameter to $DISPLAY" 

CONTAINER_NAME=gpd-ros

docker start $CONTAINER_NAME
docker exec -it $CONTAINER_NAME terminator
# docker attach $CONTAINER_NAME

# disallow x server connection
xhost -local:root
