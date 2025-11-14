#!/bin/bash
#
# AutoSDV Container Run Script
# Use this to run the container after pulling from DockerHub
#

DOCKER_IMAGE="misuhsieh001/autosdv:2025.02-latest"
CONTAINER_NAME="autosdv_container"

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' already exists."
    read -p "Remove and recreate? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker rm -f "${CONTAINER_NAME}"
    else
        echo "Starting existing container..."
        docker start -ai "${CONTAINER_NAME}"
        exit 0
    fi
fi

# Run new container
echo "Creating and starting container..."
docker run -it --name "${CONTAINER_NAME}" \
    --gpus all \
    --net host \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/AutoSDV:rw \
    -v /dev:/dev:rw \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    "${DOCKER_IMAGE}" /bin/bash

