#!/bin/bash

IMAGE_NAME="asserehab/mobile-robots-lab:latest"
CONTAINER_NAME="MobileRobots"
WORKSPACE_DIR="$(pwd)"

echo "🔹 Pulling Docker image (safe to repeat)..."
docker pull $IMAGE_NAME

# Check if container exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "✅ Container exists."

    # Check if container is running
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "▶️ Container already running."
    else
        echo "⏯️ Container exists but stopped. Starting it..."
        docker start $CONTAINER_NAME
    fi

else
    echo "🆕 Container does not exist. Creating & running it..."

    docker run -dit \
      --name $CONTAINER_NAME \
      --network host \
      -e DISPLAY=:0 \
      -e WAYLAND_DISPLAY=wayland-0 \
      -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
      -e QT_QPA_PLATFORM=xcb \
      -v /mnt/wslg:/mnt/wslg \
      -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
      -v "$WORKSPACE_DIR":/workspace \
      $IMAGE_NAME
fi

echo "🖥️ Opening terminal inside container..."
docker exec -it $CONTAINER_NAME bash
