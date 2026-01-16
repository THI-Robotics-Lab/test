docker pull asserehab/mobile-robots-lab

docker run -it \
  --network host \
   --name MobileRobots \
  -e DISPLAY=:0 \
  -e WAYLAND_DISPLAY=wayland-0 \
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
  -e QT_QPA_PLATFORM=xcb \
  -v /mnt/wslg:/mnt/wslg \
  -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  moblab
