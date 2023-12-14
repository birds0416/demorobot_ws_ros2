#!/bin/bash
xhost +

docker run -it \
--net=host \
--ipc=host \
--pid=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
-v /home/cornersdev/demorobot/demorobot_ws/server:/root/demorobot_ws \
-p 9090:9090 \
-p 8888:8888 \
demorobot/server:0.0.2.4.1 /bin/bash
