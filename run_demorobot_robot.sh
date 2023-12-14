#!/bin/bash
xhost +

docker run -it \
--net=host \
--ipc=host \
--pid=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/cornersdev/demorobot/demorobot_ws/robot \
--volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
-v /home/cornersdev/demorobot/demorobot_ws/robot:/root/demorobot_ws \
-p 9091:9091 \
-p 8889:8889 \
demorobot/robot:0.0.1.4 /bin/bash
