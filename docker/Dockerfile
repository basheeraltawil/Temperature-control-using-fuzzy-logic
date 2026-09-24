# ROS 2 Humble image with the agriclimate stack (digital twin by default).
#   docker build -f docker/Dockerfile -t agriclimate .
#   docker run --rm -it agriclimate                                   # digital twin
#   docker run --rm -it --net=host agriclimate \
#       ros2 launch agriclimate_ros hardware.launch.py plc_host:=192.168.0.10
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/agriclimate
COPY . .
RUN pip3 install --no-cache-dir -e ".[ai,field]"
RUN . /opt/ros/humble/setup.sh && cd ros2_ws && colcon build
RUN printf '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /opt/agriclimate/ros2_ws/install/setup.bash\nexec "$@"\n' \
      > /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "agriclimate_ros", "digital_twin.launch.py", "time_scale:=60"]
