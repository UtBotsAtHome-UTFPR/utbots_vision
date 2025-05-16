FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libgl1 \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu118 \
    ultralytics \
    supervision \
    cv-bridge

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

RUN . /opt/ros/humble/setup.sh && \
    colcon build

CMD ["bash"]
